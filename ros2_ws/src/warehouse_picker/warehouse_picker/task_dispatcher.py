#!/usr/bin/env python3
"""
Seeds the fleet's shared task list, and tops it up as work is completed.

This is not an allocator -- it never decides which robot does what. It only
publishes jobs onto the latched `/fleet/tasks` topic; the robots auction them
out among themselves. Killing this node stops new work arriving and changes
nothing else, which is the test of whether it is really a coordinator in
disguise.

    ros2 run warehouse_picker dispatcher --ros-args -p tasks:=12
"""

import json
import os
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from .allocation import Task
from .occupancy import GridMap
from .stations import derive_stations

TASK_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=20)


class TaskDispatcher(Node):
    def __init__(self):
        super().__init__("task_dispatcher")
        self.declare_parameter("tasks", 12)
        self.declare_parameter("seed", 7)
        self.declare_parameter("map_path", "")
        self.declare_parameter("layout_path", "")
        self.declare_parameter("period", 3.0)

        share = None
        map_path = self.get_parameter("map_path").value
        layout_path = self.get_parameter("layout_path").value
        if not map_path or not layout_path:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("warehouse_picker")
            map_path = map_path or os.path.join(share, "maps", "warehouse_plan.json")
            layout_path = layout_path or os.path.join(share, "config",
                                                      "warehouse_layout.json")

        grid = GridMap.load_plan(map_path)
        layout = json.loads(open(layout_path).read())
        start = (layout["robots"][0]["x"], layout["robots"][0]["y"]) \
            if layout.get("robots") else None
        self.stations = derive_stations(grid, layout, reachable_from=start)
        if not self.stations:
            raise RuntimeError("no reachable pick faces; check the map")

        self.rng = random.Random(int(self.get_parameter("seed").value))
        self.pub = self.create_publisher(String, "/fleet/tasks", TASK_QOS)
        self.create_subscription(String, "/fleet/tasks", self.on_tasks, TASK_QOS)

        self.issued = {}
        self.done = set()
        self.total = int(self.get_parameter("tasks").value)
        self.create_timer(float(self.get_parameter("period").value), self.top_up)
        self.get_logger().info(
            f"dispatcher: {len(self.stations)} reachable pick faces, "
            f"keeping {self.total} jobs in flight")

    def on_tasks(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.done.update(d.get("done", []))

    def top_up(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        live = {tid: t for tid, t in self.issued.items() if tid not in self.done}
        made = 0
        while len(live) < self.total:
            pick, drop = self._crossing_pair()
            tid = f"t{len(self.issued):03d}"
            task = Task(tid, pick.xy(), drop.xy(), created=now,
                        label=f"{pick.name} -> {drop.name}")
            self.issued[tid] = task
            live[tid] = task
            made += 1
        if made:
            self.get_logger().info(f"issued {made} task(s); "
                                   f"{len(self.done)} completed so far")
        self.pub.publish(String(data=json.dumps({
            "from": "dispatcher",
            "tasks": [t.as_dict() for t in live.values()],
            "done": sorted(self.done),
            "bids": {},
        }, separators=(",", ":"))))

    def _crossing_pair(self):
        """Pick and drop on opposite halves of the building, so routes overlap."""
        north = [s for s in self.stations if s.y > 0]
        south = [s for s in self.stations if s.y <= 0]
        if north and south:
            if self.rng.random() < 0.5:
                return self.rng.choice(north), self.rng.choice(south)
            return self.rng.choice(south), self.rng.choice(north)
        return tuple(self.rng.sample(self.stations, 2))


def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
