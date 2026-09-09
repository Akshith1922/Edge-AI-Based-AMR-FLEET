#!/usr/bin/env python3
"""
ROS 2 node wrapper around `EdgeAgent`.

This file is deliberately thin. It converts ROS messages into the four things
the agent understands -- a pose, a scan, a peer broadcast, a task broadcast --
and converts the agent's answer back into a Twist. All of the decision-making
lives in `agent_core` and its neighbours, which import no ROS at all, so the
behaviour you get here is the behaviour the twin measured and the unit tests
check.

    ros2 run warehouse_picker edge_agent --ros-args -p robot_id:=amr_1

On the mesh being decentralised: `/fleet/mesh` is a ROS 2 topic, and ROS 2 has
no broker. DDS discovers peers over multicast and every message travels
directly from publisher to subscriber, so a topic here really is a peer-to-peer
mesh and not a hub wearing a disguise -- kill any one node, including the one
you might call the leader, and the others carry on. What it is *not* is a claim
that the robots would find each other across a network without multicast; on a
real shop floor that is a DDS discovery-server or a Zenoh bridge, and it is a
deployment concern rather than a change to any of this code.
"""

import json
import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .agent_core import EdgeAgent
from .navigation import Limits
from .occupancy import GridMap
from .stations import charger_station

CONTROL_HZ = 10.0

# Best-effort and shallow: a heartbeat that arrives late is worse than useless,
# and a robot that blocks on delivering one to a peer that has gone offline is
# exactly the single point of failure this design exists to avoid.
MESH_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=10)
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
# Tasks are latched so a robot that joins late inherits the outstanding work.
TASK_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=20)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EdgeFleetAgentNode(Node):
    def __init__(self):
        super().__init__("edge_fleet_agent")

        self.declare_parameter("robot_id", "amr_1")
        self.declare_parameter("map_path", "")
        self.declare_parameter("layout_path", "")
        self.declare_parameter("policy", "cooperative")
        self.declare_parameter("max_speed", 0.8)
        self.declare_parameter("battery", 100.0)

        self.robot_id = self.get_parameter("robot_id").value
        map_path = self.get_parameter("map_path").value or self._default_map()

        grid = GridMap.load_plan(map_path)
        charger = charger_station(grid)
        limits = Limits(v_max=float(self.get_parameter("max_speed").value),
                        dt=1.0 / CONTROL_HZ)

        self.agent = EdgeAgent(
            self.robot_id, grid, limits=limits,
            charger=charger.xy() if charger else None,
            home=self._home_from_layout(),
            battery=float(self.get_parameter("battery").value),
            clock=self._now,
            policy=self.get_parameter("policy").value)

        self.cmd_pub = self.create_publisher(Twist, f"/{self.robot_id}/cmd_vel", 10)
        self.mesh_pub = self.create_publisher(String, "/fleet/mesh", MESH_QOS)
        self.task_pub = self.create_publisher(String, "/fleet/tasks", TASK_QOS)
        self.telemetry_pub = self.create_publisher(String, "/fleet/telemetry", MESH_QOS)

        self.create_subscription(Odometry, f"/{self.robot_id}/odom",
                                 self.on_odom, SENSOR_QOS)
        self.create_subscription(LaserScan, f"/{self.robot_id}/scan",
                                 self.on_scan, SENSOR_QOS)
        self.create_subscription(String, "/fleet/mesh", self.on_mesh, MESH_QOS)
        self.create_subscription(String, "/fleet/tasks", self.on_tasks, TASK_QOS)

        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
        self._odom_seen = False
        self.get_logger().info(
            f"[{self.robot_id}] edge agent online: {grid.w}x{grid.h} map @ "
            f"{grid.resolution} m, policy={self.agent.policy}, "
            f"charger={'yes' if charger else 'none'}")

    def _home_from_layout(self):
        """Standby bay: this robot's spawn pose, read from the world layout."""
        path = self.get_parameter("layout_path").value
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("warehouse_picker"),
                                "config", "warehouse_layout.json")
        try:
            layout = json.loads(Path(path).read_text())
        except (OSError, ValueError):
            return None
        for robot in layout.get("robots", []):
            if robot.get("name") == self.robot_id:
                return (robot["x"], robot["y"])
        return None

    def _default_map(self):
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("warehouse_picker"),
                            "maps", "warehouse_plan.json")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- subscriptions -----------------------------------------------------

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.agent.set_pose(p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation),
                            msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if not self._odom_seen:
            self._odom_seen = True
            self.get_logger().info(
                f"[{self.robot_id}] odometry acquired at ({p.x:.2f}, {p.y:.2f})")

    def on_scan(self, msg):
        self.agent.set_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                            msg.range_max, now=self._now())

    def on_mesh(self, msg):
        self.agent.on_mesh(msg.data, self._now())

    def on_tasks(self, msg):
        self.agent.on_tasks(msg.data, self._now())

    # -- the loop ----------------------------------------------------------

    def control_loop(self):
        out = self.agent.step(self._now())

        twist = Twist()
        twist.linear.x = float(out.v)
        twist.angular.z = float(out.w)
        self.cmd_pub.publish(twist)

        self.mesh_pub.publish(String(data=out.mesh))
        self.task_pub.publish(String(data=out.tasks))
        self.telemetry_pub.publish(String(data=_json(out.telemetry)))


def _json(obj):
    import json
    return json.dumps(obj, separators=(",", ":"))


def main(args=None):
    rclpy.init(args=args)
    node = EdgeFleetAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())      # always leave the robot stopped
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
