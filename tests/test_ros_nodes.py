"""
Exercise the ROS glue without ROS.

`edge_fleet_agent`, `task_dispatcher` and `fleet_dashboard` are thin, but thin
is not the same as trivially correct: a mistyped parameter name, a callback
with the wrong arity, or a quaternion converted the wrong way round all fail
only at runtime, and finding them means a full Gazebo launch. Standing up a
small `rclpy` stub means they fail here instead, in two seconds.

The stub is deliberately shallow. It is not a ROS simulator and does not try to
be one -- it records what was published and lets the tests invoke callbacks by
hand, which is enough to check the wiring.
"""

import json
import math
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "ros2_ws" / "src" / "warehouse_picker"
sys.path.insert(0, str(PKG))


# --------------------------------------------------------------------------
# the stub
# --------------------------------------------------------------------------

class _Pub:
    def __init__(self, topic):
        self.topic = topic
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Sub:
    def __init__(self, topic, cb):
        self.topic, self.cb = topic, cb


class _Timer:
    def __init__(self, period, cb):
        self.period, self.cb = period, cb


class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(self.t * 1e9))


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(msg)

    warn = warning = error = info


class _Node:
    def __init__(self, name):
        self.name = name
        self._params = {}
        self.pubs = {}
        self.subs = {}
        self.timers = []
        self._clock = _Clock()
        self._logger = _Logger()

    def declare_parameter(self, name, default):
        self._params.setdefault(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def set_parameter_value(self, name, value):
        self._params[name] = value

    def create_publisher(self, _type, topic, _qos):
        pub = _Pub(topic)
        self.pubs[topic] = pub
        return pub

    def create_subscription(self, _type, topic, cb, _qos):
        sub = _Sub(topic, cb)
        self.subs[topic] = sub
        return sub

    def create_timer(self, period, cb):
        timer = _Timer(period, cb)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass


def _msg_type(*fields):
    def make(**kwargs):
        obj = types.SimpleNamespace()
        for f in fields:
            setattr(obj, f, kwargs.get(f))
        return obj
    return make


def install_stub():
    """Register a minimal rclpy and the message packages the nodes import."""
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda **_kw: None
    rclpy.shutdown = lambda: None
    rclpy.ok = lambda: True
    rclpy.spin = lambda _node: None

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node
    rclpy.node = node_mod

    qos = types.ModuleType("rclpy.qos")
    for name in ("ReliabilityPolicy", "HistoryPolicy", "DurabilityPolicy"):
        setattr(qos, name, types.SimpleNamespace(
            BEST_EFFORT=0, RELIABLE=1, KEEP_LAST=2, TRANSIENT_LOCAL=3, VOLATILE=4))
    qos.QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    rclpy.qos = qos

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.qos"] = qos

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    geo = types.ModuleType("geometry_msgs")
    geo_msg = types.ModuleType("geometry_msgs.msg")
    geo_msg.Twist = Twist
    geo.msg = geo_msg
    sys.modules["geometry_msgs"] = geo
    sys.modules["geometry_msgs.msg"] = geo_msg

    nav = types.ModuleType("nav_msgs")
    nav_msg = types.ModuleType("nav_msgs.msg")
    nav_msg.Odometry = _msg_type("pose", "twist")
    nav.msg = nav_msg
    sys.modules["nav_msgs"] = nav
    sys.modules["nav_msgs.msg"] = nav_msg

    sen = types.ModuleType("sensor_msgs")
    sen_msg = types.ModuleType("sensor_msgs.msg")
    sen_msg.LaserScan = _msg_type("ranges", "angle_min", "angle_increment",
                                  "range_max")
    sen.msg = sen_msg
    sys.modules["sensor_msgs"] = sen
    sys.modules["sensor_msgs.msg"] = sen_msg

    class String:
        def __init__(self, data=""):
            self.data = data

    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = String
    std.msg = std_msg
    sys.modules["std_msgs"] = std
    sys.modules["std_msgs.msg"] = std_msg

    ament = types.ModuleType("ament_index_python")
    pkgs = types.ModuleType("ament_index_python.packages")
    pkgs.get_package_share_directory = lambda name: str(PKG)
    ament.packages = pkgs
    sys.modules["ament_index_python"] = ament
    sys.modules["ament_index_python.packages"] = pkgs
    return String, Twist


String, Twist = install_stub()

from warehouse_picker import edge_fleet_agent, fleet_dashboard, task_dispatcher  # noqa: E402
from warehouse_picker.protocol import FleetState                                # noqa: E402


def odom(x, y, yaw, v=0.0):
    return types.SimpleNamespace(
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0,
                                              z=math.sin(yaw / 2),
                                              w=math.cos(yaw / 2)))),
        twist=types.SimpleNamespace(twist=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=v),
            angular=types.SimpleNamespace(z=0.0))))


class TestEdgeAgentNode(unittest.TestCase):
    def setUp(self):
        self.node = edge_fleet_agent.EdgeFleetAgentNode()

    def test_it_wires_up_the_topics_it_needs(self):
        self.assertIn("/amr_1/cmd_vel", self.node.pubs)
        self.assertIn("/fleet/mesh", self.node.pubs)
        self.assertIn("/fleet/tasks", self.node.pubs)
        self.assertIn("/fleet/telemetry", self.node.pubs)
        for topic in ("/amr_1/odom", "/amr_1/scan", "/fleet/mesh", "/fleet/tasks"):
            self.assertIn(topic, self.node.subs)

    def test_it_runs_a_control_loop_at_ten_hertz(self):
        self.assertTrue(self.node.timers)
        self.assertAlmostEqual(self.node.timers[0].period, 0.1, places=6)

    def test_odometry_yaw_and_velocity_are_converted(self):
        """Pose is composed onto the spawn (see the frame test below); yaw and
        velocity are checked here against the composition."""
        _sx, _sy, syaw = self.node.spawn
        self.node.subs["/amr_1/odom"].cb(odom(0.0, 0.0, 1.2, 0.4))
        self.assertAlmostEqual(self.node.agent.yaw, 1.2 + syaw, places=6)
        self.assertAlmostEqual(self.node.agent.v, 0.4)

    def test_odometry_is_composed_onto_the_spawn_pose(self):
        """Gazebo's DiffDrive reports odometry relative to where the robot
        started, so feeding it straight to a planner holding a map of the whole
        building puts every robot at the map origin. amr_1 spawns at
        (-2.9, -21.0) facing north; one metre 'forward' in the odometry frame
        is one metre north in the world."""
        self.assertIsNotNone(self.node.spawn)
        sx, sy, syaw = self.node.spawn
        self.assertAlmostEqual(sx, -2.9, places=3)
        self.assertAlmostEqual(sy, -21.0, places=3)

        self.node.subs["/amr_1/odom"].cb(odom(0.0, 0.0, 0.0))
        self.assertAlmostEqual(self.node.agent.x, sx, places=3)
        self.assertAlmostEqual(self.node.agent.y, sy, places=3)
        self.assertAlmostEqual(self.node.agent.yaw, syaw, places=3)

        self.node.subs["/amr_1/odom"].cb(odom(1.0, 0.0, 0.0))
        self.assertAlmostEqual(self.node.agent.x, sx, places=3)
        self.assertAlmostEqual(self.node.agent.y, sy + 1.0, places=3)

        self.node.subs["/amr_1/odom"].cb(odom(0.0, 1.0, 0.0))
        self.assertAlmostEqual(self.node.agent.x, sx - 1.0, places=3)
        self.assertAlmostEqual(self.node.agent.y, sy, places=3)

    def test_the_composed_pose_starts_on_free_floor(self):
        """The whole point: an uncomposed pose would put the robot at (0, 0),
        which on this map is open floor too -- so the failure is silent. Check
        the robot starts where the world says it does."""
        self.node.subs["/amr_1/odom"].cb(odom(0.0, 0.0, 0.0))
        grid = self.node.agent.grid
        self.assertFalse(grid.at(*grid.world_to_grid(self.node.agent.x,
                                                     self.node.agent.y)))
        self.assertGreater(math.hypot(self.node.agent.x, self.node.agent.y), 5.0)

    def test_a_scan_reaches_the_agent(self):
        scan = types.SimpleNamespace(ranges=[3.0] * 91, angle_min=-1.47,
                                     angle_increment=2 * 1.47 / 90,
                                     range_max=5.0)
        self.node.subs["/amr_1/scan"].cb(scan)
        self.assertIsNotNone(self.node.agent._scan)
        self.assertEqual(len(self.node.agent._scan[0]), 91)

    def test_the_loop_publishes_a_twist_and_a_parsable_heartbeat(self):
        self.node.subs["/amr_1/odom"].cb(odom(0.0, 0.0, 0.0))
        self.node.timers[0].cb()
        cmd = self.node.pubs["/amr_1/cmd_vel"].sent[-1]
        self.assertIsInstance(cmd.linear.x, float)
        self.assertIsInstance(cmd.angular.z, float)
        mesh = self.node.pubs["/fleet/mesh"].sent[-1].data
        self.assertIsNotNone(FleetState.from_json(mesh))
        telemetry = json.loads(self.node.pubs["/fleet/telemetry"].sent[-1].data)
        self.assertEqual(telemetry["id"], "amr_1")

    def test_a_peers_heartbeat_is_taken_in(self):
        peer = FleetState("amr_2")
        peer.x, peer.y = 3.5, -17.5
        self.node.subs["/fleet/mesh"].cb(String(data=peer.to_json()))
        self.assertEqual([p["id"] for p in self.node.agent.peers.alive()], ["amr_2"])

    def test_it_ignores_the_echo_of_its_own_heartbeat(self):
        self.node.subs["/amr_1/odom"].cb(odom(0.0, 0.0, 0.0))
        self.node.timers[0].cb()
        mine = self.node.pubs["/fleet/mesh"].sent[-1].data
        self.node.subs["/fleet/mesh"].cb(String(data=mine))
        self.assertEqual(self.node.agent.peers.alive(), [])

    def test_a_malformed_message_does_not_take_the_node_down(self):
        for junk in ("", "null", "{", '{"v":99}', "[1,2,3]"):
            self.node.subs["/fleet/mesh"].cb(String(data=junk))
            self.node.subs["/fleet/tasks"].cb(String(data=junk))
        self.assertEqual(self.node.agent.peers.alive(), [])

    def test_it_drives_once_it_has_a_pose_and_a_task(self):
        """Pickup is up the aisle the robot spawns in, so it should set off
        without needing to turn round first."""
        self.node.subs["/fleet/tasks"].cb(String(data=json.dumps({
            "from": "dispatcher", "done": [], "bids": {},
            "tasks": [{"id": "t0", "pickup": [-2.9, -14.0], "dropoff": [-2.9, -10.0],
                       "priority": 1.0, "created": 0.0, "label": "t0"}]})))
        moved = False
        for i in range(80):
            self.node._clock.t = i * 0.1
            self.node.subs["/amr_1/odom"].cb(odom(0.0, 0.0, 0.0))
            self.node.timers[0].cb()
            if self.node.pubs["/amr_1/cmd_vel"].sent[-1].linear.x > 0.05:
                moved = True
                break
        self.assertTrue(moved, "node never commanded forward motion")


class TestDispatcherNode(unittest.TestCase):
    def test_it_issues_reachable_tasks_and_stops_reissuing_completed_ones(self):
        node = task_dispatcher.TaskDispatcher()
        node.timers[0].cb()
        msg = json.loads(node.pubs["/fleet/tasks"].sent[-1].data)
        self.assertEqual(len(msg["tasks"]), node.total)
        first = msg["tasks"][0]["id"]

        node.subs["/fleet/tasks"].cb(String(data=json.dumps(
            {"from": "amr_1", "tasks": [], "done": [first], "bids": {}})))
        node.timers[0].cb()
        msg = json.loads(node.pubs["/fleet/tasks"].sent[-1].data)
        self.assertNotIn(first, [t["id"] for t in msg["tasks"]])
        self.assertEqual(len(msg["tasks"]), node.total)

    def test_pickup_and_dropoff_are_never_the_same_place(self):
        node = task_dispatcher.TaskDispatcher()
        node.timers[0].cb()
        for task in json.loads(node.pubs["/fleet/tasks"].sent[-1].data)["tasks"]:
            self.assertNotEqual(task["pickup"], task["dropoff"])


class TestDashboardNode(unittest.TestCase):
    def setUp(self):
        self.node = fleet_dashboard.FleetDashboard()

    def tearDown(self):
        self.node.server.shutdown()
        self.node.server.server_close()

    def test_it_listens_on_the_topic_the_agents_actually_publish(self):
        self.assertIn("/fleet/mesh", self.node.subs)
        self.assertIn("/fleet/telemetry", self.node.subs)

    def test_a_heartbeat_shows_up_in_the_snapshot(self):
        state = FleetState("amr_2")
        state.x, state.y, state.battery, state.mode = 4.0, -3.0, 71.5, "TO_PICKUP"
        self.node.subs["/fleet/mesh"].cb(String(data=state.to_json()))
        import time
        snap = self.node.store.snapshot(time.time())
        self.assertEqual(len(snap["robots"]), 1)
        robot = snap["robots"][0]
        self.assertEqual(robot["id"], "amr_2")
        self.assertAlmostEqual(robot["battery"], 71.5)
        self.assertEqual(robot["mode"], "TO_PICKUP")
        self.assertTrue(robot["online"])

    def test_a_robot_that_goes_quiet_is_marked_offline(self):
        import time
        state = FleetState("amr_2")
        self.node.subs["/fleet/mesh"].cb(String(data=state.to_json()))
        snap = self.node.store.snapshot(time.time() + 60.0)
        self.assertFalse(snap["robots"][0]["online"])

    def test_the_page_carries_the_real_map_bounds(self):
        self.assertIn(b"[-15.0, -25.0, 15.0, 25.0]", fleet_dashboard.Handler.page)
        self.assertTrue(fleet_dashboard.Handler.map_png.startswith(b"\x89PNG"))


if __name__ == "__main__":
    unittest.main()
