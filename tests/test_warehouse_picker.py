"""
Tests for the Gazebo-side fleet stack (`ros2_ws/src/warehouse_picker`).

None of these import ROS. That is the point of keeping the decision-making in
`agent_core` and its neighbours: the behaviour that runs on the robot can be
checked without a simulator, a middleware or a network.
"""

import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "ros2_ws" / "src" / "warehouse_picker"
sys.path.insert(0, str(PKG))

from warehouse_picker.agent_core import EdgeAgent                    # noqa: E402
from warehouse_picker.allocation import Task, TaskPool, route_cost   # noqa: E402
from warehouse_picker.navigation import (AStar, Limits, LocalPlanner,  # noqa: E402
                                         Obstacle, PathTracker, wrap)
from warehouse_picker.occupancy import GridMap, ROBOT_RADIUS, SAFETY_MARGIN  # noqa: E402
from warehouse_picker.protocol import (Coordinator, FleetState, PeerTable,  # noqa: E402
                                       find_wait_cycle, rank_key)
from warehouse_picker.stations import derive_stations                # noqa: E402

LAYOUT_PATH = PKG / "config" / "warehouse_layout.json"
PLAN_PATH = PKG / "maps" / "warehouse_plan.json"


def load_layout():
    return json.loads(LAYOUT_PATH.read_text())


def load_plan():
    return GridMap.load_plan(PLAN_PATH)


class TestMap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layout = load_layout()
        cls.plan = load_plan()

    def test_bounds_are_the_building(self):
        self.assertEqual(self.layout["bounds"], [-15.0, -25.0, 15.0, 25.0])

    def test_racks_are_solid_and_aisles_are_not(self):
        fine = GridMap.from_layout(self.layout, 0.25)
        self.assertTrue(fine.at(*fine.world_to_grid(0.094, -13.0)),
                        "shelf_big_3 should be occupied")
        self.assertFalse(fine.at(*fine.world_to_grid(2.9, -13.0)),
                         "the aisle beside it should be free")

    def test_perimeter_walls_are_closed(self):
        fine = GridMap.from_layout(self.layout, 0.05)
        for row in range(0, fine.h, 37):
            self.assertTrue(fine.at(0, row))
            self.assertTrue(fine.at(fine.w - 1, row))

    def test_outside_the_map_is_never_drivable(self):
        self.assertTrue(self.plan.at(-1, 0))
        self.assertTrue(self.plan.at(0, self.plan.h + 5))

    def test_plan_grid_is_inflated(self):
        """A cell 0.3 m from a rack is free on the raw map and blocked here."""
        fine = GridMap.from_layout(self.layout, 0.05)
        x, y = 1.35, -13.0            # just off shelf_big_3's east face
        self.assertLess(fine.clearance(x, y), ROBOT_RADIUS + SAFETY_MARGIN)
        self.assertFalse(fine.at(*fine.world_to_grid(x, y)))
        self.assertTrue(self.plan.at(*self.plan.world_to_grid(x, y)))

    def test_clearance_layer_survives_the_round_trip(self):
        self.assertIsNotNone(self.plan.clearance_cm)
        wide = self.plan.free_width(0.0, 20.0)
        narrow = self.plan.free_width(-2.9, -13.0)
        self.assertGreater(wide, narrow)
        self.assertGreater(narrow, 2.0)

    def test_corridor_width_does_not_depend_on_where_in_it_you_stand(self):
        """Reading the aisle width off the robot's own clearance makes a robot
        hugging one rack look like it is in a single-file corridor, and then
        every ordinary passing manoeuvre is treated as a head-on."""
        centre = self.plan.free_width(2.9, -13.0)
        hugging = self.plan.free_width(1.9, -13.0)
        self.assertAlmostEqual(centre, hugging, delta=0.3)
        self.assertGreater(hugging, 2.5)

    def test_floor_is_one_connected_region(self):
        regions = self.plan.connected_regions()
        free = sum(1 for v in self.plan.cells if not v)
        self.assertGreater(len(regions[0]) / free, 0.99)

    def test_raycast_hits_a_rack_at_the_right_distance(self):
        fine = GridMap.from_layout(self.layout, 0.05)
        # From the aisle centre at x=2.9 looking west, shelf_big_3's east face
        # is at x = 0.094 + 2.1/2 = 1.144.
        hit = fine.raycast(2.9, -13.0, math.pi, 10.0)
        self.assertAlmostEqual(hit, 2.9 - 1.144, delta=0.08)

    def test_raycast_returns_max_range_down_an_open_aisle(self):
        fine = GridMap.from_layout(self.layout, 0.05)
        self.assertAlmostEqual(fine.raycast(2.9, -13.0, math.pi / 2, 3.0), 3.0,
                               delta=1e-6)


class TestNav2Export(unittest.TestCase):
    """The published map has to be loadable by map_server, not just by us."""

    @classmethod
    def setUpClass(cls):
        cls.pgm = PKG / "maps" / "warehouse.pgm"
        cls.yaml = PKG / "maps" / "warehouse.yaml"
        cls.grid = GridMap.from_layout(load_layout(), 0.05)

    def test_the_yaml_describes_the_grid_that_was_exported(self):
        text = self.yaml.read_text()
        self.assertIn("image: warehouse.pgm", text)
        self.assertIn(f"resolution: {self.grid.resolution}", text)
        self.assertIn(f"origin: [{self.grid.origin_x}, {self.grid.origin_y}, 0.0]",
                      text)
        self.assertIn("negate: 0", text)

    def test_the_pgm_reads_back_as_the_same_occupancy(self):
        raw = self.pgm.read_bytes()
        self.assertTrue(raw.startswith(b"P5"))
        # header: magic, optional comments, "w h", maxval, then binary
        fields, i = [], 2
        while len(fields) < 3:
            while i < len(raw) and raw[i:i + 1].isspace():
                i += 1
            if raw[i:i + 1] == b"#":
                while raw[i:i + 1] not in (b"\n", b""):
                    i += 1
                continue
            start = i
            while i < len(raw) and not raw[i:i + 1].isspace():
                i += 1
            fields.append(int(raw[start:i]))
        width, height, maxval = fields
        pixels = raw[i + 1:]
        self.assertEqual((width, height), (self.grid.w, self.grid.h))
        self.assertEqual(maxval, 255)
        self.assertEqual(len(pixels), width * height)

        # Image row 0 is the top of the map, i.e. maximum y.
        for row in (0, self.grid.h // 3, self.grid.h - 1):
            for col in (0, self.grid.w // 2, self.grid.w - 1):
                exported = pixels[(self.grid.h - 1 - row) * width + col]
                self.assertEqual(exported == 0, bool(self.grid.at(col, row)),
                                 f"cell ({col}, {row}) disagrees with the PGM")


class TestPlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = load_plan()
        cls.astar = AStar(cls.plan)

    def test_finds_a_route_the_length_of_the_building(self):
        path = self.astar.plan((-2.9, -21.0), (0.0, 12.0))
        self.assertTrue(path)
        length = sum(math.hypot(path[i + 1][0] - path[i][0],
                                path[i + 1][1] - path[i][1])
                     for i in range(len(path) - 1))
        straight = math.hypot(0.0 - (-2.9), 12.0 - (-21.0))
        self.assertLess(length, straight * 1.6)

    def test_every_step_of_a_route_is_drivable(self):
        path = self.astar.plan((-12.9, -19.0), (8.5, 14.0))
        self.assertTrue(path)
        for (x, y) in path:
            self.assertFalse(self.plan.at(*self.plan.world_to_grid(x, y)))

    def test_refuses_the_column_blocked_aisle(self):
        """The 1.85 m aisle at x=-7.85 has structural columns in it, so no
        route exists into the middle of it. Returning one would be worse than
        returning nothing."""
        self.assertFalse(self.astar.plan((-2.9, -21.0), (-7.85, -11.0)))

    def test_simplify_keeps_the_path_clear(self):
        path = self.astar.plan((-2.9, -21.0), (2.9, -21.0))
        simple = self.astar.simplify(path)
        self.assertLessEqual(len(simple), len(path))
        for i in range(len(simple) - 1):
            self.assertTrue(self.astar.line_is_clear(simple[i], simple[i + 1]))

    def test_penalties_push_the_route_onto_another_aisle(self):
        start, goal = (-2.9, -21.0), (-2.9, -6.0)
        plain = self.astar.plan(start, goal)
        blocked = {self.plan.world_to_grid(x, y): 500.0
                   for x in (-3.4, -2.9, -2.4)
                   for y in [-20.0 + 0.25 * i for i in range(40)]}
        detour = self.astar.plan(start, goal, blocked)
        self.assertTrue(detour)
        self.assertNotEqual(plain, detour)

    def test_goals_inside_inflation_snap_out_of_it(self):
        inside = (1.35, -13.0)
        self.assertTrue(self.plan.at(*self.plan.world_to_grid(*inside)))
        path = self.astar.plan((2.9, -13.0), inside)
        self.assertTrue(path)


class TestLocalPlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = load_plan()
        cls.lim = Limits(dt=0.1)
        cls.local = LocalPlanner(cls.plan, cls.lim)

    def test_drives_forward_down_a_clear_aisle(self):
        v, w = self.local.compute(2.9, -18.0, math.pi / 2, 0.4, 0.0,
                                  (2.9, -14.0), [])
        self.assertGreater(v, 0.3)
        self.assertLess(abs(w), 0.3)

    def test_refuses_to_drive_into_an_obstacle(self):
        wall = [Obstacle(2.9, -17.4 + 0.05 * i) for i in range(12)]
        v, _w = self.local.compute(2.9, -18.0, math.pi / 2, 0.4, 0.0,
                                   (2.9, -14.0), wall)
        self.assertLessEqual(v, 0.0)

    def test_speed_cap_is_respected(self):
        v, _w = self.local.compute(2.9, -18.0, math.pi / 2, 0.4, 0.0,
                                   (2.9, -14.0), [], speed_cap=0.15)
        self.assertLessEqual(v, 0.1500001)

    def test_turns_in_place_when_the_goal_is_behind(self):
        v, w = self.local.compute(2.9, -18.0, math.pi / 2, 0.0, 0.0,
                                  (2.9, -22.0), [])
        self.assertAlmostEqual(v, 0.0, places=6)
        self.assertGreater(abs(w), 0.3)

    def test_a_scan_return_at_the_planning_margin_is_still_drivable(self):
        """The map is inflated by radius+safety, so a route may legitimately
        run that close to a rack; demanding the same margin again from the
        lidar echo of that rack is what stalled the robot in a 1.3 m gap."""
        echo = [Obstacle(2.9 + (ROBOT_RADIUS + SAFETY_MARGIN), -17.0 + 0.1 * i)
                for i in range(8)]
        v, _w = self.local.compute(2.9, -18.0, math.pi / 2, 0.4, 0.0,
                                   (2.9, -14.0), echo)
        self.assertGreater(v, 0.0)


class TestPathTracker(unittest.TestCase):
    def test_carrot_sits_exactly_one_lookahead_along_the_path(self):
        tracker = PathTracker(lookahead=1.0)
        tracker.set_path([(0, 0), (2, 0), (4, 0), (6, 0)])
        for x, expected in ((0.0, 1.0), (3.5, 4.5), (5.4, 6.0)):
            cx, cy = tracker.carrot(x, 0)
            self.assertAlmostEqual(cx, expected, places=6)
            self.assertAlmostEqual(cy, 0.0, places=6)

    def test_the_carrot_is_never_behind_the_robot(self):
        """A cursor that does not advance hands back a waypoint the robot has
        already passed, and the robot then drives backwards along its route."""
        tracker = PathTracker(lookahead=1.0)
        tracker.set_path([(0, 0), (2, 0), (4, 0), (6, 0)])
        for x in (0.0, 1.7, 3.5, 4.9):
            cx, _cy = tracker.carrot(x, 0.4)      # 0.4 m off the path, as a
            self.assertGreater(cx, x)             # real robot always is

    def test_remaining_length_shrinks_as_the_robot_advances(self):
        tracker = PathTracker(lookahead=1.0)
        tracker.set_path([(0, 0), (5, 0), (10, 0)])
        far = tracker.remaining_length(0, 0)
        near = tracker.remaining_length(9, 0)
        self.assertGreater(far, near)


class TestProtocol(unittest.TestCase):
    def test_message_round_trip(self):
        state = FleetState("amr_1")
        state.x, state.y, state.yaw = 1.5, -2.5, 0.75
        state.claim = [(3, 4), (3, 5)]
        state.eta = [0.5, 1.0]
        parsed = FleetState.from_json(state.to_json())
        self.assertEqual(parsed["id"], "amr_1")
        self.assertEqual(parsed["claim"], [(3, 4), (3, 5)])
        self.assertAlmostEqual(parsed["x"], 1.5)

    def test_garbage_and_wrong_versions_are_dropped(self):
        self.assertIsNone(FleetState.from_json("not json"))
        self.assertIsNone(FleetState.from_json('{"v":0,"id":"x"}'))
        self.assertIsNone(FleetState.from_json('{"v":2}'))

    def test_rank_is_a_total_order_and_ageing_wins(self):
        low = {"priority": 1.0, "lamport": 5, "id": "amr_1"}
        high = {"priority": 1.4, "lamport": 5, "id": "amr_1"}
        self.assertGreater(rank_key(high), rank_key(low))
        a = {"priority": 1.0, "lamport": 5, "id": "amr_1"}
        b = {"priority": 1.0, "lamport": 5, "id": "amr_2"}
        self.assertNotEqual(rank_key(a), rank_key(b))

    def test_peers_expire(self):
        table = PeerTable("amr_1", ttl=1.0)
        other = FleetState("amr_2")
        table.observe(FleetState.from_json(other.to_json()), 0.0)
        self.assertEqual(len(table.alive()), 1)
        self.assertEqual(table.tick(0.5), set())
        self.assertEqual(table.tick(2.0), {"amr_2"})
        self.assertEqual(table.alive(), [])

    def test_a_robot_ignores_its_own_broadcast(self):
        table = PeerTable("amr_1")
        mine = FleetState("amr_1")
        self.assertFalse(table.observe(FleetState.from_json(mine.to_json()), 0.0))

    def test_wait_cycle_detection(self):
        peers = [{"id": "amr_2", "waiting_for": "amr_3"},
                 {"id": "amr_3", "waiting_for": "amr_1"}]
        cycle = find_wait_cycle("amr_1", "amr_2", peers)
        self.assertEqual(set(cycle), {"amr_1", "amr_2", "amr_3"})

    def test_a_chain_that_terminates_is_not_a_cycle(self):
        peers = [{"id": "amr_2", "waiting_for": "amr_3"},
                 {"id": "amr_3", "waiting_for": None}]
        self.assertIsNone(find_wait_cycle("amr_1", "amr_2", peers))

    def test_stop_and_wait_recovers_from_a_stall(self):
        """The control arm is given the timeout-and-backoff that real
        stop-and-wait systems have. Without it the scheme gridlocks outright
        and comparing against it proves nothing."""
        plan = load_plan()
        me = FleetState("amr_1")
        me.x, me.y, me.yaw = 2.9, -18.0, math.pi / 2
        peer = {"id": "amr_2", "x": 3.6, "y": -17.2, "yaw": math.pi / 2,
                "v": 0.0, "priority": 1.0, "lamport": 0, "claim": [], "eta": [],
                "waiting_for": None, "blocked": [], "retreating": False}
        coord = Coordinator(plan, policy="stop_and_wait")
        self.assertEqual(coord.decide(me, [peer], 0.0, stalled_for=0.0).speed_cap,
                         0.0)
        freed = coord.decide(me, [peer], 11.0, stalled_for=11.0)
        self.assertNotEqual(freed.speed_cap, 0.0)

    def test_stop_and_wait_freezes_and_cooperative_does_not(self):
        plan = load_plan()
        me = FleetState("amr_1")
        me.x, me.y, me.yaw = 2.9, -18.0, math.pi / 2
        peer = {"id": "amr_2", "x": 3.6, "y": -17.2, "yaw": math.pi / 2,
                "v": 0.4, "priority": 1.0, "lamport": 0, "claim": [], "eta": [],
                "waiting_for": None, "blocked": [], "retreating": False}

        naive = Coordinator(plan, policy="stop_and_wait").decide(me, [peer], 0.0)
        self.assertEqual(naive.speed_cap, 0.0)

        coop = Coordinator(plan).decide(me, [peer], 0.0)
        self.assertNotEqual(coop.speed_cap, 0.0)

    def test_a_superior_peers_claim_becomes_expensive_not_impassable(self):
        plan = load_plan()
        me = FleetState("amr_1")
        me.x, me.y = 2.9, -18.0
        cell = plan.world_to_grid(2.9, -16.0)
        peer = {"id": "amr_2", "x": 2.9, "y": -16.0, "yaw": math.pi / 2, "v": 0.3,
                "priority": 9.0, "lamport": 0, "claim": [cell], "eta": [0.5],
                "waiting_for": None, "blocked": [], "retreating": False}
        decision = Coordinator(plan).decide(me, [peer], 0.0)
        self.assertIn(cell, decision.penalties)
        self.assertLess(decision.penalties[cell], 1e5)


class TestAllocation(unittest.TestCase):
    def make_pool(self, robot_id="amr_1"):
        pool = TaskPool(robot_id)
        pool.add(Task("t0", (0, 0), (1, 1)))
        pool.add(Task("t1", (2, 2), (3, 3)))
        return pool

    def test_auction_settles_deterministically(self):
        pool = self.make_pool()
        pool.place_bid("t0", "amr_1", 5.0, 0.0)
        pool.place_bid("t0", "amr_2", 9.0, 0.0)
        self.assertIsNone(pool.settle(0.1))         # window still open
        self.assertEqual(pool.settle(2.0), "t0")    # cheapest bid is mine

    def test_the_more_expensive_bidder_does_not_win(self):
        pool = self.make_pool()
        pool.place_bid("t0", "amr_1", 9.0, 0.0)
        pool.place_bid("t0", "amr_2", 5.0, 0.0)
        self.assertIsNone(pool.settle(2.0))

    def test_ties_break_on_id_identically_everywhere(self):
        for me, expected in (("amr_1", "t0"), ("amr_2", None)):
            pool = TaskPool(me)
            pool.add(Task("t0", (0, 0), (1, 1)))
            pool.place_bid("t0", "amr_1", 5.0, 0.0)
            pool.place_bid("t0", "amr_2", 5.0, 0.0)
            self.assertEqual(pool.settle(2.0), expected)

    def test_a_peers_task_is_not_offered_to_me(self):
        pool = self.make_pool()
        pool.observe_peer("amr_2", "t0", 0.0)
        self.assertEqual([t.id for t in pool.open_tasks(0.0)], ["t1"])

    def test_a_silent_peers_task_returns_to_the_pool(self):
        pool = self.make_pool()
        pool.observe_peer("amr_2", "t0", 0.0)
        self.assertEqual(len(pool.open_tasks(0.0)), 1)
        self.assertEqual(len(pool.open_tasks(1000.0)), 2)   # lease expired

    def test_conflicting_claims_resolve_to_the_lower_id(self):
        loser = TaskPool("amr_2")
        loser.add(Task("t0", (0, 0), (1, 1)))
        loser.take("t0")
        loser.observe_peer("amr_1", "t0", 0.0)
        self.assertTrue(loser.resolve_conflicts(0.0))
        self.assertIsNone(loser.mine)

        winner = TaskPool("amr_1")
        winner.add(Task("t0", (0, 0), (1, 1)))
        winner.take("t0")
        winner.observe_peer("amr_2", "t0", 0.0)
        self.assertFalse(winner.resolve_conflicts(0.0))
        self.assertEqual(winner.mine, "t0")

    def test_merge_is_idempotent_and_commutative(self):
        a, b = TaskPool("amr_1"), TaskPool("amr_2")
        a.add(Task("t0", (0, 0), (1, 1)))
        b.add(Task("t1", (2, 2), (3, 3)))
        msg_a, msg_b = a.announce(0.0), b.announce(0.0)
        a.merge(msg_b, 0.0)
        a.merge(msg_b, 0.0)
        b.merge(msg_a, 0.0)
        self.assertEqual(sorted(a.tasks), sorted(b.tasks))

    def test_completion_is_a_tombstone_that_survives_regossip(self):
        pool = self.make_pool()
        pool.complete("t0")
        pool.merge(json.dumps({"from": "amr_2",
                               "tasks": [Task("t0", (0, 0), (1, 1)).as_dict()],
                               "done": [], "bids": {}}), 0.0)
        self.assertNotIn("t0", pool.tasks)

    def test_ageing_makes_a_neglected_task_more_attractive(self):
        pool = self.make_pool()
        task = pool.tasks["t0"]
        fresh = pool.effective_cost(task, 10.0, 0.0)
        stale = pool.effective_cost(task, 10.0, 30.0)
        self.assertLess(stale, fresh)


class TestLiveness(unittest.TestCase):
    """A stall the coordination layer cannot name must still not be permanent."""

    def make_agent(self, robot_id="amr_1", home=None):
        self.clock = [0.0]
        return EdgeAgent(robot_id, load_plan(), limits=Limits(dt=0.1), home=home,
                         clock=lambda: self.clock[0])

    def test_stalling_is_measured_by_progress_not_by_speed(self):
        """A robot shuffling back and forth in a jam clears any speed
        threshold repeatedly while going nowhere, so a speed test never sees
        the stall and the watchdog never fires."""
        agent = self.make_agent()
        for i in range(140):
            self.clock[0] = i * 0.1
            # twitch either side of the stall speed, net displacement ~0
            agent.set_pose(2.9 + 0.02 * (i % 2), -18.0, math.pi / 2,
                           0.3 if i % 2 else -0.3)
            agent._track_history.append((self.clock[0], agent.x, agent.y))
        self.assertGreater(agent._stalled_for(self.clock[0]), 5.0)

    def test_a_robot_that_is_getting_somewhere_is_not_stalled(self):
        agent = self.make_agent()
        for i in range(140):
            self.clock[0] = i * 0.1
            agent.set_pose(2.9, -18.0 + i * 0.05, math.pi / 2, 0.5)
            agent._track_history.append((self.clock[0], agent.x, agent.y))
        self.assertEqual(agent._stalled_for(self.clock[0]), 0.0)

    def test_only_one_robot_in_a_knot_backs_off(self):
        """Three robots that all back off at once are exactly as jammed, only
        further apart. The lowest-ranked one goes; the rest hold."""
        low = self.make_agent("amr_1")
        low.set_pose(2.9, -18.0, 0.0)
        high = self.make_agent("amr_3")
        high.set_pose(2.9, -18.0, 0.0)
        knot = [{"id": "amr_2", "x": 3.2, "y": -18.0, "yaw": 0.0, "v": 0.0,
                 "priority": 1.0, "lamport": 0, "stalled": True},
                {"id": "amr_3", "x": 2.6, "y": -18.0, "yaw": 0.0, "v": 0.0,
                 "priority": 1.0, "lamport": 0, "stalled": True}]
        self.assertTrue(low._my_turn_to_escape(knot))
        self.assertFalse(high._my_turn_to_escape(
            [dict(p, stalled=True) for p in knot if p["id"] != "amr_3"]
            + [{"id": "amr_1", "x": 3.2, "y": -18.0, "yaw": 0.0, "v": 0.0,
                "priority": 1.0, "lamport": 0, "stalled": True}]))

    def test_a_peer_that_is_moving_does_not_hold_up_the_escape(self):
        agent = self.make_agent("amr_3")
        agent.set_pose(2.9, -18.0, 0.0)
        self.assertTrue(agent._my_turn_to_escape(
            [{"id": "amr_1", "x": 3.2, "y": -18.0, "yaw": 0.0, "v": 0.6,
              "priority": 1.0, "lamport": 0, "stalled": False}]))

    def test_it_will_not_reverse_into_something(self):
        agent = self.make_agent()
        agent.set_pose(2.9, -18.0, math.pi / 2)
        behind = [{"id": "amr_2", "x": 2.9, "y": -18.6, "yaw": math.pi / 2,
                   "v": 0.0}]
        self.assertFalse(agent._reverse_is_clear(behind))
        self.assertTrue(agent._reverse_is_clear([]))

    def test_an_idle_robot_goes_back_to_its_standby_bay(self):
        """Otherwise it parks on the pick face it just delivered to and every
        other robot has to route around it for the rest of the shift."""
        home = (-12.9, -19.0)
        agent = self.make_agent(home=home)
        agent.set_pose(2.9, -18.0, math.pi / 2)
        for i in range(120):
            self.clock[0] = i * 0.1
            out = agent.step(self.clock[0])
        self.assertEqual(agent.state.mode, "STANDBY")
        self.assertEqual(agent.state.goal, home)
        # The bay is behind and to the side, so the first move is a turn.
        self.assertGreater(abs(out.v) + abs(out.w), 0.0)

    def test_a_robot_with_no_standby_bay_simply_stays_put(self):
        agent = self.make_agent(home=None)
        agent.set_pose(2.9, -18.0, math.pi / 2)
        for i in range(120):
            self.clock[0] = i * 0.1
            out = agent.step(self.clock[0])
        self.assertEqual(agent.state.mode, "IDLE")
        self.assertEqual((out.v, out.w), (0.0, 0.0))

    def test_work_wins_over_going_to_standby(self):
        """A robot already driving to its bay must still be able to take a job;
        otherwise it spends the trip home ignoring work it could have won."""
        agent = self.make_agent(home=(-12.9, -19.0))
        # Creep west so the robot is making progress and the stall watchdog,
        # which is not what this test is about, stays out of the way.
        for i in range(120):
            self.clock[0] = i * 0.1
            agent.set_pose(2.9 - i * 0.02, -18.0, math.pi, 0.2)
            agent.step(self.clock[0])
        self.assertEqual(agent.state.mode, "STANDBY")

        agent.submit_task(Task("t0", (2.9, -10.0), (2.9, -6.0)))
        for i in range(120, 190):
            self.clock[0] = i * 0.1
            agent.set_pose(2.9 - i * 0.02, -18.0, math.pi, 0.2)
            agent.step(self.clock[0])
        self.assertEqual(agent.state.mode, "TO_PICKUP")
        self.assertEqual(agent.state.task, "t0")


class TestBlockedAisle(unittest.TestCase):
    """Discovering, sharing and routing around something the map does not know."""

    def make_agent(self, robot_id="amr_1"):
        self.clock = [0.0]
        return EdgeAgent(robot_id, load_plan(), limits=Limits(dt=0.1),
                         clock=lambda: self.clock[0])

    def test_a_reported_block_closes_the_route_through_it(self):
        agent = self.make_agent()
        agent.set_pose(-2.9, -20.0, math.pi / 2)
        plain = agent.planner.plan((-2.9, -20.0), (-2.9, -6.0))
        self.assertTrue(plain)

        wall = {agent.grid.world_to_grid(x, -13.0)
                for x in (-4.5, -4.0, -3.5, -3.0, -2.5, -2.0, -1.5, -1.2)}
        detour = agent.planner.plan((-2.9, -20.0), (-2.9, -6.0),
                                    {cell: 1e6 for cell in wall})
        self.assertTrue(detour, "a blocked aisle must not make the goal unreachable")
        crossed = {agent.grid.world_to_grid(x, y) for (x, y) in detour}
        self.assertFalse(crossed & wall, "route still goes through the blockage")

    def test_a_block_a_peer_found_is_adopted(self):
        agent = self.make_agent()
        agent.set_pose(-2.9, -20.0, math.pi / 2)
        finder = FleetState("amr_2")
        cell = agent.grid.world_to_grid(-2.9, -13.0)
        finder.blocked = [cell]
        agent.on_mesh(finder.to_json(), 0.0)
        self.assertIn(cell, agent._blocked)

    def test_a_block_expires_rather_than_being_believed_forever(self):
        agent = self.make_agent()
        agent.set_pose(-2.9, -20.0, math.pi / 2)
        finder = FleetState("amr_2")
        cell = agent.grid.world_to_grid(-2.9, -13.0)
        finder.blocked = [cell]
        agent.on_mesh(finder.to_json(), 0.0)
        self.clock[0] = 10_000.0
        agent._expire_blocks(self.clock[0])
        self.assertNotIn(cell, agent._blocked)

    def test_a_reported_block_changes_the_route_not_the_corridor_rules(self):
        """A blocked cell is already closed in the planner. Counting it as a
        chokepoint as well makes robots give way and retreat around an obstacle
        they have already routed past, which measured 50% slower on an
        open-plan workload than not doing it."""
        plan = load_plan()
        coord = Coordinator(plan)
        self.assertFalse(coord.is_single_file(-2.9, -13.0))
        self.assertTrue(coord.is_single_file(-7.85, -13.0)
                        or plan.free_width(-7.85, -13.0) >= coord.pass_width)

    def test_the_warehouse_itself_has_no_chokepoints(self):
        """Recorded because it explains the benchmark: every aisle here takes
        two Tugbots abreast, so corridor coordination has nothing to manage and
        contention has to come from density or from blockages."""
        plan = load_plan()
        pass_width = 2 * (ROBOT_RADIUS + SAFETY_MARGIN) + 0.25
        single = 0
        for row in range(0, plan.h, 6):
            for col in range(0, plan.w, 6):
                if plan.at(col, row):
                    continue
                if plan.free_width(*plan.grid_to_world(col, row)) < pass_width:
                    single += 1
        self.assertEqual(single, 0)


class TestStations(unittest.TestCase):
    def test_pick_faces_are_all_reachable_free_cells(self):
        plan = load_plan()
        layout = load_layout()
        stations = derive_stations(plan, layout, reachable_from=(-2.9, -21.0))
        self.assertGreater(len(stations), 40)
        for st in stations:
            self.assertFalse(plan.at(*plan.world_to_grid(st.x, st.y)))


class TestEdgeAgent(unittest.TestCase):
    def make_agent(self, robot_id="amr_1", policy="cooperative"):
        self.clock = [0.0]
        plan = load_plan()
        return EdgeAgent(robot_id, plan, limits=Limits(dt=0.1),
                         clock=lambda: self.clock[0], policy=policy)

    def test_no_odometry_means_no_motion(self):
        agent = self.make_agent()
        out = agent.step(0.0)
        self.assertEqual((out.v, out.w), (0.0, 0.0))
        self.assertIn("odometry", out.telemetry["reason"])

    def test_idle_with_no_work_is_stationary(self):
        agent = self.make_agent()
        agent.set_pose(2.9, -18.0, math.pi / 2)
        out = agent.step(0.0)
        self.assertEqual((out.v, out.w), (0.0, 0.0))
        self.assertEqual(out.telemetry["mode"], "IDLE")

    def test_it_wins_an_uncontested_task_and_sets_off(self):
        agent = self.make_agent()
        agent.set_pose(2.9, -18.0, math.pi / 2)
        agent.submit_task(Task("t0", (2.9, -10.0), (2.9, -6.0)))
        moved = False
        for i in range(60):
            self.clock[0] = i * 0.1
            out = agent.step(self.clock[0])
            if out.v > 0.05:
                moved = True
                break
        self.assertTrue(moved, "agent never accelerated toward its pickup")
        self.assertEqual(agent.state.mode, "TO_PICKUP")

    def test_its_broadcast_parses_as_a_peer_state(self):
        agent = self.make_agent()
        agent.set_pose(2.9, -18.0, 0.0)
        out = agent.step(0.0)
        self.assertIsNotNone(FleetState.from_json(out.mesh))
        self.assertIsInstance(json.loads(out.tasks), dict)

    def test_it_reaches_its_speed_limit_on_a_clear_aisle(self):
        """A lookahead shorter than the local planner's rollout makes the robot
        overshoot its own aim point, which silently caps it at about 70% of the
        speed it was configured for."""
        agent = self.make_agent()
        agent.submit_task(Task("t0", (-2.9, -6.0), (-2.9, -4.0)))
        lim = agent.lim
        x, y, yaw, v, w = -2.9, -21.0, math.pi / 2, 0.0, 0.0
        peak = 0.0
        for i in range(160):
            self.clock[0] = i * 0.1
            agent.set_pose(x, y, yaw, v, w)
            out = agent.step(self.clock[0])
            v += max(-lim.a_lin * 0.1, min(lim.a_lin * 0.1, out.v - v))
            w += max(-lim.a_ang * 0.1, min(lim.a_ang * 0.1, out.w - w))
            v = max(lim.v_min, min(lim.v_max, v))
            yaw += w * 0.1
            x += v * math.cos(yaw) * 0.1
            y += v * math.sin(yaw) * 0.1
            peak = max(peak, v)
        self.assertGreater(peak, 0.95 * lim.v_max)

    def test_oncoming_traffic_makes_it_move_over_not_stop(self):
        """Two robots nose-to-nose on open floor each sit inside the other's
        clearance envelope and creep to a mutual standstill. Passing on an
        agreed side breaks that without either having to give way."""
        plan = load_plan()
        me = FleetState("amr_1")
        me.x, me.y, me.yaw = 0.0, 14.0, 0.0        # wide open northern hall
        peer = {"id": "amr_2", "x": 2.6, "y": 14.0, "yaw": math.pi, "v": 0.5,
                "priority": 9.0, "lamport": 0, "claim": [], "eta": [],
                "waiting_for": None, "blocked": [], "retreating": False}
        decision = Coordinator(plan).decide(me, [peer], 0.0)
        self.assertNotEqual(decision.lateral_bias, 0.0)
        self.assertIsNone(decision.retreat_to)
        self.assertNotEqual(decision.speed_cap, 0.0)

    def test_a_peer_directly_ahead_slows_it_down(self):
        agent = self.make_agent()
        agent.submit_task(Task("t0", (2.9, -10.0), (2.9, -6.0)))
        for i in range(40):
            self.clock[0] = i * 0.1
            agent.set_pose(2.9, -18.0 + i * 0.02, math.pi / 2, 0.4)
            free = agent.step(self.clock[0])

        blocker = FleetState("amr_9")
        blocker.x, blocker.y, blocker.yaw = 2.9, -16.6, math.pi / 2
        blocker.v = 0.0
        blocker.priority = 9.0
        self.clock[0] += 0.1
        agent.on_mesh(blocker.to_json(), self.clock[0])
        agent.set_pose(2.9, -17.2, math.pi / 2, 0.4)
        held = agent.step(self.clock[0])
        self.assertLess(held.v, free.v)

    def test_a_lost_peer_stops_constraining_it(self):
        agent = self.make_agent()
        agent.set_pose(2.9, -18.0, math.pi / 2, 0.0)
        peer = FleetState("amr_9")
        peer.x, peer.y = 3.4, -17.6
        agent.on_mesh(peer.to_json(), 0.0)
        self.assertEqual(len(agent.peers.alive()), 1)
        self.clock[0] = 30.0
        agent.step(30.0)
        self.assertEqual(agent.peers.alive(), [])


if __name__ == "__main__":
    unittest.main()
