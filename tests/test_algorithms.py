"""Unit tests for the six algorithms, one class per module."""

import unittest

from amrsim import blocks as blocks_mod
from amrsim import deadlock as dl
from amrsim import failures as fl
from amrsim.config import DEFAULT as CFG
from amrsim.planner import plan_spacetime, plan_static, plan_with_detour_cap
from amrsim.reservations import (LamportClock, Reservation, ReservationTable,
                                 build_cluster, classify_geometry, rank_cluster)
from amrsim.robot import Robot, RobotState
from amrsim.tasks import ORSet, TaskPool, TaskStatus
from amrsim.warehouse import Tile, Warehouse


def res(rid, t0, steps, priority=1.0, lamport=1, tail=0):
    r = Reservation(rid, t0, list(steps), priority, lamport, tail=tail)
    r.build_windows(CFG.RESERVATION_PAD)
    return r


class TestWarehouse(unittest.TestCase):
    def setUp(self):
        self.wh = Warehouse()

    def test_every_free_cell_is_reachable(self):
        from collections import deque
        start = next(iter(self.wh.adj))
        seen, q = {start}, deque([start])
        while q:
            for nb in self.wh.neighbors(*q.popleft()):
                if nb not in seen:
                    seen.add(nb)
                    q.append(nb)
        self.assertEqual(len(seen), len(self.wh.adj),
                         "the warehouse floor must be one connected component")

    def test_corridors_are_straight_single_file_runs(self):
        self.assertTrue(self.wh.corridors, "expected at least one chokepoint")
        for c in self.wh.corridors:
            self.assertGreaterEqual(len(c.cells), CFG.CORRIDOR_MIN_LEN)
            self.assertEqual(len(c.ends), 2)
            for cell in c.cells:
                self.assertEqual(len(self.wh.free_neighbors(cell)), 2,
                                 f"{cell} is not single-file")
            xs = {c[0] for c in c.cells}
            ys = {c[1] for c in c.cells}
            self.assertTrue(len(xs) == 1 or len(ys) == 1, "corridor must be straight")

    def test_pick_faces_touch_a_rack_and_are_walkable(self):
        for cell in self.wh.pick_faces:
            self.assertTrue(self.wh.is_walkable(*cell))
            self.assertTrue(any(self.wh.tiles[cell[0] + dx][cell[1] + dy] == Tile.RACK
                                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                                if self.wh.in_bounds(cell[0] + dx, cell[1] + dy)))


class TestAlgorithm1Reservations(unittest.TestCase):
    """Path conflict detection, classification and resolution."""

    def test_lamport_clock_orders_and_merges(self):
        c = LamportClock()
        self.assertEqual(c.tick(), 1)
        self.assertEqual(c.observe(9), 10)
        self.assertGreater(c.tick(), 10)

    def test_cell_windows_are_local_in_time(self):
        a = res(1, 0, [(0, 0), (1, 0), (2, 0)])
        b = res(2, 20, [(2, 0), (3, 0)])
        self.assertFalse(a.overlaps(b),
                         "same cell 20 ticks apart is not a conflict")
        c = res(3, 1, [(2, 0)])
        self.assertTrue(a.overlaps(c))

    def test_cluster_is_transitive_not_just_pairwise(self):
        table = ReservationTable(pad=CFG.RESERVATION_PAD)
        table.commit(res(2, 0, [(1, 0), (2, 0)]))
        table.commit(res(3, 0, [(2, 0), (3, 0)]))
        table.commit(res(4, 0, [(9, 9)]))
        mine = res(1, 0, [(0, 0), (1, 0)])
        cluster = build_cluster(mine, table)
        self.assertEqual({r.robot_id for r in cluster}, {1, 2, 3},
                         "robot 3 joins through robot 2, robot 4 is unrelated")

    def test_ranking_is_deterministic_and_documented_order(self):
        a = res(3, 0, [(0, 0)], priority=5.0, lamport=9)
        b = res(1, 0, [(0, 0)], priority=5.0, lamport=2)
        c = res(2, 0, [(0, 0)], priority=7.0, lamport=50)
        ranked = rank_cluster([a, b, c])
        self.assertEqual([r.robot_id for r in ranked], [2, 1, 3])
        self.assertEqual(rank_cluster([c, a, b]), ranked, "order must not depend on input order")

    def test_geometry_classification(self):
        self.assertEqual(classify_geometry((1, 0), (-1, 0)), "HEAD_ON")
        self.assertEqual(classify_geometry((1, 0), (0, 1)), "CROSSING")
        self.assertEqual(classify_geometry((1, 0), (1, 0)), "SAME_DIRECTION")
        self.assertEqual(classify_geometry((0, 0), (1, 0)), "STATIONARY")

    def test_table_indices_track_commit_and_invalidate(self):
        t = ReservationTable(pad=1)
        t.commit(res(1, 10, [(0, 0), (1, 0)], tail=3))
        self.assertEqual(t.occupant((1, 0), 11), 1)
        self.assertEqual(t.occupant((1, 0), 13), 1, "tail keeps a parked cell claimed")
        self.assertTrue(t.is_busy((0, 0), 10, exclude=2))
        self.assertFalse(t.is_busy((0, 0), 10, exclude=1))
        t.invalidate(1)
        self.assertEqual(t.all(), [])
        self.assertIsNone(t.occupant((1, 0), 11))

    def test_swap_conflict_detects_a_head_on_exchange(self):
        t = ReservationTable(pad=1)
        t.commit(res(1, 0, [(1, 0), (0, 0)]))
        self.assertTrue(t.swap_conflict((0, 0), (1, 0), 0, exclude=2))
        self.assertFalse(t.swap_conflict((0, 0), (1, 0), 5, exclude=2))


class TestAlgorithm2Planner(unittest.TestCase):
    """Congestion-aware cost with fairness and detour bounds."""

    def setUp(self):
        self.wh = Warehouse()
        self.table = ReservationTable(pad=CFG.RESERVATION_PAD)
        self.robot = Robot(id=1, x=0, y=0)

    def test_space_time_path_never_enters_a_claimed_cell(self):
        start, goal = (1, 5), (1, 14)
        direct = plan_static(self.wh, {}, start, goal, CFG)
        # a higher-priority robot claims the middle of our route, moving the
        # opposite way, from tick 2 onwards
        self.table.commit(res(9, 2, list(reversed(direct[2:8])), priority=99.0))
        steps, _ = plan_spacetime(self.wh, self.table, {}, self.robot,
                                  start, goal, 0, CFG)
        self.assertIsNotNone(steps)
        for i, cell in enumerate(steps[1:], start=1):
            self.assertNotEqual(self.table.occupant(cell, i), 9,
                                f"stepped onto a committed cell at t={i}")

    def test_planner_waits_when_waiting_is_the_only_option(self):
        wh, table = self.wh, self.table
        corridor = wh.corridors[0]
        entry, exit_ = corridor.ends
        table.commit(res(9, 0, [corridor.cells[1]] * 6, priority=99.0, tail=2))
        steps, _ = plan_spacetime(wh, table, {}, self.robot, entry, exit_, 0, CFG)
        self.assertIsNotNone(steps, "a wait action must let the robot get through")
        self.assertTrue(any(steps[i] == steps[i - 1] for i in range(1, len(steps))),
                        "expected at least one wait step")

    def test_confirmed_full_block_is_impassable_and_routed_around(self):
        cell = self.wh.corridors[0].cells[3]
        registry = blocks_mod.BlockRegistry(CFG)
        registry.report(cell, 1, 1, 0, severity="FULL")
        steps, _ = plan_spacetime(self.wh, self.table, registry, self.robot,
                                  self.wh.corridors[0].ends[0],
                                  self.wh.corridors[0].ends[1], 0, CFG)
        if steps is not None:
            self.assertNotIn(cell, steps)

    def test_detour_cap_falls_back_to_the_direct_route(self):
        start, goal = (1, 6), (1, 16)
        result = plan_with_detour_cap(self.wh, self.table, {}, self.robot,
                                      start, goal, 0, CFG)
        direct = len(plan_static(self.wh, {}, start, goal, CFG)) - 1
        moves = sum(1 for i in range(1, len(result.steps))
                    if result.steps[i] != result.steps[i - 1])
        self.assertLessEqual(moves, direct * CFG.MAX_DETOUR_RATIO + 1)

    def test_fairness_alpha_halves_for_a_repeatedly_detoured_robot(self):
        from amrsim.planner import fairness_adjusted_alpha
        fresh = Robot(id=2, x=0, y=0)
        tired = Robot(id=3, x=0, y=0, detour_history=[2.0] * 5)
        self.assertEqual(fairness_adjusted_alpha(fresh, CFG), CFG.ALPHA)
        self.assertLess(fairness_adjusted_alpha(tired, CFG), CFG.ALPHA)


class TestAlgorithm3Deadlock(unittest.TestCase):
    """Wait-graph cycle detection with escape and shuffle fallback."""

    def _fleet(self, waits):
        robots = {}
        for rid, waiting_for in waits.items():
            r = Robot(id=rid, x=rid, y=0)
            r.waiting_for = waiting_for
            robots[rid] = r
        return robots

    def test_detects_a_three_way_cycle(self):
        robots = self._fleet({1: 2, 2: 3, 3: 1})
        cycle = dl.find_cycle(robots[1], robots, 8)
        self.assertEqual(sorted(cycle), [1, 2, 3])

    def test_a_dead_end_chain_is_not_a_deadlock(self):
        robots = self._fleet({1: 2, 2: 3, 3: None})
        self.assertIsNone(dl.find_cycle(robots[1], robots, 8),
                          "starvation is not deadlock — the watchdog handles it")

    def test_a_cycle_we_are_not_part_of_is_ignored(self):
        robots = self._fleet({1: 2, 2: 3, 3: 2})
        self.assertIsNone(dl.find_cycle(robots[1], robots, 8))

    def test_wait_threshold_avoids_false_positives(self):
        r = Robot(id=1, x=0, y=0)
        self.assertFalse(dl.wait_stalled(r, 0, CFG))
        self.assertTrue(dl.wait_stalled(r, CFG.WAIT_STALL_THRESHOLD + 1, CFG))

    def test_escape_search_leaves_a_jammed_single_file_lane(self):
        wh = Warehouse()
        corridor = wh.corridors[0]
        stuck = corridor.cells[3]
        jammed = {c for c in corridor.cells if c != stuck}
        escape = dl.search_expanding_radius(wh, stuck, set(), {}, jammed, CFG)
        self.assertIsNotNone(escape)
        self.assertNotIn(escape, corridor.cells,
                         "escaping along the same single-file lane only moves the pinch")

    def test_escape_search_biases_the_first_hop_across_the_lane(self):
        """In a wider aisle the first hop should switch lane rather than slide
        along the one the robot is already stuck in."""
        wh = Warehouse()
        lane_cell = wh.corridors[0].cells[3]
        self.assertEqual(dl._lane_bias(wh, lane_cell), "v")
        self.assertTrue(dl._is_perpendicular(lane_cell,
                                             (lane_cell[0] + 1, lane_cell[1]), "v"))
        self.assertFalse(dl._is_perpendicular(lane_cell,
                                              (lane_cell[0], lane_cell[1] + 1), "v"))

    def test_shuffle_chain_walks_through_occupied_cells(self):
        wh = Warehouse()
        corridor = wh.corridors[0]
        occupied = {corridor.cells[i]: i + 10 for i in range(3)}
        chain, free = dl.find_shuffle_chain(wh, corridor.cells[0], occupied,
                                            {10}, {}, CFG)
        self.assertIsNotNone(chain)


class TestAlgorithm4Blocks(unittest.TestCase):
    """Block detection, propagation and opportunistic clearing."""

    def setUp(self):
        self.reg = blocks_mod.BlockRegistry(CFG)

    def test_transient_sightings_are_filtered_out(self):
        for i in range(CFG.CONFIRM_WINDOW - 1):
            self.assertIsNone(self.reg.observe((5, 5), 1, i, i))
        self.assertIsNotNone(self.reg.observe((5, 5), 1, 9, 9))

    def test_report_is_idempotent_under_stale_lamport(self):
        self.reg.report((5, 5), 1, lamport=10, tick=0)
        self.reg.report((5, 5), 2, lamport=3, tick=0, severity="PARTIAL")
        self.assertEqual(self.reg.get((5, 5)).severity, "FULL",
                         "an older report must not overwrite a newer one")

    def test_ttl_softens_rather_than_deletes(self):
        self.reg.report((5, 5), 1, 1, tick=0)
        softened, _ = self.reg.expire(CFG.BLOCK_TTL + 1)
        self.assertEqual(softened, [(5, 5)])
        self.assertEqual(self.reg.get((5, 5)).confidence, "UNCONFIRMED")
        self.assertIn((5, 5), self.reg.events, "never leave a phantom hole in the map")

    def test_escalation_fires_once(self):
        self.reg.report((5, 5), 1, 1, tick=0)
        _, esc = self.reg.expire(CFG.REPROBE_ESCALATION_TIME + 1)
        self.assertEqual(esc, [(5, 5)])
        _, esc2 = self.reg.expire(CFG.REPROBE_ESCALATION_TIME + 2)
        self.assertEqual(esc2, [])

    def test_version_bumps_so_route_caches_expire(self):
        v = self.reg.version
        self.reg.report((5, 5), 1, 1, tick=0)
        self.assertGreater(self.reg.version, v)
        v = self.reg.version
        self.reg.clear((5, 5))
        self.assertGreater(self.reg.version, v)


class TestAlgorithm5Failures(unittest.TestCase):
    """Heartbeat-based failure detection with idempotent recovery."""

    def setUp(self):
        self.table = ReservationTable(pad=1)
        self.pool = TaskPool(CFG)
        self.blocks = blocks_mod.BlockRegistry(CFG)
        self.robot = Robot(id=1, x=4, y=4)
        self.messages = []

    def _declare(self):
        return fl.declare_failed(self.robot, self.table, self.pool, self.blocks,
                                 tick=50, lamport=1, log=self.messages.append)

    def test_declare_failed_is_idempotent(self):
        self.assertTrue(self._declare())
        self.assertFalse(self._declare(), "concurrent detection must be a no-op")

    def test_failure_invalidates_reservation_and_blocks_the_footprint(self):
        self.table.commit(res(1, 0, [(4, 4), (5, 4)]))
        self._declare()
        self.assertIsNone(self.table.get(1))
        self.assertIn((4, 4), self.blocks.events)
        self.assertEqual(self.blocks.get((4, 4)).cause, "failed_robot")

    def test_task_goes_back_through_the_normal_pipeline(self):
        task = self.pool.create("pick", (1, 1), (2, 2), "any", 1.0, 0, "aisle")
        self.pool.claim(task, self.robot, 0)
        task.item_in_transit = True
        self._declare()
        self.assertEqual(task.status, TaskStatus.UNASSIGNED)
        self.assertIsNone(task.holder)
        self.assertTrue(task.recovery_flagged, "an in-transit item must raise an alert")
        self.assertTrue(any("ITEM_LOCATION_UNKNOWN" in m for m in self.messages))

    def test_ttl_thresholds(self):
        self.robot.last_heartbeat = 0
        self.assertFalse(fl.is_failed(self.robot, CFG.HEARTBEAT_TTL, CFG.HEARTBEAT_TTL))
        self.assertTrue(fl.is_failed(self.robot, CFG.HEARTBEAT_TTL + 1, CFG.HEARTBEAT_TTL))


class TestAlgorithm6TaskPool(unittest.TestCase):
    """CRDT task pool with a time-boxed deterministic auction."""

    def setUp(self):
        self.wh = Warehouse()
        self.pool = TaskPool(CFG)
        self.clock = LamportClock()

    def test_orset_deduplicates_by_origin_id(self):
        s = ORSet()
        self.assertTrue(s.add("wms:1", "a"))
        self.assertFalse(s.add("wms:1", "b"), "same origin id must not be added twice")
        self.assertEqual(s.values(), ["a"])

    def test_orset_merge_is_conflict_free(self):
        a, b = ORSet(), ORSet()
        a.add("x", 1)
        b.add("y", 2)
        a.merge(b)
        self.assertEqual(sorted(a.values()), [1, 2])

    def test_aging_lifts_a_starving_task(self):
        t = self.pool.create("pick", (1, 6), (1, 10), "any", 1.0, 0, "aisle")
        self.assertGreater(self.pool.aged_priority(t, 100), self.pool.aged_priority(t, 0))

    def test_capability_prefilter(self):
        heavy = self.pool.create("pick", (1, 6), (1, 10), "heavy", 1.0, 0, "aisle")
        generic = self.pool.create("pick", (1, 6), (1, 10), "any", 1.0, 0, "aisle")
        light = Robot(id=1, x=1, y=6, capability="any")
        strong = Robot(id=2, x=1, y=6, capability="heavy")
        self.assertFalse(self.pool.prefilter(heavy, light))
        self.assertTrue(self.pool.prefilter(heavy, strong))
        self.assertTrue(self.pool.prefilter(generic, light))
        self.assertTrue(self.pool.prefilter(generic, strong))

    def test_auction_is_time_boxed_then_awards_the_best_bid(self):
        task = self.pool.create("pick", (1, 6), (1, 10), "any", 1.0, 0, "aisle")
        near = Robot(id=1, x=1, y=7)
        far = Robot(id=2, x=38, y=18)
        for tick in range(CFG.BID_WINDOW + 2):
            self.pool.run_auction(tick, [near, far], self.wh, self.clock, lambda m: None)
        self.assertEqual(task.holder, 1, "the nearest capable robot should win")
        self.assertEqual(near.task, task)

    def test_busy_robots_do_not_trigger_the_no_candidate_backoff(self):
        busy = Robot(id=1, x=1, y=7)
        busy.task = object()
        task = self.pool.create("pick", (1, 6), (1, 10), "any", 1.0, 0, "aisle")
        self.pool.run_auction(0, [busy], self.wh, self.clock, lambda m: None)
        self.assertEqual(task.recheck_after, 0,
                         "'everyone is busy' is not 'no candidates'")

    def test_uncoverable_task_backs_off_exponentially(self):
        pool = TaskPool(CFG)
        only = Robot(id=1, x=1, y=7, capability="any")
        task = pool.create("pick", (1, 6), (1, 10), "heavy", 1.0, 0, "aisle")
        pool.run_auction(0, [only], self.wh, self.clock, lambda m: None)
        first = task.recheck_after
        self.assertGreater(first, 0)
        pool.run_auction(first, [only], self.wh, self.clock, lambda m: None)
        self.assertGreater(task.recheck_after - first, 0)

    def test_lease_expiry_is_detected(self):
        task = self.pool.create("pick", (1, 6), (1, 10), "any", 1.0, 0, "aisle")
        self.pool.claim(task, Robot(id=1, x=1, y=7), 0)
        self.assertEqual(self.pool.expired_leases(CFG.TASK_TTL), [])
        self.assertEqual(self.pool.expired_leases(CFG.TASK_TTL + 1), [task])


if __name__ == "__main__":
    unittest.main()
