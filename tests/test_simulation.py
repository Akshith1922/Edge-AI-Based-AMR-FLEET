"""
End-to-end invariants.

These are the properties the whole design exists to guarantee, so they are
checked every tick of every scenario, in both modes, over several seeds —
not just at the end of one lucky run.
"""

import unittest

from amrsim import scenarios
from amrsim.config import DEFAULT as CFG
from amrsim.engine import Simulation
from amrsim.robot import RobotState

SCENARIOS = sorted(s["name"] for s in scenarios.catalogue())
SEEDS = (1, 4, 9)
TICKS = 260


def run(mode, scenario, seed, robots=8, check=None):
    sim = Simulation(mode=mode, seed=seed, num_robots=robots)
    scenarios.build(sim, scenario, seed=seed)
    for _ in range(TICKS):
        sim.step()
        if check:
            check(sim)
    return sim


class TestInvariants(unittest.TestCase):
    def _assert_physics(self, sim):
        seen = {}
        for r in sim.robots:
            if not r.is_alive():
                continue
            self.assertTrue(sim.warehouse.is_walkable(*r.pos()),
                            f"robot {r.id} left the floor at {r.pos()} (tick {sim.tick})")
            self.assertNotIn(r.pos(), seen,
                             f"robots {seen.get(r.pos())} and {r.id} share "
                             f"{r.pos()} at tick {sim.tick}")
            seen[r.pos()] = r.id

    def test_no_collisions_in_any_scenario_or_mode(self):
        for scenario in SCENARIOS:
            for mode in ("coordinated", "baseline"):
                for seed in SEEDS:
                    with self.subTest(scenario=scenario, mode=mode, seed=seed):
                        sim = run(mode, scenario, seed, check=self._assert_physics)
                        self.assertEqual(sim.metrics.collisions, 0)

    def test_robots_never_teleport(self):
        """Every move is to an adjacent cell or a stay — one cell per tick."""
        sim = Simulation(mode="coordinated", seed=3, num_robots=8)
        scenarios.build(sim, "rush_hour")
        last = {r.id: r.pos() for r in sim.robots}
        for _ in range(TICKS):
            sim.step()
            for r in sim.robots:
                if not r.is_alive():
                    continue
                d = sim.warehouse.manhattan(last[r.id], r.pos())
                self.assertLessEqual(d, 1, f"robot {r.id} jumped {d} cells")
                last[r.id] = r.pos()

    def test_work_actually_gets_done(self):
        for mode in ("coordinated", "baseline"):
            with self.subTest(mode=mode):
                sim = run(mode, "rush_hour", 5)
                self.assertGreater(len(sim.metrics.completed), 5,
                                   "the fleet made almost no progress — check liveness")

    def test_no_robot_is_stuck_for_the_whole_run(self):
        """Liveness: every robot either carries work or is parked, and none sits
        blocked indefinitely."""
        sim = Simulation(mode="coordinated", seed=6, num_robots=8)
        scenarios.build(sim, "rush_hour")
        blocked_for = {r.id: 0 for r in sim.robots}
        for _ in range(TICKS):
            sim.step()
            for r in sim.robots:
                stuck = r.state in (RobotState.WAITING, RobotState.QUEUED,
                                    RobotState.IN_RESOLUTION)
                blocked_for[r.id] = blocked_for[r.id] + 1 if stuck else 0
                self.assertLess(blocked_for[r.id], 60,
                                f"robot {r.id} has been blocked for "
                                f"{blocked_for[r.id]} consecutive ticks")

    def test_run_is_deterministic_for_a_given_seed(self):
        a = run("coordinated", "rush_hour_fixed", 2)
        b = run("coordinated", "rush_hour_fixed", 2)
        self.assertEqual(a.metrics.summary(), b.metrics.summary())
        self.assertEqual([r.pos() for r in a.robots], [r.pos() for r in b.robots])

    def test_reservation_table_is_conflict_free_inside_the_planning_window(self):
        """Reservations must be mutually exclusive for the cooperative window.

        Beyond it the search deliberately stops deconflicting — every robot
        replans each tick and only ever executes its next step, so the window
        is exactly the horizon that has to be exclusive.
        """
        sim = Simulation(mode="coordinated", seed=8, num_robots=10)
        scenarios.build(sim, "rush_hour")
        for _ in range(180):
            sim.step()
            claims = {}
            for res in sim.table.all():
                for i, cell in enumerate(res.steps[:CFG.COOP_WINDOW + 1]):
                    key = (cell, res.t0 + i)
                    self.assertNotIn(key, claims,
                                     f"cell {cell} double-claimed at t={res.t0 + i} "
                                     f"by robots {claims.get(key)} and {res.robot_id}")
                    claims[key] = res.robot_id


class TestFleetEndurance(unittest.TestCase):
    """The fleet must not quietly retire itself while work is queued."""

    def test_a_flat_robot_charges_and_returns_to_service(self):
        sim = Simulation(mode="coordinated", seed=3, num_robots=6)
        scenarios.build(sim, "rush_hour")
        sim.run(30)
        victim = next(r for r in sim.robots if r.is_alive())
        victim.battery = CFG.BATTERY_MIN_BID - 1

        sim.step()
        self.assertTrue(victim.charging, "a low robot must be flagged for charge")
        self.assertFalse(sim.pool.prefilter(sim.pool.create(
            "pick", (1, 6), (1, 10), "any", 1.0, sim.tick, "aisle"), victim),
            "a charging robot must not bid for new work")

        for _ in range(900):
            sim.step()
            if not victim.charging:
                break
        self.assertFalse(victim.charging, "the robot never finished charging")
        self.assertGreaterEqual(victim.battery, CFG.BATTERY_RESUME)

    def test_fleet_keeps_working_over_a_long_run(self):
        """The regression this guards: batteries drained, every robot fell below
        the bidding floor, nothing recharged them, and the whole fleet parked in
        the charging bays with a full backlog of unassigned tasks."""
        sim = Simulation(mode="coordinated", seed=7, num_robots=8)
        scenarios.build(sim, "rush_hour")
        sim.run(2500)
        early = len(sim.metrics.completed)
        sim.run(1500)
        self.assertGreater(len(sim.metrics.completed) - early, 50,
                           "the fleet stopped delivering part-way through the run")
        self.assertGreater(min(r.battery for r in sim.robots if r.is_alive()), 0.0,
                           "a robot ran completely flat")
        working = [r for r in sim.robots if r.is_alive() and r.task is not None]
        self.assertTrue(working, "every robot is idle while work is outstanding")

    def test_the_task_stream_never_runs_dry(self):
        sim = Simulation(mode="coordinated", seed=5, num_robots=8)
        scenarios.build(sim, "rush_hour")
        sim.run(1400)
        received = sim.pool.stats(sim.tick)["received"]
        sim.run(400)
        self.assertGreater(sim.pool.stats(sim.tick)["received"], received,
                           "the scenario stopped releasing work at its horizon")

    def test_the_backlog_stays_bounded(self):
        sim = Simulation(mode="coordinated", seed=7, num_robots=8)
        scenarios.build(sim, "rush_hour")
        sim.run(1200)
        mid = sim.pool.stats(sim.tick)["outstanding"]
        sim.run(2000)
        self.assertLessEqual(sim.pool.stats(sim.tick)["outstanding"], mid * 2 + 10,
                             "the release policy is not holding the queue in check")

    def test_pipeline_counts_reconcile(self):
        sim = Simulation(mode="coordinated", seed=2, num_robots=8)
        scenarios.build(sim, "rush_hour")
        for _ in range(400):
            sim.step()
            s = sim.pool.stats(sim.tick)
            self.assertEqual(
                s["delivered"] + s["in_progress"] + s["auction"] + s["pending"]
                + s["recovery"], s["received"],
                "the pipeline buckets must always sum to what was received")
            self.assertEqual(s["outstanding"], s["received"] - s["delivered"])

    def test_scheduled_and_recurring_events_both_fire(self):
        sim = Simulation(mode="coordinated", seed=1, num_robots=4)
        once, every = [], []
        sim.schedule(5, lambda s: once.append(s.tick))
        sim.every_tick(lambda s: every.append(s.tick), start=3)
        sim.run(10)
        self.assertEqual(once, [5], "a scheduled event must fire exactly once")
        self.assertEqual(every, list(range(3, 11)))

    def test_history_carries_every_series_the_dashboard_plots(self):
        sim = Simulation(mode="coordinated", seed=1, num_robots=6)
        scenarios.build(sim, "rush_hour")
        sim.run(40)
        sample = list(sim.metrics.history)[-1]
        for key in ("t", "received", "done", "pending", "in_progress", "waiting",
                    "moving", "charging", "avg", "avg_all", "battery", "battery_min"):
            self.assertIn(key, sample)
        self.assertLessEqual(sample["battery_min"], sample["battery"])
        self.assertLessEqual(sample["waiting"], len(sim.robots))


class TestCoordinationBehaviour(unittest.TestCase):
    """The behaviours the coordinated mode is supposed to buy."""

    def test_failure_is_detected_far_faster_than_the_baseline_timeout(self):
        coord = run("coordinated", "failure_storm", 3)
        base = run("baseline", "failure_storm", 3)
        self.assertTrue(coord.metrics.reassignment_latency)
        self.assertLess(max(coord.metrics.reassignment_latency),
                        min(base.metrics.reassignment_latency),
                        "heartbeat TTL should beat a fixed timeout every time")

    def test_coordination_avoids_unplanned_stops(self):
        coord = run("coordinated", "rush_hour", 7, robots=10)
        base = run("baseline", "rush_hour", 7, robots=10)
        self.assertLess(coord.metrics.hard_stops, base.metrics.hard_stops / 4,
                        "planned yielding should replace emergency stops")

    def test_a_block_is_detected_gossiped_and_routed_around(self):
        """Drop an obstacle directly onto a robot's committed path: it must be
        confirmed, shared, and then avoided by the whole fleet."""
        sim = Simulation(mode="coordinated", seed=1, num_robots=8)
        scenarios.build(sim, "rush_hour", faults=False)
        sim.run(60)
        mover = next(r for r in sim.robots
                     if r.is_alive() and len(r.remaining_path()) > 6)
        cell = mover.remaining_path()[4]
        sim.place_obstacle(cell)
        self.assertNotIn(cell, sim.blocks.events, "nothing is known before it is sensed")

        sim.run(CFG.CONFIRM_WINDOW + 3)
        self.assertIn(cell, sim.blocks.events, "the obstacle was never confirmed")

        for _ in range(40):
            sim.step()
            for r in sim.robots:
                self.assertNotIn(cell, r.remaining_path(),
                                 "a confirmed full block must never be planned through")
                self.assertNotEqual(r.pos(), cell)

    def test_a_lifted_obstacle_is_reprobed_and_the_block_retracted(self):
        sim = Simulation(mode="coordinated", seed=1, num_robots=8)
        scenarios.build(sim, "rush_hour")
        sim.run(60)
        mover = next(r for r in sim.robots
                     if r.is_alive() and len(r.remaining_path()) > 6)
        cell = mover.remaining_path()[4]
        sim.place_obstacle(cell)
        sim.run(CFG.CONFIRM_WINDOW + 3)
        self.assertIn(cell, sim.blocks.events)

        sim.remove_obstacle(cell)
        sim.run(200)
        self.assertNotIn(cell, sim.blocks.events,
                         "a cleared aisle must be reopened, not left as a phantom wall")

    def test_a_killed_robot_releases_its_task_and_blocks_its_cell(self):
        sim = Simulation(mode="coordinated", seed=2, num_robots=8)
        scenarios.build(sim, "rush_hour")
        sim.run(60)
        victim = next(r for r in sim.robots if r.task is not None)
        held = victim.task.id
        sim.silence_robot(victim.id)
        sim.run(CFG.HEARTBEAT_TTL + 3)
        self.assertEqual(victim.state, RobotState.FAILED)
        self.assertIsNone(victim.task)
        self.assertIn(victim.pos(), sim.blocks.events)
        task = sim.pool.by_id[held]
        self.assertNotEqual(task.holder, victim.id)

    def test_a_recovered_robot_rejoins_the_fleet(self):
        sim = Simulation(mode="coordinated", seed=2, num_robots=8)
        scenarios.build(sim, "rush_hour")
        sim.run(40)
        victim = sim.robots[0]
        sim.silence_robot(victim.id)
        sim.run(CFG.HEARTBEAT_TTL + 3)
        self.assertEqual(victim.state, RobotState.FAILED)
        sim.revive_robot(victim.id)
        sim.run(6)
        self.assertNotEqual(victim.state, RobotState.FAILED)
        self.assertNotIn(victim.pos(), sim.blocks.events,
                         "recovery must lift the footprint block it created")

    def test_chokepoints_serialise_traffic_without_starving_anyone(self):
        sim = Simulation(mode="coordinated", seed=4, num_robots=10)
        scenarios.build(sim, "chokepoint_duel")
        sim.run(300)
        self.assertEqual(sim.metrics.collisions, 0)
        self.assertGreater(len(sim.metrics.completed), 10)
        for corridor in sim.locks.snapshot():
            self.assertLessEqual(len(corridor["inside"]), len(corridor["cells"]))


if __name__ == "__main__":
    unittest.main()
