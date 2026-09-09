#!/usr/bin/env python3
"""
Headless digital twin of the Gazebo fleet.

Same warehouse, same map, and -- this is the point -- the *same* `EdgeAgent`
code that runs on the robots. Only the plumbing differs: instead of ROS topics
and Gazebo physics, the agent gets a differential-drive integrator and a lidar
synthesised by ray-casting the 5 cm occupancy grid. So a result here is a
result about the code that ships, not about a separate model of it.

That buys two things Gazebo cannot: it runs faster than real time on a laptop
with no GPU, and it can run the identical workload twice under two different
coordination policies, which is the only honest way to measure what the
coordination is worth.

    python3 tools/twin.py                        # cooperative, 3 robots
    python3 tools/twin.py --compare              # cooperative vs stop-and-wait
    python3 tools/twin.py --robots 5 --trace results/trace.json

Standard library only.
"""

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "ros2_ws" / "src" / "warehouse_picker"
sys.path.insert(0, str(PKG))

from warehouse_picker.agent_core import EdgeAgent                    # noqa: E402
from warehouse_picker.allocation import Task                        # noqa: E402
from warehouse_picker.navigation import Limits, wrap                 # noqa: E402
from warehouse_picker.occupancy import GridMap, ROBOT_RADIUS         # noqa: E402
from warehouse_picker.stations import (                              # noqa: E402
    charger_station, derive_stations, dock_stations)

# Matches the Tugbot's scan_front sensor, subsampled: 84.2 deg either side,
# 5 m range. Fewer beams than the real 674 because the twin pays for every one
# and the agent subsamples to ~90 anyway.
SCAN_FOV = 1.47043991089
SCAN_BEAMS = 91
SCAN_RANGE = 5.0
CONTROL_HZ = 10.0


class Body:
    """Differential-drive kinematics with first-order actuator lag."""

    __slots__ = ("x", "y", "yaw", "v", "w", "lim", "radius")

    def __init__(self, x, y, yaw, lim):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.lim = lim
        self.radius = ROBOT_RADIUS

    def apply(self, v_cmd, w_cmd, dt):
        lim = self.lim
        dv = max(-lim.a_lin * dt, min(lim.a_lin * dt, v_cmd - self.v))
        dw = max(-lim.a_ang * dt, min(lim.a_ang * dt, w_cmd - self.w))
        self.v = max(lim.v_min, min(lim.v_max, self.v + dv))
        self.w = max(-lim.w_max, min(lim.w_max, self.w + dw))
        self.yaw = wrap(self.yaw + self.w * dt)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt


class Obstruction:
    """Something in an aisle that the map does not know about.

    A dropped pallet, a stopped forklift, a spill. It exists only in the lidar
    and in the collision check -- never in the occupancy grid -- so the fleet
    has to *discover* it, agree it is there, and route around it. That is the
    whole point: a blockage the planner already knows about is not a test of
    anything.
    """

    __slots__ = ("x", "y", "radius", "appears_at", "label")

    def __init__(self, x, y, radius, appears_at=0.0, label=""):
        self.x, self.y, self.radius = x, y, radius
        self.appears_at = appears_at
        self.label = label or f"pallet at ({x:.1f}, {y:.1f})"

    def present(self, now):
        return now >= self.appears_at


def simulate_scan(grid, body, circles, rng=None):
    """Ray-cast the static map, then clip each beam on anything in its way.

    `circles` are the other robots and any unmapped obstructions, as objects
    with `.x`, `.y` and `.radius`.
    """
    ranges = []
    for i in range(SCAN_BEAMS):
        a = -SCAN_FOV + 2 * SCAN_FOV * i / (SCAN_BEAMS - 1)
        theta = body.yaw + a
        r = grid.raycast(body.x, body.y, theta, SCAN_RANGE)
        dx, dy = math.cos(theta), math.sin(theta)
        for ob in circles:
            radius = getattr(ob, "radius", ROBOT_RADIUS)
            ox, oy = ob.x - body.x, ob.y - body.y
            along = ox * dx + oy * dy
            if along <= 0 or along - radius > r:
                continue
            perp2 = ox * ox + oy * oy - along * along
            if perp2 > radius * radius:
                continue
            hit = along - math.sqrt(radius * radius - perp2)
            if 0 < hit < r:
                r = hit
        if rng is not None and r < SCAN_RANGE:
            r = max(0.02, r + rng.gauss(0.0, 0.01))
        ranges.append(r)
    return ranges, -SCAN_FOV, 2 * SCAN_FOV / (SCAN_BEAMS - 1), SCAN_RANGE


class Twin:
    def __init__(self, policy="cooperative", robots=3, seed=7, tasks=12,
                 duration=420.0, trace=False, verbose=False, scenario="crossing"):
        self.policy = policy
        self.scenario = scenario
        self.seed = seed
        self.rng = random.Random(seed)
        self.duration = duration
        self.dt = 1.0 / CONTROL_HZ
        self.verbose = verbose

        self.fine = GridMap.load_layout(PKG / "config" / "warehouse_layout.json", 0.05)
        self.plan = GridMap.load_plan(PKG / "maps" / "warehouse_plan.json")
        self.layout = json.loads((PKG / "config" / "warehouse_layout.json").read_text())

        spawns = [(r["x"], r["y"], r["yaw"]) for r in self.layout["robots"]]
        while len(spawns) < robots:
            # Extra robots for stress runs queue up behind the declared ones.
            sx, sy, syaw = spawns[len(spawns) % len(self.layout["robots"])]
            spawns.append((sx, sy - 2.2 * (1 + len(spawns) // 3), syaw))
        spawns = spawns[:robots]

        charger = charger_station(self.plan)
        self.stations = derive_stations(self.plan, self.layout,
                                        reachable_from=(spawns[0][0], spawns[0][1]))
        self.docks = dock_stations(self.plan, self.layout)

        lim = Limits(dt=self.dt)
        self.agents, self.bodies = [], []
        for i, (sx, sy, syaw) in enumerate(spawns):
            agent = EdgeAgent(f"amr_{i + 1}", self.plan, limits=lim,
                              charger=charger.xy() if charger else None,
                              home=(sx, sy),
                              clock=lambda: self.now, policy=policy)
            self.agents.append(agent)
            self.bodies.append(Body(sx, sy, syaw, lim))

        self.tasks = self._make_tasks(tasks)
        self.obstructions = self._make_obstructions()
        self.now = 0.0
        self.collisions = []
        self.wall_hits = []
        self.obstruction_hits = []
        self.completion_times = {}
        self.trace = [] if trace else None

    # -- workload ----------------------------------------------------------

    def _make_tasks(self, count):
        """The workload. Two shapes, and the difference between them is the
        whole story of what coordination is worth.

        `crossing` scatters pickups and drops across the building. Robots meet
        occasionally and by accident.

        `rush_hour` puts every pickup at a northern rack face and every drop on
        the southern loading docks, so the entire fleet has to funnel through
        the same two north-south aisles at the same time. This is what the
        brief means by "overlapping paths", and it is the case worth measuring:
        an uncoordinated fleet spends it stopped.
        """
        rng = random.Random(self.seed * 31 + 7)
        north = [s for s in self.stations if s.y > 2.0]
        south = [s for s in self.stations if s.y <= 2.0]
        out = []
        for i in range(count):
            if self.scenario == "blocked_aisle" and self.stations:
                # Force traffic up and down the southern aisles, which is where
                # the blockage lands.
                deep = [s for s in self.stations if s.y < -6.0] or self.stations
                pick = rng.choice(deep)
                drop = (self.docks[i % len(self.docks)] if self.docks
                        else rng.choice(self.stations))
                if i % 2:
                    pick, drop = drop, pick
            elif self.scenario == "rush_hour" and north and self.docks:
                pick = rng.choice(north)
                drop = self.docks[i % len(self.docks)]
            elif north and south:
                pick, drop = ((rng.choice(north), rng.choice(south)) if i % 2 == 0
                              else (rng.choice(south), rng.choice(north)))
            else:
                pick, drop = rng.sample(self.stations, 2)
            out.append(Task(f"t{i:02d}", pick.xy(), drop.xy(),
                            priority=1.0, created=0.0,
                            label=f"{pick.name} -> {drop.name}"))
        return out

    def _make_obstructions(self):
        """The `blocked_aisle` scenario's chokepoint.

        Every aisle in this warehouse is wide enough for two Tugbots abreast --
        `tools/twin.py --measure-corridors` reports not one single-file cell on
        the whole floor -- so nothing in the layout itself forces robots to take
        turns. A pallet stack dropped part-way across one of the two southern
        aisles does: it leaves a 1.2 m gap, which one robot fits through and two
        do not, and it appears after the run has started so the fleet has to
        find it rather than plan around it from the outset.
        """
        if self.scenario != "blocked_aisle":
            return []
        # The west aisle runs x in [-4.77, -0.97]. A 1.3 m pallet stack against
        # its western rack leaves x in [-2.17, -0.97] open: 1.2 m.
        return [Obstruction(-3.47, -12.0, 1.30, appears_at=45.0,
                            label="pallet stack across the west aisle")]

    def live_obstructions(self):
        return [o for o in self.obstructions if o.present(self.now)]

    # -- the loop ----------------------------------------------------------

    def run(self):
        for agent in self.agents:
            for task in self.tasks:
                agent.submit_task(task)

        started = time.time()
        steps = int(self.duration / self.dt)
        remaining = {t.id for t in self.tasks}

        for step in range(steps):
            self.now = step * self.dt

            hazards = self.live_obstructions()
            meshes, task_msgs = [], []
            for agent, body in zip(self.agents, self.bodies):
                others = [b for b in self.bodies if b is not body] + hazards
                agent.set_pose(body.x, body.y, body.yaw, body.v, body.w)
                agent.set_scan(*simulate_scan(self.fine, body, others, self.rng),
                               now=self.now)
                out = agent.step(self.now)
                body.apply(out.v, out.w, self.dt)
                meshes.append(out.mesh)
                task_msgs.append(out.tasks)

            # Broadcast domain: every robot hears every other robot. Packets are
            # delivered after all agents have stepped, so nobody acts on a peer
            # state from the same instant it was produced -- the one-cycle delay
            # a real radio has.
            for i, agent in enumerate(self.agents):
                for j, mesh in enumerate(meshes):
                    if i != j:
                        agent.on_mesh(mesh, self.now)
                        agent.on_tasks(task_msgs[j], self.now)

            self._check_collisions()

            outstanding = self._outstanding()
            for tid in list(remaining):
                if tid not in outstanding:
                    self.completion_times[tid] = self.now
                    remaining.discard(tid)

            if self.trace is not None and step % 5 == 0:
                self.trace.append({
                    "t": round(self.now, 2),
                    "r": [{"id": a.id, "x": round(b.x, 2), "y": round(b.y, 2),
                           "yaw": round(b.yaw, 2), "b": round(a.state.battery, 1),
                           "m": a.state.mode, "w": a.state.waiting_for,
                           "p": [[round(px, 1), round(py, 1)]
                                 for px, py in a.tracker.path[a.tracker.index:]],
                           "why": a._decision_reason}
                          for a, b in zip(self.agents, self.bodies)],
                    "done": len(self.completion_times),
                })

            if not remaining:
                break
            if self.verbose and step % 300 == 0:
                print(f"    t={self.now:6.1f}s  done={len(self.completion_times):2d}"
                      f"/{len(self.tasks)}  collisions={len(self.collisions)}")

        self.wall_clock = time.time() - started
        return self.report()

    def _outstanding(self):
        """Tasks not yet completed, as agreed by the fleet's replicas."""
        done = set()
        for agent in self.agents:
            done |= agent.pool.done
        return {t.id for t in self.tasks} - done

    def _check_collisions(self):
        touch = 2 * ROBOT_RADIUS
        hazards = self.live_obstructions()
        for i in range(len(self.bodies)):
            bi = self.bodies[i]
            if self.fine.at(*self.fine.world_to_grid(bi.x, bi.y)):
                self.wall_hits.append((round(self.now, 2), self.agents[i].id))
            for ob in hazards:
                if math.hypot(bi.x - ob.x, bi.y - ob.y) < ROBOT_RADIUS + ob.radius:
                    self.obstruction_hits.append((round(self.now, 2),
                                                  self.agents[i].id, ob.label))
            for j in range(i + 1, len(self.bodies)):
                bj = self.bodies[j]
                if math.hypot(bi.x - bj.x, bi.y - bj.y) < touch:
                    self.collisions.append((round(self.now, 2),
                                            self.agents[i].id, self.agents[j].id))

    # -- results -----------------------------------------------------------

    def report(self):
        completed = len(self.completion_times)
        makespan = max(self.completion_times.values()) if self.completion_times else self.now
        return {
            # Throughput is the metric that survives a run being cut short.
            # Makespan only means "time to finish the batch" when the batch
            # actually finished; on a censored run it silently reports how long
            # the tasks that *did* finish took, which flatters whichever arm
            # finished fewer of them.
            "throughput_per_min": round(completed / max(self.now, 1e-6) * 60.0, 3),
            "complete": completed == len(self.tasks),
            # When the two arms deliver different numbers of tasks, neither
            # makespan nor mean task time compares them honestly: the arm that
            # dropped work looks *faster*, because the jobs it never finished
            # are the slow ones and they never enter its average. The curve is
            # the time each successive delivery landed, so the arms can be
            # compared over the same number of deliveries.
            "completion_curve": sorted(round(t, 1)
                                       for t in self.completion_times.values()),
            "policy": self.policy,
            "robots": len(self.agents),
            "tasks": len(self.tasks),
            "completed": completed,
            "makespan_s": round(makespan, 1),
            "mean_task_s": round(sum(self.completion_times.values()) / completed, 1)
            if completed else None,
            "sim_time_s": round(self.now, 1),
            "robot_collisions": len(self.collisions),
            "wall_contacts": len(self.wall_hits),
            "obstruction_contacts": len(self.obstruction_hits),
            "distance_m": round(sum(a.distance_travelled for a in self.agents), 1),
            "time_yielding_s": round(sum(a.time_yielding for a in self.agents), 1),
            "replans": sum(a.replans for a in self.agents),
            "wall_clock_s": round(self.wall_clock, 1),
        }


def print_report(rep):
    print(f"  policy            {rep['policy']}")
    print(f"  robots / tasks    {rep['robots']} / {rep['tasks']}")
    print(f"  completed         {rep['completed']}/{rep['tasks']}"
          f"{'' if rep['complete'] else '   (run cut short)'}")
    print(f"  makespan          {rep['makespan_s']} s"
          f"{'' if rep['complete'] else '  <- censored, use throughput'}")
    print(f"  throughput        {rep['throughput_per_min']} tasks/min")
    print(f"  mean task time    {rep['mean_task_s']} s")
    print(f"  robot collisions  {rep['robot_collisions']}")
    print(f"  wall contacts     {rep['wall_contacts']}")
    if rep.get("obstruction_contacts") is not None:
        print(f"  hit the blockage  {rep['obstruction_contacts']}")
    print(f"  distance driven   {rep['distance_m']} m")
    print(f"  time yielding     {rep['time_yielding_s']} s")
    print(f"  replans           {rep['replans']}")
    print(f"  (ran in {rep['wall_clock_s']} s wall clock)")


def equal_work_gain(base, coop):
    """Percentage reduction in the time to deliver the *same* number of tasks.

    Comparing makespans only works when both arms finished the batch. When one
    did not, this compares how long each took to reach the delivery count they
    both reached, which is the like-for-like question and does not reward an
    arm for abandoning its slowest jobs.
    """
    a, b = coop["completion_curve"], base["completion_curve"]
    k = min(len(a), len(b))
    if k == 0:
        return 0.0, "  (neither arm delivered anything)"
    t_coop, t_base = a[k - 1], b[k - 1]
    if t_base <= 0:
        return 0.0, ""
    return (t_base - t_coop) / t_base * 100.0, ""


def measure_corridors():
    """Report how much of the floor is too narrow for two robots to pass.

    Worth running before believing any coordination benchmark. A warehouse
    whose aisles all take two robots abreast has no chokepoints, so corridor
    coordination -- give way, throttle, passing bays -- has nothing to do, and
    a comparison against an uncoordinated fleet will come out roughly level
    however good the protocol is. That is a fact about the building, not about
    the software, and it is better to know it up front than to discover it in
    the results.
    """
    import collections
    grid = GridMap.load_plan(PKG / "maps" / "warehouse_plan.json")
    pass_width = 2 * (ROBOT_RADIUS + 0.15) + 0.25
    hist = collections.Counter()
    total = single = 0
    for row in range(0, grid.h, 2):
        for col in range(0, grid.w, 2):
            if grid.at(col, row):
                continue
            width = grid.free_width(*grid.grid_to_world(col, row))
            total += 1
            single += width < pass_width
            hist[round(width)] += 1
    if not total:
        print("no drivable floor")
        return 1
    print(f"  sampled {total} drivable cells")
    print(f"  two robots need {pass_width:.2f} m to pass")
    print(f"  single-file floor: {single} cells ({single / total * 100:.1f}%)")
    print("  corridor width distribution:")
    for width in sorted(hist):
        share = hist[width] / total * 100
        print(f"    {width:>3} m  {share:5.1f}%  " + "#" * int(share / 2))
    if single == 0:
        print("\n  No chokepoints. Contention here comes from robot density and\n"
              "  from blockages, not from the layout -- see --scenario blocked_aisle.")
    return 0


def run_sweep(args):
    """Measure the gain as a function of how congested the floor is.

    A single number for "coordination makes the fleet 20% faster" is close to
    meaningless without saying how many robots were sharing how many aisles --
    on an empty floor no policy can beat any other, because nothing ever has to
    yield. The sweep is the honest form of the claim.
    """
    sizes = [int(v) for v in args.sweep.split(",") if v.strip()]
    rows = []
    for n in sizes:
        row = {"robots": n}
        for policy in ("stop_and_wait", "cooperative"):
            twin = Twin(policy=policy, robots=n, seed=args.seed, tasks=args.tasks,
                        duration=args.duration, scenario=args.scenario, verbose=False)
            row[policy] = twin.run()
            print(f"  {args.scenario} n={n} {policy:<14} "
                  f"makespan={row[policy]['makespan_s']:6.1f}s "
                  f"done={row[policy]['completed']}/{row[policy]['tasks']} "
                  f"collisions={row[policy]['robot_collisions']} "
                  f"yield={row[policy]['time_yielding_s']:.0f}s")
        rows.append(row)

    print(f"\n=== {args.scenario}: {args.tasks} tasks, seed {args.seed}, "
          f"{args.duration:.0f}s window ===")
    print(f"  {'robots':>6} {'delivered':>11} {'stop-and-wait':>14} "
          f"{'cooperative':>12} {'gain':>8} {'collisions':>11} {'contention':>11}")
    for row in rows:
        base, coop = row["stop_and_wait"], row["cooperative"]
        both_done = base["complete"] and coop["complete"]
        gain, note = equal_work_gain(base, coop)
        metric = f"{gain:>7.1f}%"
        if not both_done:
            note = f"  (over {min(base['completed'], coop['completed'])} deliveries)"
        row["gain_pct"] = round(gain, 1)
        delivered = f"{base['completed']}/{coop['completed']}"
        print(f"  {row['robots']:>6} {delivered:>11} {base['makespan_s']:>13.1f}s "
              f"{coop['makespan_s']:>11.1f}s {metric} "
              f"{base['robot_collisions']}/{coop['robot_collisions']:>9} "
              f"{base['time_yielding_s']:>10.0f}s{note}")
    print("  columns pair stop-and-wait/cooperative; a makespan on a run that "
          "did not\n  deliver everything is censored, so those rows compare "
          "throughput instead.")

    if args.trace:
        args.trace.parent.mkdir(parents=True, exist_ok=True)
        args.trace.write_text(json.dumps({"scenario": args.scenario,
                                          "tasks": args.tasks, "seed": args.seed,
                                          "rows": rows}, indent=1))
        print(f"\n  sweep -> {args.trace}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robots", type=int, default=3)
    ap.add_argument("--tasks", type=int, default=12)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--duration", type=float, default=420.0)
    ap.add_argument("--policy", default="cooperative",
                    choices=["cooperative", "stop_and_wait"])
    ap.add_argument("--scenario", default="rush_hour",
                    choices=["crossing", "rush_hour", "blocked_aisle"],
                    help="rush_hour funnels the fleet through two aisles; "
                         "blocked_aisle drops an unmapped pallet stack into one "
                         "of them part-way through the run")
    ap.add_argument("--compare", action="store_true",
                    help="run both policies on the identical workload")
    ap.add_argument("--sweep", type=str, default="",
                    help="comma-separated fleet sizes to compare, e.g. 3,4,5,6")
    ap.add_argument("--trace", type=Path, help="write a playback trace here")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--measure-corridors", action="store_true",
                    help="report how much of the floor is single-file, then exit")
    args = ap.parse_args()

    if args.measure_corridors:
        return measure_corridors()

    if args.sweep:
        return run_sweep(args)

    if not args.compare:
        twin = Twin(policy=args.policy, robots=args.robots, seed=args.seed,
                    tasks=args.tasks, duration=args.duration, scenario=args.scenario,
                    trace=args.trace is not None, verbose=not args.quiet)
        rep = twin.run()
        print()
        print_report(rep)
        if args.trace:
            args.trace.parent.mkdir(parents=True, exist_ok=True)
            args.trace.write_text(json.dumps({
                "report": rep,
                "bounds": twin.layout["bounds"],
                "stations": [s.as_dict() for s in twin.stations],
                "frames": twin.trace,
            }, separators=(",", ":")))
            print(f"\n  trace -> {args.trace} ({args.trace.stat().st_size/1024:.0f} KB)")
        return 0 if rep["robot_collisions"] == 0 else 1

    results = {}
    for policy in ("stop_and_wait", "cooperative"):
        print(f"\n=== {policy} ===")
        twin = Twin(policy=policy, robots=args.robots, seed=args.seed,
                    tasks=args.tasks, duration=args.duration, scenario=args.scenario,
                    trace=args.trace is not None and policy == "cooperative",
                    verbose=not args.quiet)
        results[policy] = twin.run()
        print_report(results[policy])
        if args.trace and policy == "cooperative":
            args.trace.parent.mkdir(parents=True, exist_ok=True)
            args.trace.write_text(json.dumps({
                "report": results[policy], "baseline": results["stop_and_wait"],
                "bounds": twin.layout["bounds"],
                "stations": [s.as_dict() for s in twin.stations],
                "frames": twin.trace,
            }, separators=(",", ":")))

    base, coop = results["stop_and_wait"], results["cooperative"]
    print("\n=== comparison ===")
    if not (base["completed"] and coop["completed"]):
        print("  one of the arms delivered nothing; nothing to compare")
        return 1

    k = min(base["completed"], coop["completed"])
    gain, note = equal_work_gain(base, coop)
    print(f"  delivered  {base['completed']}/{base['tasks']} -> "
          f"{coop['completed']}/{coop['tasks']}")
    print(f"  time to deliver {k} tasks   "
          f"{base['completion_curve'][k - 1]:.1f} s -> "
          f"{coop['completion_curve'][k - 1]:.1f} s   ({gain:+.1f}%){note}")
    if base["complete"] and coop["complete"]:
        ms = ((base["makespan_s"] - coop["makespan_s"]) / base["makespan_s"] * 100
              if base["makespan_s"] else 0.0)
        print(f"  makespan   {base['makespan_s']:.1f} s -> "
              f"{coop['makespan_s']:.1f} s   ({ms:+.1f}%)")
    print(f"  throughput {base['throughput_per_min']:.2f} -> "
          f"{coop['throughput_per_min']:.2f} tasks/min")
    print(f"  yielding   {base['time_yielding_s']:.0f} s -> "
          f"{coop['time_yielding_s']:.0f} s of fleet time")
    print(f"  collisions {base['robot_collisions']} -> {coop['robot_collisions']}")

    target = 20.0
    print(f"\n  success criteria: zero inter-robot collisions, and at least "
          f"{target:.0f}% less\n  time to deliver the same work")
    ok = coop["robot_collisions"] == 0 and gain >= target
    print(f"  -> {'MET' if ok else 'NOT MET'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
