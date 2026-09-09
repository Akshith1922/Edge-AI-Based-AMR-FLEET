"""
Scenarios. Each builds an identical world for both modes — same map, same task
list, same faults, same tick budget — so the comparison in the evaluation
chapter is genuinely apples-to-apples.

``rush_hour`` is the documented base case: traffic funnelled through the same
few single-file picking aisles, an aisle blocked mid-run, and a robot dropping
off the network while carrying an item.
"""

import random

SCENARIOS = {}


def scenario(name, label, description):
    def wrap(fn):
        SCENARIOS[name] = {"name": name, "label": label,
                           "description": description, "build": fn}
        return fn
    return wrap


class Points:
    """Layout-derived task endpoints, so scenarios never hard-code coordinates."""

    def __init__(self, wh):
        aisles = sorted({c[0] for c in wh.pick_faces if 4 <= c[0] <= 32})
        self.hot_aisles = aisles[len(aisles) // 2 - 1:len(aisles) // 2 + 1] or aisles[:2]
        self.hot = [c for c in wh.pick_faces if c[0] in self.hot_aisles]
        self.cold = [c for c in wh.pick_faces if c[0] not in self.hot_aisles]
        self.receiving = [(1, y) for y in (6, 10, 14, 18) if wh.is_walkable(1, y)]
        self.pickpack = [(38, y) for y in (5, 7, 9) if wh.is_walkable(38, y)]
        self.shipping = [(38, y) for y in (14, 16, 18) if wh.is_walkable(38, y)]
        self.outbound = self.pickpack + self.shipping

    def face(self, rng, hot_bias=0.65):
        pool = self.hot if (self.hot and rng.random() < hot_bias) else self.cold
        return rng.choice(pool or self.cold)


def _task(kind, pickup, dropoff, cap, priority=1.0):
    return lambda s: s.spawn_task(kind, pickup, dropoff, capability=cap,
                                  priority=priority)


def _faults(sim, rng, block_tick=60, fail_tick=110, clear_tick=220):
    """The two documented faults: a blocked aisle and a silent robot."""
    cell = sim.warehouse.fault_aisle[len(sim.warehouse.fault_aisle) // 2]

    def drop(s):
        s.place_obstacle(cell)

    def clear(s):
        s.remove_obstacle(cell)

    def fail(s):
        carrying = [r for r in s.robots if r.is_alive() and r.task is not None]
        if carrying:
            s.silence_robot(rng.choice(carrying).id)

    sim.schedule(block_tick, drop)
    sim.schedule(fail_tick, fail)
    sim.schedule(clear_tick, clear)


@scenario("rush_hour", "Cross-Aisle Rush Hour",
          "Continuous inbound/outbound traffic funnelled through the two busiest "
          "single-file picking aisles, with an aisle blocked at t=60 and a robot "
          "going silent at t=110. The open-ended demo case.")
def rush_hour(sim, seed=7, rate=0.34, backlog_cap=4, faults=True):
    rng = random.Random(seed)
    pts = Points(sim.warehouse)
    sim.scenario_name = "rush_hour"

    def burst(s):
        for _ in range(2):
            s.spawn_task("pick", pts.face(rng), rng.choice(pts.outbound), priority=2.0)

    sim.schedule(3, burst)
    if faults:
        _faults(sim, rng)

    def background(s):
        # Release policy: hold new work once the outstanding queue reaches
        # backlog_cap per robot. Without a cap an open-ended demo just
        # accumulates a backlog it can never clear, which tells you nothing
        # except that arrivals outpace the fleet.
        if rng.random() >= rate:
            return
        if s.pool.stats(s.tick)["outstanding"] > backlog_cap * len(s.robots):
            return
        if rng.random() < 0.5:
            cap = "heavy" if rng.random() < 0.2 else "any"
            s.spawn_task("putaway", rng.choice(pts.receiving), pts.face(rng),
                         capability=cap, priority=1.0)
        else:
            s.spawn_task("pick", pts.face(rng), rng.choice(pts.outbound),
                         priority=1.0)

    # A continuous stream, not a pre-scheduled block: the demo should never
    # run out of work and leave the fleet standing in the charging bays.
    sim.every_tick(background, start=4)
    return sim


@scenario("rush_hour_fixed", "Rush Hour (deterministic)",
          "The same stress episode with a closed, fixed task list instead of a "
          "random stream, so both modes are measured on an identical workload. "
          "This is the scenario the benchmark uses.")
def rush_hour_fixed(sim, seed=7, count=48, spacing=4, faults=True):
    rng = random.Random(seed)
    pts = Points(sim.warehouse)
    sim.scenario_name = "rush_hour_fixed"

    # Pre-generate the whole workload once, then schedule it at fixed ticks:
    # both modes then see the exact same tasks arriving at the exact same time.
    for i in range(count):
        if rng.random() < 0.5:
            cap = "heavy" if rng.random() < 0.2 else "any"
            job = _task("putaway", rng.choice(pts.receiving), pts.face(rng), cap)
        else:
            job = _task("pick", pts.face(rng), rng.choice(pts.outbound), "any")
        sim.schedule(4 + i * spacing, job)

    if faults:
        _faults(sim, rng)
    sim.total_expected = count
    return sim


@scenario("chokepoint_duel", "Chokepoint Duel",
          "The minimal readable case: robots sent into the same single-file "
          "aisle from opposite ends at the same instant, over and over.")
def chokepoint_duel(sim, seed=3, rounds=8, **_):
    wh = sim.warehouse
    sim.scenario_name = "chokepoint_duel"
    aisle = wh.corridors[0]
    north, south = aisle.ends
    far_north = (north[0], max(0, north[1] - 2))
    far_south = (south[0], min(wh.h - 1, south[1] + 2))

    def duel(s):
        s.spawn_task("pick", aisle.cells[-1], far_north, priority=2.0)
        s.spawn_task("pick", aisle.cells[0], far_south, priority=2.0)
        s.spawn_task("pick", far_north, far_south, priority=1.8)
        s.spawn_task("pick", far_south, far_north, priority=1.8)

    for i in range(rounds):
        sim.schedule(2 + i * 40, duel)
    return sim


@scenario("failure_storm", "Failure Storm",
          "Robots drop off the network in waves while carrying items, exercising "
          "TTL detection, the item-recovery alert and re-auction under load.")
def failure_storm(sim, seed=11, **_):
    rng = random.Random(seed)
    pts = Points(sim.warehouse)
    sim.scenario_name = "failure_storm"

    def burst(s):
        for _ in range(6):
            s.spawn_task("pick", pts.face(rng), rng.choice(pts.outbound), priority=1.5)

    def kill(n):
        def fn(s):
            carrying = [r for r in s.robots if r.is_alive() and r.task is not None]
            rng.shuffle(carrying)
            for victim in carrying[:n]:
                s.silence_robot(victim.id)
        return fn

    for t in (2, 50, 100, 150):
        sim.schedule(t, burst)
    sim.schedule(30, kill(1))
    sim.schedule(70, kill(2))
    sim.schedule(130, kill(2))
    return sim


@scenario("aisle_gridlock", "Aisle Gridlock Drill",
          "A robot dies inside a single-file aisle with another right behind "
          "it. The dead robot becomes a static obstacle, the one behind is "
          "walled in, and Algorithm 3's escape search has to reverse it out.")
def aisle_gridlock(sim, seed=5, **_):
    rng = random.Random(seed)
    pts = Points(sim.warehouse)
    sim.scenario_name = "aisle_gridlock"
    aisle = sim.warehouse.corridors[0]

    def feed(s):
        # send several robots down the same aisle, nose to tail
        for cell in (aisle.cells[1], aisle.cells[3], aisle.cells[5]):
            s.spawn_task("pick", cell, rng.choice(pts.outbound), priority=2.0)

    def kill_inside(s):
        """Silence whichever robot is deepest inside the aisle."""
        inside = [r for r in s.robots if r.is_alive() and r.pos() in aisle.cells]
        if inside:
            s.silence_robot(max(inside, key=lambda r: aisle.cells.index(r.pos())).id)

    for i in range(4):
        sim.schedule(2 + i * 70, feed)
        sim.schedule(20 + i * 70, kill_inside)
        sim.schedule(55 + i * 70, lambda s: [s.revive_robot(r.id) for r in s.robots])
    return sim


def build(sim, name="rush_hour", **kwargs):
    if name not in SCENARIOS:
        raise KeyError(f"unknown scenario {name!r}; have {sorted(SCENARIOS)}")
    SCENARIOS[name]["build"](sim, **kwargs)
    return sim


def catalogue():
    return [{"name": s["name"], "label": s["label"], "description": s["description"]}
            for s in SCENARIOS.values()]
