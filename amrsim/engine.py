"""
The simulation engine: one warehouse, one fleet, one tick at a time.

Two modes share the same map, task stream and fault schedule so they can be
compared fairly:

``coordinated``
    The six documented algorithms — Lamport-ordered reservations with n-way
    cluster resolution and chokepoint locks (1), congestion-aware A* with a
    fairness term and a detour cap (2), wait-graph deadlock detection with
    escape and shuffle (3), gossiped block events with confirm/soften/clear
    (4), heartbeat TTL failure detection with idempotent recovery (5), and a
    CRDT task pool with a time-boxed auction (6).

``baseline``
    The uncoordinated control arm: independent shortest-path A*, hard-stop on
    contact with a fixed-timeout random backoff, a central nearest-idle
    dispatcher, obstacles discovered only on physical arrival, and a long
    fixed failure timeout instead of heartbeat TTLs.

Tick order is fixed and deliberate: sense and agree on the world first, then
allocate work, then plan, then move, then repair.
"""

import random

from . import blocks as blocks_mod
from . import deadlock as dl
from . import failures as fl
from .config import DEFAULT
from .corridors import CorridorLocks
from .metrics import Metrics
from .planner import (plan_spacetime, plan_static, plan_with_detour_cap)
from .reservations import (LamportClock, Reservation, ReservationTable,
                           build_cluster, classify_geometry, rank_cluster)
from .robot import Robot, RobotState
from .tasks import TaskPool, TaskStatus
from .warehouse import Warehouse

ALGORITHMS = {
    "A1": "Path conflicts",
    "A2": "Congestion cost",
    "A3": "Deadlock recovery",
    "A4": "Dynamic re-routing",
    "A5": "Failure recovery",
    "A6": "Task allocation",
}


class Simulation:
    def __init__(self, mode="coordinated", seed=7, num_robots=None, cfg=DEFAULT):
        assert mode in ("coordinated", "baseline")
        self.cfg = cfg
        self.mode = mode
        self.seed = seed
        self.rng = random.Random(seed)
        self.tick = 0

        self.warehouse = Warehouse()
        self.table = ReservationTable(pad=cfg.RESERVATION_PAD)
        self.clock = LamportClock()
        self.pool = TaskPool(cfg)
        self.blocks = blocks_mod.BlockRegistry(cfg)
        self.locks = CorridorLocks(self.warehouse)
        self.metrics = Metrics(mode, cfg)

        self.robots = []
        self._spawn_fleet(num_robots or cfg.NUM_ROBOTS)
        self.by_id = {r.id: r for r in self.robots}

        self.obstacles = set()          # ground truth: what is physically there
        self.events = []                # scripted scenario events
        self.log = []                   # (tick, level, text)
        self.activity = {k: 0 for k in ALGORITHMS}
        self.scenario_name = "custom"
        self.total_expected = None

        self._known_blocks = set()      # baseline: obstacles discovered on arrival
        self._route_cache = {}          # (start, goal) -> static path, per block epoch
        self._block_epoch = -1
        self._full_blocks = frozenset()
        self._unreachable_since = {}
        self._failure_started = {}
        self._gridlock_streak = {}

    # ================================================================ setup
    def _spawn_fleet(self, n):
        caps = ["any", "any", "heavy", "any", "any", "heavy", "any", "heavy"]
        bays = self.warehouse.parking_bays
        for i in range(n):
            bay = bays[i % len(bays)]
            x, y = (bay[0], bay[1] - (i // len(bays)))
            r = Robot(id=i + 1, x=x, y=y, capability=caps[i % len(caps)])
            r.home = bay if i < len(bays) else (bay[0], bay[1] - 1)
            self.robots.append(r)

    def schedule(self, tick, fn):
        self.events.append((tick, fn))

    def emit(self, text, level="info"):
        self.log.append((self.tick, level, text))
        if len(self.log) > 400:
            del self.log[:100]

    def note(self, algo):
        self.activity[algo] = self.activity.get(algo, 0) + 1

    # -------------------------------------------------------- scenario hooks
    def spawn_task(self, kind, pickup, dropoff, capability="any", priority=1.0):
        task = self.pool.create(kind, pickup, dropoff, capability, priority,
                                self.tick, self.warehouse.zone_of(*pickup))
        if task:
            self.emit(f"T{task.id} released ({kind}, {capability})", "task")
        return task

    def place_obstacle(self, cell):
        """Physically drop something in an aisle. Robots must *discover* it."""
        self.obstacles.add(cell)
        self.emit(f"Obstacle dropped at {cell} (not yet detected)", "warn")

    def remove_obstacle(self, cell):
        self.obstacles.discard(cell)
        self.emit(f"Obstacle at {cell} removed (awaiting reprobe)", "info")

    def silence_robot(self, robot_id):
        """Stop a robot's heartbeat: the failure is only *declared* once its
        TTL expires, exactly as the algorithm specifies."""
        r = self.by_id.get(robot_id)
        if r and r.is_alive() and not r.heartbeat_frozen:
            r.heartbeat_frozen = True
            self._failure_started[robot_id] = self.tick
            self.emit(f"Robot {robot_id} went silent (heartbeat stopped)", "warn")

    def revive_robot(self, robot_id):
        r = self.by_id.get(robot_id)
        if r:
            r.heartbeat_frozen = False
            r.last_heartbeat = self.tick

    # ================================================================= tick
    def step(self):
        self.tick += 1
        self.metrics.ticks = self.tick
        self.activity = {k: 0 for k in ALGORITHMS}
        self._full_blocks = frozenset(self.blocks.full_blocks())

        for t, fn in self.events:
            if t == self.tick:
                fn(self)

        self._heartbeat_pass()          # Algorithm 5
        self._failure_pass()            # Algorithm 5
        self._sensing_pass()            # Algorithm 4
        self._allocation_pass()         # Algorithm 6
        self._target_pass()
        self._planning_pass()           # Algorithms 1 + 2 + 4
        self._movement_pass()
        self._deadlock_pass()           # Algorithm 3
        self._collision_check()
        self._completion_pass()

        self.metrics.distance = sum(r.distance for r in self.robots)
        self.metrics.sample(self.tick, self.robots, self.pool)

    def run(self, ticks):
        for _ in range(ticks):
            self.step()
        return self.metrics

    # ---------------------------------------------------- Algorithm 5 passes
    def _heartbeat_pass(self):
        for r in self.robots:
            if r.state != RobotState.FAILED and not r.heartbeat_frozen:
                r.last_heartbeat = self.tick

    def _failure_pass(self):
        ttl = (self.cfg.HEARTBEAT_TTL if self.mode == "coordinated"
               else self.cfg.BASELINE_FAILURE_TIMEOUT)
        for r in self.robots:
            if r.state == RobotState.FAILED:
                if not r.heartbeat_frozen:
                    fl.handle_recovery(r, self.table, self.blocks, self.tick,
                                       lambda m: self.emit(m, "good"))
                    self.note("A5")
                continue
            if fl.is_failed(r, self.tick, ttl):
                if fl.declare_failed(r, self.table, self.pool, self.blocks, self.tick,
                                     self.clock.tick(), lambda m: self.emit(m, "bad")):
                    self.metrics.failure_events += 1
                    started = self._failure_started.get(r.id, self.tick)
                    self.metrics.reassignment_latency.append(self.tick - started)
                    self.note("A5")

        # Lease expiry: a robot that silently stops making progress loses its
        # task to the pool. Same Lease object Algorithm 6 hands out.
        if self.mode == "coordinated":
            for task in self.pool.expired_leases(self.tick):
                holder = self.by_id.get(task.holder)
                self.pool.release_for_reassignment(task, self.tick,
                                                   item_in_transit=task.item_in_transit)
                if holder is not None and holder.task is task:
                    holder.task = None
                    holder.clear_plan()
                self.emit(f"T{task.id} lease expired — returned to the pool", "warn")
                self.note("A5")

    # ---------------------------------------------------- Algorithm 4 passes
    def _sensing_pass(self):
        """Robots scan ahead, confirm persistent obstacles, and opportunistically
        clear stale reports as they pass. Ground truth lives in `self.obstacles`;
        the fleet only ever acts on what it has actually detected."""
        if self.mode == "baseline":
            # the control arm has no gossip: a robot only learns about an
            # obstacle by arriving next to it, and never shares the fact.
            for r in self.robots:
                if not r.is_alive():
                    continue
                for nb in self.warehouse.free_neighbors(r.pos()):
                    if nb in self.obstacles and nb not in self._known_blocks:
                        self._known_blocks.add(nb)
                        self.blocks.report(nb, r.id, self.clock.tick(), self.tick)
                        self.metrics.reroute_events += 1
                        r.clear_plan()
                        self.emit(f"Robot {r.id} ran into the obstacle at {nb}", "warn")
            return

        for r in self.robots:
            if not r.is_alive():
                continue
            for cell in self._scan_cells(r):
                if cell in self.obstacles:
                    known = self.blocks.get(cell)
                    ev = self.blocks.observe(cell, r.id, self.clock.tick(), self.tick)
                    if ev is not None and known is None:
                        self.metrics.reroute_events += 1
                        self.note("A4")
                        self.emit(f"Robot {r.id} confirmed a block at {cell} — gossiped", "warn")
                        self._invalidate_paths_through(cell)
                    elif ev is not None:
                        self.blocks.refresh(cell, self.tick)
                else:
                    ev = self.blocks.get(cell)
                    if ev is not None and ev.cause == "obstacle":
                        self.blocks.clear(cell)
                        self.note("A4")
                        self.emit(f"Robot {r.id} reprobed {cell} — clear, block retracted", "good")

        softened, escalated = self.blocks.expire(self.tick)
        for cell in softened:
            self.emit(f"Block at {cell} softened to UNCONFIRMED (TTL)", "info")
        for cell in escalated:
            self._assign_reprobe(cell)

    def _scan_cells(self, robot):
        """What the robot can see: its immediate surroundings plus the next few
        cells of its committed path."""
        seen = set(self.warehouse.free_neighbors(robot.pos()))
        seen.update(c for c in robot.remaining_path()[1:4])
        return seen

    def _invalidate_paths_through(self, cell):
        """``trigger_immediate_replan`` — event-driven, not a periodic poll."""
        for r in self.robots:
            if r.is_alive() and cell in r.remaining_path():
                r.clear_plan()
                self.metrics.reroute_latency.append(1)

    def _assign_reprobe(self, cell):
        target = self.warehouse.nearest_free(
            cell, lambda c: c not in self.obstacles and c not in self.blocks, 4)
        if target is None:
            return
        task = self.pool.create("reprobe", target, target, "any", 2.0, self.tick,
                                self.warehouse.zone_of(*target))
        if task is not None:
            task.probe_cell = cell
        self.emit(f"Block at {cell} unconfirmed too long — reprobe task queued", "info")
        self.note("A4")

    # ---------------------------------------------------- Algorithm 6 passes
    def _allocation_pass(self):
        if self.mode == "coordinated":
            self.pool.run_auction(self.tick, self.robots, self.warehouse, self.clock,
                                  lambda m: (self.emit(m, "task"), self.note("A6")))
        else:
            self.pool.dispatch_nearest(self.tick, self.robots, self.warehouse)

    # ------------------------------------------------------- target selection
    def _target_pass(self):
        for r in self.robots:
            if not r.is_alive():
                r.target = None
                continue
            if r.task is None:
                home = getattr(r, "home", self.warehouse.parking_bays[0])
                r.target = None if r.pos() == home else home
                r.phase = "park"
                if r.target is None and r.state != RobotState.FAILED:
                    r.state = RobotState.IDLE
                continue
            if self.mode == "coordinated":
                self.pool.renew_lease(r.task, self.tick)
            r.target = r.task.pickup if r.phase == "to_pickup" else r.task.dropoff

    # ------------------------------------------------- Algorithms 1 + 2 + 4
    def _planning_pass(self):
        if self.mode == "baseline":
            self._baseline_plan()
            return

        self.table.clear()
        self.locks.update(self.by_id, self.tick)
        self.locks.begin_tick()

        # Stationary robots claim their cell *first*. A parked or idle robot is
        # a static obstacle, and if it only reserved its cell after the movers
        # had planned, a mover would happily route straight through it, get
        # physically blocked, replan into the same cell, and repeat forever.
        active = []
        for robot in self.robots:
            if not robot.is_alive():
                continue
            if robot.target is None:
                self._commit_hold(robot)
            else:
                active.append(robot)

        # Rank rule shared with Algorithms 3 and 6, plus the chokepoint
        # starvation bonus: the robot that has queued longest for an aisle
        # plans first, and therefore wins it. That is the FIFO in-transit lock.
        order = sorted(
            active,
            key=lambda r: (-(r.priority_score(self.warehouse)
                             + self.locks.starvation_bonus(r.id, self.tick)),
                           r.last_plan_tick, r.id),
        )

        for robot in order:
            direct = self.direct_route(robot.pos(), robot.target)
            if direct is None:
                # Goal is fully walled off by a confirmed block: stand down out
                # of the way and wait for clearance (Algorithm 4). If it stays
                # unreachable, the task goes back to the pool so one bad
                # endpoint cannot strand a robot for the rest of the run.
                self._wait_for_clearance(robot)
                since = self._unreachable_since.setdefault(robot.id, self.tick)
                if robot.task is not None and self.tick - since > self.cfg.STALE_WAIT_WATCHDOG:
                    task = robot.task
                    self.pool.release_for_reassignment(task, self.tick,
                                                       item_in_transit=task.item_in_transit)
                    task.recheck_after = self.tick + self.cfg.MAX_REQUEUE_DELAY
                    robot.task = None
                    robot.clear_plan()
                    del self._unreachable_since[robot.id]
                    self.emit(f"T{task.id} endpoint unreachable — requeued with backoff", "warn")
                    self.note("A4")
                continue
            self._unreachable_since.pop(robot.id, None)

            forbidden = self._corridor_gate(robot, direct)

            keep, blocker = self._plan_still_valid(robot, forbidden)
            if keep:
                self._commit_path(robot, robot.remaining_path())
                robot.last_action = "PROCEED"
                continue

            if blocker is not None:
                self._record_conflict(robot, blocker)

            result = plan_with_detour_cap(self.warehouse, self.table, self.blocks,
                                          robot, robot.pos(), robot.target, self.tick,
                                          self.cfg, forbidden=forbidden,
                                          direct_len=max(1, len(direct) - 1))
            self.note("A2")
            if result.steps:
                robot.path = result.steps
                robot.path_index = 0
                robot.path_t0 = self.tick
                robot.last_plan_tick = self.tick
                robot.last_action = result.action
                robot.replans += 1
                self.metrics.replans += 1
                self._commit_path(robot, result.steps)
                robot.state = (RobotState.MOVING if result.steps[1:2] != [robot.pos()]
                               else RobotState.WAITING)
            else:
                self._hold_at_mouth(robot, direct, forbidden)

    def _plan_still_valid(self, robot, forbidden):
        """Can the robot keep the path it already committed to? Returns
        ``(ok, blocking_robot_id)``. This *is* Algorithm 1's cluster check: a
        path stays committed until somebody who outranks this robot has taken
        space it needs."""
        rem = robot.remaining_path()
        if len(rem) < 2 or rem[-1] != robot.target:
            return False, None
        if self.tick - robot.last_plan_tick >= self.cfg.REPLAN_INTERVAL:
            return False, None            # periodic opportunistic replan
        prev = rem[0]
        # Only the cooperative window is re-validated: beyond it, nobody's plan
        # is authoritative, so checking it would cause needless replan churn.
        for i, cell in enumerate(rem[:self.cfg.COOP_WINDOW + 1]):
            at = self.tick + i
            if cell in self._full_blocks:
                return False, None
            if cell in forbidden:
                return False, None
            who = self.table.occupant(cell, at)
            if who is not None and who != robot.id:
                return False, who
            if cell != prev and self.table.swap_conflict(prev, cell, at - 1, exclude=robot.id):
                return False, self.table.occupant(cell, at - 1)
            prev = cell
        return True, None

    def _record_conflict(self, robot, blocker_id):
        """Build the n-way cluster around the contested space, rank it with the
        shared rule, and classify the geometry — the reporting half of
        Algorithm 1. The ranking itself is enforced by the planning order:
        whoever ranks higher has already committed, so the loser is the one
        replanning here."""
        other = self.table.get(blocker_id)
        if other is None:
            return
        mine = Reservation(robot.id, self.tick, robot.remaining_path() or [robot.pos()],
                           robot.priority_score(self.warehouse), self.clock.tick())
        mine.build_windows(self.cfg.RESERVATION_PAD)
        cluster = build_cluster(mine, self.table)
        if len(cluster) <= 1:
            return
        ranked = rank_cluster(cluster)
        geometry = classify_geometry(mine.direction(), other.direction())
        robot.last_geometry = geometry
        robot.waiting_for = blocker_id
        self.metrics.conflict_events += 1
        self.note("A1")
        if len(cluster) > 2:
            self.emit(f"{len(cluster)}-way conflict cluster "
                      f"{[r.robot_id for r in ranked]} — robot {ranked[0].robot_id} wins",
                      "warn")

    def _commit_path(self, robot, steps):
        res = Reservation(robot_id=robot.id, t0=self.tick, steps=list(steps),
                          priority_score=robot.priority_score(self.warehouse),
                          lamport_ts=self.clock.tick(),
                          tail=self.cfg.COOP_WINDOW if len(steps) <= 1 else 4)
        self.table.commit(res)

    def _commit_hold(self, robot):
        """A robot with nowhere to go still occupies a cell, and must be planned
        around like any other obstacle."""
        robot.clear_plan()
        robot.last_action = "IDLE"
        res = Reservation(robot_id=robot.id, t0=self.tick, steps=[robot.pos()],
                          priority_score=robot.priority_score(self.warehouse),
                          lamport_ts=self.clock.tick(), tail=self.cfg.COOP_WINDOW)
        self.table.commit(res)
        if robot.state not in (RobotState.FAILED,):
            robot.state = RobotState.IDLE

    def _wait_for_clearance(self, robot):
        """``plan_around_block`` with no route left: step out of the traffic
        lane and hold, reusing Algorithm 3's free-cell search."""
        occupied = {r.pos() for r in self.robots if r.is_alive() and r.id != robot.id}
        spot = dl.search_expanding_radius(self.warehouse, robot.pos(), set(),
                                          self.blocks, occupied, self.cfg)
        robot.state = RobotState.WAITING
        robot.last_action = "WAIT_CLEARANCE"
        if spot and len(self.warehouse.free_neighbors(robot.pos())) <= 2:
            steps = plan_spacetime(self.warehouse, self.table, self.blocks, robot,
                                   robot.pos(), spot, self.tick, self.cfg)[0]
            if steps:
                robot.path, robot.path_index = steps, 0
                robot.last_plan_tick = self.tick
                self._commit_path(robot, steps)
                return
        self._commit_hold(robot)
        robot.state = RobotState.WAITING

    # ------------------------------------- Algorithm 1 component 3: chokepoints
    def _corridor_gate(self, robot, direct):
        """Claim the in-transit lock for the next chokepoint on the route.

        Registering interest is what puts the robot in the aisle's FIFO queue;
        the queue is then honoured through the planning order rather than by
        blocking cells, so a robot can still choose to wait a couple of ticks
        and go through — which is exactly the ``select_action`` trade-off the
        reference describes, and usually cheaper than driving around a rack.
        """
        for cell in direct[:self.cfg.CORRIDOR_LOOKAHEAD + 1]:
            cid = self.warehouse.corridor_of.get(cell)
            if cid is None:
                continue
            corridor = self.locks.corridors[cid]
            direction = 1 if cell == corridor.cells[0] else -1
            self.note("A1")
            if self.locks.request(robot, corridor, direction, self.tick,
                                  robot.priority_score(self.warehouse)):
                self.metrics.corridor_grants += 1
            break
        return self.locks.forbidden_for(robot.id)

    def _hold_at_mouth(self, robot, direct, forbidden):
        """No legal route right now — usually because a chokepoint lock is held
        the other way. Queue at the mouth of the aisle we are waiting on rather
        than milling about in the open."""
        standoff = None
        for idx, cell in enumerate(direct):
            if cell in forbidden:
                # stand one cell clear of the mouth, never on it
                standoff = next((c for c in direct[max(0, idx - 2):idx]
                                 if c not in forbidden), None)
                break
        robot.last_action = "QUEUE"
        robot.state = RobotState.QUEUED
        if standoff and standoff != robot.pos():
            steps = plan_spacetime(self.warehouse, self.table, self.blocks, robot,
                                   robot.pos(), standoff, self.tick, self.cfg,
                                   forbidden=forbidden)[0]
            if steps:
                robot.path, robot.path_index = steps, 0
                robot.last_plan_tick = self.tick
                self._commit_path(robot, steps)
                return
        self._commit_hold(robot)
        robot.state = RobotState.QUEUED
        robot.wait_ticks += 1
        self.metrics.conflict_wait_ticks += 1

    # ------------------------------------------------------- baseline planner
    def _baseline_plan(self):
        for robot in self.robots:
            if not robot.is_alive() or robot.target is None:
                if robot.is_alive():
                    robot.clear_plan()
                    robot.state = RobotState.IDLE
                continue
            if robot.has_plan() and robot.path[-1] == robot.target:
                continue
            path = self.direct_route(robot.pos(), robot.target)
            if path is None:
                robot.state = RobotState.WAITING
                robot.clear_plan()
                continue
            robot.path, robot.path_index = path, 0
            robot.last_plan_tick = self.tick
            robot.replans += 1
            robot.state = RobotState.MOVING
            robot.last_action = "PROCEED"

    # ---------------------------------------------------------------- motion
    def _movement_pass(self):
        movers = [r for r in self.robots if r.is_alive()]
        intent = {}
        for r in movers:
            if r.resume_at > self.tick:
                intent[r.id] = r.pos()        # still spinning back up to speed
            else:
                intent[r.id] = r.next_cell() if r.has_plan() else r.pos()

        occupied = {r.pos(): r.id for r in movers}
        static_blocks = self._full_blocks

        # 1. two robots must never target the same cell (guaranteed by the
        #    reservation table in coordinated mode; enforced here for baseline)
        contenders = {}
        for rid, cell in intent.items():
            if cell != self.by_id[rid].pos():
                contenders.setdefault(cell, []).append(rid)
        for cell, ids in contenders.items():
            if len(ids) < 2:
                continue
            ids.sort(key=lambda i: (-self.by_id[i].priority_score(self.warehouse), i))
            for loser in ids[1:]:
                intent[loser] = self.by_id[loser].pos()
                self._block_robot(self.by_id[loser], self.by_id[ids[0]].id)

        # 2. resolve move chains; robots in a rotation cycle all move together
        decided = {}
        pending = set()
        for r in movers:
            cell = intent[r.id]
            if cell == r.pos():
                decided[r.id] = False
            elif cell in static_blocks or not self.warehouse.is_walkable(*cell):
                decided[r.id] = False
                self._block_robot(r, None)
            else:
                pending.add(r.id)

        changed = True
        while changed and pending:
            changed = False
            for rid in list(pending):
                cell = intent[rid]
                holder = occupied.get(cell)
                if holder is None or holder == rid or decided.get(holder) is True:
                    decided[rid] = True
                    pending.discard(rid)
                    changed = True
                elif decided.get(holder) is False:
                    decided[rid] = False
                    pending.discard(rid)
                    changed = True
                    self._block_robot(self.by_id[rid], holder)
        for rid in pending:          # a clean rotation: everybody advances
            decided[rid] = True

        # 3. apply
        for r in movers:
            if decided.get(r.id):
                r.advance_to(intent[r.id], self.cfg, self.tick)
                r.waiting_for = None
                r.wait_started = None
                r.state = RobotState.MOVING if r.has_plan() else (
                    RobotState.IDLE if r.task is None else RobotState.MOVING)
            elif r.resume_at > self.tick:
                r.state = RobotState.WAITING
                self.metrics.conflict_wait_ticks += 1
            elif intent[r.id] == r.pos() and r.has_plan():
                # a *planned* wait: the space-time path deliberately holds here
                r.advance_to(r.pos(), self.cfg, self.tick)
                r.state = RobotState.WAITING
                self.metrics.conflict_wait_ticks += 1
                if r.waiting_for is None:
                    r.waiting_for = self._who_is_in_my_way(r)
                dl.wait_stalled(r, self.tick, self.cfg)

        if self.mode == "baseline":
            self._baseline_backoff(occupied)

    def _block_robot(self, robot, blocker_id):
        """An *unplanned* stop: the robot expected to move and could not. It
        pays the deceleration/re-acceleration cost before it rolls again."""
        robot.state = RobotState.WAITING
        robot.waiting_for = blocker_id
        robot.wait_ticks += 1
        # Charge the deceleration/re-acceleration cost once per stop *event* —
        # a robot already standing still does not brake again every tick.
        if self.tick - robot.last_moved_tick <= 1 and robot.resume_at <= self.tick:
            robot.resume_at = self.tick + self.cfg.HARD_STOP_RESUME_TICKS
            self.metrics.hard_stops += 1
        self.metrics.conflict_wait_ticks += 1
        dl.wait_stalled(robot, self.tick, self.cfg)

    def _who_is_in_my_way(self, robot):
        """Fill in the wait-graph edge: whichever robot is sitting on the cells
        we would occupy next if nothing were in the way."""
        ahead = [c for c in robot.remaining_path()[1:5] if c != robot.pos()]
        if not ahead and robot.target is not None:
            direct = self.direct_route(robot.pos(), robot.target)
            ahead = direct[1:3] if direct else []
        for cell in ahead[:3]:
            for other in self.robots:
                if other.id != robot.id and other.is_alive() and other.pos() == cell:
                    return other.id
        return None

    def _baseline_backoff(self, occupied):
        """The control arm's only escape hatch: after a fixed timeout, step to a
        random free neighbour and hope. No cycle detection, no coordination."""
        for r in self.robots:
            if not r.is_alive() or r.state != RobotState.WAITING:
                continue
            if r.wait_started is None:
                r.wait_started = self.tick
            if (self.tick - r.wait_started) <= self.cfg.BASELINE_STALL_TIMEOUT:
                continue
            options = [n for n in self.warehouse.free_neighbors(r.pos())
                       if n not in occupied and n not in self._full_blocks]
            if options:
                nxt = self.rng.choice(options)
                r.path, r.path_index = [r.pos(), nxt], 0
                r.state = RobotState.MOVING
            r.wait_started = None

    # ---------------------------------------------------- Algorithm 3 passes
    def _deadlock_pass(self):
        if self.mode != "coordinated":
            return
        stuck = [r for r in self.robots
                 if r.is_alive() and r.state in (RobotState.WAITING, RobotState.QUEUED)]
        waiting = [r for r in stuck if r.waiting_for is not None]

        # Liveness watchdog: being blocked by something that is not itself
        # waiting never forms a cycle, so cycle detection alone would let such
        # a robot starve. A bounded timeout forces a fresh plan.
        for r in stuck:
            if r.wait_started is None:
                r.wait_started = self.tick
            if (self.tick - r.wait_started) > self.cfg.STALE_WAIT_WATCHDOG:
                r.clear_plan()
                r.wait_started = None
                r.last_plan_tick = -999
                self.locks.withdraw(r.id)
                self.emit(f"Robot {r.id} stale-wait watchdog fired — forcing a replan", "warn")
                self.note("A3")

        for r in waiting:
            if not dl.wait_stalled(r, self.tick, self.cfg):
                continue
            cycle = dl.find_cycle(r, self.by_id, len(self.robots))
            if not cycle:
                continue
            resolver = min(cycle)
            if r.id != resolver:
                r.state = RobotState.IN_RESOLUTION      # freeze, await the resolver
                continue

            self.metrics.deadlock_events += 1
            self.note("A3")
            started = r.wait_started or self.tick
            exclude = {self.by_id[i].pos() for i in cycle if i in self.by_id}
            occupied = {rb.pos() for rb in self.robots if rb.is_alive() and rb.id != r.id}

            escape = dl.search_expanding_radius(self.warehouse, r.pos(), exclude,
                                                self.blocks, occupied, self.cfg)
            if escape and self._move_to(r, escape):
                self.metrics.deadlock_recovery.append(self.tick - started)
                self.emit(f"Deadlock {cycle} resolved — robot {r.id} escaped to {escape}", "good")
                self._unfreeze(cycle, r.id)
                continue

            occupied_by = {rb.pos(): rb.id for rb in self.robots if rb.is_alive()}
            chain, free_cell = dl.find_shuffle_chain(self.warehouse, r.pos(), occupied_by,
                                                     set(cycle), self.blocks, self.cfg)
            if chain:
                for rid in reversed(chain):
                    mover = self.by_id.get(rid)
                    if mover is None:
                        continue
                    spot = dl.search_expanding_radius(self.warehouse, mover.pos(), exclude,
                                                      self.blocks,
                                                      {rb.pos() for rb in self.robots
                                                       if rb.is_alive() and rb.id != rid},
                                                      self.cfg)
                    if spot:
                        self._move_to(mover, spot)
                        mover.state = RobotState.SHUFFLING
                self.metrics.deadlock_recovery.append(self.tick - started)
                self.emit(f"Deadlock {cycle} resolved via shuffle chain {chain}", "good")
                self._unfreeze(cycle, r.id)
                continue

            self.metrics.gridlocks += 1
            self.emit(f"GRIDLOCK among {cycle} — no escape cell, retrying after cooldown", "bad")
            self._unfreeze(cycle, r.id)
            streak = self._gridlock_streak.get(tuple(sorted(cycle)), 0) + 1
            self._gridlock_streak[tuple(sorted(cycle))] = streak
            if streak >= 3:
                self._break_deadlock_by_release(cycle)

    def _move_to(self, robot, cell):
        steps = plan_spacetime(self.warehouse, self.table, self.blocks, robot,
                               robot.pos(), cell, self.tick, self.cfg,
                               forbidden=self.locks.forbidden_for(robot.id))[0]
        if steps is None:
            steps = plan_static(self.warehouse, self.blocks, robot.pos(), cell, self.cfg)
        if not steps or len(steps) < 2:
            return False
        robot.path, robot.path_index = steps, 0
        robot.last_plan_tick = self.tick
        robot.state = RobotState.MOVING
        robot.waiting_for = None
        robot.wait_started = None
        self._commit_path(robot, steps)
        return True

    def _unfreeze(self, cycle, resolver_id):
        for rid in cycle:
            if rid == resolver_id:
                continue
            other = self.by_id.get(rid)
            if other and other.state == RobotState.IN_RESOLUTION:
                other.clear_plan()
                other.waiting_for = None
                other.wait_started = None
                other.last_plan_tick = -999
                other.state = RobotState.MOVING if other.task else RobotState.IDLE

    def _break_deadlock_by_release(self, cycle):
        """Last resort: a structurally contested spot is not fixable locally, so
        the lowest-priority robot's task goes back to the pool and it parks."""
        members = [self.by_id[i] for i in cycle if i in self.by_id and self.by_id[i].task]
        if not members:
            return
        loser = min(members, key=lambda rb: rb.priority_score(self.warehouse))
        self.pool.release_for_reassignment(loser.task, self.tick)
        loser.task = None
        loser.clear_plan()
        loser.last_plan_tick = -999
        self.locks.withdraw(loser.id)
        self._gridlock_streak.clear()
        self.emit(f"Circuit breaker: robot {loser.id}'s task returned to the pool", "warn")

    # ------------------------------------------------------------ invariants
    def _collision_check(self):
        seen = {}
        for r in self.robots:
            if not r.is_alive():
                continue
            if r.pos() in seen:
                self.metrics.collisions += 1
                self.emit(f"COLLISION at {r.pos()}: robots {seen[r.pos()]} and {r.id}", "bad")
            seen[r.pos()] = r.id

    def _completion_pass(self):
        for r in self.robots:
            if not r.is_alive() or r.task is None:
                continue
            task = r.task
            if r.phase == "to_pickup" and r.pos() == task.pickup:
                r.phase = "to_dropoff"
                task.item_in_transit = True
                r.clear_plan()
                r.last_plan_tick = -999
                if task.kind == "reprobe":
                    self.blocks.clear(getattr(task, "probe_cell", task.pickup))
                self.emit(f"Robot {r.id} picked up T{task.id}", "info")
            elif r.phase == "to_dropoff" and r.pos() == task.dropoff:
                elapsed = self.tick - task.created_tick
                self.pool.complete(task, self.tick)
                if task.kind != "reprobe":     # housekeeping is not throughput
                    self.metrics.completed.append((task.id, elapsed))
                task.item_in_transit = False
                r.tasks_done += 1
                r.task = None
                r.clear_plan()
                r.last_plan_tick = -999
                r.state = RobotState.IDLE
                self.locks.withdraw(r.id)
                self.emit(f"T{task.id} delivered by robot {r.id} in {elapsed} ticks", "good")

    # -------------------------------------------------------------- routing
    def direct_route(self, start, goal):
        """Block-aware, distance-only route, memoised per block epoch. This is
        the *direct distance estimate* Algorithm 2's detour cap is measured
        against, and it is needed several times per robot per tick."""
        epoch = self.blocks.version
        if epoch != self._block_epoch:
            self._block_epoch = epoch
            self._route_cache.clear()
        key = (start, goal)
        if key not in self._route_cache:
            if len(self._route_cache) > 6000:
                self._route_cache.clear()
            self._route_cache[key] = plan_static(self.warehouse, self.blocks,
                                                 start, goal, self.cfg)
        return self._route_cache[key]

    # ---------------------------------------------------------------- output
    def snapshot(self, include_layout=False):
        m = self.metrics
        data = {
            "tick": self.tick,
            "mode": self.mode,
            "seed": self.seed,
            "scenario": self.scenario_name,
            "robots": [r.as_dict() for r in self.robots],
            "tasks": [t.as_dict() for t in self.pool.all()
                      if t.status != TaskStatus.DONE][:60],
            "blocks": self.blocks.snapshot(),
            "obstacles": [list(c) for c in self.obstacles],
            "corridors": self.locks.snapshot(),
            "heat": [[c[0], c[1], v] for c, v in self.table.heat(self.tick).items()],
            "activity": self.activity,
            "metrics": m.summary(),
            "history": list(m.history)[-180:],
            "log": [{"t": t, "level": lv, "text": tx} for t, lv, tx in self.log[-40:]],
            "auctions": self.pool.auction_log[-10:],
            "total_expected": self.total_expected,
        }
        if include_layout:
            data["layout"] = self.warehouse.static_layout()
        return data
