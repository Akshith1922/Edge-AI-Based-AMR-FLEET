"""
The edge agent: everything one robot decides, with no ROS anywhere in it.

`EdgeAgent` takes in odometry, a laser scan and whatever its neighbours have
broadcast, and produces a velocity command plus its own broadcast. That is the
whole interface. `edge_fleet_agent.py` wires it to ROS topics; the headless
twin wires it to a kinematic model; the unit tests wire it to nothing at all.
Keeping the decision-making free of the transport is what makes it possible to
test a fleet's behaviour without starting a simulator.

Layers, outermost first:

    task allocation   which pallet am I fetching?          allocation.py
    coordination      whose corridor is this?              protocol.py
    global planning   which aisles get me there?           navigation.AStar
    local planning    what is in front of me right now?    navigation.LocalPlanner

Each layer can only constrain the one below it, never reach past it. The
coordination layer, for instance, cannot steer -- it can make cells expensive
and it can cap speed, and that is deliberately all. So a coordination bug can
make the fleet slow or silly, but it cannot drive a robot into a rack, because
the local planner rejects every trajectory that collides regardless of what it
has been asked to do.
"""

import math
import time

from .allocation import TaskPool, route_cost, UNREACHABLE
from .navigation import (PEER_MARGIN, AStar, Limits, LocalPlanner, Obstacle,
                         PathTracker, scan_to_obstacles, wrap)
from .protocol import (CLAIM_HORIZON_M, Coordinator, FleetState, PeerTable,
                       AGING_RATE, BASE_PRIORITY, STALL_SPEED)

IDLE = "IDLE"
TO_PICKUP = "TO_PICKUP"
TO_DROPOFF = "TO_DROPOFF"
TO_CHARGER = "TO_CHARGER"
CHARGING = "CHARGING"
SERVICING = "SERVICING"

GOAL_TOLERANCE = 0.45
SERVICE_TIME_S = 2.0
REPLAN_INTERVAL_S = 1.5
SCAN_STALE_S = 1.0

BATTERY_PER_METRE = 0.09
BATTERY_IDLE_PER_S = 0.012
BATTERY_LOW = 22.0
BATTERY_FULL = 95.0
CHARGE_RATE_PER_S = 6.0

# A robot that cannot find a safe trajectory for this long has hit something
# the map does not know about, and says so.
BLOCK_CONFIRM_S = 2.0
BLOCK_TTL_S = 45.0

# Auction throttling: how often an idle robot re-bids, and how many of the
# nearest open tasks it is willing to cost a route to.
BID_INTERVAL_S = 1.0
BID_CANDIDATES = 5


class AgentOutput:
    __slots__ = ("v", "w", "mesh", "tasks", "telemetry")

    def __init__(self, v, w, mesh, tasks, telemetry):
        self.v, self.w = v, w
        self.mesh, self.tasks = mesh, tasks
        self.telemetry = telemetry


class EdgeAgent:
    def __init__(self, robot_id, grid, limits=None, charger=None,
                 battery=100.0, clock=time.monotonic, policy="cooperative"):
        self.id = robot_id
        self.grid = grid
        self.lim = limits or Limits()
        self.clock = clock
        self.charger = charger

        self.planner = AStar(grid)
        self.local = LocalPlanner(grid, self.lim)
        self.tracker = PathTracker(lookahead=max(0.8, 4 * grid.resolution))
        self.peers = PeerTable(robot_id)
        self.coord = Coordinator(grid, self.lim.radius, self.lim.safety, policy)
        self.policy = policy
        self.pool = TaskPool(robot_id)

        self.state = FleetState(robot_id)
        self.state.battery = battery
        self.state.mode = IDLE

        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.have_pose = False

        self._scan = None
        self._scan_time = -1e9
        self._last_plan = -1e9
        self._plan_goal = None
        self._stalled_since = None
        self._service_until = 0.0
        self._blocked = {}            # cell -> expiry time
        self._block_since = None
        self._last_bid = -1e9
        self._last_step = None
        self._last_xy = None
        self._decision_reason = "boot"

        # metrics
        self.distance_travelled = 0.0
        self.tasks_done = 0
        self.time_yielding = 0.0
        self.time_blocked = 0.0
        self.replans = 0

    # -- inputs ------------------------------------------------------------

    def set_pose(self, x, y, yaw, v=0.0, w=0.0):
        if self.have_pose and self._last_xy is not None:
            self.distance_travelled += math.hypot(x - self._last_xy[0],
                                                  y - self._last_xy[1])
        self.x, self.y, self.yaw = x, y, wrap(yaw)
        self.v, self.w = v, w
        self._last_xy = (x, y)
        self.have_pose = True

    def set_scan(self, ranges, angle_min, angle_increment, range_max, now=None):
        self._scan = (list(ranges), angle_min, angle_increment, range_max)
        self._scan_time = self.clock() if now is None else now

    def on_mesh(self, text, now=None):
        now = self.clock() if now is None else now
        msg = FleetState.from_json(text)
        if self.peers.observe(msg, now):
            for cell in msg["blocked"]:
                self._blocked.setdefault(cell, now + BLOCK_TTL_S)
            return True
        return False

    def on_tasks(self, text, now=None):
        now = self.clock() if now is None else now
        return self.pool.merge(text, now)

    def submit_task(self, task):
        return self.pool.add(task)

    # -- the loop ----------------------------------------------------------

    def step(self, now=None):
        now = self.clock() if now is None else now
        dt = 0.0 if self._last_step is None else max(0.0, now - self._last_step)
        self._last_step = now

        if not self.have_pose:
            return self._output(0.0, 0.0, now, "waiting for odometry")

        lost = self.peers.tick(now)
        for pid in lost:
            # A silent peer's work is nobody's until someone bids for it again.
            self.pool.forget_peer(pid)

        self._expire_blocks(now)
        self._update_battery(dt)
        self._run_allocation(now)
        self._bid_if_idle(now)
        goal = self._goal_for_mode(now)

        if goal is None:
            self.tracker.clear()
            self.state.goal = None
            self._decision_reason = "no work"
            return self._output(0.0, 0.0, now, self._decision_reason)

        self.state.goal = goal
        reached = math.hypot(goal[0] - self.x, goal[1] - self.y) < GOAL_TOLERANCE
        if reached:
            self._on_goal_reached(now)
            return self._output(0.0, 0.0, now, "arrived")

        # --- decide with the fleet, then plan against that decision --------
        self._publish_claim(now)
        peers = self.peers.alive()
        stalled_for = 0.0 if self._stalled_since is None else now - self._stalled_since
        decision = self.coord.decide(self.state, peers, now,
                                     stalled_for=stalled_for)
        self._decision_reason = decision.reason
        self.state.waiting_for = decision.waiting_for
        if decision.yielded:
            self.time_yielding += dt
            self.state.priority += AGING_RATE * dt      # ageing beats starvation
        else:
            self.state.priority = max(BASE_PRIORITY,
                                      self.state.priority - AGING_RATE * dt)

        penalties = dict(decision.penalties)
        for cell in self._blocked:
            penalties[cell] = penalties.get(cell, 0.0) + 1e6   # effectively closed

        self._maybe_replan(goal, penalties, now)

        # --- retreat overrides path following ------------------------------
        if decision.retreat_to is not None:
            self.state.retreating = True
            return self._drive_retreat(decision.retreat_to, now)
        self.state.retreating = False

        carrot = self.tracker.carrot(self.x, self.y)
        if carrot is None:
            self._decision_reason = "no route"
            return self._output(0.0, 0.0, now, self._decision_reason)

        obstacles = self._obstacles(now, peers)
        v_cmd, w_cmd = self.local.compute(self.x, self.y, self.yaw, self.v, self.w,
                                          carrot, obstacles, decision.speed_cap)

        self._track_stall(v_cmd, now, dt, decision)
        return self._output(v_cmd, w_cmd, now, self._decision_reason)

    # -- pieces ------------------------------------------------------------

    def _obstacles(self, now, peers):
        """What the local planner must not hit: lidar returns and peers.

        Peers are added explicitly rather than left to the lidar because a
        neighbour's *intent* is knowable and its position is not always visible
        -- around a rack corner, the broadcast arrives before the reflection
        does. The lidar is what catches everything nobody announced.
        """
        obstacles = []
        if self._scan and now - self._scan_time < SCAN_STALE_S:
            ranges, a_min, a_inc, r_max = self._scan
            # Roughly 90 beams is all a 0.4 m robot can act on, whatever the
            # sensor reports; a 674-beam Tugbot scan subsamples to every 8th.
            stride = max(1, len(ranges) // 90)
            obstacles = scan_to_obstacles(self.x, self.y, self.yaw, ranges,
                                          a_min, a_inc, r_max, stride=stride)
        for peer in peers:
            # Grow the disc along the peer's own velocity: where it will be
            # matters more than where it was when the packet left.
            lead = max(0.0, peer["v"]) * self.lim.horizon * 0.5
            obstacles.append(Obstacle(peer["x"] + lead * math.cos(peer["yaw"]),
                                      peer["y"] + lead * math.sin(peer["yaw"]),
                                      radius=self.lim.radius, source="peer",
                                      margin=PEER_MARGIN))
        return obstacles

    def _maybe_replan(self, goal, penalties, now):
        stale = now - self._last_plan > REPLAN_INTERVAL_S
        changed = self._plan_goal != goal
        if not (stale or changed or not self.tracker.active):
            return
        path = self.planner.plan((self.x, self.y), goal, penalties)
        if not path and penalties:
            # Every route is contested. Drop the soft penalties and take the
            # plain shortest path; the throttle and the local planner still
            # keep the robot out of trouble, and refusing to move is worse.
            hard = {c: p for c, p in penalties.items() if p >= 1e5}
            path = self.planner.plan((self.x, self.y), goal, hard)
        if path:
            self.tracker.set_path(self.planner.simplify(path))
            self._plan_goal = goal
            self._last_plan = now
            self.replans += 1

    def _publish_claim(self, now):
        cells = self.tracker.cells_ahead(self.grid, self.x, self.y, CLAIM_HORIZON_M)
        speed = max(0.25, abs(self.v))
        self.state.claim = cells
        self.state.eta = [i * self.grid.resolution / speed for i in range(len(cells))]

    def _drive_retreat(self, target, now):
        """Reverse towards a passing bay without turning round to do it."""
        dx, dy = target[0] - self.x, target[1] - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.25:
            return self._output(0.0, 0.0, now, "waiting in passing bay")
        bearing = wrap(math.atan2(dy, dx) - self.yaw)
        # The bay is behind us, so the steering error is measured against the
        # robot's *rear*: driving backwards, a positive error steers the other way.
        rear_err = wrap(bearing - math.pi)
        v = max(self.lim.v_min, -0.25)
        w = max(-0.6, min(0.6, -1.2 * rear_err))
        return self._output(v, w, now, "retreating")

    def _track_stall(self, v_cmd, now, dt, decision):
        moving = abs(self.v) > STALL_SPEED or abs(v_cmd) > STALL_SPEED
        if moving:
            self._stalled_since = None
        elif self._stalled_since is None:
            self._stalled_since = now

        if self.local.last_reason == "blocked" and decision.speed_cap != 0.0:
            self.time_blocked += dt
            if self._block_since is None:
                self._block_since = now
            elif now - self._block_since > BLOCK_CONFIRM_S:
                self._report_block(now)
                self._block_since = None
        else:
            self._block_since = None

    def _report_block(self, now):
        """Mark the floor just ahead as impassable and tell everyone.

        Only cells the *map* thinks are free are worth reporting: a robot stuck
        against a known rack has a localisation problem, not a blocked aisle,
        and gossiping that would poison every peer's map.
        """
        ahead = []
        for d in (0.5, 0.9, 1.3):
            cx = self.x + d * math.cos(self.yaw)
            cy = self.y + d * math.sin(self.yaw)
            cell = self.grid.world_to_grid(cx, cy)
            if self.grid.at(*cell) == 0:
                ahead.append(cell)
        for cell in ahead:
            self._blocked[cell] = now + BLOCK_TTL_S
        if ahead:
            self.state.blocked = ahead
            self.state.lamport = self.peers.lamport = self.peers.lamport + 1
            self._plan_goal = None          # force a reroute around it
            self._decision_reason = f"aisle blocked at {ahead[0]}, rerouting"

    def _expire_blocks(self, now):
        for cell, expiry in list(self._blocked.items()):
            if now > expiry:
                del self._blocked[cell]
        self.state.blocked = [c for c in self.state.blocked if c in self._blocked]

    def _update_battery(self, dt):
        if self.state.mode == CHARGING:
            self.state.battery = min(100.0, self.state.battery + CHARGE_RATE_PER_S * dt)
            return
        self.state.battery = max(
            0.0, self.state.battery
            - abs(self.v) * dt * BATTERY_PER_METRE
            - BATTERY_IDLE_PER_S * dt)

    # -- task state machine ------------------------------------------------

    def _run_allocation(self, now):
        # Peers' heartbeats are the only source of truth about what peers are
        # doing, so fold them in before deciding anything.
        for peer in self.peers.alive():
            self.pool.observe_peer(peer["id"], peer.get("task"), now)
        if self.pool.resolve_conflicts(now):
            # A lower-numbered robot claimed the same task; stand down.
            self._abandon_task("claim lost to a lower id")

        won = self.pool.settle(now)
        if won is not None and self.pool.mine is None and self.state.mode == IDLE:
            self.pool.take(won)

        if self.state.mode in (TO_CHARGER, CHARGING):
            self.pool.release()
            self.state.task = None
            return

        if self.state.battery < BATTERY_LOW and self.charger:
            self.pool.release()
            self.state.mode = TO_CHARGER
            self.state.task = None
            self._plan_goal = None
            return

        mine = self.pool.assigned_to(self.id, now)
        if mine is None:
            if self.state.mode in (TO_PICKUP, TO_DROPOFF):
                self._abandon_task("task released")
            return

        self.state.task = mine.id
        if self.state.mode == IDLE:
            self.state.mode = TO_PICKUP
            self._plan_goal = None
        return

    def _abandon_task(self, reason):
        self.state.mode = IDLE
        self.state.task = None
        self._plan_goal = None
        self.tracker.clear()
        self._decision_reason = reason

    def _bid_if_idle(self, now):
        """Bid on the best open task. Idle robots only.

        Bidding is throttled and the candidate list is capped, for the same
        reason a real fleet does both: costing a task means planning a route to
        it, and re-planning routes to every open pallet at the 10 Hz control
        rate swamps the CPU the local planner needs. Once a second, over the
        few nearest candidates, converges just as fast in practice -- the
        auction settles in under a second either way -- and leaves the control
        loop its budget.
        """
        if self.state.mode != IDLE or self.pool.mine is not None:
            return
        if now - self._last_bid < BID_INTERVAL_S:
            return
        open_tasks = self.pool.open_tasks(now)
        if not open_tasks:
            return
        self._last_bid = now
        open_tasks.sort(key=lambda t: math.hypot(t.pickup[0] - self.x,
                                                 t.pickup[1] - self.y))
        best, best_cost = None, UNREACHABLE
        for task in open_tasks[:BID_CANDIDATES]:
            raw = route_cost(self.planner, (self.x, self.y), task)
            cost = self.pool.effective_cost(task, raw, now)
            if cost < best_cost:
                best, best_cost = task, cost
        if best is not None and best_cost < UNREACHABLE:
            # Bid true cost. There is nothing to game: every robot applies the
            # same rule to the same numbers, so shading a bid only wins work
            # this robot is bad at.
            self.pool.place_bid(best.id, self.id, best_cost, now)

    def _goal_for_mode(self, now):
        mode = self.state.mode
        if mode == SERVICING:
            return None
        if mode == TO_CHARGER:
            return self.charger
        if mode == CHARGING:
            return None
        task = self.pool.assigned_to(self.id, now)
        if task is None:
            return None
        if mode == TO_PICKUP:
            return task.pickup
        if mode == TO_DROPOFF:
            return task.dropoff
        return None

    def _on_goal_reached(self, now):
        mode = self.state.mode
        self.tracker.clear()
        self._plan_goal = None
        if mode == TO_CHARGER:
            self.state.mode = CHARGING
            return
        if mode == CHARGING:
            return
        task = self.pool.assigned_to(self.id, now)
        if task is None:
            self.state.mode = IDLE
            return
        if mode == TO_PICKUP:
            self.state.mode = TO_DROPOFF
            self._service_until = now + SERVICE_TIME_S
        elif mode == TO_DROPOFF:
            self.pool.complete(task.id)
            self.tasks_done += 1
            self.state.mode = IDLE
            self.state.task = None

    # -- output ------------------------------------------------------------

    def _output(self, v, w, now, reason):
        if self.state.mode == CHARGING and self.state.battery >= BATTERY_FULL:
            self.state.mode = IDLE

        self.state.seq += 1
        self.state.lamport = self.peers.lamport = self.peers.lamport + 1
        self.state.stamp = now
        self.state.x, self.state.y, self.state.yaw = self.x, self.y, self.yaw
        self.state.v, self.state.w = v, w

        telemetry = {
            "id": self.id, "mode": self.state.mode, "reason": reason,
            "battery": round(self.state.battery, 1),
            "x": round(self.x, 3), "y": round(self.y, 3), "yaw": round(self.yaw, 3),
            "v": round(v, 3), "w": round(w, 3),
            "task": self.state.task,
            "goal": self.state.goal,
            "path": [[round(px, 2), round(py, 2)] for px, py in self.tracker.path],
            "peers": sorted(self.peers.peers),
            "waiting_for": self.state.waiting_for,
            "priority": round(self.state.priority, 2),
            "blocked": [list(c) for c in self._blocked],
            "distance": round(self.distance_travelled, 2),
            "tasks_done": self.tasks_done,
            "replans": self.replans,
        }
        return AgentOutput(v, w, self.state.to_json(),
                           self.pool.announce(now), telemetry)
