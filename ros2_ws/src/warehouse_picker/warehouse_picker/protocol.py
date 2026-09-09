"""
The decentralised layer: what robots tell each other, and what they do about it.

There is no coordinator process. Every robot broadcasts the same message to
every other robot on a shared channel, keeps a table of what it has heard, and
decides for itself. Losing a peer costs that peer's contribution to everyone
else's decisions and nothing more; losing *any* robot, including the one you
might be tempted to call the leader, leaves the rest running.

The single design decision worth arguing about is what "yielding" means. The
obvious implementation -- stop dead when a peer is close -- is what the first
version of this agent did, and it is both slow and deadlock-prone: two robots
that stop for each other never move again. Here a robot yields in three
escalating ways, and only ever reaches the third:

1. **Reroute.** A peer's claimed cells become *expensive* to plan through, not
   impassable. If another aisle is within the detour bound, the yielding robot
   simply goes around and neither robot slows down at all.
2. **Throttle.** If there is no reasonable detour, the yielding robot caps its
   speed to hold a time gap behind the peer. It keeps rolling, so it does not
   pay the deceleration-and-reacceleration cost twice, and it is already moving
   when the corridor clears.
3. **Retreat.** Only for a genuine head-on in a single-file aisle, where one of
   the two has to give up ground: the lower-ranked robot reverses to the
   nearest bay wide enough to pass in.

Rank is `(priority, -lamport, id)`, compared as a tuple, and priority *ages
upward while a robot is blocked*, so a robot that keeps losing eventually wins
and starvation is bounded rather than merely unlikely.

No ROS imports.
"""

import json
import math

PROTOCOL_VERSION = 2

# How far ahead a robot publishes its intent. Long enough that a peer can react
# before the corridor is contested, short enough that the claim is still true
# by the time it arrives.
CLAIM_HORIZON_M = 6.0

# A peer that has said nothing for this long is presumed unreachable and its
# claims stop constraining anyone. Short enough that a robot which loses radio
# mid-aisle does not hold the corridor; long enough to ride out the packet loss
# a warehouse full of metal produces.
HEARTBEAT_TTL = 2.0

# A robot's priority climbs by this much per second spent yielding, and is what
# guarantees that no robot is starved indefinitely at a busy junction.
AGING_RATE = 0.08
BASE_PRIORITY = 1.0

# Contention geometry
HEAD_ON_DOT = -0.55          # cos of the heading difference that counts as opposed
CLAIM_PENALTY = 14.0         # A* cost added to a superior peer's claimed cell
COURTESY_PENALTY = 1.5       # ... and to an inferior peer's, to spread traffic

# Two robots that meet nose-to-nose on open floor will each refuse to close
# the last metre, because each is inside the other's clearance envelope, and
# they creep to a mutual standstill that neither the head-on rule nor the
# throttle sees -- there is no corridor to contest and no claim overlap, so as
# far as the coordination layer is concerned nothing is wrong. Robots pass on
# an agreed side instead. Both apply the same offset, so the symmetry is broken
# by the convention rather than by a negotiation, and neither has to stop.
PASS_SIDE = -1.0             # -1 keeps right (clockwise); +1 keeps left
PASS_OFFSET_M = 0.75
ONCOMING_RADIUS = 4.5

MIN_TIME_GAP = 2.0           # seconds of following distance when throttling
STALL_SPEED = 0.05           # below this a robot counts as not moving
DEADLOCK_STALL_S = 3.0       # stalled this long -> look for a wait cycle
RETREAT_TIME_S = 4.0         # how long a retreat manoeuvre is committed to


def rank_key(state):
    """Ordering over robots. Larger wins. Total, and identical on every node."""
    return (state.get("priority", BASE_PRIORITY),
            -state.get("lamport", 0),
            state.get("id", ""))


class FleetState:
    """One robot's broadcast state -- the entire contents of the mesh message."""

    __slots__ = ("id", "seq", "lamport", "stamp", "x", "y", "yaw", "v", "w",
                 "battery", "mode", "goal", "task", "claim", "eta", "priority",
                 "waiting_for", "blocked", "retreating")

    def __init__(self, robot_id):
        self.id = robot_id
        self.seq = 0
        self.lamport = 0
        self.stamp = 0.0
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.battery = 100.0
        self.mode = "IDLE"
        self.goal = None            # (x, y) or None
        self.task = None            # task id or None
        self.claim = []             # [(col, row), ...] cells reserved ahead
        self.eta = []               # seconds until arrival at each claim cell
        self.priority = BASE_PRIORITY
        self.waiting_for = None     # peer id this robot is held up behind
        self.blocked = []           # aisle cells reported impassable
        self.retreating = False

    def to_json(self):
        return json.dumps({
            "v": PROTOCOL_VERSION, "id": self.id, "seq": self.seq,
            "lamport": self.lamport, "stamp": round(self.stamp, 3),
            "x": round(self.x, 3), "y": round(self.y, 3), "yaw": round(self.yaw, 4),
            "vel": round(self.v, 3), "w": round(self.w, 3),
            "battery": round(self.battery, 2), "mode": self.mode,
            "goal": None if self.goal is None else [round(self.goal[0], 2),
                                                   round(self.goal[1], 2)],
            "task": self.task,
            "claim": [[c, r] for (c, r) in self.claim],
            "eta": [round(e, 2) for e in self.eta],
            "priority": round(self.priority, 3),
            "waiting_for": self.waiting_for,
            "blocked": [[c, r] for (c, r) in self.blocked],
            "retreating": self.retreating,
        }, separators=(",", ":"))

    @staticmethod
    def from_json(text):
        """Parse a peer message. Returns a plain dict, or None if unusable.

        Anything malformed or from an incompatible protocol version is dropped
        silently: one robot running old firmware must not be able to wedge the
        rest of the fleet.
        """
        try:
            d = json.loads(text)
        except (ValueError, TypeError):
            return None
        if not isinstance(d, dict) or d.get("v") != PROTOCOL_VERSION:
            return None
        if not isinstance(d.get("id"), str):
            return None
        try:
            return {
                "id": d["id"], "seq": int(d.get("seq", 0)),
                "lamport": int(d.get("lamport", 0)),
                "stamp": float(d.get("stamp", 0.0)),
                "x": float(d["x"]), "y": float(d["y"]), "yaw": float(d["yaw"]),
                "v": float(d.get("vel", 0.0)), "w": float(d.get("w", 0.0)),
                "battery": float(d.get("battery", 100.0)),
                "mode": str(d.get("mode", "IDLE")),
                "goal": tuple(d["goal"]) if d.get("goal") else None,
                "task": d.get("task"),
                "claim": [(int(c), int(r)) for c, r in d.get("claim", [])],
                "eta": [float(e) for e in d.get("eta", [])],
                "priority": float(d.get("priority", BASE_PRIORITY)),
                "waiting_for": d.get("waiting_for"),
                "blocked": [(int(c), int(r)) for c, r in d.get("blocked", [])],
                "retreating": bool(d.get("retreating", False)),
            }
        except (KeyError, TypeError, ValueError):
            return None


class PeerTable:
    """Peers heard from recently, and the Lamport clock that orders them."""

    def __init__(self, robot_id, ttl=HEARTBEAT_TTL):
        self.robot_id = robot_id
        self.ttl = ttl
        self.peers = {}          # id -> state dict
        self.last_seen = {}      # id -> local receive time
        self.lamport = 0
        self.lost = set()        # peers whose heartbeat expired

    def observe(self, msg, now):
        if msg is None or msg["id"] == self.robot_id:
            return False
        prev = self.peers.get(msg["id"])
        if prev is not None and msg["seq"] < prev["seq"]:
            return False                     # out-of-order datagram
        self.lamport = max(self.lamport, msg["lamport"]) + 1
        self.peers[msg["id"]] = msg
        self.last_seen[msg["id"]] = now
        self.lost.discard(msg["id"])
        return True

    def tick(self, now):
        """Expire silent peers. Returns the set newly declared lost."""
        newly = set()
        for pid, seen in list(self.last_seen.items()):
            if now - seen > self.ttl:
                del self.last_seen[pid]
                self.peers.pop(pid, None)
                if pid not in self.lost:
                    newly.add(pid)
                    self.lost.add(pid)
        return newly

    def alive(self):
        return list(self.peers.values())


class Decision:
    """What the coordination layer wants the motion layer to do this tick."""

    __slots__ = ("speed_cap", "penalties", "waiting_for", "retreat_to",
                 "reason", "yielded", "lateral_bias")

    def __init__(self):
        self.speed_cap = None       # None = no limit
        self.penalties = {}         # cell -> extra A* cost
        self.waiting_for = None
        self.retreat_to = None      # (x, y) to reverse towards, or None
        self.reason = "clear"
        self.yielded = False
        self.lateral_bias = 0.0     # metres to shift the aim point sideways


# The control arm. `stop_and_wait` is the textbook uncoordinated scheme, and the
# one the previous version of this agent implemented: broadcast position, and if
# a robot with a higher fixed priority is within a radius, stop until it leaves.
# It is here so the cooperative policy is measured against something real rather
# than against an estimate.
#
# It is given the deadlock escape such systems have in practice -- after a
# timeout, back off for a randomised interval and proceed regardless of
# priority. Without it the scheme simply gridlocks and any comparison against it
# is a straw man; with it, it recovers, and what the comparison then measures is
# the cost of recovering *after* the fact instead of not deadlocking in the
# first place.
STOP_AND_WAIT_RADIUS = 2.2
BASELINE_STALL_TIMEOUT = 10.0
BASELINE_BACKOFF_S = 4.0


class Coordinator:
    """Turns the peer table into a Decision, from one robot's point of view."""

    def __init__(self, grid, robot_radius=0.40, safety=0.15, policy="cooperative"):
        self.grid = grid
        self.policy = policy
        self.pass_width = 2.0 * (robot_radius + safety) + 0.25
        self._retreat_until = 0.0
        self._retreat_target = None
        self._backoff_until = 0.0

    # -- helpers -----------------------------------------------------------

    def is_single_file(self, x, y):
        """True where two robots cannot pass each other."""
        return self.grid.free_width(x, y) < self.pass_width

    def nearest_passing_bay(self, x, y, yaw, max_back=6.0):
        """Search backwards along the robot's own heading for somewhere to wait.

        Reversing is the only direction that is guaranteed to be free -- the
        robot has just driven through it.
        """
        best = None
        step = self.grid.resolution
        d = step
        while d <= max_back:
            bx = x - d * math.cos(yaw)
            by = y - d * math.sin(yaw)
            if self.grid.at(*self.grid.world_to_grid(bx, by)):
                break
            if not self.is_single_file(bx, by):
                best = (bx, by)
                break
            d += step
        return best

    # -- the decision ------------------------------------------------------

    def decide(self, me, peers, now, stalled_for=0.0):
        """`me` is this robot's FleetState; `peers` the live peer dicts."""
        decision = Decision()
        if not peers:
            self._retreat_target = None
            return decision

        if self.policy == "stop_and_wait":
            return self._stop_and_wait(me, peers, decision, now, stalled_for)

        my_key = rank_key({"priority": me.priority, "lamport": me.lamport, "id": me.id})
        my_claim = {cell: eta for cell, eta in zip(me.claim, me.eta)}

        # --- 1. build the planning penalty layer --------------------------
        for peer in peers:
            superior = rank_key(peer) > my_key
            weight = CLAIM_PENALTY if superior else COURTESY_PENALTY
            for i, cell in enumerate(peer["claim"]):
                eta = peer["eta"][i] if i < len(peer["eta"]) else float(i) * 0.5
                # A cell someone reaches in 10 s is barely a constraint; one
                # they are entering now is a hard one.
                decay = 1.0 / (1.0 + 0.35 * eta)
                decision.penalties[cell] = (decision.penalties.get(cell, 0.0)
                                            + weight * decay)

        # --- 2. who, if anyone, am I actually in conflict with? -----------
        conflict = None
        conflict_eta = math.inf
        for peer in peers:
            if rank_key(peer) <= my_key:
                continue                       # I outrank them; they yield
            for i, cell in enumerate(peer["claim"]):
                if cell not in my_claim:
                    continue
                peer_eta = peer["eta"][i] if i < len(peer["eta"]) else i * 0.5
                mine_eta = my_claim[cell]
                # Only a *space-time* overlap is a conflict. Two robots using
                # the same cell twenty seconds apart is normal traffic.
                if abs(peer_eta - mine_eta) < MIN_TIME_GAP and mine_eta < conflict_eta:
                    conflict, conflict_eta = peer, mine_eta
                    break

        # --- 3. oncoming traffic ------------------------------------------
        # Anywhere there is room to pass, both robots simply move over. Only
        # where there is not does anyone have to give way.
        head_on = None
        for peer in peers:
            dx, dy = peer["x"] - me.x, peer["y"] - me.y
            dist = math.hypot(dx, dy)
            if dist > ONCOMING_RADIUS or dist < 1e-6:
                continue
            facing = math.cos(me.yaw) * math.cos(peer["yaw"]) + \
                math.sin(me.yaw) * math.sin(peer["yaw"])
            towards = (math.cos(me.yaw) * dx + math.sin(me.yaw) * dy) / dist
            if facing >= HEAD_ON_DOT or towards <= 0.4:
                continue
            if not self.is_single_file(me.x, me.y):
                # Wide enough for two: shift over and keep going. Closer
                # traffic gets a firmer nudge.
                decision.lateral_bias = PASS_SIDE * PASS_OFFSET_M * min(
                    1.0, (ONCOMING_RADIUS - dist) / (ONCOMING_RADIUS - 1.0))
                decision.reason = f"passing {peer['id']}"
                continue
            if rank_key(peer) > my_key:
                head_on = peer
                break

        now_retreating = self._retreat_target is not None and now < self._retreat_until
        if head_on or now_retreating:
            if not now_retreating:
                bay = self.nearest_passing_bay(me.x, me.y, me.yaw)
                if bay:
                    self._retreat_target = bay
                    self._retreat_until = now + RETREAT_TIME_S
                else:
                    # Nowhere to go: hold position and let the other robot come
                    # through. This is the one case where stopping is correct.
                    decision.speed_cap = 0.0
                    decision.waiting_for = head_on["id"] if head_on else None
                    decision.reason = "head-on, no bay"
                    decision.yielded = True
                    return decision
            decision.retreat_to = self._retreat_target
            decision.waiting_for = head_on["id"] if head_on else decision.waiting_for
            decision.reason = "retreating to passing bay"
            decision.yielded = True
            return decision
        self._retreat_target = None

        # --- 4. throttle behind a superior peer ---------------------------
        if conflict is not None:
            gap = math.hypot(conflict["x"] - me.x, conflict["y"] - me.y)
            # Hold a time gap rather than a distance gap: the faster the robot
            # ahead, the more room it needs, and the closer we may safely sit
            # when it is crawling.
            target = max(0.0, (gap - self.pass_width) / MIN_TIME_GAP)
            decision.speed_cap = max(0.0, min(target, conflict["v"] + 0.15))
            decision.waiting_for = conflict["id"]
            decision.reason = f"throttled behind {conflict['id']}"
            decision.yielded = True

        # --- 5. deadlock: a cycle in the wait-for graph --------------------
        if stalled_for > DEADLOCK_STALL_S:
            cycle = find_wait_cycle(me.id, decision.waiting_for, peers)
            if cycle:
                # Everyone in the cycle sees the same cycle and computes the
                # same loser, so exactly one robot backs off -- no negotiation
                # round trip, and no chance of all of them backing off at once.
                loser = min(cycle, key=lambda pid: _cycle_key(pid, me, peers))
                if loser == me.id:
                    bay = self.nearest_passing_bay(me.x, me.y, me.yaw, max_back=9.0)
                    if bay:
                        self._retreat_target = bay
                        self._retreat_until = now + RETREAT_TIME_S
                        decision.retreat_to = bay
                        decision.speed_cap = None
                    decision.reason = f"deadlock {'->'.join(cycle)}: I back off"
                else:
                    decision.reason = f"deadlock {'->'.join(cycle)}: {loser} backs off"
                decision.yielded = True

        return decision

    def _stop_and_wait(self, me, peers, decision, now, stalled_for):
        """Freeze if a statically higher-priority robot is close. No planning
        penalties, no throttling, no passing side, no retreat -- the naive
        scheme, plus the stall timeout and randomised backoff that keeps it from
        simply gridlocking."""
        if now < self._backoff_until:
            decision.reason = "backing off after a stall"
            return decision
        if stalled_for > BASELINE_STALL_TIMEOUT:
            # Deterministic per-robot jitter, so a run is reproducible and two
            # robots in the same jam do not pick the same backoff.
            jitter = ((hash(me.id) % 97) / 97.0) * BASELINE_BACKOFF_S
            self._backoff_until = now + BASELINE_BACKOFF_S + jitter
            decision.reason = "stall timeout, backing off"
            return decision
        for peer in peers:
            dist = math.hypot(peer["x"] - me.x, peer["y"] - me.y)
            if dist < STOP_AND_WAIT_RADIUS and peer["id"] > me.id:
                decision.speed_cap = 0.0
                decision.waiting_for = peer["id"]
                decision.reason = f"stopped for {peer['id']}"
                decision.yielded = True
                return decision
        return decision


def _cycle_key(pid, me, peers):
    if pid == me.id:
        return rank_key({"priority": me.priority, "lamport": me.lamport, "id": me.id})
    for p in peers:
        if p["id"] == pid:
            return rank_key(p)
    return (math.inf, 0, pid)


def find_wait_cycle(my_id, my_waiting_for, peers, max_depth=8):
    """Walk the wait-for chain from this robot and report a cycle through it.

    Bounded, and only ever walks *forward* from self, so every robot in a cycle
    independently discovers the same member set without any of them having to
    hold the whole fleet's graph.
    """
    if not my_waiting_for:
        return None
    waits = {p["id"]: p.get("waiting_for") for p in peers}
    waits[my_id] = my_waiting_for
    chain = [my_id]
    seen = {my_id}
    node = my_waiting_for
    for _ in range(max_depth):
        if node is None:
            return None
        if node in seen:
            start = chain.index(node)
            cycle = chain[start:]
            return cycle if my_id in cycle else None
        chain.append(node)
        seen.add(node)
        node = waits.get(node)
    return None
