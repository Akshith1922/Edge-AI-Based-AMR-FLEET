"""
Algorithm 1 — Path Conflict Detection, Classification and Resolution.

This module owns the shared reservation substrate that Algorithms 2, 3, 4 and
5 all reuse unmodified:

* :class:`LamportClock`   — causal ordering for deterministic tie-breaks.
* :class:`Reservation`    — ``cells[] / t_enter / t_exit / priority_score /
  lamport_ts / robot_id / status``, stored as per-cell occupancy windows so a
  conflict only fires where two robots are close in space *and* time.
* :class:`ReservationTable` — the gossiped table. This is a single-process
  simulation, so "gossip" is a shared dict; it is logically equivalent to a
  fully-converged gossip network with zero latency. Everything that makes the
  protocol *correct* — the Lamport clock, the deterministic ranking, the
  transitive cluster walk — is implemented for real.
* :func:`build_cluster` / :func:`rank_cluster` / :func:`classify_geometry` —
  the n-way cluster BFS, the ``(priority DESC, lamport ASC, id ASC)`` ranking
  used identically by Algorithms 1, 3 and 6, and the angle-based conflict
  classifier.
"""

import math
from dataclasses import dataclass, field


class LamportClock:
    """Monotonic logical clock; ``observe`` merges a remote timestamp."""

    __slots__ = ("t",)

    def __init__(self):
        self.t = 0

    def tick(self):
        self.t += 1
        return self.t

    def observe(self, other):
        self.t = max(self.t, other) + 1
        return self.t


@dataclass
class Reservation:
    """A robot's claim on space-time.

    ``steps[i]`` is the cell the robot intends to occupy at tick ``t0 + i``.
    After the last step the robot is stationary, so its final cell stays
    claimed for ``tail`` further ticks — a parked robot is still an obstacle.
    """

    robot_id: int
    t0: int
    steps: list
    priority_score: float
    lamport_ts: int
    tail: int = 0
    status: str = "COMMITTED"
    windows: dict = field(default_factory=dict, repr=False)

    # -------------------------------------------------------------- windows
    def build_windows(self, pad):
        """Per-cell occupancy windows with a safety pad, for congestion and
        cluster overlap. Feasibility uses the exact `steps` instead."""
        win = {}
        for i, cell in enumerate(self.steps):
            t = self.t0 + i
            lo, hi = t - pad, t + pad
            if cell in win:
                win[cell] = (min(win[cell][0], lo), max(win[cell][1], hi))
            else:
                win[cell] = (lo, hi)
        if self.steps and self.tail:
            last = self.steps[-1]
            end = self.t0 + len(self.steps) - 1 + self.tail
            lo, hi = win[last]
            win[last] = (lo, max(hi, end))
        self.windows = win
        return win

    @property
    def cells(self):
        return list(self.windows.keys())

    @property
    def t_enter(self):
        return self.t0

    @property
    def t_exit(self):
        return self.t0 + max(0, len(self.steps) - 1) + self.tail

    def cell_at(self, t):
        """Where this robot plans to be at absolute tick `t` (None if unknown)."""
        i = t - self.t0
        if i < 0:
            return None
        if i < len(self.steps):
            return self.steps[i]
        if self.steps and i <= len(self.steps) - 1 + self.tail:
            return self.steps[-1]
        return None

    def window_for(self, cell):
        return self.windows.get(cell)

    def overlaps(self, other):
        if self.robot_id == other.robot_id:
            return False
        for c in set(self.windows) & set(other.windows):
            a0, a1 = self.windows[c]
            b0, b1 = other.windows[c]
            if not (a1 < b0 or b1 < a0):
                return True
        return False

    def latest_exit_among(self, cells):
        rel = [self.windows[c][1] for c in cells if c in self.windows]
        return max(rel) if rel else self.t_exit

    def direction(self):
        if len(self.steps) < 2:
            return (0, 0)
        (x0, y0), (x1, y1) = self.steps[0], self.steps[-1]
        return (x1 - x0, y1 - y0)


class ReservationTable:
    """Shared, CRDT-flavoured reservation store with occupancy indices."""

    def __init__(self, pad=1):
        self.pad = pad
        self.by_robot = {}
        self._occ = {}       # cell -> {tick: robot_id}       exact, for feasibility
        self._win = {}       # cell -> [(robot_id, lo, hi)]   padded, for congestion

    # ---------------------------------------------------------------- CRUD
    def clear(self):
        self.by_robot.clear()
        self._occ.clear()
        self._win.clear()

    def commit(self, res: Reservation):
        self.invalidate(res.robot_id)
        res.status = "COMMITTED"
        res.build_windows(self.pad)
        self.by_robot[res.robot_id] = res
        for i, cell in enumerate(res.steps):
            self._occ.setdefault(cell, {})[res.t0 + i] = res.robot_id
        if res.steps and res.tail:
            last, base = res.steps[-1], res.t0 + len(res.steps) - 1
            for t in range(base + 1, base + res.tail + 1):
                self._occ.setdefault(last, {}).setdefault(t, res.robot_id)
        for cell, (lo, hi) in res.windows.items():
            self._win.setdefault(cell, []).append((res.robot_id, lo, hi))

    def invalidate(self, robot_id):
        res = self.by_robot.pop(robot_id, None)
        if res is None:
            return
        for cell in set(res.steps):
            slot = self._occ.get(cell)
            if slot:
                for t, rid in list(slot.items()):
                    if rid == robot_id:
                        del slot[t]
                if not slot:
                    del self._occ[cell]
        for cell in list(res.windows):
            entries = [e for e in self._win.get(cell, []) if e[0] != robot_id]
            if entries:
                self._win[cell] = entries
            else:
                self._win.pop(cell, None)

    def get(self, robot_id):
        return self.by_robot.get(robot_id)

    def all(self):
        return list(self.by_robot.values())

    # ------------------------------------------------------------ queries
    def occupant(self, cell, tick):
        return self._occ.get(cell, {}).get(tick)

    def is_busy(self, cell, tick, exclude=None):
        who = self._occ.get(cell, {}).get(tick)
        return who is not None and who != exclude

    def swap_conflict(self, frm, to, tick, exclude=None):
        """True if some other robot moves `to`->`frm` while we move `frm`->`to`."""
        who = self._occ.get(to, {}).get(tick)
        if who is None or who == exclude:
            return False
        return self._occ.get(frm, {}).get(tick + 1) == who

    def congestion_at(self, cell, tick, exclude=None):
        """Algorithm 2's ``congestion_weight``: sum of overlap fractions between
        our one-tick window and every reservation window on this cell."""
        total = 0.0
        span = float(2 * self.pad + 1)
        lo, hi = tick - self.pad, tick + self.pad
        for rid, wlo, whi in self._win.get(cell, ()):
            if rid == exclude:
                continue
            overlap = min(hi, whi) - max(lo, wlo) + 1
            if overlap > 0:
                total += overlap / span
        return total

    def overlapping(self, res: Reservation):
        return [r for r in self.by_robot.values()
                if r.robot_id != res.robot_id and r.overlaps(res)]

    def heat(self, tick, span=12):
        """Reservation density per cell over the next `span` ticks (dashboard)."""
        heat = {}
        for res in self.by_robot.values():
            for i, cell in enumerate(res.steps[:span]):
                if res.t0 + i >= tick:
                    heat[cell] = heat.get(cell, 0) + 1
        return heat


# --------------------------------------------------------------------------
# cluster / ranking / classification — shared by Algorithms 1, 3 and 6
# --------------------------------------------------------------------------
def build_cluster(res, table):
    """Transitive overlap BFS. Handles n-way conflicts, not just pairs."""
    cluster = {res.robot_id: res}
    frontier = [res]
    while frontier:
        cur = frontier.pop()
        for other in table.overlapping(cur):
            if other.robot_id not in cluster:
                cluster[other.robot_id] = other
                frontier.append(other)
    return list(cluster.values())


def rank_cluster(cluster):
    """The one ranking rule used everywhere: priority DESC, lamport ASC, id ASC."""
    return sorted(cluster, key=lambda r: (-r.priority_score, r.lamport_ts, r.robot_id))


def classify_geometry(vec_a, vec_b):
    """Angle between two heading vectors -> conflict geometry."""
    if vec_a == (0, 0) or vec_b == (0, 0):
        return "STATIONARY"
    dot = vec_a[0] * vec_b[0] + vec_a[1] * vec_b[1]
    mag = math.hypot(*vec_a) * math.hypot(*vec_b)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, dot / (mag + 1e-9)))))
    if angle > 150:
        return "HEAD_ON"
    if 60 <= angle <= 120:
        return "CROSSING"
    if angle < 30:
        return "SAME_DIRECTION"
    return "OBLIQUE"
