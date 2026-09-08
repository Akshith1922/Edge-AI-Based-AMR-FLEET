"""
Algorithm 1, Component 3 — Chokepoint / In-Transit Lock.

Every single-file run of cells in the map (found automatically by
``Warehouse._compute_corridors``) is a lockable resource. Two robots cannot
pass inside one, so contention there has to be resolved *before* they meet,
not after.

The lock is deliberately **not** a hard gate on the planner. Space-time
reservations already make a head-on entry infeasible — a robot simply cannot
commit to a cell another robot has claimed at that tick. Forbidding corridor
cells outright on top of that only removes the planner's ability to express
"wait a moment and then go through", which is usually the cheapest option and
the one the reference's ``select_action`` is supposed to pick.

So the lock does the two things reservations alone cannot:

1. **FIFO fairness.** It records who has been waiting on a chokepoint and for
   how long, and turns that into a priority bonus. Since the fleet plans in
   priority order, the robot that has waited longest plans first and therefore
   wins the aisle — a real first-come-first-served queue, expressed through
   the ranking rule Algorithms 1, 3 and 6 already share, instead of a second
   competing mechanism.

2. **Drive-through commitment.** Once a robot is inside a corridor with
   somebody behind it, the entry mouth is closed to it: it must continue out
   the far end. Without that, a robot that stops at a pick face and then wants
   to reverse traps itself against its own follower — the one deadlock a
   single-file aisle cannot shuffle its way out of.
"""


class _CorridorState:
    __slots__ = ("direction", "inside", "interest", "served", "flips")

    def __init__(self):
        self.direction = 0       # 0 = free, +1 / -1 = current flow direction
        self.inside = set()      # robot ids physically in the corridor
        self.interest = {}       # robot_id -> [since_tick, direction, priority]
        self.served = 0
        self.flips = 0


class CorridorLocks:
    #: priority added per tick of queueing, so a starved robot eventually wins
    STARVATION_GAIN = 0.12
    #: interest lapses if it is not renewed for this many ticks (a lease)
    INTEREST_TTL = 3

    def __init__(self, warehouse):
        self.wh = warehouse
        self.corridors = {c.id: c for c in warehouse.corridors}
        self.state = {cid: _CorridorState() for cid in self.corridors}
        self._renewed = {}

    # ------------------------------------------------------------- interest
    def request(self, robot, corridor, direction, tick, priority):
        """Register (or renew) interest in a chokepoint. Returns True when the
        aisle is already ours — free, or flowing our way with nobody opposed."""
        st = self.state[corridor.id]
        entry = st.interest.get(robot.id)
        if entry is None:
            st.interest[robot.id] = [tick, direction, priority]
        else:
            entry[1] = direction
            entry[2] = priority
        self._renewed.setdefault(corridor.id, set()).add(robot.id)

        if robot.id in st.inside:
            return True
        opposed = any(st.interest[rid][1] != direction
                      for rid in st.inside if rid in st.interest)
        return st.direction in (0, direction) and not opposed

    def starvation_bonus(self, robot_id, tick):
        """How much priority this robot has earned by waiting at chokepoints."""
        best = 0.0
        for st in self.state.values():
            entry = st.interest.get(robot_id)
            if entry is not None and robot_id not in st.inside:
                best = max(best, (tick - entry[0]) * self.STARVATION_GAIN)
        return min(best, 6.0)

    def forbidden_for(self, robot_id):
        """Drive-through commitment: the entry mouth of any corridor we are
        inside of while another robot is in there with us."""
        out = set()
        for cid, st in self.state.items():
            if robot_id in st.inside and st.direction and len(st.inside) > 1:
                corridor = self.corridors[cid]
                out.add(corridor.ends[0] if st.direction > 0 else corridor.ends[1])
        return out

    def withdraw(self, robot_id):
        for st in self.state.values():
            st.interest.pop(robot_id, None)

    # ---------------------------------------------------------------- tick
    def begin_tick(self):
        self._renewed = {}

    def update(self, robots_by_id, tick):
        """Refresh occupancy and flow direction, and expire lapsed interest."""
        for cid, st in self.state.items():
            cells = set(self.corridors[cid].cells)
            was_inside = st.inside
            st.inside = {r.id for r in robots_by_id.values()
                         if r.is_alive() and r.pos() in cells}
            st.served += len(was_inside - st.inside)

            renewed = self._renewed.get(cid, set())
            for rid, entry in list(st.interest.items()):
                robot = robots_by_id.get(rid)
                if robot is None or not robot.is_alive():
                    del st.interest[rid]
                elif rid not in renewed and rid not in st.inside:
                    if tick - entry[0] > self.INTEREST_TTL:
                        del st.interest[rid]      # lease lapsed: they rerouted

            if st.inside:
                directions = {st.interest[rid][1] for rid in st.inside
                              if rid in st.interest}
                new_dir = directions.pop() if len(directions) == 1 else st.direction
                if new_dir and new_dir != st.direction:
                    st.flips += 1
                    st.direction = new_dir
            else:
                st.direction = 0

    # -------------------------------------------------------------- reports
    def snapshot(self):
        return [{
            "id": cid,
            "cells": [list(c) for c in self.corridors[cid].cells],
            "direction": st.direction,
            "inside": sorted(st.inside),
            "queue": sorted(rid for rid in st.interest if rid not in st.inside),
            "served": st.served,
            "flips": st.flips,
        } for cid, st in self.state.items()]
