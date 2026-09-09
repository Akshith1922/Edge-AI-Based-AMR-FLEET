"""
Decentralised task allocation: a sealed-bid auction with no auctioneer.

Tasks are gossiped on a shared channel. Every robot holds the same pool, bids
its own true cost for the task it wants most, and then -- this is the part that
removes the need for a central allocator -- *every* robot independently applies
the same tie-break rule to the same set of bids and reaches the same answer.
The winner discovers it has won at the same instant everyone else does.

Two properties make that safe:

* The pool is an OR-set. Adds and completions commute, so robots that saw
  events in different orders still converge, and a task cannot be lost because
  two robots learned about it from different neighbours.
* A claim is a *lease*. If the holder goes quiet for longer than the lease, the
  task returns to the pool automatically -- which is also how a robot that has
  driven into a dead radio spot releases its work without anyone noticing it
  failed.

Starvation is handled by ageing: a task's effective cost falls the longer it
sits unclaimed, so an awkwardly-placed pick eventually outbids convenient ones.

No ROS imports.
"""

import json
import math

LEASE_S = 25.0          # a claim must be refreshed this often
BID_WINDOW_S = 0.6      # collect bids for this long before settling
AGING_RATE = 0.35       # cost units subtracted per second a task waits
UNREACHABLE = float("inf")


class Task:
    __slots__ = ("id", "pickup", "dropoff", "priority", "created", "label")

    def __init__(self, tid, pickup, dropoff, priority=1.0, created=0.0, label=""):
        self.id = tid
        self.pickup = tuple(pickup)
        self.dropoff = tuple(dropoff)
        self.priority = priority
        self.created = created
        self.label = label or tid

    def as_dict(self):
        return {"id": self.id, "pickup": list(self.pickup),
                "dropoff": list(self.dropoff), "priority": self.priority,
                "created": self.created, "label": self.label}

    @staticmethod
    def from_dict(d):
        return Task(d["id"], d["pickup"], d["dropoff"],
                    d.get("priority", 1.0), d.get("created", 0.0),
                    d.get("label", ""))


class TaskPool:
    """One robot's replica of the fleet's work list.

    Claims are *not* shared mutable state. Early versions of this kept a
    `task -> holder` map that every robot merged from every other robot, and it
    diverged exactly as you would expect: two replicas award the same task to
    different robots, each overwrites the other on the next gossip round, and a
    robot halfway through a delivery discovers it no longer owns the job and
    restarts from the pickup. Tasks ping-ponged and never completed.

    So a robot is the sole authority on one thing only -- what *it* is doing --
    and it publishes that in its own heartbeat. Everything else is derived:
    a task is held if some robot says it is holding it. Two robots that claim
    the same task (possible, briefly, when they settled an auction on different
    bid sets) resolve it with a total order on robot id: the lower id keeps it,
    the higher one drops it the moment it sees the conflict. There is no state
    to converge, so there is nothing to diverge.
    """

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.tasks = {}          # id -> Task
        self.done = set()        # OR-set tombstones
        self.mine = None         # the task id this robot is executing
        self.peer_holds = {}     # robot id -> (task id, last heard)
        self.bids = {}           # task id -> {robot id -> (cost, stamp)}

    # -- pool contents -----------------------------------------------------

    def add(self, task):
        if task.id in self.done or task.id in self.tasks:
            return False
        self.tasks[task.id] = task
        return True

    def complete(self, task_id):
        self.done.add(task_id)
        self.tasks.pop(task_id, None)
        self.bids.pop(task_id, None)
        if self.mine == task_id:
            self.mine = None
        for rid, (tid, seen) in list(self.peer_holds.items()):
            if tid == task_id:
                self.peer_holds.pop(rid, None)

    # -- who holds what ----------------------------------------------------

    def observe_peer(self, robot_id, task_id, now):
        """Fold in what a peer's heartbeat says it is working on."""
        if robot_id == self.robot_id:
            return
        if task_id is None:
            self.peer_holds.pop(robot_id, None)
        else:
            self.peer_holds[robot_id] = (task_id, now)

    def forget_peer(self, robot_id):
        """A peer went silent: whatever it held is available again."""
        self.peer_holds.pop(robot_id, None)

    def _live_holds(self, now):
        return {tid: rid for rid, (tid, seen) in self.peer_holds.items()
                if now - seen <= LEASE_S}

    def holder(self, task_id, now):
        if self.mine == task_id:
            return self.robot_id
        return self._live_holds(now).get(task_id)

    def open_tasks(self, now):
        held = set(self._live_holds(now))
        if self.mine:
            held.add(self.mine)
        return [t for tid, t in self.tasks.items() if tid not in held]

    def assigned_to(self, robot_id, now):
        """The Task this robot is executing, or None. Only meaningful for self."""
        if robot_id != self.robot_id:
            tid = next((t for r, (t, _s) in self.peer_holds.items() if r == robot_id),
                       None)
            return self.tasks.get(tid)
        return self.tasks.get(self.mine) if self.mine else None

    def take(self, task_id):
        self.mine = task_id
        self.bids.pop(task_id, None)

    def release(self, task_id=None):
        if task_id is None or self.mine == task_id:
            self.mine = None

    def resolve_conflicts(self, now):
        """Give up a task a lower-numbered robot also claims.

        Every robot applies the same comparison to the same pair of ids, so
        exactly one of the two lets go and it happens without a round trip.
        """
        if self.mine is None:
            return False
        for tid, rid in self._live_holds(now).items():
            if tid == self.mine and rid < self.robot_id:
                self.mine = None
                return True
        return False

    # -- auction -----------------------------------------------------------

    def effective_cost(self, task, raw_cost, now):
        """Raw route cost, discounted by how long the task has been waiting."""
        if raw_cost >= UNREACHABLE:
            return UNREACHABLE
        waited = max(0.0, now - task.created)
        return raw_cost / max(0.2, task.priority) - AGING_RATE * waited

    def place_bid(self, task_id, robot_id, cost, now):
        if cost >= UNREACHABLE:
            return
        self.bids.setdefault(task_id, {})[robot_id] = (cost, now)

    def settle(self, now):
        """Close every auction whose window has elapsed.

        Returns the task id this robot won, or None. Losing bids are simply
        discarded: the winner announces itself in its next heartbeat, and that
        is how the losers find out.
        """
        won = None
        for tid, bids in list(self.bids.items()):
            if tid not in self.tasks or self.holder(tid, now) is not None:
                self.bids.pop(tid, None)
                continue
            fresh = {rid: c for rid, (c, stamp) in bids.items()
                     if now - stamp <= LEASE_S}
            if not fresh:
                self.bids.pop(tid, None)
                continue
            if now - min(stamp for (_c, stamp) in bids.values()) < BID_WINDOW_S:
                continue                     # still collecting
            # Lowest cost wins; ties fall to the lexicographically first id, so
            # every robot in the fleet computes the same winner.
            winner = min(fresh.items(), key=lambda kv: (kv[1], kv[0]))[0]
            self.bids.pop(tid, None)
            if winner == self.robot_id and won is None and self.mine is None:
                won = tid
        return won

    # -- gossip ------------------------------------------------------------

    def announce(self, now):
        """The message this robot broadcasts to synchronise the pool."""
        return json.dumps({
            "from": self.robot_id,
            "tasks": [t.as_dict() for t in self.tasks.values()],
            "done": sorted(self.done),
            "bids": {tid: {rid: round(c, 3) for rid, (c, _s) in b.items()}
                     for tid, b in self.bids.items()},
        }, separators=(",", ":"))

    def merge(self, text, now):
        """Fold a peer's announcement in. Commutative and idempotent."""
        try:
            d = json.loads(text)
        except (ValueError, TypeError):
            return False
        if not isinstance(d, dict) or d.get("from") == self.robot_id:
            return False
        for tid in d.get("done", []):
            self.complete(tid)
        for raw in d.get("tasks", []):
            try:
                self.add(Task.from_dict(raw))
            except (KeyError, TypeError):
                continue
        for tid, bids in (d.get("bids") or {}).items():
            if tid in self.done:
                continue
            slot = self.bids.setdefault(tid, {})
            for rid, cost in bids.items():
                if rid not in slot:
                    slot[rid] = (float(cost), now)
        return True


def route_cost(planner, start, task, penalties=None):
    """Bid value: metres to the pickup plus metres from pickup to dropoff."""
    legs = ((start, task.pickup), (task.pickup, task.dropoff))
    total = 0.0
    for a, b in legs:
        path = planner.plan(a, b, penalties)
        if not path:
            return UNREACHABLE
        total += sum(math.hypot(path[i + 1][0] - path[i][0],
                                path[i + 1][1] - path[i][1])
                     for i in range(len(path) - 1))
    return total
