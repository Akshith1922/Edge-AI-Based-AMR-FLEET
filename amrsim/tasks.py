"""
Algorithm 6 — CRDT Task Pool with Time-Boxed Deterministic Auction.

The pool is an add-wins OR-Set keyed by ``source_id:origin_timestamp``, so the
same task arriving twice from two gossip paths is de-duplicated instead of
being executed twice. Allocation is a real time-boxed auction: bids are
collected for ``BID_WINDOW`` ticks and then ranked with the same
``score DESC, lamport ASC, robot_id ASC`` rule used by Algorithms 1 and 3, so
every robot independently computes the same winner without a central
dispatcher.

Starvation is handled by aging the priority, and no-candidate tasks back off
exponentially rather than re-auctioning every tick.
"""

import itertools
from dataclasses import dataclass, field
from enum import Enum

_ids = itertools.count(1)


class TaskStatus(Enum):
    UNASSIGNED = "unassigned"
    AUCTION = "auction"
    CLAIMED = "claimed"
    NEEDS_RECOVERY = "needs_recovery"
    DONE = "done"


@dataclass
class Task:
    id: int
    kind: str                     # putaway | pick | reprobe
    pickup: tuple
    dropoff: tuple
    required_capability: str
    priority: float
    created_tick: int
    origin_id: str
    zone: str = "aisle"
    status: TaskStatus = TaskStatus.UNASSIGNED
    holder: int = None
    lease_expiry: int = None
    claimed_tick: int = None
    done_tick: int = None
    item_in_transit: bool = False
    auction_opened: int = None
    bids: dict = field(default_factory=dict, repr=False)
    requeue_delay: float = 0.0
    recheck_after: int = 0
    reassignments: int = 0
    recovery_flagged: bool = False
    probe_cell: tuple = None

    def as_dict(self):
        return {"id": self.id, "kind": self.kind, "pickup": list(self.pickup),
                "dropoff": list(self.dropoff), "status": self.status.value,
                "holder": self.holder, "priority": round(self.priority, 2),
                "capability": self.required_capability,
                "in_transit": self.item_in_transit,
                "bids": len(self.bids), "age": self.created_tick,
                "recovery": self.recovery_flagged}


@dataclass
class Bid:
    task_id: int
    robot_id: int
    score: float
    lamport_ts: int


class ORSet:
    """Add-wins set with tombstones — the CRDT underneath the pool."""

    def __init__(self):
        self.adds = {}
        self.removes = set()

    def add(self, key, value):
        if key in self.removes:
            return False
        if key in self.adds:
            return False              # de-duplicated by origin id
        self.adds[key] = value
        return True

    def contains(self, key):
        return key in self.adds and key not in self.removes

    def values(self):
        return [v for k, v in self.adds.items() if k not in self.removes]

    def merge(self, other):
        """Conflict-free merge: adds win unless explicitly removed."""
        self.removes |= other.removes
        for k, v in other.adds.items():
            self.adds.setdefault(k, v)
        return self


class TaskPool:
    def __init__(self, cfg):
        self.cfg = cfg
        self.set = ORSet()
        self.by_id = {}
        self.auction_log = []
        self._stats_cache = (None, None)

    # ---------------------------------------------------------------- CRUD
    def create(self, kind, pickup, dropoff, capability, priority, tick, zone,
               source_id="wms"):
        origin = f"{source_id}:{tick}:{pickup}:{dropoff}:{next(_ids)}"
        task = Task(id=next(_ids), kind=kind, pickup=pickup, dropoff=dropoff,
                    required_capability=capability, priority=priority,
                    created_tick=tick, origin_id=origin, zone=zone)
        return task if self.add(task) else None

    def add(self, task):
        """``TaskPool.add`` — deduplicated by origin id, then gossiped."""
        if not self.set.add(task.origin_id, task):
            return False
        self.by_id[task.id] = task
        return True

    def merge(self, peer):
        self.set.merge(peer.set)
        for t in self.set.values():
            self.by_id.setdefault(t.id, t)

    def all(self):
        return self.set.values()

    def open_tasks(self):
        return [t for t in self.all()
                if t.status in (TaskStatus.UNASSIGNED, TaskStatus.AUCTION)]

    def pending_count(self):
        return len(self.open_tasks())

    def stats(self, tick=None, exclude_kinds=("reprobe",)):
        """Live counts across the whole pipeline, for the dashboard.

        `received` is every task the WMS has released so far, and the four
        buckets below it always sum back to that number — housekeeping
        reprobe tasks are excluded so the figures match what an operator
        would call an order.
        """
        if tick is not None and self._stats_cache[0] == tick:
            return self._stats_cache[1]     # several callers per tick
        counts = {"received": 0, "pending": 0, "auction": 0,
                  "in_progress": 0, "delivered": 0, "recovery": 0}
        for t in self.all():
            if t.kind in exclude_kinds:
                continue
            counts["received"] += 1
            if t.status == TaskStatus.DONE:
                counts["delivered"] += 1
            elif t.status == TaskStatus.CLAIMED:
                counts["in_progress"] += 1
            elif t.status == TaskStatus.AUCTION:
                counts["auction"] += 1
            elif t.status == TaskStatus.NEEDS_RECOVERY:
                counts["recovery"] += 1
            else:
                counts["pending"] += 1
        counts["outstanding"] = (counts["received"] - counts["delivered"])
        self._stats_cache = (tick, counts)
        return counts

    # ------------------------------------------------------------- scoring
    def aged_priority(self, task, tick):
        """``aging_adjusted_score``: a task nobody wants climbs until someone does."""
        return task.priority + self.cfg.AGING_RATE * (tick - task.created_tick)

    def prefilter(self, task, robot):
        """Capability and spatial-locality prefilter — cheap rejection before
        anybody computes a score or sends a bid."""
        if robot.capability != task.required_capability and task.required_capability != "any":
            return False
        if robot.charging:
            return False        # on charge: the engine releases it at BATTERY_RESUME
        return True

    SPECIALIST_RESERVE = 2.0

    def score(self, task, robot, tick, warehouse):
        """A robot's bid: distance, battery, workload and the aged priority.

        The one non-obvious term is the specialist reserve. A heavy-capable
        robot can also carry a generic load, so without it the scarce heavy
        robots get consumed by ordinary work and every heavy task then waits
        for one to free up. Bidding slightly lower on work anyone could do
        keeps them available for the work only they can do.
        """
        dist = warehouse.manhattan(robot.pos(), task.pickup)
        dist_term = 12.0 / (1.0 + dist)
        battery_term = robot.battery / 100.0
        workload_term = 1.0 if robot.task is None else 0.0
        reserve = (self.SPECIALIST_RESERVE
                   if task.required_capability == "any" and robot.capability != "any"
                   else 0.0)
        return (dist_term + battery_term + workload_term
                + self.aged_priority(task, tick) - reserve)

    # ------------------------------------------------------------- auction
    def run_auction(self, tick, robots, warehouse, clock, log):
        """One pass of the time-boxed auction over every open task.

        Tasks are offered in aged-priority order, so a job that has been
        waiting climbs past newer ones instead of starving behind them.
        """
        alive = {r.id: r for r in robots if r.is_alive()}
        busy = {rid for rid, r in alive.items() if r.task is not None}

        for task in sorted(self.open_tasks(),
                           key=lambda t: (-self.aged_priority(t, tick), t.id)):
            if tick < task.recheck_after:
                continue

            capable = [r for r in alive.values() if self.prefilter(task, r)]
            if not capable:
                # Genuinely nobody can ever do this job: back off. Note the
                # distinction — "every robot is busy right now" is NOT no
                # candidates, and backing off there is what makes an auction
                # look slower than a central dispatcher for no reason.
                self._no_candidates(task, tick)
                continue

            free = [r for r in capable if r.id not in busy]
            if not free and not task.bids:
                continue                      # nothing to bid yet; retry next tick

            if task.status == TaskStatus.UNASSIGNED:
                task.status = TaskStatus.AUCTION
                task.auction_opened = tick
                task.bids = {}
                task.requeue_delay = 0.0

            # every capable robot bids independently, with identical logic
            for robot in free:
                task.bids[robot.id] = Bid(task.id, robot.id,
                                          self.score(task, robot, tick, warehouse),
                                          clock.tick())

            if tick - task.auction_opened < self.cfg.BID_WINDOW:
                continue                      # the window is still open

            # A bidder can have been claimed or declared failed since it bid —
            # stale bids are simply dropped, never trusted.
            live = [b for b in task.bids.values()
                    if b.robot_id in alive and b.robot_id not in busy]
            if not live:
                continue                      # bidders got taken; re-run next tick

            ranked = sorted(live, key=lambda b: (-b.score, b.lamport_ts, b.robot_id))
            winner = alive[ranked[0].robot_id]
            self.claim(task, winner, tick)
            busy.add(winner.id)
            self.auction_log.append((tick, task.id, winner.id, len(live)))
            log(f"T{task.id} auctioned to robot {winner.id} ({len(live)} bids)")

    def _no_candidates(self, task, tick):
        """``on_no_candidates``: exponential backoff, never a busy re-auction loop."""
        task.status = TaskStatus.UNASSIGNED
        task.requeue_delay = min(max(1.0, task.requeue_delay * self.cfg.REQUEUE_BACKOFF),
                                 self.cfg.MAX_REQUEUE_DELAY)
        task.recheck_after = tick + int(task.requeue_delay)

    def claim(self, task, robot, tick):
        task.status = TaskStatus.CLAIMED
        task.holder = robot.id
        task.claimed_tick = tick
        task.lease_expiry = tick + self.cfg.TASK_TTL     # same Lease Algorithm 5 watches
        task.bids = {}
        robot.task = task
        robot.phase = "to_pickup"

    def renew_lease(self, task, tick):
        task.lease_expiry = tick + self.cfg.TASK_TTL

    def expired_leases(self, tick):
        return [t for t in self.all()
                if t.status == TaskStatus.CLAIMED and t.lease_expiry is not None
                and tick > t.lease_expiry]

    def release_for_reassignment(self, task, tick, item_in_transit=False):
        """Re-enters the normal allocation pipeline — never a special-case path."""
        task.reassignments += 1
        task.holder = None
        task.lease_expiry = None
        task.bids = {}
        task.recheck_after = 0
        if item_in_transit:
            # the item is somewhere between pickup and dropoff: flag it for the
            # operator, reset the leg, and let the pool re-auction it normally
            task.status = TaskStatus.NEEDS_RECOVERY
            task.recovery_flagged = True
            task.item_in_transit = False
        task.status = TaskStatus.UNASSIGNED

    def complete(self, task, tick):
        task.status = TaskStatus.DONE
        task.done_tick = tick
        task.holder = None

    # ------------------------------------------------------------- baseline
    def dispatch_nearest(self, tick, robots, warehouse):
        """The control arm: a central dispatcher assigning the nearest idle
        robot, first-come-first-served. No bidding, no aging, no lease."""
        idle = [r for r in robots if r.is_alive() and r.task is None]
        for task in sorted(self.open_tasks(), key=lambda t: t.id):
            cands = [r for r in idle if self.prefilter(task, r)]
            if not cands:
                continue
            cands.sort(key=lambda r: (warehouse.manhattan(r.pos(), task.pickup), r.id))
            self.claim(task, cands[0], tick)
            idle.remove(cands[0])
