"""
Algorithm 4 — Block Detection, Propagation and Opportunistic Clearing.

A ``BlockEvent`` is gossiped exactly like a reservation and carries a
confidence level and a TTL. The lifecycle is deliberately never destructive:

    CONFIRMED --(TTL expires)--> UNCONFIRMED --(reprobe clears)--> removed
                                        |
                              (escalation) assign a reprobe task

Softening rather than deleting is what stops a stale report becoming a
phantom wall that quietly makes part of the warehouse unreachable forever.
"""

from dataclasses import dataclass


@dataclass
class BlockEvent:
    cell: tuple
    reported_by: int
    lamport_ts: int
    confidence: str = "CONFIRMED"     # CONFIRMED | UNCONFIRMED
    severity: str = "FULL"            # FULL | PARTIAL
    last_refreshed: int = 0
    escalated: bool = False
    cause: str = "obstacle"           # obstacle | failed_robot

    def as_dict(self):
        return {"cell": list(self.cell), "confidence": self.confidence,
                "severity": self.severity, "cause": self.cause,
                "reported_by": self.reported_by, "age": self.last_refreshed}


class BlockRegistry:
    """Dict-like view of the gossiped ``blocked_edges`` table."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.events = {}
        self.pending = {}     # cell -> consecutive scans, the CONFIRM_WINDOW filter
        self.version = 0      # bumped on every mutation, so route caches can expire

    # ------------------------------------------------------------ dict view
    def get(self, cell):
        return self.events.get(cell)

    def __contains__(self, cell):
        return cell in self.events

    def items(self):
        return self.events.items()

    def full_blocks(self):
        return {c for c, e in self.events.items()
                if e.confidence == "CONFIRMED" and e.severity == "FULL"}

    # -------------------------------------------------------------- reports
    def observe(self, cell, robot_id, lamport, tick, severity="FULL", cause="obstacle"):
        """A scan saw something. Only a *persistent* obstacle becomes an event,
        which filters out robots and people passing through."""
        self.pending[cell] = self.pending.get(cell, 0) + 1
        if self.pending[cell] < self.cfg.CONFIRM_WINDOW:
            return None
        return self.report(cell, robot_id, lamport, tick, severity, cause)

    def report(self, cell, robot_id, lamport, tick, severity="FULL", cause="obstacle"):
        """``on_block_event``: idempotent, last-writer-wins by Lamport clock."""
        existing = self.events.get(cell)
        if existing is not None and existing.lamport_ts > lamport:
            return existing
        ev = BlockEvent(cell=cell, reported_by=robot_id, lamport_ts=lamport,
                        severity=severity, last_refreshed=tick, cause=cause)
        self.events[cell] = ev
        self.version += 1
        return ev

    def refresh(self, cell, tick):
        ev = self.events.get(cell)
        if ev is not None:
            if ev.confidence != "CONFIRMED":
                self.version += 1
            ev.last_refreshed = tick
            ev.confidence = "CONFIRMED"
            ev.escalated = False

    def clear(self, cell):
        """``on_clear_event``: the next robot through treats it cautiously
        anyway, because the pending counter is reset, not trusted."""
        self.pending.pop(cell, None)
        ev = self.events.pop(cell, None)
        if ev is not None:
            self.version += 1
        return ev

    # ---------------------------------------------------------------- tick
    def expire(self, tick):
        """``on_tick_check_expiry`` plus the escalation fallback."""
        softened, escalated = [], []
        for cell, ev in self.events.items():
            age = tick - ev.last_refreshed
            if age > self.cfg.BLOCK_TTL and ev.confidence == "CONFIRMED":
                ev.confidence = "UNCONFIRMED"
                self.version += 1
                softened.append(cell)
            if age > self.cfg.REPROBE_ESCALATION_TIME and not ev.escalated:
                ev.escalated = True
                escalated.append(cell)
        return softened, escalated

    def snapshot(self):
        return [e.as_dict() for e in self.events.values()]
