"""Run metrics, matching the evaluation methodology in the report."""

from collections import deque


class Metrics:
    def __init__(self, mode, cfg):
        self.mode = mode
        self.cfg = cfg
        self.ticks = 0
        self.completed = []            # (task_id, ticks_from_creation)
        self.conflict_events = 0
        self.conflict_wait_ticks = 0
        self.replans = 0
        self.deadlock_events = 0
        self.deadlock_recovery = []
        self.gridlocks = 0
        self.reroute_events = 0
        self.reroute_latency = []
        self.failure_events = 0
        self.reassignment_latency = []
        self.collisions = 0
        self.hard_stops = 0
        self.corridor_grants = 0
        self.distance = 0
        self.history = deque(maxlen=cfg.HISTORY_LEN)

    # ------------------------------------------------------------- derived
    def throughput(self):
        """Completed tasks per 100 ticks — the headline productivity number."""
        return (len(self.completed) / self.ticks * 100.0) if self.ticks else 0.0

    def avg_task_time(self):
        return (sum(t for _, t in self.completed) / len(self.completed)
                if self.completed else 0.0)

    def p90_task_time(self):
        if not self.completed:
            return 0.0
        vals = sorted(t for _, t in self.completed)
        return vals[min(len(vals) - 1, int(0.9 * len(vals)))]

    def avg_deadlock_recovery(self):
        return _avg(self.deadlock_recovery)

    def avg_reassignment_latency(self):
        return _avg(self.reassignment_latency)

    def sample(self, tick, robots, pool):
        from .robot import RobotState
        self.history.append({
            "t": tick,
            "done": len(self.completed),
            "pending": pool.pending_count(),
            "waiting": sum(1 for r in robots
                           if r.state in (RobotState.WAITING, RobotState.QUEUED,
                                          RobotState.IN_RESOLUTION)),
            "moving": sum(1 for r in robots if r.state == RobotState.MOVING),
            "avg": round(self.avg_task_time(), 1),
        })

    def summary(self):
        return {
            "mode": self.mode,
            "ticks": self.ticks,
            "tasks_completed": len(self.completed),
            "throughput_per_100": round(self.throughput(), 2),
            "avg_task_time": round(self.avg_task_time(), 2),
            "p90_task_time": round(self.p90_task_time(), 2),
            "conflict_events": self.conflict_events,
            "conflict_wait_ticks": self.conflict_wait_ticks,
            "deadlock_events": self.deadlock_events,
            "avg_deadlock_recovery": round(self.avg_deadlock_recovery(), 2),
            "gridlocks": self.gridlocks,
            "reroute_events": self.reroute_events,
            "avg_reroute_latency": round(_avg(self.reroute_latency), 2),
            "failure_events": self.failure_events,
            "avg_reassignment_latency": round(self.avg_reassignment_latency(), 2),
            "collisions": self.collisions,
            "hard_stops": self.hard_stops,
            "corridor_grants": self.corridor_grants,
            "total_distance": self.distance,
            "replans": self.replans,
        }


def _avg(values):
    return sum(values) / len(values) if values else 0.0
