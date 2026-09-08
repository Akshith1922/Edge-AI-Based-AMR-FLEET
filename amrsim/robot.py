"""
The per-robot agent state.

Everything here is state a real robot would hold *locally*: its pose, its
battery, the task it has claimed, its committed path, and the small pieces of
protocol state the six algorithms exchange — ``waiting_for`` (Algorithm 3),
``last_heartbeat`` (Algorithm 5), ``detour_history`` (Algorithm 2's fairness
term) and the corridor grants it holds (Algorithm 1).
"""

from dataclasses import dataclass, field
from enum import Enum


class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    WAITING = "waiting"
    QUEUED = "queued"              # holding at a chokepoint mouth
    IN_RESOLUTION = "in_resolution"
    SHUFFLING = "shuffling"
    FAILED = "failed"
    RECOVERING = "recovering"


@dataclass
class Robot:
    id: int
    x: int
    y: int
    capability: str = "any"
    battery: float = 100.0

    state: RobotState = RobotState.IDLE
    path: list = field(default_factory=list)      # cells, index i == tick t0 + i
    path_index: int = 0
    path_t0: int = 0
    task: object = None
    phase: str = "to_pickup"                      # to_pickup | to_dropoff | park
    target: tuple = None

    # protocol state
    waiting_for: int = None
    wait_started: int = None
    last_heartbeat: int = 0
    heartbeat_frozen: bool = False
    failed_at: int = None
    detour_history: list = field(default_factory=list)
    entered_corridor: dict = field(default_factory=dict)
    last_plan_tick: int = -999
    resume_at: int = 0
    last_moved_tick: int = 0
    last_action: str = "IDLE"
    last_geometry: str = None

    # metrics
    distance: int = 0
    wait_ticks: int = 0
    tasks_done: int = 0
    replans: int = 0

    # ------------------------------------------------------------------ pose
    def pos(self):
        return (self.x, self.y)

    def is_alive(self):
        return self.state != RobotState.FAILED

    def is_active(self):
        return self.state not in (RobotState.FAILED,)

    def heading(self):
        remaining = self.remaining_path()
        if len(remaining) < 2:
            return (0, 0)
        for cell in remaining[1:]:
            if cell != remaining[0]:
                return (cell[0] - remaining[0][0], cell[1] - remaining[0][1])
        return (0, 0)

    def remaining_path(self):
        return self.path[self.path_index:]

    def next_cell(self):
        rem = self.remaining_path()
        return rem[1] if len(rem) > 1 else self.pos()

    def has_plan(self):
        return self.path_index < len(self.path) - 1

    # -------------------------------------------------------------- priority
    def priority_score(self, warehouse):
        """The score every ranking in the system is built on.

        Distance-to-target, battery and workload as documented, plus an
        explicit bonus for a robot that is already carrying an item: pre-empting
        a half-finished delivery wastes strictly more work than delaying one
        that has not started, so a mid-delivery robot must reliably outrank a
        fresh one in a contested cell.
        """
        battery_term = self.battery / 100.0
        if self.task is None:
            return 0.4 + battery_term * 0.2               # parked robots yield
        target = self.target or self.task.pickup
        dist = warehouse.manhattan(self.pos(), target)
        dist_term = 2.0 / (1.0 + dist)
        transit_bonus = 1.5 if self.task.item_in_transit else 0.0
        return dist_term + battery_term + self.task.priority + transit_bonus + 1.0

    # ---------------------------------------------------------------- motion
    def advance_to(self, cell, cfg, tick=0):
        if cell != self.pos():
            self.distance += 1
            self.last_moved_tick = tick
            self.battery = max(0.0, self.battery - cfg.BATTERY_DRAIN_PER_CELL)
        else:
            self.wait_ticks += 1
        self.x, self.y = cell
        self.path_index += 1

    def clear_plan(self):
        self.path = []
        self.path_index = 0

    def as_dict(self):
        return {
            "id": self.id, "x": self.x, "y": self.y,
            "capability": self.capability, "battery": round(self.battery, 1),
            "state": self.state.value, "task": self.task.id if self.task else None,
            "phase": self.phase if self.task else None,
            "target": list(self.target) if self.target else None,
            "waiting_for": self.waiting_for,
            "action": self.last_action, "geometry": self.last_geometry,
            "path": [list(c) for c in self.remaining_path()[:40]],
            "distance": self.distance, "wait_ticks": self.wait_ticks,
            "stalled": self.resume_at,
            "tasks_done": self.tasks_done, "replans": self.replans,
        }
