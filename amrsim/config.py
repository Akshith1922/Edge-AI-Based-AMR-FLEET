"""
Central configuration.

Every tunable in the simulation lives here, named after the constant it
corresponds to in *EdgeAI_Fleet_Coordination_Algorithms* so the code can be
read side-by-side with the algorithm reference.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    # ---- fleet -----------------------------------------------------------
    NUM_ROBOTS: int = 8
    BATTERY_DRAIN_PER_CELL: float = 0.04
    # Charging. Without these a robot's battery only ever falls, and once it
    # drops under the bidding floor the robot is retired for good — the fleet
    # quietly parks itself while work piles up in the pool. A robot sitting on
    # a charging bay draws current; hysteresis between the two thresholds stops
    # it accepting one job at the floor and immediately dying again.
    CHARGE_RATE_PER_TICK: float = 0.9
    BATTERY_MIN_BID: float = 25.0     # below this a robot stops bidding
    BATTERY_RESUME: float = 65.0      # and stays on charge until it reaches this

    # An *unplanned* stop is not free. The safety node in the reference design
    # runs at 20 Hz and cuts speed the moment something enters HARD_STOP_RADIUS,
    # so a robot that gets surprised has to decelerate to zero and accelerate
    # again before it covers another cell. A planned wait costs nothing extra,
    # because the robot knew about it in advance and simply eased off. This is
    # the whole practical difference between yielding and being stopped, so it
    # is charged explicitly rather than assumed away. Set to 0 to disable.
    HARD_STOP_RESUME_TICKS: int = 2

    # ---- Algorithm 1 : path conflicts -----------------------------------
    RESERVATION_PAD: int = 1          # ticks of buffer around each cell window
    # The reference's ``cascades(depth)`` recursion bound has no counterpart
    # here: a rerouted path is checked against the reservation table inside the
    # same planning pass that produced it, and each robot plans exactly once
    # per tick in rank order, so a reroute cannot set off a chain of further
    # reroutes. Cascading is bounded by construction rather than by a depth cap.
    CORRIDOR_LOOKAHEAD: int = 8       # how far ahead a robot requests an aisle lock
    CORRIDOR_MIN_LEN: int = 2         # runs shorter than this are not locked

    # ---- Algorithm 2 : congestion-aware cost ----------------------------
    ALPHA: float = 0.35               # congestion weight
    FAIRNESS_THRESHOLD: float = 1.30  # mean detour ratio above which ALPHA halves
    MAX_DETOUR_RATIO: float = 1.80    # bounded detour cap
    DETOUR_HISTORY: int = 8
    REPLAN_INTERVAL: int = 8          # ticks between opportunistic replans
    PLAN_HORIZON: int = 96            # space-time A* time depth (> map diameter)
    COOP_WINDOW: int = 16             # ticks of cooperative (space-time) search
    HEURISTIC_WEIGHT: float = 1.15    # mild weighting: fewer nodes, near-optimal
    MAX_EXPANSIONS: int = 60000       # hard bound on A* work per call
    WAIT_COST: float = 0.9            # cost of a "stay put" action:
    #  slightly cheaper than moving, so holding for a tick is preferred
    #  over a detour of equal length — the wait/reroute trade-off in
    #  Algorithm 1's select_action, expressed in the cost function.

    # ---- Algorithm 3 : deadlocks ----------------------------------------
    WAIT_STALL_THRESHOLD: int = 4     # ticks of waiting before cycle check
    R0: int = 1
    MAX_SEARCH_RADIUS: int = 8
    SHUFFLE_MAX_DEPTH: int = 5
    STALE_WAIT_WATCHDOG: int = 12     # liveness net for non-cyclic starvation

    # ---- Algorithm 4 : dynamic re-routing -------------------------------
    CONFIRM_WINDOW: int = 3           # scans an obstacle must persist for
    BLOCK_TTL: int = 90
    REPROBE_ESCALATION_TIME: int = 140
    PARTIAL_BLOCK_PENALTY: float = 6.0
    CAUTIOUS_PENALTY: float = 2.0

    # ---- Algorithm 5 : failure recovery ---------------------------------
    HEARTBEAT_TTL: int = 15           # coordinated detection window
    BASELINE_FAILURE_TIMEOUT: int = 45

    # ---- Algorithm 6 : task allocation ----------------------------------
    AGING_RATE: float = 0.02
    TASK_TTL: int = 90                # lease length
    BID_WINDOW: int = 1               # time-boxed auction, in ticks
    MAX_REQUEUE_DELAY: int = 30
    REQUEUE_BACKOFF: float = 1.5

    # ---- baseline (uncoordinated control arm) ---------------------------
    BASELINE_STALL_TIMEOUT: int = 10  # fixed-timeout random backoff

    # ---- metrics ---------------------------------------------------------
    HISTORY_LEN: int = 600


DEFAULT = Config()
