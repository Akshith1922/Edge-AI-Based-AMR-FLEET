"""
Algorithm 2 — Congestion-Aware Path Cost with Fairness and Detour Bounds,
extended by Algorithm 4's block-aware cost term.

The planner is a **space-time A\\***: a search node is ``(cell, t)`` and the
available actions are "move to a free neighbour" and "wait here". That single
change is what makes the whole system safe by construction — a path is only
returned if it never occupies a cell at the same tick as a committed
reservation and never swaps places with an oncoming robot, so WAIT and REROUTE
fall out of the search itself rather than being bolted on afterwards.

Cost of entering a cell, exactly as documented:

    base(edge) * block_penalty(cell)  +  alpha(robot) * (1 + jitter(robot)) *
    congestion_weight(cell, window)
"""

import heapq
import math


def deterministic_jitter(robot_id):
    """Bounded, per-robot, and stable across ticks: breaks the symmetry that
    makes a whole fleet reroute onto the same alternative aisle at once."""
    return 0.05 * ((robot_id * 2654435761) % 100) / 100.0


def fairness_adjusted_alpha(robot, cfg):
    """A robot that has been detoured repeatedly gets a discounted congestion
    weight, so the same unlucky robot is not made to yield forever."""
    if not robot.detour_history:
        return cfg.ALPHA
    avg = sum(robot.detour_history) / len(robot.detour_history)
    return cfg.ALPHA * 0.5 if avg > cfg.FAIRNESS_THRESHOLD else cfg.ALPHA


def block_factor(cell, blocks, cfg):
    """Algorithm 4's contribution: INF for a confirmed full block, a heavy
    multiplier for a partial one, a mild one for an unconfirmed report."""
    ev = blocks.get(cell)
    if ev is None:
        return 1.0
    if ev.confidence == "CONFIRMED":
        return math.inf if ev.severity == "FULL" else cfg.PARTIAL_BLOCK_PENALTY
    return cfg.CAUTIOUS_PENALTY


class PlanResult:
    __slots__ = ("steps", "action", "expansions", "detour_ratio", "blocked_by")

    def __init__(self, steps, action="PROCEED", expansions=0,
                 detour_ratio=1.0, blocked_by=None):
        self.steps = steps
        self.action = action              # PROCEED | WAIT | REROUTE | REVERSE
        self.expansions = expansions
        self.detour_ratio = detour_ratio
        self.blocked_by = blocked_by

    def __bool__(self):
        return bool(self.steps)


def plan_spacetime(wh, table, blocks, robot, start, goal, tick, cfg,
                   respect_reservations=True, use_congestion=True,
                   forbidden=frozenset(), avoid=frozenset(), horizon=None):
    """Windowed cooperative space-time A* from `start` at `tick` to `goal`.

    Returns ``(steps, expansions)`` where ``steps[i]`` is the cell occupied at
    ``tick + i``, or ``(None, expansions)``.

    Only the first ``COOP_WINDOW`` ticks are searched in space-time; past the
    window other robots' plans are too stale to be worth honouring, so the
    search collapses to an ordinary spatial A*. Safety does not depend on the
    tail of the path — every robot re-validates and re-plans every tick, and
    only the *next* step is ever executed, which always lies inside the
    window. This is what keeps the search a few thousand nodes instead of a
    few hundred thousand.
    """
    if start == goal:
        return [start], 0
    horizon = horizon or cfg.PLAN_HORIZON
    window = min(cfg.COOP_WINDOW, horizon)
    weight = fairness_adjusted_alpha(robot, cfg) * (1.0 + deterministic_jitter(robot.id))
    hw = cfg.HEURISTIC_WEIGHT
    rid = robot.id
    gx, gy = goal
    adj = wh.adj
    busy, swap, congestion = table.is_busy, table.swap_conflict, table.congestion_at

    # Resolve the block costs once instead of per expansion.
    hard = {c for c, e in blocks.items()
            if e.confidence == "CONFIRMED" and e.severity == "FULL"}
    soft = {c: (cfg.PARTIAL_BLOCK_PENALTY if e.confidence == "CONFIRMED"
                else cfg.CAUTIOUS_PENALTY)
            for c, e in blocks.items() if c not in hard}

    open_heap = [((abs(start[0] - gx) + abs(start[1] - gy)) * hw, 0.0, start, 0)]
    best = {(start, 0): 0.0}
    came = {}
    expansions = 0

    while open_heap:
        _, g, cell, t = heapq.heappop(open_heap)
        if best.get((cell, t), math.inf) < g - 1e-9:
            continue
        if cell == goal:
            return _reconstruct(came, (cell, t)), expansions
        if t >= horizon:
            continue
        expansions += 1
        if expansions > cfg.MAX_EXPANSIONS:
            break

        cooperative = t < window
        at = tick + t
        options = (cell,) + adj[cell] if cooperative else adj[cell]
        for nxt in options:
            if nxt in hard or nxt in forbidden:
                continue
            if nxt in avoid and nxt != goal:
                continue
            if cooperative and respect_reservations:
                if busy(nxt, at + 1, rid):
                    continue
                if nxt != cell and swap(cell, nxt, at, rid):
                    continue
            cost = (cfg.WAIT_COST if nxt == cell else 1.0) * soft.get(nxt, 1.0)
            if use_congestion and cooperative:
                cost += weight * congestion(nxt, at + 1, rid)
            ng = g + cost
            # past the window time no longer changes anything, so states are
            # deduplicated on the cell alone and the search stays small
            key = (nxt, t + 1 if t + 1 <= window else window + 1)
            if ng < best.get(key, math.inf) - 1e-9:
                best[key] = ng
                came[(nxt, t + 1)] = (cell, t)
                heapq.heappush(open_heap,
                               (ng + (abs(nxt[0] - gx) + abs(nxt[1] - gy)) * hw,
                                ng, nxt, t + 1))
    return None, expansions


def plan_static(wh, blocks, start, goal, cfg, avoid=frozenset(), honour_blocks=True):
    """Plain distance-only A* on the static map — the direct-distance estimate
    used for the detour cap, and the baseline arm's only planner."""
    if start == goal:
        return [start]
    open_heap = [(0, 0, start)]
    g_score = {start: 0}
    came = {}
    closed = set()
    while open_heap:
        _, g, cell = heapq.heappop(open_heap)
        if cell in closed:
            continue
        closed.add(cell)
        if cell == goal:
            path = [cell]
            while cell in came:
                cell = came[cell]
                path.append(cell)
            return path[::-1]
        for nxt in wh.neighbors(*cell):
            if nxt in avoid and nxt != goal:
                continue
            if honour_blocks and block_factor(nxt, blocks, cfg) == math.inf:
                continue
            ng = g + 1
            if ng < g_score.get(nxt, math.inf):
                g_score[nxt] = ng
                came[nxt] = cell
                heapq.heappush(open_heap, (ng + abs(nxt[0] - goal[0]) + abs(nxt[1] - goal[1]),
                                           ng, nxt))
    return None


def plan_with_detour_cap(wh, table, blocks, robot, start, goal, tick, cfg,
                         forbidden=frozenset(), avoid=frozenset(), direct_len=None):
    """``plan_path`` from the reference: congestion-aware search first, and if
    the result exceeds ``MAX_DETOUR_RATIO`` of the direct distance, fall back
    to the distance-only route so a congestion term can never send a robot on
    an unbounded detour. Records the realised ratio for fairness accounting."""
    if direct_len is None:
        direct = plan_static(wh, blocks, start, goal, cfg)
        direct_len = max(1, len(direct) - 1) if direct else max(1, wh.manhattan(start, goal))

    steps, expansions = plan_spacetime(wh, table, blocks, robot, start, goal, tick, cfg,
                                       forbidden=forbidden, avoid=avoid)
    action = "PROCEED"
    if steps is None:
        # No congestion-aware route: retry ignoring the soft cost, then give up
        # to the caller, which falls back to WAIT / out-of-way (Algorithm 4).
        steps, exp2 = plan_spacetime(wh, table, blocks, robot, start, goal, tick, cfg,
                                     use_congestion=False, forbidden=forbidden, avoid=avoid)
        expansions += exp2
        action = "REROUTE"
    if steps is None:
        return PlanResult(None, action="WAIT", expansions=expansions)

    moves = sum(1 for i in range(1, len(steps)) if steps[i] != steps[i - 1])
    waits = len(steps) - 1 - moves
    ratio = moves / direct_len
    if ratio > cfg.MAX_DETOUR_RATIO:
        capped, exp2 = plan_spacetime(wh, table, blocks, robot, start, goal, tick, cfg,
                                      use_congestion=False, forbidden=forbidden, avoid=avoid)
        expansions += exp2
        if capped is not None:
            capped_moves = sum(1 for i in range(1, len(capped)) if capped[i] != capped[i - 1])
            if capped_moves < moves:
                steps, moves, ratio = capped, capped_moves, capped_moves / direct_len
                action = "REROUTE"

    robot.detour_history.append(ratio)
    if len(robot.detour_history) > cfg.DETOUR_HISTORY:
        robot.detour_history.pop(0)
    if waits > 0 and action == "PROCEED":
        action = "WAIT"
    return PlanResult(steps, action=action, expansions=expansions, detour_ratio=ratio)


def _reconstruct(came, node):
    path = [node[0]]
    while node in came:
        node = came[node]
        path.append(node[0])
    path.reverse()
    return path
