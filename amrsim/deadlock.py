"""
Algorithm 3 — Wait-Graph Cycle Detection with Escape and Shuffle Fallback.

``waiting_for`` is piggybacked on the heartbeat, so the wait graph costs no
extra messages. Detection is a bounded chain walk (never a full graph search,
so latency stays predictable), and resolution escalates:

    expanding-radius escape  ->  shuffle chain  ->  gridlock alert

Only the lowest-id member of a confirmed cycle acts; everyone else freezes in
``IN_RESOLUTION`` so two robots never try to resolve the same cycle at once.
"""

from collections import deque


def wait_stalled(robot, tick, cfg):
    """True once a robot has been waiting long enough to be worth inspecting.
    The threshold is what stops a one-tick yield being called a deadlock."""
    if robot.wait_started is None:
        robot.wait_started = tick
    return (tick - robot.wait_started) > cfg.WAIT_STALL_THRESHOLD


def find_cycle(robot, robots_by_id, fleet_size):
    """Bounded walk along ``waiting_for`` edges. Returns the cycle members or
    None for a dead end (which is starvation, not deadlock — see the
    stale-wait watchdog in the engine)."""
    chain = [robot.id]
    current = robot.waiting_for
    for _ in range(fleet_size):
        if current is None:
            return None
        if current == robot.id:
            return chain
        if current in chain:
            return None          # a cycle we are not part of
        chain.append(current)
        nxt = robots_by_id.get(current)
        current = nxt.waiting_for if nxt else None
    return None


def search_expanding_radius(wh, origin, exclude_cells, blocks, occupied, cfg):
    """``search_expanding_radius``: nearest free cell, searched in growing
    rings, biased so the *first* hop is perpendicular to the corridor the
    robot is stuck in. Sliding further along the same lane only relocates the
    pinch point; stepping into the parallel lane is what actually lets the
    other robot through."""
    radius = cfg.R0
    while radius <= cfg.MAX_SEARCH_RADIUS:
        cell = _nearest_free(wh, origin, radius, exclude_cells, blocks, occupied)
        if cell is not None:
            return cell
        radius *= 2
    return None


def _nearest_free(wh, origin, radius, exclude_cells, blocks, occupied):
    seen = {origin}
    q = deque([(origin, 0)])
    lane_bias = _lane_bias(wh, origin)
    while q:
        cell, d = q.popleft()
        if d > radius:
            continue
        if (cell != origin and cell not in exclude_cells and cell not in occupied
                and _passable(wh, cell, blocks)):
            return cell
        if d == radius:
            continue
        nbs = wh.free_neighbors(cell)
        if cell == origin and lane_bias:
            nbs.sort(key=lambda n: 0 if _is_perpendicular(origin, n, lane_bias) else 1)
        for nb in nbs:
            if nb not in seen:
                seen.add(nb)
                q.append((nb, d + 1))
    return None


def _lane_bias(wh, cell):
    """The axis the robot's current lane runs along, or None if it is in the open."""
    nbs = wh.free_neighbors(cell)
    horizontal = [n for n in nbs if n[1] == cell[1]]
    vertical = [n for n in nbs if n[0] == cell[0]]
    if len(horizontal) >= 2 and len(vertical) == 0:
        return "h"
    if len(vertical) >= 2 and len(horizontal) == 0:
        return "v"
    return None


def _is_perpendicular(origin, nb, lane_bias):
    return (lane_bias == "h" and nb[0] == origin[0]) or (lane_bias == "v" and nb[1] == origin[1])


def _passable(wh, cell, blocks):
    if not wh.is_walkable(*cell):
        return False
    ev = blocks.get(cell)
    return not (ev is not None and ev.confidence == "CONFIRMED" and ev.severity == "FULL")


def find_shuffle_chain(wh, origin, occupied_by, exclude_ids, blocks, cfg):
    """BFS *through* occupied cells to the nearest genuinely free one. Returns
    the robot ids to shift, nearest-to-free first, so a packed aisle can
    ripple open one robot at a time."""
    visited = {origin}
    q = deque([(origin, [])])
    while q:
        cell, chain = q.popleft()
        if len(chain) > cfg.SHUFFLE_MAX_DEPTH:
            continue
        for nb in wh.free_neighbors(cell):
            if nb in visited:
                continue
            visited.add(nb)
            occupant = occupied_by.get(nb)
            if occupant is None:
                if _passable(wh, nb, blocks):
                    return chain, nb
                continue
            if occupant in exclude_ids:
                continue
            q.append((nb, chain + [occupant]))
    return None, None
