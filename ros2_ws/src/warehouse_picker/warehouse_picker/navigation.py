"""
Navigation: a global A* over the static map and a local DWA that drives to it.

The split matters. The A* layer knows the warehouse but not the present
moment; the DWA layer knows only what the lidar can see right now but obeys
the robot's actual velocity and acceleration limits. Together they give the
property the previous agent was missing: the robot follows a route that exists
on the map, *and* it will not drive into something the map does not know about
-- a pallet left in an aisle, a person, or another AMR.

Nothing here imports ROS, so the same code runs on the robot, in the headless
twin, and under unit test.
"""

import math
from heapq import heappush, heappop

TAU = 2.0 * math.pi


def wrap(a):
    """Fold an angle into [-pi, pi)."""
    return (a + math.pi) % TAU - math.pi


class Limits:
    """Kinematic envelope of the platform, in SI units."""

    __slots__ = ("v_max", "v_min", "w_max", "a_lin", "a_ang",
                 "radius", "safety", "dt", "horizon")

    def __init__(self, v_max=0.8, v_min=-0.25, w_max=1.2, a_lin=0.6, a_ang=2.4,
                 radius=0.40, safety=0.15, dt=0.1, horizon=1.6):
        self.v_max, self.v_min, self.w_max = v_max, v_min, w_max
        self.a_lin, self.a_ang = a_lin, a_ang
        self.radius, self.safety = radius, safety
        self.dt, self.horizon = dt, horizon


# --------------------------------------------------------------------------
# global planner
# --------------------------------------------------------------------------

# 8-connected; diagonals cost sqrt(2) so the path length is a real distance.
_NEIGHBOURS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
               (1, 1, 1.4142135623730951), (1, -1, 1.4142135623730951),
               (-1, 1, 1.4142135623730951), (-1, -1, 1.4142135623730951))


class AStar:
    """A* over an inflated grid, with a soft penalty layer for peer traffic.

    `penalties` maps a cell to extra cost. This is how a robot yields without
    stopping: the cells another robot has claimed become expensive rather than
    impassable, so the search takes the next aisle over if one is cheap enough
    and only queues behind the peer if it genuinely is the best option.
    """

    def __init__(self, grid, max_expansions=120000):
        self.grid = grid
        self.max_expansions = max_expansions
        self.last_expansions = 0

    def _free(self, col, row):
        return self.grid.at(col, row) == 0

    def nearest_free(self, col, row, max_radius=24):
        """Snap a cell into free space -- goals land inside inflation often."""
        if self._free(col, row):
            return (col, row)
        for r in range(1, max_radius + 1):
            best, best_d = None, None
            for dc in range(-r, r + 1):
                for dr in (-r, r):
                    for (c, rw) in ((col + dc, row + dr), (col + dr, row + dc)):
                        if self._free(c, rw):
                            d = dc * dc + r * r
                            if best_d is None or d < best_d:
                                best, best_d = (c, rw), d
            if best:
                return best
        return None

    # How much open floor a robot would *like* around it, and what a metre of
    # missing clearance is worth in extra path length. Without this the search
    # is indifferent between the middle of a 4 m aisle and a route that scrapes
    # along the inflation boundary, and it will happily pick the latter because
    # it is a few centimetres shorter -- leaving the local planner to thread a
    # gap with no room to correct in.
    COMFORT_M = 1.10
    COMFORT_WEIGHT = 1.4

    def comfort_cost(self, cell):
        clearance_cm = self.grid.clearance_cm
        if clearance_cm is None:
            return 0.0
        col, row = cell
        gap = clearance_cm[row * self.grid.w + col] / 100.0
        if gap >= self.COMFORT_M:
            return 0.0
        return self.COMFORT_WEIGHT * (self.COMFORT_M - gap) / self.COMFORT_M

    # Weighted A*. At 1.3 the search expands about a quarter of the nodes it
    # does at 1.05 and the paths come out within a couple of percent of optimal
    # on this map -- a trade worth making on a Raspberry Pi that replans every
    # 1.5 s while also running a local planner at 10 Hz.
    HEURISTIC_WEIGHT = 1.3

    def plan(self, start_xy, goal_xy, penalties=None, weight=None):
        """World-coordinate A*. Returns a list of ``(x, y)`` waypoints, or []."""
        grid = self.grid
        start = self.nearest_free(*grid.world_to_grid(*start_xy))
        goal = self.nearest_free(*grid.world_to_grid(*goal_xy))
        self.last_expansions = 0
        if start is None or goal is None:
            return []
        if start == goal:
            return [grid.grid_to_world(*goal)]

        weight = self.HEURISTIC_WEIGHT if weight is None else weight
        pen = penalties or {}
        gx, gy = goal
        open_heap = [(0.0, start)]
        came = {}
        cost = {start: 0.0}
        seen = set()
        expansions = 0

        while open_heap:
            _, cur = heappop(open_heap)
            if cur in seen:
                continue
            seen.add(cur)
            if cur == goal:
                break
            expansions += 1
            if expansions > self.max_expansions:
                self.last_expansions = expansions
                return []
            cc, cr = cur
            base = cost[cur]
            for (dc, dr, step) in _NEIGHBOURS:
                nc, nr = cc + dc, cr + dr
                if not self._free(nc, nr):
                    continue
                if dc and dr:
                    # no cutting a diagonal through the corner of a rack
                    if not (self._free(cc + dc, cr) and self._free(cc, cr + dr)):
                        continue
                nxt = (nc, nr)
                g = base + step + pen.get(nxt, 0.0) + self.comfort_cost(nxt)
                if g < cost.get(nxt, math.inf):
                    cost[nxt] = g
                    came[nxt] = cur
                    h = math.hypot(nc - gx, nr - gy)
                    heappush(open_heap, (g + weight * h, nxt))

        self.last_expansions = expansions
        if goal not in came and goal != start:
            return []

        cells = [goal]
        while cells[-1] != start:
            cells.append(came[cells[-1]])
        cells.reverse()
        return [grid.grid_to_world(c, r) for (c, r) in cells]

    def simplify(self, path, tolerance=None):
        """Drop waypoints that lie on a straight, obstacle-free run.

        Halves the number of points on a warehouse aisle, which makes the
        lookahead search cheap and the path easier to read on the dashboard.
        """
        if len(path) < 3:
            return list(path)
        tol = tolerance if tolerance is not None else self.grid.resolution * 0.6
        out = [path[0]]
        anchor = 0
        for i in range(2, len(path)):
            ax, ay = path[anchor]
            cx, cy = path[i]
            straight = True
            seg = math.hypot(cx - ax, cy - ay)
            if seg < 1e-9:
                continue
            for j in range(anchor + 1, i):
                px, py = path[j]
                # perpendicular distance from the chord
                d = abs((cx - ax) * (ay - py) - (ax - px) * (cy - ay)) / seg
                if d > tol:
                    straight = False
                    break
            if straight and self.line_is_clear((ax, ay), (cx, cy)):
                continue
            out.append(path[i - 1])
            anchor = i - 1
        out.append(path[-1])
        return out

    def line_is_clear(self, a, b):
        grid = self.grid
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(dist / (grid.resolution * 0.5)))
        for i in range(steps + 1):
            t = i / steps
            if grid.at(*grid.world_to_grid(a[0] + t * (b[0] - a[0]),
                                           a[1] + t * (b[1] - a[1]))):
                return False
        return True


# --------------------------------------------------------------------------
# path bookkeeping
# --------------------------------------------------------------------------

class PathTracker:
    """Holds a path and answers 'where should I aim right now?'.

    The cursor is found by projecting the robot onto the path, not by waiting
    for it to come within a fixed distance of the next waypoint. That
    distinction is not cosmetic: with a proximity test, a robot that is pushed
    off the path, that jumps after a replan, or that simply passes a waypoint
    wide never advances its cursor at all, and then hands the local planner a
    carrot that is *behind* it. The robot drives backwards along its own route
    or stalls, and the failure looks like an obstacle-avoidance bug rather than
    a bookkeeping one.

    The search only ever runs forward from the current cursor, so a route that
    doubles back on itself -- out of an aisle and back down the next one -- does
    not snap the cursor onto the returning leg.
    """

    def __init__(self, lookahead=1.1):
        self.path = []
        self.lookahead = lookahead
        self.index = 0

    def set_path(self, path):
        self.path = list(path)
        self.index = 0

    def clear(self):
        self.path = []
        self.index = 0

    @property
    def active(self):
        return bool(self.path) and self.index < len(self.path)

    def locate(self, x, y):
        """Closest point on the remaining path: ``(segment, point, distance)``."""
        if not self.path:
            return None
        if len(self.path) == 1:
            p = self.path[0]
            return (0, p, math.hypot(p[0] - x, p[1] - y))
        best = None
        for i in range(self.index, len(self.path) - 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            dx, dy = bx - ax, by - ay
            span = dx * dx + dy * dy
            t = 0.0 if span < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx +
                                                            (y - ay) * dy) / span))
            px, py = ax + t * dx, ay + t * dy
            d = math.hypot(px - x, py - y)
            if best is None or d < best[2]:
                best = (i, (px, py), d)
        return best

    def advance(self, x, y):
        located = self.locate(x, y)
        if located:
            self.index = located[0]
        return located

    def carrot(self, x, y):
        """The point `lookahead` metres further along the path than the robot."""
        if not self.path:
            return None
        located = self.advance(x, y)
        if located is None:
            return None
        seg, point, _d = located
        remaining = self.lookahead
        px, py = point
        for i in range(seg + 1, len(self.path)):
            wx, wy = self.path[i]
            step = math.hypot(wx - px, wy - py)
            if step >= remaining:
                if step < 1e-9:
                    return (wx, wy)
                t = remaining / step
                return (px + t * (wx - px), py + t * (wy - py))
            remaining -= step
            px, py = wx, wy
        return self.path[-1]

    def remaining_length(self, x, y):
        located = self.advance(x, y)
        if located is None:
            return 0.0
        seg, point, dist = located
        total = dist
        px, py = point
        for i in range(seg + 1, len(self.path)):
            wx, wy = self.path[i]
            total += math.hypot(wx - px, wy - py)
            px, py = wx, wy
        return total

    def cells_ahead(self, grid, x, y, distance):
        """Grid cells the robot expects to occupy within `distance` metres.

        This is what gets broadcast as the robot's *intent*, and what peers
        test their own route against.
        """
        located = self.advance(x, y)
        if located is None:
            return []
        seg, point, _d = located
        out = []
        travelled = 0.0
        px, py = point
        for i in range(seg + 1, len(self.path)):
            wx, wy = self.path[i]
            step = math.hypot(wx - px, wy - py)
            samples = max(1, int(step / grid.resolution))
            for s in range(1, samples + 1):
                t = s / samples
                cell = grid.world_to_grid(px + t * (wx - px), py + t * (wy - py))
                if not out or out[-1] != cell:
                    out.append(cell)
            travelled += step
            px, py = wx, wy
            if travelled >= distance:
                break
        return out


# --------------------------------------------------------------------------
# local planner
# --------------------------------------------------------------------------

# Two different margins, for two different reasons.
#
# The map is inflated by `radius + safety`, so a planned route is allowed to
# run right along that boundary. The lidar then sees the very rack the map has
# already accounted for -- and if the local planner demands another full
# `radius + safety` from that return, it rejects the route its own global
# planner just produced. Aisles a robot fits through become untraversable, and
# it stalls nose-first in the gap. So a scan return is rejected at the
# *physical* bound plus a small buffer, and the extra room is bought back by
# the clearance term in the score, which keeps the robot mid-aisle when there
# is room to be there.
#
# A peer is different: it is moving, its broadcast position is a cycle old, and
# nothing in the static map accounts for it. That one keeps the full margin.
SCAN_MARGIN = 0.06
PEER_MARGIN = 0.18


class Obstacle:
    """A point obstacle in world coordinates, from lidar or from a peer."""
    __slots__ = ("x", "y", "radius", "source", "margin")

    def __init__(self, x, y, radius=0.0, source="scan", margin=SCAN_MARGIN):
        self.x, self.y, self.radius, self.source = x, y, radius, source
        self.margin = margin


def scan_to_obstacles(x, y, yaw, ranges, angle_min, angle_increment,
                      range_max, stride=8):
    """Project a LaserScan into world-frame points.

    `stride` subsamples the beam list. A 674-beam Tugbot scan at 10 Hz is far
    more resolution than a 0.4 m-radius robot can act on, and the local planner
    cost is linear in the point count, so every 8th beam is plenty.
    """
    out = []
    n = len(ranges)
    for i in range(0, n, stride):
        r = ranges[i]
        if r is None or r != r:            # NaN
            continue
        if r <= 0.0 or r >= range_max * 0.999 or math.isinf(r):
            continue
        a = yaw + angle_min + i * angle_increment
        out.append(Obstacle(x + r * math.cos(a), y + r * math.sin(a)))
    return out


class LocalPlanner:
    """Dynamic Window Approach over the reachable ``(v, w)`` set.

    Every candidate is rolled forward `horizon` seconds and scored on three
    things: does it end up pointing at the carrot, does it keep clear of
    obstacles, and does it make progress. Candidates that would collide are
    discarded outright, which is what makes the collision avoidance a hard
    guarantee rather than a preference.
    """

    def __init__(self, grid, limits, v_samples=5, w_samples=11):
        self.grid = grid
        self.lim = limits
        self.v_samples = v_samples
        self.w_samples = w_samples
        # Progress is what the robot is for; clearance is a preference on top
        # of a guarantee, because a trajectory that would actually collide has
        # already been discarded before it is scored. Weighting clearance
        # heavily against an *admissible* trajectory only makes the fleet
        # crawl down aisles it has plenty of room in.
        self.weights = {"heading": 1.6, "clearance": 0.9, "velocity": 1.2,
                        "path": 1.8}
        self.last_reason = "idle"

    def _window(self, v, w):
        lim = self.lim
        dt = lim.dt
        v_lo = max(lim.v_min, v - lim.a_lin * dt * 3)
        v_hi = min(lim.v_max, v + lim.a_lin * dt * 3)
        w_lo = max(-lim.w_max, w - lim.a_ang * dt * 3)
        w_hi = min(lim.w_max, w + lim.a_ang * dt * 3)
        return v_lo, v_hi, w_lo, w_hi

    def _rollout(self, x, y, yaw, v, w):
        """Forward-simulate, sampling the result every other integration step.

        The integration has to be fine to stay accurate; the *collision check*
        does not, because consecutive samples are far closer together than the
        robot radius, so a sample pair cannot straddle an obstacle.
        """
        lim = self.lim
        steps = max(2, int(lim.horizon / lim.dt))
        poses = []
        for i in range(steps):
            yaw += w * lim.dt
            x += v * math.cos(yaw) * lim.dt
            y += v * math.sin(yaw) * lim.dt
            if i % 2 == 1 or i == steps - 1:
                poses.append((x, y, yaw))
        return poses

    def compute(self, x, y, yaw, v, w, carrot, obstacles, speed_cap=None):  # noqa: C901
        """Return ``(v_cmd, w_cmd)``.

        `speed_cap` is how the coordination layer throttles a robot without
        taking over steering -- it yields by easing off, not by freezing.
        """
        lim = self.lim
        clear_need = lim.radius + lim.safety
        cap = lim.v_max if speed_cap is None else max(0.0, min(lim.v_max, speed_cap))

        # Nothing outside this radius can be reached within the rollout, so it
        # cannot affect the answer. Filtering once here rather than inside the
        # inner loop is the difference between a control cycle that fits in
        # 100 ms on a Pi and one that does not.
        reach = abs(lim.v_max) * lim.horizon + lim.radius + PEER_MARGIN + 0.5
        obstacles = [ob for ob in obstacles
                     if abs(ob.x - x) < reach and abs(ob.y - y) < reach]
        v_lo, v_hi, w_lo, w_hi = self._window(v, w)
        v_hi = min(v_hi, cap)

        goal_dir = math.atan2(carrot[1] - y, carrot[0] - x)
        heading_err = abs(wrap(goal_dir - yaw))

        # A tight turn is not a driving problem, it is a pointing problem:
        # spinning on the spot beats arcing into the rack opposite.
        if heading_err > 1.15 and cap > 0.0:
            turn = math.copysign(min(lim.w_max, 0.9), wrap(goal_dir - yaw))
            if self._rollout_ok(x, y, yaw, 0.0, turn, obstacles, clear_need):
                self.last_reason = "turn-in-place"
                return 0.0, turn

        best = None
        best_score = -math.inf
        vs = [v_lo + (v_hi - v_lo) * i / (self.v_samples - 1)
              for i in range(self.v_samples)] if v_hi > v_lo else [v_hi]
        ws = [w_lo + (w_hi - w_lo) * i / (self.w_samples - 1)
              for i in range(self.w_samples)] if w_hi > w_lo else [w_lo]

        for cv in vs:
            if cv > cap:
                continue
            for cw in ws:
                poses = self._rollout(x, y, yaw, cv, cw)
                clearance = self._clearance(poses, obstacles, clear_need)
                if clearance is None:
                    continue
                ex, ey, eyaw = poses[-1]
                head = 1.0 - abs(wrap(math.atan2(carrot[1] - ey, carrot[0] - ex)
                                      - eyaw)) / math.pi
                gain = (math.hypot(carrot[0] - x, carrot[1] - y)
                        - math.hypot(carrot[0] - ex, carrot[1] - ey))
                # Normalise against the furthest the robot could possibly get,
                # so "made 90% of the available progress" scores the same
                # whatever the speed limit is, and a flat-out run down a clear
                # aisle actually wins.
                reachable = max(1e-6, lim.v_max * lim.horizon)
                score = (self.weights["heading"] * head
                         + self.weights["clearance"] * min(clearance, 1.5) / 1.5
                         + self.weights["velocity"] * (cv / lim.v_max if lim.v_max else 0)
                         + self.weights["path"] * max(-1.0, min(1.0, gain / reachable)))
                if score > best_score:
                    best_score, best = score, (cv, cw)

        if best is None:
            # Nothing in the dynamic window is safe. Decelerate hard; if we are
            # already stopped, back out slowly -- reversing out of a dead end is
            # a legitimate manoeuvre and the only one left here.
            self.last_reason = "blocked"
            if abs(v) > 0.05:
                return 0.0, 0.0
            if self._rollout_ok(x, y, yaw, lim.v_min, 0.0, obstacles, clear_need * 0.8):
                return lim.v_min, 0.0
            return 0.0, math.copysign(0.6, wrap(goal_dir - yaw) or 1.0)

        self.last_reason = "tracking"
        return best

    def _rollout_ok(self, x, y, yaw, v, w, obstacles, need):
        return self._clearance(self._rollout(x, y, yaw, v, w), obstacles, need) is not None

    def _clearance(self, poses, obstacles, need):
        """Smallest obstacle gap along a rollout, or None if it collides.

        `need` is only the *reporting* threshold now; each obstacle carries the
        margin it is actually entitled to.
        """
        worst = 100.0
        body = self.lim.radius
        grid = self.grid
        origin_x, origin_y, res = grid.origin_x, grid.origin_y, grid.resolution
        w_cells, h_cells, cells = grid.w, grid.h, grid.cells
        for (px, py, _yaw) in poses:
            col = int((px - origin_x) / res)
            row = int((py - origin_y) / res)
            if not (0 <= col < w_cells and 0 <= row < h_cells):
                return None
            if cells[row * w_cells + col]:
                return None                 # the static map already says no
            for ob in obstacles:
                dx = ob.x - px
                dy = ob.y - py
                d2 = dx * dx + dy * dy
                # Compare squared distances first; only pay for the square root
                # on the handful of obstacles that are actually close.
                limit = body + ob.margin + ob.radius
                if d2 < limit * limit:
                    return None
                if d2 < (worst + ob.radius) * (worst + ob.radius):
                    gap = math.sqrt(d2) - ob.radius
                    if gap < worst:
                        worst = gap
        return worst
