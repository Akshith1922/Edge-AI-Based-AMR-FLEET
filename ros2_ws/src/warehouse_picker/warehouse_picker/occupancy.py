"""
The warehouse map.

An occupancy grid rasterised from the world's real collision geometry, plus
the two things every consumer of a map actually needs: a distance field (how
far is this cell from the nearest rack?) and a ray caster (what would a lidar
beam hit from here?).

Two resolutions are used and they have different jobs:

* **0.05 m** -- the published map. This is what Nav2, RViz and a SLAM run
  compare against, and it is what the headless twin ray-casts for simulated
  lidar, because a 5 cm cell is finer than the sensor's own range noise.
* **0.25 m** -- the planning grid. A* on a 120x200 grid finishes in a few
  milliseconds on a Raspberry Pi; on a 600x1000 grid it does not. Coarsening
  is conservative: a coarse cell is occupied if *any* fine cell inside it is.

Standard library only.
"""

import base64
import json
import math
from collections import deque
from pathlib import Path

FREE = 0
OCCUPIED = 1

# Measured off the Tugbot's own collision mesh (meshes/base/tugbot_simp.stl):
# the chassis is 0.587 m long x 0.583 m wide and its furthest point sits 0.369 m
# from base_link. Rounded up to 0.40 m so the value covers the wheels, and the
# gripper arm that sticks out to 0.48 m behind is left to the safety margin --
# it trails the robot rather than leading it.
ROBOT_RADIUS = 0.40
SAFETY_MARGIN = 0.15


class GridMap:
    """Occupancy grid. Row 0 is the *bottom* (minimum y) row."""

    __slots__ = ("origin_x", "origin_y", "resolution", "w", "h", "cells",
                 "_dist", "clearance_cm")

    def __init__(self, origin_x, origin_y, resolution, w, h, cells=None):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.w, self.h = w, h
        self.cells = cells if cells is not None else bytearray(w * h)
        self._dist = None
        # Distance from each cell to the nearest *real* obstacle, in
        # centimetres, carried over from the full-resolution map. The planning
        # grid is inflated, so its own distance field measures the wrong thing;
        # this is what tells a robot whether an aisle is wide enough for two.
        self.clearance_cm = None

    # ---- coordinates -----------------------------------------------------

    def world_to_grid(self, x, y):
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def grid_to_world(self, col, row):
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def in_bounds(self, col, row):
        return 0 <= col < self.w and 0 <= row < self.h

    def at(self, col, row):
        if not self.in_bounds(col, row):
            return OCCUPIED           # outside the building is not drivable
        return self.cells[row * self.w + col]

    def mark(self, col, row, value=OCCUPIED):
        if self.in_bounds(col, row):
            self.cells[row * self.w + col] = value

    # ---- construction ----------------------------------------------------

    @classmethod
    def from_layout(cls, layout, resolution=0.05):
        x0, y0, x1, y1 = layout["bounds"]
        w = int(math.ceil((x1 - x0) / resolution))
        h = int(math.ceil((y1 - y0) / resolution))
        grid = cls(x0, y0, resolution, w, h)
        for box in layout["boxes"]:
            grid._fill_box(box["x"], box["y"], box["sx"], box["sy"], box["yaw"])
        for tri in layout["wall_tris"]:
            grid._fill_triangle((tri[0], tri[1]), (tri[2], tri[3]), (tri[4], tri[5]))
        return grid

    @classmethod
    def load_layout(cls, path, resolution=0.05):
        return cls.from_layout(json.loads(Path(path).read_text()), resolution)

    @classmethod
    def load_plan(cls, path):
        """Load the pre-inflated planning grid written by tools/build_map.py.

        Agents load this instead of rasterising the layout at boot: it is
        already coarsened and already inflated, so a robot is ready to plan in
        milliseconds rather than seconds.
        """
        d = json.loads(Path(path).read_text())
        grid = cls(d["origin_x"], d["origin_y"], d["resolution"], d["w"], d["h"],
                   bytearray(int(c) for c in d["cells"]))
        if d.get("clearance_cm"):
            grid.clearance_cm = bytearray(base64.b64decode(d["clearance_cm"]))
        return grid

    def free_width(self, x, y):
        """Metres of open floor around a world point, from the source map.

        Two robots need roughly ``2 * (radius + margin)`` to pass; anywhere
        this returns less than that is single-file and has to be coordinated
        rather than merely avoided.
        """
        if self.clearance_cm is None:
            return self.clearance(x, y) * 2.0
        col, row = self.world_to_grid(x, y)
        if not self.in_bounds(col, row):
            return 0.0
        return self.clearance_cm[row * self.w + col] / 100.0 * 2.0

    def connected_regions(self):
        """Free-space components, largest first, as lists of ``(col, row)``."""
        seen = bytearray(self.w * self.h)
        out = []
        for r0 in range(self.h):
            for c0 in range(self.w):
                i = r0 * self.w + c0
                if seen[i] or self.cells[i]:
                    continue
                seen[i] = 1
                queue = deque([(c0, r0)])
                cells = []
                while queue:
                    c, r = queue.popleft()
                    cells.append((c, r))
                    for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        nc, nr = c + dc, r + dr
                        if 0 <= nc < self.w and 0 <= nr < self.h:
                            j = nr * self.w + nc
                            if not seen[j] and not self.cells[j]:
                                seen[j] = 1
                                queue.append((nc, nr))
                out.append(cells)
        out.sort(key=len, reverse=True)
        return out

    def _fill_box(self, cx, cy, sx, sy, yaw):
        c, s = math.cos(-yaw), math.sin(-yaw)
        hx, hy = abs(sx) / 2.0, abs(sy) / 2.0
        ex = (abs(sx) * abs(math.cos(yaw)) + abs(sy) * abs(math.sin(yaw))) / 2.0
        ey = (abs(sx) * abs(math.sin(yaw)) + abs(sy) * abs(math.cos(yaw))) / 2.0
        c0, r0 = self.world_to_grid(cx - ex, cy - ey)
        c1, r1 = self.world_to_grid(cx + ex, cy + ey)
        for row in range(max(0, r0), min(self.h - 1, r1) + 1):
            for col in range(max(0, c0), min(self.w - 1, c1) + 1):
                wx, wy = self.grid_to_world(col, row)
                dx, dy = wx - cx, wy - cy
                # rotate the query point into the box frame
                if abs(dx * c - dy * s) <= hx and abs(dx * s + dy * c) <= hy:
                    self.cells[row * self.w + col] = OCCUPIED

    def _fill_triangle(self, a, b, c):
        """Rasterise a triangle *and* its edges.

        The edges matter more than the interior here: a vertical wall panel
        projects onto the floor plane as a zero-area line, so interior fill
        alone would draw nothing at all and the map would have no walls.
        """
        self._draw_segment(a, b)
        self._draw_segment(b, c)
        self._draw_segment(c, a)

        area = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
        if abs(area) < 1e-9:
            return
        c0, r0 = self.world_to_grid(min(a[0], b[0], c[0]), min(a[1], b[1], c[1]))
        c1, r1 = self.world_to_grid(max(a[0], b[0], c[0]), max(a[1], b[1], c[1]))
        for row in range(max(0, r0), min(self.h - 1, r1) + 1):
            for col in range(max(0, c0), min(self.w - 1, c1) + 1):
                px, py = self.grid_to_world(col, row)
                w0 = ((b[0] - a[0]) * (py - a[1]) - (px - a[0]) * (b[1] - a[1])) / area
                w1 = ((c[0] - b[0]) * (py - b[1]) - (px - b[0]) * (c[1] - b[1])) / area
                w2 = 1.0 - w0 - w1
                if w0 >= 0 and w1 >= 0 and w2 >= 0:
                    self.cells[row * self.w + col] = OCCUPIED

    def _draw_segment(self, p, q):
        length = math.hypot(q[0] - p[0], q[1] - p[1])
        steps = max(1, int(length / (self.resolution * 0.5)) + 1)
        for i in range(steps + 1):
            t = i / steps
            self.mark(*self.world_to_grid(p[0] + t * (q[0] - p[0]),
                                          p[1] + t * (q[1] - p[1])))

    # ---- derived products ------------------------------------------------

    def subsample(self, factor):
        """Lower-resolution copy sampled at each coarse cell's centre.

        Use this *after* inflating, never before. Inflating by `r` guarantees
        no free gap narrower than `2r` survives, so as long as the coarse cell
        is smaller than `2r` a centre sample cannot step over an obstacle --
        while the obvious alternative -- marking a coarse cell occupied if any
        fine cell in it is -- additionally rounds every rack outward by up to a
        full cell and closes aisles that are genuinely drivable. The 1.85 m
        aisle between the two western racks is exactly such a case.
        """
        w, h = self.w // factor, self.h // factor
        out = GridMap(self.origin_x, self.origin_y, self.resolution * factor, w, h)
        half = factor // 2
        for row in range(h):
            src = (row * factor + half) * self.w + half
            for col in range(w):
                out.cells[row * w + col] = self.cells[src + col * factor]
        return out

    def distance_field(self):
        """Metres from each cell to the nearest obstacle (two-pass chamfer).

        The 3-4 chamfer kernel is within ~2% of true Euclidean distance, which
        is far tighter than the clearance decisions it feeds.
        """
        if self._dist is not None:
            return self._dist
        big = float(self.w + self.h) * 2.0
        d = [0.0 if v else big for v in self.cells]
        w, h = self.w, self.h
        a, b = 1.0, 1.4142135623730951
        for row in range(h):
            base = row * w
            for col in range(w):
                i = base + col
                if d[i] == 0.0:
                    continue
                best = d[i]
                if col > 0:
                    best = min(best, d[i - 1] + a)
                if row > 0:
                    best = min(best, d[i - w] + a)
                    if col > 0:
                        best = min(best, d[i - w - 1] + b)
                    if col < w - 1:
                        best = min(best, d[i - w + 1] + b)
                d[i] = best
        for row in range(h - 1, -1, -1):
            base = row * w
            for col in range(w - 1, -1, -1):
                i = base + col
                if d[i] == 0.0:
                    continue
                best = d[i]
                if col < w - 1:
                    best = min(best, d[i + 1] + a)
                if row < h - 1:
                    best = min(best, d[i + w] + a)
                    if col > 0:
                        best = min(best, d[i + w - 1] + b)
                    if col < w - 1:
                        best = min(best, d[i + w + 1] + b)
                d[i] = best
        self._dist = [v * self.resolution for v in d]
        return self._dist

    def inflated(self, radius):
        """A copy with every cell within `radius` of an obstacle marked occupied."""
        dist = self.distance_field()
        out = GridMap(self.origin_x, self.origin_y, self.resolution, self.w, self.h,
                      bytearray(1 if v <= radius else 0 for v in dist))
        return out

    def clearance(self, x, y):
        """Metres from a world point to the nearest obstacle."""
        col, row = self.world_to_grid(x, y)
        if not self.in_bounds(col, row):
            return 0.0
        return self.distance_field()[row * self.w + col]

    def raycast(self, x, y, theta, max_range):
        """Distance to the first obstacle along a ray, or `max_range`.

        Amanatides & Woo grid traversal: one cell per iteration, no oversampling
        and no missed cells at shallow angles.
        """
        col, row = self.world_to_grid(x, y)
        if not self.in_bounds(col, row):
            return 0.0
        if self.cells[row * self.w + col]:
            return 0.0
        dx, dy = math.cos(theta), math.sin(theta)
        step_x = 1 if dx > 0 else -1
        step_y = 1 if dy > 0 else -1
        res = self.resolution
        inf = float("inf")
        if dx == 0:
            t_max_x, t_delta_x = inf, inf
        else:
            bound = self.origin_x + (col + (1 if dx > 0 else 0)) * res
            t_max_x = (bound - x) / dx
            t_delta_x = res / abs(dx)
        if dy == 0:
            t_max_y, t_delta_y = inf, inf
        else:
            bound = self.origin_y + (row + (1 if dy > 0 else 0)) * res
            t_max_y = (bound - y) / dy
            t_delta_y = res / abs(dy)

        while True:
            if t_max_x < t_max_y:
                if t_max_x > max_range:
                    return max_range
                col += step_x
                t = t_max_x
                t_max_x += t_delta_x
            else:
                if t_max_y > max_range:
                    return max_range
                row += step_y
                t = t_max_y
                t_max_y += t_delta_y
            if not self.in_bounds(col, row):
                return min(t, max_range)
            if self.cells[row * self.w + col]:
                return min(t, max_range)

    # ---- export ----------------------------------------------------------

    def save_pgm(self, path):
        """Nav2 / map_server format: 254 free, 0 occupied, image row 0 = max y."""
        path = Path(path)
        body = bytearray()
        for row in range(self.h - 1, -1, -1):
            off = row * self.w
            body.extend(bytes(0 if v else 254 for v in self.cells[off:off + self.w]))
        header = f"P5\n# warehouse_picker generated map\n{self.w} {self.h}\n255\n"
        path.write_bytes(header.encode("ascii") + bytes(body))
        return path

    def save_yaml(self, path, image_name, occupied_thresh=0.65, free_thresh=0.196):
        Path(path).write_text(
            f"image: {image_name}\n"
            f"resolution: {self.resolution}\n"
            f"origin: [{self.origin_x}, {self.origin_y}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: {occupied_thresh}\n"
            f"free_thresh: {free_thresh}\n"
            f"mode: trinary\n")
        return Path(path)

    def occupied_fraction(self):
        return sum(self.cells) / float(self.w * self.h)
