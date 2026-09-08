"""
Warehouse model: a static occupancy grid, named zones, pick faces, parking
bays, and automatically-derived single-file corridors.

Layout (flow left to right, matching the shop-floor described in the report):

    RECEIVING -> STORAGE (5 rack banks, 2-wide aisles) -> PICK/PACK -> SHIPPING
                        |                                      |
                  central corridor with two pinch points   CHARGING BAYS

Coordinates are ``(x, y)`` with the origin top-left so grid space and screen
space agree, which keeps the renderer trivial.
"""

from collections import deque
from enum import IntEnum

WIDTH = 40
HEIGHT = 24


class Tile(IntEnum):
    FREE = 0
    RACK = 1      # shelving; impassable, but tasks pick from its aisle face
    WALL = 2      # structural wall / pinch point


class Zone:
    """A named rectangular region, used for task generation and spatial hashing."""

    __slots__ = ("name", "label", "x0", "y0", "x1", "y1", "kind")

    def __init__(self, name, label, x0, y0, x1, y1, kind):
        self.name, self.label, self.kind = name, label, kind
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1

    def contains(self, x, y):
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1

    def cells(self):
        return [(x, y) for x in range(self.x0, self.x1 + 1)
                for y in range(self.y0, self.y1 + 1)]

    def as_dict(self):
        return {"name": self.name, "label": self.label, "kind": self.kind,
                "x0": self.x0, "y0": self.y0, "x1": self.x1, "y1": self.y1}


class Corridor:
    """A run of single-file cells that can only be traversed one way at a time."""

    __slots__ = ("id", "cells", "ends")

    def __init__(self, cid, cells, ends):
        self.id = cid
        self.cells = cells          # ordered list, ends[0] -> ends[1]
        self.ends = ends            # the two mouth cells just outside the run

    def direction_towards(self, cell):
        """+1 if `cell` is on the ends[1] side, -1 if on the ends[0] side."""
        return 1 if cell == self.ends[1] else -1

    def as_dict(self):
        return {"id": self.id, "cells": [list(c) for c in self.cells],
                "ends": [list(e) for e in self.ends]}


# Two banks of five racks, separated by *single-file* picking aisles — the
# realistic warehouse geometry, and the reason coordination is needed at all:
# every interior aisle is a chokepoint that two robots cannot share.
RACK_BANDS = ((4, 10), (13, 19))          # y ranges of the two rack banks
RACK_COLUMNS = (4, 10, 16, 22, 28)        # x start of each 5-wide rack block
RACK_W = 5
# Structural pillars that pinch the central east-west corridor to one lane.
PINCHES = (((11, 11), (12, 11), (13, 11)),      # -> single file along y = 12
           ((24, 12), (25, 12), (26, 12)))      # -> single file along y = 11


class Warehouse:
    def __init__(self):
        self.w, self.h = WIDTH, HEIGHT
        self.tiles = [[Tile.FREE] * self.h for _ in range(self.w)]

        for x0 in RACK_COLUMNS:
            for (y0, y1) in RACK_BANDS:
                self._fill(x0, y0, x0 + RACK_W - 1, y1, Tile.RACK)
        for pinch in PINCHES:
            for (x, y) in pinch:
                self.tiles[x][y] = Tile.WALL

        self.zones = {
            z.name: z for z in (
                Zone("receiving", "Receiving", 0, 4, 2, 19, "inbound"),
                Zone("storage_w", "Storage West", 4, 4, 20, 19, "storage"),
                Zone("storage_e", "Storage East", 22, 4, 32, 19, "storage"),
                Zone("pickpack", "Pick / Pack", 36, 4, 39, 10, "outbound"),
                Zone("shipping", "Shipping", 36, 13, 39, 19, "outbound"),
                Zone("charging", "Charging", 2, 21, 13, 23, "service"),
            )
        }

        self.adj = {(x, y): tuple(self._neighbors_of(x, y))
                    for x in range(self.w) for y in range(self.h)
                    if self.tiles[x][y] == Tile.FREE}
        self.pick_faces = self._compute_pick_faces()
        self.parking_bays = [(x, 22) for x in range(3, 15, 2)]
        from .config import DEFAULT as _cfg
        self.corridors = self._compute_corridors(_cfg.CORRIDOR_MIN_LEN)
        self.corridor_of = {c: cor.id for cor in self.corridors for c in cor.cells}
        # the aisle used by the scripted "blocked aisle" fault
        self.fault_aisle = [(15, y) for y in range(4, 11)]

    # ------------------------------------------------------------------ grid
    def _fill(self, x0, y0, x1, y1, tile):
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                if self.in_bounds(x, y):
                    self.tiles[x][y] = tile

    def in_bounds(self, x, y):
        return 0 <= x < self.w and 0 <= y < self.h

    def is_walkable(self, x, y):
        return self.in_bounds(x, y) and self.tiles[x][y] == Tile.FREE

    def _neighbors_of(self, x, y):
        for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
            nx, ny = x + dx, y + dy
            if self.is_walkable(nx, ny):
                yield (nx, ny)

    def neighbors(self, x, y):
        return self.adj.get((x, y), ())

    def free_neighbors(self, cell):
        return list(self.adj.get(cell, ()))

    def zone_of(self, x, y):
        for name, z in self.zones.items():
            if z.contains(x, y):
                return name
        return "aisle"

    def manhattan(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # ------------------------------------------------------- derived features
    def _compute_pick_faces(self):
        """Free aisle cells that touch a rack — the cells a task picks from."""
        faces = []
        for x in range(self.w):
            for y in range(self.h):
                if self.tiles[x][y] != Tile.FREE:
                    continue
                touching = any(self.in_bounds(x + dx, y + dy)
                               and self.tiles[x + dx][y + dy] == Tile.RACK
                               for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)))
                if touching:
                    faces.append((x, y))
        return faces

    def _compute_corridors(self, min_len=2):
        """Find every straight single-file run of free cells.

        A cell is *narrow* when it has exactly two walkable neighbours and
        those neighbours are opposite each other — i.e. two robots physically
        cannot pass. Connected narrow cells form one corridor, which
        Algorithm 1's chokepoint lock then serialises.
        """
        narrow = set()
        for x in range(self.w):
            for y in range(self.h):
                if not self.is_walkable(x, y):
                    continue
                nbs = self.free_neighbors((x, y))
                if len(nbs) != 2:
                    continue
                (ax, ay), (bx, by) = nbs
                if (ax + bx) // 2 == x and (ay + by) // 2 == y and (ax, ay) != (bx, by):
                    narrow.add((x, y))

        corridors, seen, cid = [], set(), 0
        for cell in sorted(narrow):
            if cell in seen:
                continue
            run, q = [], deque([cell])
            seen.add(cell)
            while q:
                cur = q.popleft()
                run.append(cur)
                for nb in self.neighbors(*cur):
                    if nb in narrow and nb not in seen:
                        seen.add(nb)
                        q.append(nb)
            if len(run) < min_len:
                continue
            ordered = self._order_run(run)
            ends = self._mouths(ordered)
            if len(ends) != 2:
                continue
            corridors.append(Corridor(cid, ordered, ends))
            cid += 1
        return corridors

    @staticmethod
    def _order_run(run):
        cells = set(run)
        adj = {c: [n for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]),
                               (c[0], c[1] + 1), (c[0], c[1] - 1)) if n in cells]
               for c in run}
        start = next((c for c in sorted(run) if len(adj[c]) == 1), sorted(run)[0])
        ordered, prev, cur = [start], None, start
        while True:
            nxt = next((n for n in adj[cur] if n != prev), None)
            if nxt is None or nxt in ordered:
                break
            ordered.append(nxt)
            prev, cur = cur, nxt
        return ordered

    def _mouths(self, ordered):
        """The walkable cells immediately outside each end of the run."""
        run = set(ordered)
        ends = []
        for end in (ordered[0], ordered[-1]):
            outs = [n for n in self.neighbors(*end) if n not in run]
            if len(outs) == 1:
                ends.append(outs[0])
        return ends

    # ------------------------------------------------------------- utilities
    def nearest_free(self, origin, predicate, max_radius=12):
        """BFS outward from `origin` for the closest cell satisfying `predicate`."""
        seen, q = {origin}, deque([(origin, 0)])
        while q:
            cell, d = q.popleft()
            if d > max_radius:
                return None
            if cell != origin and predicate(cell):
                return cell
            for nb in self.neighbors(*cell):
                if nb not in seen:
                    seen.add(nb)
                    q.append((nb, d + 1))
        return None

    def static_layout(self):
        """Compact description of the immutable map, sent once to the dashboard."""
        return {
            "w": self.w, "h": self.h,
            "tiles": [[int(self.tiles[x][y]) for x in range(self.w)] for y in range(self.h)],
            "zones": [z.as_dict() for z in self.zones.values()],
            "corridors": [c.as_dict() for c in self.corridors],
            "parking": [list(p) for p in self.parking_bays],
        }
