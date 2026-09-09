"""
Where the work is: pick faces, drop bays and the charger.

Rather than hand-listing coordinates that drift out of step with the world file
the moment a rack moves, stations are *derived* from the same layout the map is
built from. Every rack contributes pick faces along its long sides, offset into
the aisle by a robot's turning clearance and then snapped onto a cell the
planner can actually route to. A rack whose aisle is walled off by a structural
column contributes nothing, because none of its faces survive the snap.

No ROS imports.
"""

import math

APPROACH_M = 1.05        # how far into the aisle a robot parks to reach a face
FACE_SPACING_M = 3.0     # along a long rack, one face every few metres
SNAP_RADIUS_M = 1.6


class Station:
    __slots__ = ("name", "x", "y", "kind", "owner")

    def __init__(self, name, x, y, kind, owner=""):
        self.name, self.x, self.y, self.kind, self.owner = name, x, y, kind, owner

    def xy(self):
        return (self.x, self.y)

    def as_dict(self):
        return {"name": self.name, "x": round(self.x, 2), "y": round(self.y, 2),
                "kind": self.kind, "owner": self.owner}

    def __repr__(self):
        return f"<Station {self.name} ({self.x:.1f},{self.y:.1f}) {self.kind}>"


def _snap(grid, x, y):
    """Nearest free planning cell, or None if the spot is walled in."""
    col, row = grid.world_to_grid(x, y)
    if grid.at(col, row) == 0:
        return grid.grid_to_world(col, row)
    reach = int(math.ceil(SNAP_RADIUS_M / grid.resolution))
    best, best_d = None, None
    for dr in range(-reach, reach + 1):
        for dc in range(-reach, reach + 1):
            if grid.at(col + dc, row + dr):
                continue
            d = dc * dc + dr * dr
            if best_d is None or d < best_d:
                best, best_d = (col + dc, row + dr), d
    return grid.grid_to_world(*best) if best else None


def derive_stations(grid, layout, reachable_from=None):
    """Every pick face in the warehouse, as `Station`s the planner can reach.

    `reachable_from` is an optional ``(x, y)``; when given, faces that no route
    exists to are dropped rather than handed to the allocator as bids that can
    never be satisfied.
    """
    stations = []
    for box in layout["boxes"]:
        if box["model"] not in ("shelf", "shelf_big"):
            continue
        sx, sy, yaw = box["sx"], box["sy"], box["yaw"]
        # The long axis is the aisle-facing one; step along it, and offset
        # perpendicular to it on both sides.
        if abs(sx) >= abs(sy):
            long_len, half_short = abs(sx), abs(sy) / 2.0
            along = (math.cos(yaw), math.sin(yaw))
            across = (-math.sin(yaw), math.cos(yaw))
        else:
            long_len, half_short = abs(sy), abs(sx) / 2.0
            along = (-math.sin(yaw), math.cos(yaw))
            across = (math.cos(yaw), math.sin(yaw))

        count = max(1, int(long_len // FACE_SPACING_M))
        offset = half_short + APPROACH_M
        for i in range(count):
            t = (i + 0.5) / count - 0.5
            bx = box["x"] + along[0] * t * long_len
            by = box["y"] + along[1] * t * long_len
            for side, tag in ((1, "a"), (-1, "b")):
                px = bx + across[0] * offset * side
                py = by + across[1] * offset * side
                spot = _snap(grid, px, py)
                if spot is None:
                    continue
                stations.append(Station(f"{box['name']}.{i}{tag}",
                                        spot[0], spot[1], "pick", box["name"]))

    # Deduplicate: two racks facing the same aisle snap onto the same cell.
    unique, seen = [], set()
    for st in stations:
        key = grid.world_to_grid(st.x, st.y)
        if key in seen:
            continue
        seen.add(key)
        unique.append(st)

    if reachable_from is not None:
        from .navigation import AStar
        planner = AStar(grid)
        unique = [st for st in unique if planner.plan(reachable_from, st.xy())]
    return unique


def dock_stations(grid, layout):
    """Outbound bays along the building's open south wall."""
    x0, y0, x1, _y1 = layout["bounds"]
    out = []
    y = y0 + 1.6
    for i, x in enumerate([x0 + (x1 - x0) * f for f in (0.30, 0.42, 0.54, 0.66)]):
        spot = _snap(grid, x, y)
        if spot:
            out.append(Station(f"dock_{i}", spot[0], spot[1], "dock"))
    return out


def charger_station(grid, x=13.6, y=-10.6):
    """A parking spot in front of the Tugbot charging station model."""
    spot = _snap(grid, x, y)
    return Station("charger", spot[0], spot[1], "charger") if spot else None
