#!/usr/bin/env python3
"""
Build the warehouse map from the extracted layout.

    python3 tools/build_map.py

Writes, into ros2_ws/src/warehouse_picker/maps/:

    warehouse.pgm / warehouse.yaml   Nav2 + map_server, 5 cm cells
    warehouse_map.png                human-readable preview
    warehouse_plan.json              0.25 m planning grid, what the agents load

and verifies that every fleet spawn pose in the world SDF actually stands in
free space with room for the robot.

Standard library only.
"""

import argparse
import base64
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "ros2_ws" / "src" / "warehouse_picker"
sys.path.insert(0, str(PKG))

from warehouse_picker.imaging import Canvas                    # noqa: E402
from warehouse_picker.occupancy import (                       # noqa: E402
    GridMap, ROBOT_RADIUS, SAFETY_MARGIN)

FREE_RGB = (247, 247, 244)
WALL_RGB = (38, 42, 52)
HALO_RGB = (255, 214, 165)
SPAWN_RGB = (34, 139, 230)
GRID_RGB = (223, 223, 216)


def render_preview(fine, inflated, layout, path, scale=2):
    """Preview at `scale` screen pixels per map cell, drawn top-down."""
    w, h = fine.w // scale, fine.h // scale
    canvas = Canvas(w, h, FREE_RGB)
    for sy in range(h):
        row = fine.h - 1 - sy * scale          # image row 0 is max y
        for sx in range(w):
            col = sx * scale
            if fine.at(col, row):
                canvas.set(sx, sy, WALL_RGB)
            elif inflated.at(col, row):
                canvas.set(sx, sy, HALO_RGB)

    # a 5 m reference grid, so distances are readable off the picture
    step = int(round(5.0 / fine.resolution / scale))
    for gx in range(0, w, step):
        for gy in range(0, h, 3):
            if canvas.rows[gy][3 * gx:3 * gx + 3] == bytes(FREE_RGB):
                canvas.set(gx, gy, GRID_RGB)
    for gy in range(0, h, step):
        for gx in range(0, w, 3):
            if canvas.rows[gy][3 * gx:3 * gx + 3] == bytes(FREE_RGB):
                canvas.set(gx, gy, GRID_RGB)

    for robot in layout["robots"]:
        col, row = fine.world_to_grid(robot["x"], robot["y"])
        sx, sy = col // scale, (fine.h - 1 - row) // scale
        canvas.disc(sx, sy, max(2, int(ROBOT_RADIUS / fine.resolution / scale)), SPAWN_RGB)
    canvas.save(path)
    return w, h


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--plan-resolution", type=float, default=0.25)
    ap.add_argument("--layout", type=Path, default=PKG / "config" / "warehouse_layout.json")
    ap.add_argument("--out", type=Path, default=PKG / "maps")
    ap.add_argument("--ascii", action="store_true", help="print an ASCII preview")
    args = ap.parse_args()

    layout = json.loads(args.layout.read_text())
    args.out.mkdir(parents=True, exist_ok=True)

    print(f"layout: {len(layout['boxes'])} rack/box footprints, "
          f"{len(layout['wall_tris'])} wall triangles, bounds {layout['bounds']}")

    fine = GridMap.from_layout(layout, args.resolution)
    print(f"map:    {fine.w} x {fine.h} cells @ {args.resolution} m  "
          f"({fine.w * fine.resolution:.1f} x {fine.h * fine.resolution:.1f} m), "
          f"{fine.occupied_fraction() * 100:.1f}% occupied")

    fine.save_pgm(args.out / "warehouse.pgm")
    fine.save_yaml(args.out / "warehouse.yaml", "warehouse.pgm")

    clearance = ROBOT_RADIUS + SAFETY_MARGIN
    inflated = fine.inflated(clearance)
    print(f"inflated by {clearance:.2f} m -> {inflated.occupied_fraction() * 100:.1f}% "
          f"of the floor is off-limits to a robot centre")

    # Inflate at full resolution, *then* drop to the planning resolution.
    # Doing it the other way round rounds every rack outward by up to one
    # coarse cell before the safety margin is even added, which walls off
    # aisles a robot can actually drive down.
    factor = int(round(args.plan_resolution / args.resolution))
    plan = inflated.subsample(factor)
    # Sample the *uninflated* distance field at each planning cell, so agents
    # can tell a two-robot aisle from a single-file one at runtime.
    fine_dist = fine.distance_field()
    clearance_cm = bytearray(plan.w * plan.h)
    half = factor // 2
    for row in range(plan.h):
        src = (row * factor + half) * fine.w + half
        for col in range(plan.w):
            clearance_cm[row * plan.w + col] = min(
                255, int(round(fine_dist[src + col * factor] * 100)))

    plan_path = args.out / "warehouse_plan.json"
    plan_path.write_text(json.dumps({
        "origin_x": plan.origin_x, "origin_y": plan.origin_y,
        "resolution": plan.resolution, "w": plan.w, "h": plan.h,
        "cells": "".join(str(v) for v in plan.cells),
        "clearance_cm": base64.b64encode(bytes(clearance_cm)).decode("ascii"),
    }))
    print(f"plan:   {plan.w} x {plan.h} cells @ {plan.resolution} m -> "
          f"{plan_path.name} ({plan_path.stat().st_size / 1024:.0f} KB)")

    regions = plan.connected_regions()
    reachable = len(regions[0])
    free = sum(1 for v in plan.cells if not v)
    print(f"reach:  {reachable}/{free} free cells ({reachable / free * 100:.1f}%) "
          f"are one connected floor")
    for cells in regions[1:]:
        xs = [plan.grid_to_world(c, r)[0] for c, r in cells]
        ys = [plan.grid_to_world(c, r)[1] for c, r in cells]
        print(f"        unreachable pocket of {len(cells)} cells at "
              f"x[{min(xs):.1f},{max(xs):.1f}] y[{min(ys):.1f},{max(ys):.1f}] "
              f"-- too narrow for a robot, usually a structural column in an aisle")

    pw, ph = render_preview(fine, inflated, layout, args.out / "warehouse_map.png")
    print(f"png:    {pw} x {ph} preview -> warehouse_map.png")

    ok = True
    print("\nspawn poses:")
    for robot in layout["robots"]:
        gap = fine.clearance(robot["x"], robot["y"])
        verdict = "ok" if gap >= clearance else "TOO TIGHT"
        if gap < clearance:
            ok = False
        print(f"  {robot['name']:<8} ({robot['x']:>7.2f}, {robot['y']:>7.2f})  "
              f"clearance {gap:5.2f} m  [{verdict}]")

    if args.ascii:
        print()
        cols, rows = 76, 46
        for r in range(rows - 1, -1, -1):
            line = []
            for c in range(cols):
                col = int(c / cols * fine.w)
                row = int(r / rows * fine.h)
                blocked = any(fine.at(col + dc, row + dr)
                              for dc in range(0, max(1, fine.w // cols))
                              for dr in range(0, max(1, fine.h // rows)))
                line.append("#" if blocked else ".")
            print("".join(line))

    if not ok:
        print("\nAt least one spawn pose does not fit. Move it in the world SDF.",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
