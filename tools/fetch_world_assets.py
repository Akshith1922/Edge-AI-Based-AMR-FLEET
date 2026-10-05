#!/usr/bin/env python3
"""
Derive the warehouse layout from the *actual* Gazebo Fuel models.

The simulation world is assembled from MovAi's published Fuel models, so the
only trustworthy source for "where are the walls and the racks" is those
models' own collision geometry. This script reads the world SDF, downloads the
collision description of every model it includes, and writes a single compact
JSON layout file.

    python3 tools/fetch_world_assets.py

Output: ros2_ws/src/warehouse_picker/config/warehouse_layout.json

That file is committed, so the map can be rebuilt on a machine with no network
access (a Raspberry Pi on the shop floor, for instance). Re-run this only when
the world SDF changes.

Only the Python standard library is used.
"""

import argparse
import json
import math
import re
import struct
import sys
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WORLD = ROOT / "ros2_ws" / "src" / "warehouse_picker" / "worlds" / "fleet_warehouse.sdf"
OUT = ROOT / "ros2_ws" / "src" / "warehouse_picker" / "config" / "warehouse_layout.json"
CACHE = ROOT / ".fuel-cache"

# Anything whose lowest point is above a robot's head, or that a robot drives
# straight over, is not an obstacle. The band is taken from the Tugbot's own
# 2D safety scanner height (0.14 m) up to the top of its chassis.
Z_BAND = (0.12, 0.9)


def fuel_file(owner, model, relpath):
    """Fetch one file from a Fuel model, memoised on disk."""
    cached = CACHE / owner / model / relpath
    if cached.exists():
        return cached.read_bytes()
    url = f"https://fuel.gazebosim.org/1.0/{owner}/models/{model}/tip/files/{relpath}"
    with urllib.request.urlopen(url, timeout=120) as r:
        data = r.read()
    cached.parent.mkdir(parents=True, exist_ok=True)
    cached.write_bytes(data)
    return data


def parse_uri(uri):
    """`https://<host>/1.0/<owner>/models/<model>` -> (owner, model)."""
    m = re.search(r"/([^/]+)/models/([^/\s]+)", uri.strip())
    if not m:
        raise ValueError(f"unrecognised model URI: {uri!r}")
    return m.group(1), m.group(2)


def parse_pose(text):
    """SDF pose string -> (x, y, z, roll, pitch, yaw), missing values zeroed."""
    vals = [float(v) for v in (text or "").split()]
    vals += [0.0] * (6 - len(vals))
    return vals[:6]


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


def stl_triangles(data):
    """Yield the vertices of every triangle in a binary or ASCII STL."""
    if data[:5] == b"solid" and b"facet" in data[:2048]:
        nums = []
        for line in data.decode("ascii", "replace").splitlines():
            line = line.strip()
            if line.startswith("vertex"):
                nums.append(tuple(float(v) for v in line.split()[1:4]))
        for i in range(0, len(nums) - 2, 3):
            yield nums[i], nums[i + 1], nums[i + 2]
        return
    count = struct.unpack("<I", data[80:84])[0]
    for i in range(count):
        off = 84 + 50 * i
        v = struct.unpack("<12fH", data[off:off + 50])
        yield ((v[3], v[4], v[5]), (v[6], v[7], v[8]), (v[9], v[10], v[11]))


def collision_shapes(owner, model):
    """Every collision in a model, as footprints in the *model* frame.

    Returns ``(boxes, mesh_refs)`` where a box is
    ``(cx, cy, sx, sy, yaw, zmin, zmax)`` and a mesh ref is the model-relative
    path of an STL the caller still has to rasterise.
    """
    sdf = ET.fromstring(fuel_file(owner, model, "model.sdf"))
    boxes, meshes = [], []
    for link in sdf.iter("link"):
        lx, ly, _lz, _lr, _lp, lyaw = parse_pose(link.findtext("pose"))
        for col in link.findall("collision"):
            cx, cy, cz, _r, _p, cyaw = parse_pose(col.findtext("pose"))
            geom = col.find("geometry")
            if geom is None:
                continue
            box = geom.find("box")
            if box is not None:
                sx, sy, sz = [float(v) for v in box.findtext("size").split()]
                ox, oy = rotate(cx, cy, lyaw)
                boxes.append((lx + ox, ly + oy, sx, sy, lyaw + cyaw,
                              cz - sz / 2.0, cz + sz / 2.0))
                continue
            cyl = geom.find("cylinder")
            if cyl is not None:
                rad = float(cyl.findtext("radius"))
                length = float(cyl.findtext("length"))
                ox, oy = rotate(cx, cy, lyaw)
                boxes.append((lx + ox, ly + oy, 2 * rad, 2 * rad, 0.0,
                              cz - length / 2.0, cz + length / 2.0))
                continue
            mesh = geom.find("mesh")
            if mesh is not None:
                meshes.append((mesh.findtext("uri").strip(), lx, ly, lyaw))
    return boxes, meshes


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", type=Path, default=WORLD)
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    world = ET.parse(args.world).getroot().find("world")
    layout = {
        "_source": str(args.world.relative_to(ROOT)),
        "_note": "Generated by tools/fetch_world_assets.py from the Fuel model "
                 "collision geometry. Do not hand-edit.",
        "z_band": list(Z_BAND),
        "boxes": [],       # {name, model, x, y, sx, sy, yaw}
        "wall_tris": [],   # flat [x1,y1,x2,y2,x3,y3] in world coordinates
        "robots": [],      # {name, x, y, yaw}
    }
    bounds = [math.inf, math.inf, -math.inf, -math.inf]

    for inc in world.findall("include"):
        name = (inc.findtext("name") or "").strip()
        uri = inc.findtext("uri") or ""
        wx, wy, _wz, _wr, _wp, wyaw = parse_pose(inc.findtext("pose"))
        try:
            owner, model = parse_uri(uri)
        except ValueError:
            print(f"  skip {name}: {uri!r}", file=sys.stderr)
            continue

        if model == "Tugbot":
            # The fleet itself: record the spawn pose, do not bake it into the
            # static map -- the robots are what the map is *for*.
            layout["robots"].append({"name": name, "x": round(wx, 3),
                                     "y": round(wy, 3), "yaw": round(wyaw, 4)})
            continue

        print(f"  {name:<24} <- {owner}/{model}")
        boxes, meshes = collision_shapes(owner, model)
        for (bx, by, sx, sy, byaw, zmin, zmax) in boxes:
            if zmax < Z_BAND[0] or zmin > Z_BAND[1]:
                continue
            ox, oy = rotate(bx, by, wyaw)
            cx, cy = wx + ox, wy + oy
            layout["boxes"].append({
                "name": name, "model": model,
                "x": round(cx, 3), "y": round(cy, 3),
                "sx": round(sx, 3), "sy": round(sy, 3),
                "yaw": round(byaw + wyaw, 4),
            })
            # Extent of the rotated rectangle, not a circumscribing circle:
            # an 18 m x 2.1 m rack is 10 m "wide" under the circle estimate and
            # would push the map bounds outside the building.
            c, s = abs(math.cos(byaw + wyaw)), abs(math.sin(byaw + wyaw))
            ex = (abs(sx) * c + abs(sy) * s) / 2.0
            ey = (abs(sx) * s + abs(sy) * c) / 2.0
            bounds[0] = min(bounds[0], cx - ex)
            bounds[1] = min(bounds[1], cy - ey)
            bounds[2] = max(bounds[2], cx + ex)
            bounds[3] = max(bounds[3], cy + ey)

        for (uri_rel, lx, ly, lyaw) in meshes:
            rel = uri_rel.replace("model://", "").lstrip("/")
            if not rel.lower().endswith(".stl"):
                print(f"    (collision mesh {rel} is not an STL; skipped)",
                      file=sys.stderr)
                continue
            kept = 0
            for tri in stl_triangles(fuel_file(owner, model, rel)):
                zs = [v[2] for v in tri]
                if max(zs) < Z_BAND[0] or min(zs) > Z_BAND[1]:
                    continue
                flat = []
                for (vx, vy, _vz) in tri:
                    ox, oy = rotate(lx + vx, ly + vy, wyaw)
                    px, py = wx + ox, wy + oy
                    flat.extend((round(px, 3), round(py, 3)))
                    bounds[0] = min(bounds[0], px)
                    bounds[1] = min(bounds[1], py)
                    bounds[2] = max(bounds[2], px)
                    bounds[3] = max(bounds[3], py)
                layout["wall_tris"].append(flat)
                kept += 1
            print(f"    {rel}: {kept} triangles cross the robot band")

    layout["bounds"] = [round(v, 3) for v in bounds]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(layout, indent=1))
    print(f"\n{args.out.relative_to(ROOT)}: "
          f"{len(layout['boxes'])} boxes, {len(layout['wall_tris'])} wall triangles, "
          f"{len(layout['robots'])} robots, bounds {layout['bounds']}")


if __name__ == "__main__":
    main()
