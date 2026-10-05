#!/usr/bin/env python3
"""
Switch the world and launch files between Gazebo naming conventions.

Gazebo Fortress calls its systems `ignition-gazebo-*` and runs as `ign gazebo`.
Garden and later call them `gz-sim-*` and run as `gz sim`. The files here are
written for Garden and later, which is what `bash tools/doctor.sh` checks for.

    python3 tools/set_gazebo_flavour.py            # report which is set
    python3 tools/set_gazebo_flavour.py fortress   # switch to ign / ignition-gazebo
    python3 tools/set_gazebo_flavour.py garden     # switch back to gz / gz-sim

Only the plugin attributes and the launch commands are touched -- never the
comments explaining them, which is why this exists instead of a sed one-liner.
Running it twice changes nothing the second time.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "ros2_ws" / "src" / "warehouse_picker"
WORLD = PKG / "worlds" / "fleet_warehouse.sdf"
LAUNCHES = sorted((PKG / "launch").glob("*.launch.py"))

# (garden, fortress) pairs, applied only inside the constructs named below.
PLUGIN_FILE = ("gz-sim-", "ignition-gazebo-")
PLUGIN_NAME = ("gz::sim::systems::", "ignition::gazebo::systems::")
COMMAND = ('"gz", "sim", "-r"', '"ign", "gazebo", "-r"')


def swap(text, pairs, to_fortress):
    for garden, fortress in pairs:
        src, dst = (garden, fortress) if to_fortress else (fortress, garden)
        text = text.replace(src, dst)
    return text


def rewrite_world(to_fortress):
    text = WORLD.read_text()

    def fix_attr(match):
        return swap(match.group(0), [PLUGIN_FILE, PLUGIN_NAME], to_fortress)

    # Only inside filename="..." / name="..." attributes, so the comment that
    # quotes Gazebo's own server.config output is left saying what it says.
    new = re.sub(r'(?:filename|name)="[^"]*"', fix_attr, text)
    changed = new != text
    if changed:
        WORLD.write_text(new)
    return changed


def rewrite_launches(to_fortress):
    touched = []
    for path in LAUNCHES:
        text = path.read_text()
        new = swap(text, [COMMAND], to_fortress)
        if new != text:
            path.write_text(new)
            touched.append(path.name)
    return touched


def current():
    text = WORLD.read_text()
    plugins = re.findall(r'filename="([^"]+)"', text)
    if any(p.startswith("ignition-gazebo-") for p in plugins):
        return "fortress"
    if any(p.startswith("gz-sim-") for p in plugins):
        return "garden"
    return "unknown"


def main():
    if not WORLD.exists():
        raise SystemExit(f"world file not found: {WORLD}")

    if len(sys.argv) == 1:
        now = current()
        cmd = "ign gazebo" if now == "fortress" else "gz sim"
        print(f"currently set for: {now}  (launches with `{cmd}`)")
        print("pass 'fortress' or 'garden' to switch")
        return 0

    target = sys.argv[1].lower()
    if target not in ("fortress", "garden", "harmonic"):
        raise SystemExit("usage: set_gazebo_flavour.py [fortress|garden]")
    to_fortress = target == "fortress"

    if current() == ("fortress" if to_fortress else "garden"):
        print(f"already set for {target}; nothing to do")
        return 0

    world_changed = rewrite_world(to_fortress)
    launches = rewrite_launches(to_fortress)
    print(f"switched to {target}")
    print(f"  {WORLD.relative_to(ROOT)}: "
          f"{'plugin names rewritten' if world_changed else 'unchanged'}")
    for name in launches:
        print(f"  launch/{name}: gazebo command rewritten")
    if not launches:
        print("  launch files: unchanged")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
