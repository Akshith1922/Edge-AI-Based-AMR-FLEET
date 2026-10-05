# Quickstart — get the fleet running

Plain steps, in order. Copy and paste each block.

---

## What you're getting

Your three tugbots now drive themselves. Each robot:

1. **Knows the warehouse** — there's a real map built from the actual shelf and
   wall shapes in your Gazebo world.
2. **Sees** — it reads the Tugbot's own front laser scanner, so it stops for
   things the map doesn't know about (a dropped pallet, a person, another robot).
3. **Plans its own route** — A\* over the map, so it goes *around* racks instead
   of into them.
4. **Talks to the other robots directly** — no server in the middle. They share
   position, where they're going, and battery, and sort out who goes first.
5. **Picks its own jobs** — the robots bid against each other for tasks.

And there's a web dashboard showing all of it live.

---

## Step 0 — get the code

```bash
cd ~/Edge-AI-Based-AMR-FLEET      # wherever you cloned it
git fetch origin
git checkout claude/inspiring-ride-n6hwod
git pull
```

---

## Step 1 — check your machine

```bash
bash tools/doctor.sh
```

It prints `OK` or `--` for each thing you need, and gives you the exact
`sudo apt install ...` line for anything missing. Fix those, run it again.

---

## Step 2 — warm up the model download (do this once)

The first time, Gazebo downloads the warehouse and Tugbot models from the
internet. That can take a few minutes, and if it's still downloading when the
robots start, things look broken when they aren't.

Get it out of the way first:

```bash
gz sim -s -r ros2_ws/src/warehouse_picker/worlds/fleet_warehouse.sdf
```

Wait until the log stops scrolling (a minute or two), then press **Ctrl+C**.
From now on it starts in seconds.

---

## Step 3 — build and run

```bash
cd ros2_ws
colcon build --packages-select warehouse_picker
source install/setup.bash
ros2 launch warehouse_picker fleet_warehouse.launch.py
```

Then open **<http://localhost:8080>** in your browser.

That's it. Gazebo opens, six robots appear, jobs get handed out, and the robots
start driving.

---

## Step 4 — what you should see

**In Gazebo:** robots driving up and down the aisles, going around the racks,
slowing down or stepping aside for each other.

**In the browser:** each robot's position on the map, its planned route as a
dashed line, its battery, what it's doing, and in plain words *why* — like
`throttled behind amr_3` or `retreating to passing bay`.

**In the terminal**, within about 15 seconds of launching:

```
[amr_1] edge agent online: 120x200 map @ 0.25 m, policy=cooperative
[amr_1] odometry acquired; world pose (-2.90, -21.00, 90 deg)
dispatcher: 73 reachable pick faces, keeping 12 jobs in flight
```

If you see those three lines, everything is working.

---

## Useful variations

```bash
# fewer robots, no Gazebo window (much faster on a weak machine)
ros2 launch warehouse_picker fleet_warehouse.launch.py robots:=3 gui:=false

# run the OLD dumb behaviour, to show the difference side by side
ros2 launch warehouse_picker fleet_warehouse.launch.py policy:=stop_and_wait

# more jobs, slower robots
ros2 launch warehouse_picker fleet_warehouse.launch.py tasks:=20 max_speed:=0.5
```

---

## Demo without Gazebo

If the demo machine is weak, or Gazebo won't cooperate on the day, this runs
the **same robot code** with no simulator, no GPU and nothing installed:

```bash
python3 tools/twin.py --robots 6 --scenario rush_hour --tasks 12
```

Make a shareable page of a run:

```bash
python3 tools/twin.py --robots 6 --scenario rush_hour --tasks 12 \
        --trace results/trace_demo.json
python3 tools/make_demo.py results/trace_demo.json -o results/fleet_demo.html
```

`results/fleet_demo.html` is one file. Email it, put it on a USB stick, open it
on any laptop. It plays the whole run back with play/pause and a scrubber.

---

## The numbers to quote

Run this to reproduce them live:

```bash
python3 tools/twin.py --compare --robots 6 --scenario rush_hour
python3 tools/twin.py --compare --robots 6 --scenario blocked_aisle
```

| Situation | Old way (stop-and-wait) | This | |
|---|---|---|---|
| Rush hour, all traffic through two aisles | 6 of 12 jobs done | **11 of 12** | **70% faster** |
| Someone drops a pallet in an aisle | 4 of 12 jobs done | **11 of 12** | **94% faster** |
| Collisions, every run, both | 0 | **0** | |

**Say this honestly if asked:** when the work is spread out and the aisles are
empty, coordination makes almost no difference — this warehouse's aisles are
all wide enough for two robots to pass, so there's nothing to coordinate.
It pays off when the floor is busy. `python3 tools/twin.py --measure-corridors`
shows exactly that, and it's a stronger answer than pretending otherwise.

---

## If something goes wrong

Run these three, in this order. The first one that's wrong is your problem.

```bash
ros2 topic hz /amr_1/odom     # should be ~20 Hz
ros2 topic hz /amr_1/scan     # should be ~10 Hz
ros2 topic hz /amr_1/cmd_vel  # should be ~10 Hz
```

| What's wrong | What it means | Fix |
|---|---|---|
| `/amr_1/odom` silent | Gazebo isn't running, or the robot isn't called `amr_1` | `gz topic -l \| grep amr_1` — if you see `amr_1_0`, something spawned it twice |
| `/amr_1/scan` silent | The laser isn't publishing | Check the world file still has the `gz-sim-sensors-system` plugin. Without it there is no laser at all |
| `/amr_1/cmd_vel` silent | The agent isn't deciding anything | Look at the agent's terminal output — usually it never got odometry |
| All three fine, robots don't move | The agent has no job | Is the dispatcher running? `ros2 node list` |
| Robots start at the wrong place | Odometry isn't being offset by the spawn pose | Check `config/warehouse_layout.json` lists your robot names |
| Gazebo is very slow | Software rendering | `gui:=false`, and `robots:=3` |

### If you're on Gazebo Fortress

`bash tools/doctor.sh` tells you. Fortress uses `ign gazebo` instead of
`gz sim`, and older plugin names. Two changes:

```bash
sed -i 's/gz-sim-/ignition-gazebo-/g; s/gz::sim::systems::/ignition::gazebo::systems::/g' \
  ros2_ws/src/warehouse_picker/worlds/fleet_warehouse.sdf
sed -i 's/"gz", "sim", "-r"/"ign", "gazebo", "-r"/' \
  ros2_ws/src/warehouse_picker/launch/fleet_warehouse.launch.py
```

Installing Gazebo Garden or Harmonic instead is the better answer if you can.

---

## If you change the warehouse

Moved a rack? Added a shelf? Rebuild the map:

```bash
python3 tools/fetch_world_assets.py   # re-read the shapes from the model files
python3 tools/build_map.py            # rebuild the map
```

`build_map.py` checks every robot's start position still has room, and tells
you if any part of the floor became unreachable.

---

## Where things are

```
ros2_ws/src/warehouse_picker/
  warehouse_picker/
    agent_core.py    the robot's brain: what it decides, every 0.1 s
    navigation.py    route planning (A*) and steering
    protocol.py      what robots tell each other, and who gives way
    allocation.py    how jobs get shared out
    occupancy.py     the map
    edge_fleet_agent.py   connects the brain to ROS
    fleet_dashboard.py    the web page
    task_dispatcher.py    hands out jobs
  worlds/  launch/  maps/  config/

tools/
  doctor.sh          check this machine
  build_map.py       rebuild the map
  twin.py            run the fleet with no simulator
  make_demo.py       turn a run into a shareable web page
```

Everything you'd want to tune is a named constant near the top of its file,
with a comment saying why it's that value.

- Robot speed, size, safety margin → `navigation.py` (`Limits`), `occupancy.py`
- How close robots get, when they give way → `protocol.py`
- Battery, replanning rate, when to charge → `agent_core.py`

---

## Two things worth knowing for questions

**"Is it really decentralised?"** Yes. ROS 2 has no central broker — messages
go straight from robot to robot. Kill any node, including the job dispatcher,
and the rest carry on. Try it live: `ros2 node kill /task_dispatcher`, and the
fleet finishes everything already handed out.

**"How do you know they won't crash?"** The local planner throws away any
movement that would hit something — checked against both the map and the live
laser — *before* it picks one. So the coordination layer can only make the fleet
slow or silly; it can't drive a robot into a rack. Zero collisions in every run,
including the uncoordinated control arm.
