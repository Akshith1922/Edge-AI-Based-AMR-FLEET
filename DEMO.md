# Running the demo

Three ways to show the fleet, in order of how much hardware they need.

---

## 1. No simulator: the headless twin (any laptop, ~40 s)

The twin runs the **same `EdgeAgent` code** the robots run, against the same
warehouse map, with lidar synthesised by ray-casting the occupancy grid. It is
the fastest way to show the behaviour and the only way to run the same workload
twice under two different coordination policies.

```bash
python3 tools/twin.py --robots 6 --scenario rush_hour --tasks 12
```

```bash
# fleet vs the uncoordinated control arm, identical tasks and seeds
python3 tools/twin.py --compare --robots 6 --scenario crossing

# how the advantage grows with congestion
python3 tools/twin.py --sweep 3,4,5,6 --scenario crossing
```

### A playback page you can hand to someone

```bash
python3 tools/twin.py --robots 6 --scenario rush_hour --tasks 12 \
        --trace results/trace.json
python3 tools/make_demo.py results/trace.json -o results/fleet_demo.html
```

One self-contained HTML file — map embedded, trace inline, no server and no
network. Play, scrub, and step through the run; each robot shows its route,
its battery, and in plain words why it is doing what it is doing.

---

## 2. The real thing: Gazebo + ROS 2

```bash
cd ros2_ws
colcon build --packages-select warehouse_picker
source install/setup.bash
ros2 launch warehouse_picker fleet_warehouse.launch.py
```

Then open **<http://localhost:8080>** for the live fleet dashboard.

The first launch pulls the MovAi models from Fuel and takes a minute or two;
after that they are cached. The launch file waits 8 s for Gazebo before
starting the bridge and 12 s before the agents, which is enough on a warm
cache — raise the `TimerAction` periods in `launch/fleet_warehouse.launch.py`
if your machine is slower.

Useful arguments:

```bash
ros2 launch warehouse_picker fleet_warehouse.launch.py \
    robots:=6 \
    policy:=stop_and_wait \      # the uncoordinated control arm, for contrast
    gui:=false \                 # headless, much faster
    max_speed:=0.8 tasks:=12
```

### What to point at while it runs

| Watch | Where |
|---|---|
| Robots taking *different* aisles to the same area | Gazebo viewport, or the dashboard's dashed route lines |
| One robot easing off behind another instead of stopping dead | dashboard, "throttled behind amr_N" |
| Two robots passing without either stopping | the northern hall, where there is room for two |
| The give-way manoeuvre | a south aisle, "retreating to passing bay" |
| Work being auctioned with nobody in charge | kill the dispatcher: `ros2 node kill /task_dispatcher` — the fleet finishes everything already issued |
| No single point of failure | kill any agent; the rest re-bid its task within the 25 s lease |

### Checking the plumbing

```bash
ros2 topic hz /amr_1/scan          # ~10 Hz: the lidar bridge is up
ros2 topic echo /fleet/mesh --once # one robot's full broadcast
ros2 topic hz /amr_1/cmd_vel       # ~10 Hz: the agent is commanding
gz topic -l | grep amr_1           # what Gazebo is actually publishing
```

If a robot never moves, check `/amr_1/odom` first. No odometry means the bridge
did not attach, which almost always means the entity in Gazebo is not called
`amr_1` — spawning the same name twice is what renames it.

---

## 3. Mapping the warehouse with SLAM

The map in `maps/` is derived from the world's collision geometry, so it is
exact. To build the map a *robot* would, with the occlusions and drift a real
deployment has to live with:

```bash
ros2 launch warehouse_picker slam_mapping.launch.py
# in another terminal, drive amr_1 around:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
# when the map looks complete:
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_slam
```

Needs `sudo apt install ros-$ROS_DISTRO-slam-toolbox`. Compare
`~/warehouse_slam.pgm` against `maps/warehouse.pgm` — same resolution, same
origin, so they overlay directly.

---

## Rebuilding the map

Only needed after editing the world file:

```bash
python3 tools/fetch_world_assets.py   # re-read the Fuel collision geometry
python3 tools/build_map.py            # rasterise, inflate, export, verify
```

`build_map.py` verifies every spawn pose has room for a robot and reports the
connected components of the floor, so a rack moved onto a spawn point or an
aisle closed by a new obstacle shows up immediately rather than as a robot
that will not start.

---

## Talking points

**Why there is no central planner.** Each robot plans its own route, bids for
its own work, and decides for itself when to give way. `/fleet/mesh` carries
position, intent, battery and who each robot is waiting for; DDS delivers it
peer-to-peer with no broker. The dispatcher only *creates* jobs — kill it and
the fleet keeps working.

**Why yielding is not stopping.** A robot gives way by rerouting if there is
another aisle, by easing off to hold a time gap if there is not, and only
reverses to a passing bay for a true head-on in a single-file aisle. Stopping
dead is the last resort, because two robots that stop for each other never
move again — which is exactly what the control arm does, and why it gridlocks.

**Why collisions are structurally impossible, not merely unlikely.** The local
planner discards every candidate trajectory that would collide, against both
the static map and the live lidar, before any of them is scored. Coordination
can only make cells expensive and cap speed. So a coordination bug can make the
fleet slow or silly; it cannot drive a robot into a rack.

**Where the edge hardware fits.** A* over the 120×200 planning grid runs in
single-digit milliseconds and the local planner is a few thousand floating-point
operations per cycle — both sized for a Raspberry Pi 4 at 10 Hz. The map is
25 cm and pre-inflated for the same reason. Nothing here needs a GPU or a
connection to a server.
