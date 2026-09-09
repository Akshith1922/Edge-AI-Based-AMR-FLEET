# warehouse_picker — decentralised AMR fleet on Gazebo

A ROS 2 package that makes the Tugbot fleet in the MovAi warehouse drive
itself: it maps the building, plans around the racks, avoids everything the
lidar can see, and coordinates with its neighbours without a central server.

```bash
cd ros2_ws
colcon build --packages-select warehouse_picker
source install/setup.bash
ros2 launch warehouse_picker fleet_warehouse.launch.py     # then open :8080
```

---

## What was wrong before, and what fixed it

| Symptom | Cause | Fix |
|---|---|---|
| Robots drove through shelves | The agent subscribed to no sensor at all — it knew only what peers broadcast | `edge_fleet_agent` bridges and consumes the Tugbot's own `scan_front` lidar; `LocalPlanner` rejects any trajectory that would collide |
| Robots ignored `cmd_vel` | The world SDF *and* the launch file both spawned `amr_1..3`; Gazebo renamed the duplicates, and a renamed entity publishes on `/model/amr_1_0/...` | Robots are spawned only by the world file, and the bridge names are derived from the world name |
| The patrol route crossed racks | A hard-coded ±6/±5 rectangle, which passes straight through `shelf_1` at (−4.4, 5.3) and `shelf_4` at (5.6, 5.3) | Routes come from A* over a map built from the racks' own collision geometry |
| Dashboard permanently blank | It subscribed to `/fleet/p2p_mesh` expecting `target_x` / `battery`; the agent published to `/fleet/heartbeat` with neither | Both ends encode and decode with `protocol.FleetState`, so the schema cannot drift |
| Robots froze around each other | Conflict resolution was a hard stop for any peer within 2 m, on a fixed id priority | Yield escalates reroute → throttle → retreat, and priority ages upward while blocked |
| Robots stopped on the pick faces | Nothing ever told an idle robot to move | Idle robots return to a standby bay |
| World only ran on one Gazebo version | `ignition::gazebo::systems::*` plugin names hard-coded at world level | No world-level plugins; Gazebo loads its own version-correct defaults |

---

## The map

`maps/` is generated, not drawn. `tools/fetch_world_assets.py` reads the world
SDF, downloads the collision geometry of every model it includes, and writes
`config/warehouse_layout.json`:

* the warehouse building, from `warehouse_colision.stl`, keeping the 582
  triangles that cross the 0.12–0.9 m band a robot actually occupies;
* every rack, from its own `<collision><box>` — `shelf` is 3.6 × 0.6 m,
  `shelf_big` is 2.1 × 18 m, both offset −0.5 m in x by their link pose.

`tools/build_map.py` rasterises that into three products:

| File | Resolution | For |
|---|---|---|
| `warehouse.pgm` + `.yaml` | 5 cm | Nav2, RViz, comparison against a SLAM run |
| `warehouse_plan.json` | 25 cm, pre-inflated | what the agents load — A* on 120×200 finishes in single-digit milliseconds on a Pi |
| `warehouse_map.png` | preview | the dashboard background |

![The generated warehouse map](maps/warehouse_map.png)

Dark is obstacle, orange is the 0.55 m no-go halo around it (0.40 m robot
radius, measured off the Tugbot's own chassis mesh, plus 0.15 m margin), blue
dots are the fleet spawn poses.

**Inflate first, coarsen second.** Coarsening before inflating rounds every
rack outward by up to a full 25 cm cell *before* the safety margin is added,
which walls off aisles a robot can genuinely drive down. `subsample()` samples
each coarse cell's centre from the already-inflated fine map instead; that is
safe because inflating by `r` leaves no free gap narrower than `2r`, so a
centre sample cannot step over an obstacle.

**The map reports what it finds, including bad news.** `build_map.py` prints
the connected components of free space, and on this warehouse it reports:

```
reach:  15932/16006 free cells (99.5%) are one connected floor
        unreachable pocket of 74 cells at x[-8.1,-7.6] y[-14.4,-8.1]
```

That pocket is real. The aisle between `shelf_big_0` and `shelf_big_4` is
1.85 m wide, but the building has structural columns at x ≈ −7.5 every 7.5 m —
the orange pillars visible in the Gazebo viewport — and they close it. The
planner refuses routes into it, which is the correct answer.

### Mapping it for real, with SLAM

The generated map is exact, which also makes it a description of the
*simulator* rather than of a warehouse. To build the map a real robot would:

```bash
ros2 launch warehouse_picker slam_mapping.launch.py
# drive amr_1 around with teleop, then:
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_slam
```

Needs `ros-$ROS_DISTRO-slam-toolbox`. Having both is the point: the generated
map is ground truth to score the SLAM map against.

---

## Architecture

```
   task allocation    which pallet am I fetching?      allocation.py
   coordination       whose corridor is this?          protocol.py
   global planning    which aisles get me there?       navigation.AStar
   local planning     what is in front of me now?      navigation.LocalPlanner
```

Each layer can only *constrain* the one below it, never reach past it. The
coordination layer cannot steer — it can make cells expensive and it can cap
speed, and that is deliberately all it can do. So a coordination bug can make
the fleet slow or silly, but it cannot drive a robot into a rack.

None of `agent_core`, `navigation`, `protocol`, `allocation`, `occupancy` or
`stations` imports ROS. `edge_fleet_agent.py` is a thin shim that turns topics
into four inputs and one output. That is what lets the same code run under the
headless twin and under `python3 -m unittest` with no simulator at all.

### Nodes and topics

| Node | Subscribes | Publishes |
|---|---|---|
| `edge_agent` (one per robot) | `/<id>/odom`, `/<id>/scan`, `/fleet/mesh`, `/fleet/tasks` | `/<id>/cmd_vel`, `/fleet/mesh`, `/fleet/tasks`, `/fleet/telemetry` |
| `dispatcher` | `/fleet/tasks` | `/fleet/tasks` |
| `dashboard` | `/fleet/mesh`, `/fleet/telemetry` | HTTP on :8080 |

`/fleet/mesh` is best-effort with a shallow queue: a late heartbeat is worse
than no heartbeat, and a robot that blocks trying to deliver one to a peer that
has gone offline is exactly the single point of failure the design exists to
avoid. `/fleet/tasks` is `TRANSIENT_LOCAL`, so a robot that joins late inherits
the outstanding work.

**On "decentralised".** ROS 2 has no broker: DDS discovers peers over multicast
and every message goes publisher-to-subscriber, so a topic here really is a
peer-to-peer mesh. Kill any node, including the dispatcher, and the rest keep
running — the dispatcher only *creates* work, it never assigns it. What this
does not claim is that the robots would find each other across a routed network
without multicast; that is a DDS discovery-server or Zenoh bridge at deploy
time, not a change to any code here.

---

## Coordination: what "yield" means

The obvious implementation — stop dead when a peer is close — is what the
previous agent did, and it is both the slow option and a deadlock: two robots
that stop for each other never move again. Here a robot yields in three
escalating ways and normally never reaches the third.

1. **Reroute.** A peer's claimed cells become *expensive* to plan through, not
   impassable. If another aisle is within the detour bound the yielding robot
   goes around and neither robot slows down at all.
2. **Throttle.** In a single-file corridor, the yielding robot caps its
   speed to hold a time gap behind the peer. It keeps rolling, so it does
   not pay the decelerate-and-reaccelerate cost, and it is already moving
   when the corridor clears. Only in a corridor: on open floor the local
   planner already holds a safe gap from real geometry rather than from a
   broadcast position a cycle old, and an earlier version that threw a
   speed cap at every claim overlap anywhere spent four times longer
   yielding than the uncoordinated control arm while delivering less.
3. **Retreat.** Only for a genuine head-on in an aisle too narrow to pass in:
   the lower-ranked robot reverses to the nearest bay wide enough, which it
   knows from the clearance layer carried in the planning grid.

Rank is `(priority, −lamport, id)` compared as a tuple, and priority **ages
upward while a robot is yielding**, so a robot that keeps losing eventually
wins. Starvation is bounded rather than merely unlikely.

Where there *is* room for two, both robots simply move over to an agreed side
and neither slows. That only works while there is still room to move over into,
though: once two robots are inside each other's clearance envelope no aim-point
nudge can help, because the obstacle is the other robot. Below 1.9 m someone
gives way instead.

Deadlock is caught by walking the wait-for chain published in each heartbeat.
Every robot in a cycle sees the same cycle and computes the same loser, so
exactly one backs off — no negotiation round trip, and no chance of all of them
backing off at once.

### Liveness: the stalls the protocol cannot name

Coordination resolves the deadlocks it can identify. The interesting failures
are the ones it cannot, and on a real floor those are the majority: a peer
parked across a nose, a pallet the map does not know about, two robots each
politely waiting for the other. With no supervisor to come and untangle them,
one unnamed stall is permanent, so there is a backstop underneath everything
else.

A robot that has a goal and has not *been anywhere* for 8 seconds backs off and
replans, whatever the reason. Three details make that work rather than make
things worse:

* **Stalling is measured by displacement, not speed.** A robot shuffling back
  and forth in a jam clears any speed threshold repeatedly while going nowhere,
  so a speed test resets on its own twitching and the stall never registers.
* **It is measured at the top of the control loop.** Checking it at the end of
  the path-following branch means every early return (loading, retreating, no
  route) skips it, and a robot wedged inside one of those branches reports that
  it has never stalled.
* **One robot in a knot backs off at a time.** Three robots that all reverse at
  once are exactly as jammed, only further apart. Robots broadcast whether they
  are stalled; the lowest-ranked stalled robot in the cluster goes and the rest
  hold, by the same total order used everywhere else.

After three failed escapes the goal itself is the problem, usually a pick face
another robot is parked on, so the task goes back to the pool for whoever is
better placed. That is the re-allocation half of dynamic re-routing, triggered
by the robot's own experience rather than by anything telling it the aisle is
blocked.

### Idle robots go to standby

A robot with nothing to do used to stop exactly where it delivered, which is a
pick face or a drop bay: the two busiest places on the floor. It then sat there
as an obstacle nobody could negotiate with, because it never yields, and every
other robot had to route around it for the rest of the shift. Idle robots now
return to their standby bay, and can accept work on the way there.

This is worth more than it sounds. Adding it took the fleet from moving 41% of
the time to 63%, and dropped the share of decisions spent in stall-recovery
machinery from 27% to 8%: a bigger effect than any change to the coordination
rules themselves, because it removes congestion that no amount of coordination
*between the working robots* could have fixed.


### Task allocation

A sealed-bid auction with no auctioneer. Robots bid true route cost; every
robot applies the same tie-break to the same bids and reaches the same winner,
so the winner finds out at the same instant everyone else does.

Claims are **not** shared mutable state. An earlier version kept a
`task → holder` map that everyone merged, and it diverged exactly as you would
expect: two replicas award the same task to different robots, each overwrites
the other next gossip round, and a robot halfway through a delivery discovers
it no longer owns the job and restarts from the pickup. Tasks ping-ponged and
none completed. Now a robot is authoritative for one thing only — what *it* is
doing — and publishes that in its heartbeat. Everything else is derived, and
two robots that briefly claim the same task resolve it on a total order over
ids.

---

## Results

`tools/twin.py --compare` runs the identical workload twice, once under each
policy, with the same seed, the same tasks and the same spawn poses. The
control arm is the same map, the same A\*, the same lidar and the same local
planner; only conflict resolution differs, and it is given the
timeout-and-backoff a real stop-and-wait system has, because without it the
scheme simply gridlocks and beating it proves nothing.

Six Tugbots, twelve pick-and-drop tasks, 900-second window:

| Scenario | | stop-and-wait | cooperative | |
|---|---|---|---|---|
| **Rush hour** — every pickup at a northern rack face, every drop at the southern docks, so the whole fleet funnels through two aisles at once | delivered | 6 / 12 | **11 / 12** | |
| | time to deliver the same 6 | 834.4 s | **250.8 s** | **−69.9%** |
| | fleet time spent yielding | 2435 s | **924 s** | |
| | inter-robot collisions | 0 | **0** | |
| **Blocked aisle** — the same, plus a 1.3 m pallet stack dropped into one of the two aisles at t=45 s, leaving a 1.2 m gap. It exists only in the lidar, never in the map | delivered | 4 / 12 | **11 / 12** | |
| | time to deliver the same 4 | 854.0 s | **46.6 s** | **−94.5%** |
| | fleet time spent yielding | 2687 s | **268 s** | |
| | inter-robot collisions | 0 | **0** | |
| | robots that hit the pallet | 0 | **0** | |
| **Crossing** — pickups and drops scattered across the building | delivered | 12 / 12 | 12 / 12 | |
| | makespan | 231.1 s | **223.9 s** | −3.1% |
| | inter-robot collisions | 0 | **0** | |

Zero collisions in every run of both arms, which is what the local planner
guarantees — it is not a property of the coordination layer and the control
arm gets it too.

### Why the scenarios differ so much, and why that is the interesting part

Coordination is worth 70% under congestion and 3% when the floor is quiet.
That is not a scenario chosen to flatter it; it follows from the building:

```
$ python3 tools/twin.py --measure-corridors
  two robots need 1.35 m to pass
  single-file floor: 0 cells (0.0%)
    2 m    3.2%
    3 m    3.2%
    4 m   27.1%
    5 m   66.5%
```

**Not one cell of this warehouse is single-file.** Every aisle takes two
Tugbots abreast, so there is no chokepoint for give-way, throttling or passing
bays to manage, and on a scattered workload the coordinated fleet is doing
little the uncoordinated one is not. Contention here has to come from
somewhere else — from density, from all the work being at one end of the
building, or from something dropped in an aisle.

Rush hour supplies it by funnelling twelve deliveries through two aisles to
four dock points. There the uncoordinated fleet spends **2435 robot-seconds
frozen** — more than a third of all available fleet time — and delivers half
the work in the same window. The blocked aisle supplies it a second way, by
narrowing a 3.8 m aisle to a 1.2 m gap that one robot fits through and two do
not; the uncoordinated fleet manages four deliveries and spends 2687 seconds
stopped. Those are the failure modes the brief describes, and they are what the
coordination layer exists to prevent.

Worth noting separately: **neither arm ever touched the pallet**, in any run.
Avoiding an obstacle nobody put on the map is the local planner's job, and it
does it whether or not the robots are coordinating. What coordination changes
is what happens when six robots want the same 1.2 m gap.

The honest summary is that this warehouse is a generous one, and a fleet twice
this size, or a narrower building, is where the layer earns its keep every
day rather than only at rush hour.

### Reproducing

```bash
python3 tools/twin.py --compare --robots 6 --scenario rush_hour --tasks 12
python3 tools/twin.py --compare --robots 6 --scenario crossing  --tasks 12
python3 tools/twin.py --sweep 3,4,5,6 --scenario crossing        # density curve
python3 tools/twin.py --measure-corridors                        # the layout fact
```

Each `--compare` takes a few minutes on a laptop and needs nothing installed.

---


## Launch arguments

```bash
ros2 launch warehouse_picker fleet_warehouse.launch.py \
    robots:=6 policy:=cooperative gui:=true max_speed:=0.8 tasks:=12
```

`policy:=stop_and_wait` swaps in the uncoordinated control arm — same planner,
same lidar, same tasks, naive conflict resolution — which is what the benchmark
measures against.

## Tuning

Everything worth changing is a module-level constant with the reasoning next
to it: `protocol.py` for contention geometry and ageing, `agent_core.py` for
battery, replanning and bid cadence, `navigation.Limits` for the kinematic
envelope, `occupancy.py` for the robot radius and safety margin.
