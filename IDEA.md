# Idea description

*Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots in
Smart Warehouses — Bharat Electronics Limited, Smart Automation.*

---

## The problem, in one line

A warehouse fleet that asks a central server which way to turn stops turning
the moment the server, the Wi-Fi, or the one aisle without coverage lets it
down — and the bigger the fleet, the worse it gets.

## The problem, properly

Warehouse AMR fleets are usually run from a central planner. Every robot sends
its position up, the server works out everyone's routes, and sends them back.
It works, until it doesn't:

- **Latency.** Every decision makes a round trip. At a blind corner a 200 ms
  round trip is 16 cm of travel the robot could not react in.
- **Dead zones.** Metal racking blocks Wi-Fi. A robot that loses contact mid-aisle
  either stops dead and blocks the aisle, or keeps going blind.
- **One failure stops everything.** The server is a single point of failure for
  a fleet of thirty robots.
- **It gets worse with scale.** Planning for N robots centrally grows faster
  than N, so the fleet gets less responsive exactly as the warehouse gets busier.

And the usual fallback when two robots meet — *stop and wait for the higher
priority one* — is itself a trap. Two robots that stop for each other never
move again.

## The idea

**Take the decision-making off the server and put it on each robot.**

Every AMR runs the whole decision stack on its own edge computer — a Raspberry
Pi or Jetson class board. There is no central planner. Each robot:

1. **Holds the map** of the warehouse and plans its own route around the racks.
2. **Sees for itself** with its laser scanner, so it avoids whatever the map
   doesn't know about — a dropped pallet, a person, another robot.
3. **Talks peer-to-peer** with the robots near it, sharing where it is, where
   it is going, and how much battery it has left.
4. **Bids for its own work** in an auction with its neighbours, with no
   auctioneer.
5. **Negotiates right of way** directly with whoever it meets.

Switch off any robot, or any part of the infrastructure, and the rest carry on.
There is nothing left to be a single point of failure.

## What makes it work: yielding without stopping

The heart of the idea is what a robot does when another one is in its way. The
textbook answer is to stop. Ours escalates, and almost never reaches the last step:

| | What the robot does | Cost |
|---|---|---|
| **1. Reroute** | The other robot's claimed cells become *expensive* to plan through, not blocked. Take the next aisle. | Neither robot slows down |
| **2. Throttle** | No reasonable detour: ease off to hold a time gap behind it, still rolling. | No stop-start, and already moving when it clears |
| **3. Give way** | A genuine head-on in an aisle too narrow to pass: reverse into the nearest wide spot. | One robot loses a few metres |

Priority *ages upward* while a robot is waiting, so a robot that keeps losing
eventually wins — nobody is starved. And underneath all of it sits a watchdog:
a robot that has not actually *been anywhere* for eight seconds backs off and
replans whatever the reason, because on a real floor most jams have no name.

## How collisions become impossible rather than unlikely

The layers are strictly ordered, and each can only constrain the one below it:

```
task allocation    which pallet am I fetching?
coordination       whose corridor is this?
global planning    which aisles get me there?
local planning     what is in front of me right now?
```

The coordination layer **cannot steer**. It can make map cells expensive and it
can cap speed, and that is all it can do. The bottom layer throws away every
candidate movement that would hit something — checked against both the map and
the live laser — *before* it picks one.

So a bug in the clever part can make the fleet slow or silly. It cannot drive a
robot into a rack. Safety does not depend on the coordination being correct.

## What we built and measured

A working simulation of six Tugbots in a 30 × 50 m warehouse, running in
Gazebo under ROS 2, with a live fleet dashboard.

The map is not drawn by hand — it is generated from the actual collision shapes
of the warehouse and racks, and exported in the standard Nav2 format. It is
honest about what it finds: it reports that one 1.85 m aisle has structural
columns standing in it and no robot can pass, so the planner refuses to route
there.

Against an uncoordinated fleet on the identical workload — same map, same
planner, same sensors, same jobs, **only the conflict resolution differs**:

| Situation | Stop-and-wait | This | |
|---|---|---|---|
| Rush hour: all traffic through two aisles | 6 of 12 jobs | **11 of 12** | **70% faster** |
| A pallet dropped in an aisle, unmapped | 4 of 12 jobs | **11 of 12** | **94% faster** |
| Collisions, every run, both arms | 0 | **0** | |

Both targets met: **zero inter-robot collisions**, and far more than the 20%
reduction in task completion time.

Verified running in Gazebo, not just in theory: the robots acquire their correct
positions, plan, drive, deliver, and pick up new work, and across 55 ground-truth
samples the closest any two came was 0.92 m — against the 0.80 m at which they
would touch.

## One honest finding

When the work is spread out and the aisles are empty, coordination makes almost
no difference. We can show exactly why: a tool in the project measures the
warehouse and reports that **not one cell of its floor is too narrow for two
robots to pass**. With no chokepoints there is nothing to coordinate.

That is a fact about the building, not about the software, and it is worth
saying out loud: this layer earns its keep when the floor is busy or something
goes wrong, which is precisely when a warehouse can least afford a fleet that
seizes up.

## Why it is feasible

- The route planner runs in **single-digit milliseconds** on a 120 × 200 grid;
  the obstacle avoidance is a few thousand arithmetic operations per cycle.
  Both are sized for a Raspberry Pi 4 running at 10 Hz. No GPU, no server.
- It uses **ROS 2 and Gazebo**, the standard robotics stack, and the Tugbot's
  own existing sensors. Nothing exotic to buy.
- The decision-making code imports nothing robotics-specific, so the same code
  that runs on the robot runs in a laptop simulator and under unit test. There
  are **143 automated tests**, none of which need a simulator.
- It degrades sensibly: lose the network and robots fall back on their own
  sensors; lose a robot and its job returns to the pool within its lease.

## Impact

- **Throughput** — up to 70–94% more work done per shift when the floor is
  congested, which is when it matters.
- **Resilience** — no server to lose, no dead zone that stops a robot, no single
  failure that stops a fleet.
- **Safety** — collision avoidance that is structural, not a matter of the
  coordination behaving.
- **Scale** — adding a robot adds its own compute. The fleet gets more capable
  as it grows instead of less responsive.
- **Defence and strategic relevance** — the same decentralised, edge-resident,
  infrastructure-free coordination applies directly to unmanned ground vehicle
  teams operating where there is no network to depend on.
