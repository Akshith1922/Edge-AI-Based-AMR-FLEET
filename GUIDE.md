# Guide

Everything you need to run this, demo it, defend it in a review, and extend it.

---

## 1. Run it

No dependencies. Python 3.10+.

```bash
cd Edge-AI-Based-AMR-FLEET

python3 run_dashboard.py                     # opens http://127.0.0.1:8000
python3 run_benchmark.py --trials 8          # writes results/report.html
python3 -m unittest discover -s tests -t .   # 59 tests, ~7 seconds
```

Useful flags:

```bash
python3 run_dashboard.py --scenario failure_storm --robots 10 --speed 10
python3 run_dashboard.py --mode baseline --port 9000 --no-open
python3 run_benchmark.py --scenario chokepoint_duel --robots 10 --ticks 800
```

If port 8000 is taken, pass `--port`. The dashboard polls `/api/state` about
ten times a second; the simulation runs on its own thread, so pausing the
browser does not pause the fleet and vice versa.

---

## 2. A five-minute demo script

This is the order that makes the system explain itself.

**1. Start on `Cross-Aisle Rush Hour`, coordinated, speed 6.**
Point at the floor: the dark-blue vertical strips are single-file aisles — two
robots physically cannot pass in one. Watch the *Chokepoints* panel: `▶`/`◀`
shows which way each aisle is currently flowing and who is queued.

**2. Watch the event log.** You will see, in order: tasks released, auctions
won (`T58 auctioned to robot 8 (3 bids)`), pickups, deliveries, and
occasionally `3-way conflict cluster [6, 3, 7] — robot 6 wins`. That last line
is Algorithm 1 doing its job: an n-way cluster, ranked with the documented
rule, winner keeps its reservation.

**3. Tick the "Reservation heat" box.** The orange wash is the fleet's claimed
space-time over the next dozen ticks. That is the shared structure everything
else is built on.

**4. Click a floor cell in a busy aisle.** A dark red square appears — that is
the *physical* obstacle. Nothing happens yet, because nobody has seen it. When
a robot's scan covers it for `CONFIRM_WINDOW` ticks it turns into a red ✕
(a gossiped `BlockEvent`), the log says `confirmed a block ... gossiped`, and
every robot whose committed path crossed it replans immediately. Click the same
cell again to lift the obstacle, and watch a passing robot reprobe and retract
the block.

**5. Click a robot.** Its heartbeat stops. Nothing happens for 15 ticks —
that is the TTL, and it is deliberate: declaring a failure on one missed
message would be worse than waiting. At TTL the log fires
`declared FAILED`, its cell becomes a static block, its task returns to the
pool, and a new auction hands the task to somebody else. If it was carrying an
item you also get `ALERT ITEM_LOCATION_UNKNOWN`.

**6. Now hit `Baseline` and do the same things.** Same map, same tasks, same
faults. Watch three things change:
* **Hard stops** climbs steadily (coordinated stays at 0). Robots discover each
  other by arriving at the same cell and emergency-stopping.
* **Robots waiting** (bottom chart) goes from near-flat to a picket fence.
* **Reassign lag** is 45 instead of 15 — a fixed timeout instead of a heartbeat.
* All six algorithm chips go dark. Baseline uses none of them.

**7. Finish on the benchmark.** `python3 run_benchmark.py --trials 8` and open
`results/report.html`. Lead with the safety line ("no collision in any trial,
in either mode"), then completion rate and hard stops, then be upfront about
mean task time and distance — see §5.

---

## 3. Reading the dashboard

**Floor**
| | |
|---|---|
| Grey blocks | Racks |
| Dark blue strips | Single-file aisles (chokepoints) |
| Coloured rounded squares | Robots, numbered; the bar on top is battery |
| Amber ring | Waiting or queued · **Violet ring** in deadlock resolution · **Red ring** failed |
| Small amber square, top-right of a robot | Carrying an item |
| Thin coloured line | That robot's committed path |
| Amber dot / hollow violet ring | Task pickup / drop-off |
| Dark red square | Physical obstacle, not yet detected |
| Red ✕ | Confirmed block event (gossiped); faded ✕ = softened to UNCONFIRMED |
| Dashed cyan line through an aisle | Current chokepoint flow direction |

**Tiles.** `Collisions` is the safety invariant and must stay 0. `Hard stops`
counts *unplanned* stops — the number coordination is meant to drive to zero.
`Reassign lag` is the mean ticks from a robot going silent to its task being
back in the pool.

**Algorithms firing.** Lit = that algorithm did something this tick. A1 and A2
run constantly; A4 lights when a block is confirmed, softened or cleared; A5 on
a failure or lease expiry; A6 on an auction; A3 only when a wait-graph cycle or
the liveness watchdog fires (rare — see §5).

---

## 4. Where each algorithm lives

Start with `amrsim/engine.py::Simulation.step()` — it is fifteen lines and the
tick order is the architecture:

```python
self._heartbeat_pass()      # Algorithm 5   agree on who is alive
self._failure_pass()        # Algorithm 5   TTL + lease expiry
self._sensing_pass()        # Algorithm 4   detect / confirm / clear blocks
self._allocation_pass()     # Algorithm 6   CRDT pool + time-boxed auction
self._target_pass()         #               pickup / drop-off / park
self._planning_pass()       # Algorithms 1+2+4  rank, then plan in rank order
self._movement_pass()       #               execute one step, resolve chains
self._deadlock_pass()       # Algorithm 3   wait graph, escape, shuffle
self._collision_check()     #               the invariant
self._completion_pass()     #               phase transitions, delivery
```

Sense and agree on the world first, then allocate work, then plan, then move,
then repair. Reading the passes in order is reading the design.

Then read, in this order:

1. `reservations.py` — the shared substrate. `Reservation.steps` is the whole
   idea: *cell i is claimed at tick t0+i*, not "this robot owns this corridor".
2. `planner.py::plan_spacetime` — the search. Note the `cooperative` flag: the
   window is the safety horizon.
3. `engine.py::_planning_pass` and `_plan_still_valid` — where ranking becomes
   resolution. A robot keeps its path until somebody who outranks it has taken
   space it needs; the loser is the one replanning.
4. Then `tasks.py`, `failures.py`, `blocks.py`, `deadlock.py`, `corridors.py`
   in any order — each is self-contained and maps to one chapter.

Every tunable is in `config.py`, named after the reference's constant.

---

## 5. Answering hard questions

**"Why is `deadlock_events` zero? Did you implement Algorithm 3?"**
Yes — `deadlock.py`, and it is wired into `_deadlock_pass`. It is zero because
the space-time planner cannot *commit* a head-on path in the first place, so
the cycles Algorithm 3 detects almost never form. That is the correct outcome:
prevention beating cure. It stays in because prevention has holes — a robot
that fails inside an aisle, or one boxed in by an obstacle, still produces a
wait chain. `tests/test_algorithms.py::TestAlgorithm3Deadlock` exercises cycle
detection, the escape search and the shuffle chain directly, and the
`aisle_gridlock` scenario drives it from the engine.

**"Coordinated travels further and its mean task time is sometimes worse."**
True, and the benchmark prints it. Congestion avoidance costs distance — that
is what the `ALPHA` term buys. What you get for it: the whole workload finished
every seed, zero emergency stops, a tighter P90, and 3× faster failure
recovery. The mean-time comparison is density dependent and the README prints the whole
curve: baseline is ~5% ahead at 6 robots, the gap closes by 10, and coordinated
is ahead at 12 — because baseline's hard stops grow from 31 to 78 per run while
coordinated's stay at zero. Completion rate favours coordination at every
density. `run_benchmark.py --robots N` reproduces it. Lower `ALPHA` in
`config.py` to trade congestion avoidance for distance.

**"Isn't the baseline unfairly weak / unfairly strong?"**
Baseline is "safety-only", exactly as the report describes: independent
shortest-path A\*, emergency-stop on contact, fixed-timeout random backoff,
central nearest-idle dispatch, no gossip (it discovers an obstacle only by
arriving next to it), and a 45-tick failure timeout. It is *not* crippled — it
gets the same collision-free movement arbiter, which is why it also scores zero
collisions. The difference the benchmark measures is coordination quality, not
safety.

**"Why does the chokepoint lock not block cells?"**
It used to, and it made things worse — see the comment block at the top of
`corridors.py`. Reservations already make head-on entry infeasible; hard-gating
corridor cells on top of that only removed the planner's ability to express
"wait two ticks and go through", which is usually the cheapest option. The lock
now supplies what reservations cannot: FIFO fairness (queue time becomes a
priority bonus, so the longest waiter plans first and therefore wins the aisle)
and drive-through commitment (a robot with a follower behind it may not
reverse). Same guarantee, expressed through the ranking rule the other
algorithms already share.

**"Is it really collision-free?"**
`tests/test_simulation.py::TestInvariants` re-checks it *every tick* of every
scenario, in both modes, across three seeds: no two live robots share a cell,
nobody leaves the floor, nobody moves more than one cell per tick, and the
reservation table is mutually exclusive across the cooperative window.

---

## 6. Extending it

**A new scenario** — add a function to `amrsim/scenarios.py` decorated with
`@scenario("name", "Label", "description")`. It appears in the dashboard picker
and as a `--scenario` choice automatically. The hooks you have are
`sim.spawn_task(...)`, `sim.place_obstacle(cell)`, `sim.remove_obstacle(cell)`,
`sim.silence_robot(id)`, `sim.revive_robot(id)` and `sim.schedule(tick, fn)`.

**A new warehouse layout** — edit `RACK_BANDS`, `RACK_COLUMNS`, `RACK_W` and
`PINCHES` in `amrsim/warehouse.py`. Chokepoints are *derived*, not declared:
`_compute_corridors` finds every straight single-file run automatically, so a
new layout gets correct locking for free. `TestWarehouse` will tell you if you
have disconnected part of the floor.

**A new metric** — add the field in `metrics.py`, increment it in the engine,
and add it to `HEADLINE` in `run_benchmark.py` (it charts itself) or to the
`tiles` array in `web/static/app.js`.

**Tuning** — everything lives in `config.py`. The four that matter most:
`ALPHA` (congestion avoidance vs. distance), `COOP_WINDOW` (safety horizon vs.
planning cost), `REPLAN_INTERVAL` (responsiveness vs. plan churn), and
`HARD_STOP_RESUME_TICKS` (how expensive an emergency stop is).

**Making it genuinely distributed** — the honest next step. Give each robot its
own `ReservationTable` and `TaskPool`, put a message bus with latency and drop
between them, and drive convergence through `TaskPool.merge` and the Lamport
timestamps that are already there. Nothing in `reservations.py`, `blocks.py` or
`tasks.py` assumes a single writer; the engine does. That is the seam to cut
along, and it is what would let you exercise the partition and message-loss
cases the report addresses at design level only.
