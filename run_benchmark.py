#!/usr/bin/env python3
"""
Headless baseline-vs-coordinated benchmark.

Runs both modes over N seeds on an identical, closed task list and writes:

    results/benchmark.csv    one row per trial
    results/report.html      aggregated comparison with inline charts

    python3 run_benchmark.py --trials 10 --ticks 500 --robots 8
    python3 run_benchmark.py --scenario failure_storm --robots 6 8 10

No third-party packages: the charts are generated as inline SVG.
"""

import argparse
import csv
import html
import statistics
from pathlib import Path

from amrsim import scenarios
from amrsim.engine import Simulation

RESULTS = Path(__file__).parent / "results"

#: (metric key, label, "lower"|"higher" is better, unit)
HEADLINE = [
    ("tasks_completed", "Tasks delivered", "higher", ""),
    ("throughput_per_100", "Throughput / 100 ticks", "higher", ""),
    ("avg_task_time", "Average task time", "lower", "ticks"),
    ("p90_task_time", "P90 task time", "lower", "ticks"),
    ("collisions", "Collisions", "lower", ""),
    ("hard_stops", "Unplanned hard stops", "lower", ""),
    ("avg_reassignment_latency", "Failure reassignment latency", "lower", "ticks"),
    ("total_distance", "Fleet distance travelled", "lower", "cells"),
]


def run_trial(mode, seed, ticks, robots, scenario):
    sim = Simulation(mode=mode, seed=seed, num_robots=robots)
    scenarios.build(sim, scenario, seed=seed)
    sim.run(ticks)
    row = sim.metrics.summary()
    row.update(seed=seed, robots=robots, scenario=scenario)
    return row


def aggregate(rows, key):
    vals = [r[key] for r in rows]
    return statistics.mean(vals) if vals else 0.0


def bar_chart(label, unit, coord, base, better):
    """A two-bar comparison as inline SVG — no plotting library required."""
    top = max(coord, base, 1e-9)
    w_c, w_b = 100 * coord / top, 100 * base / top
    win = ("coord" if (coord > base) == (better == "higher") else "base") \
        if abs(coord - base) > 1e-9 else "tie"
    delta = ""
    if base > 1e-9:
        pct = (coord - base) / base * 100
        good = (pct > 0) == (better == "higher")
        delta = (f'<span class="delta {"good" if good else "bad"}">'
                 f'{pct:+.0f}%</span>')
    return f"""
    <figure class="cmp">
      <figcaption>{html.escape(label)} {delta}
        <em>{"higher is better" if better == "higher" else "lower is better"}</em>
      </figcaption>
      <div class="row"><span class="k">Coordinated</span>
        <svg viewBox="0 0 100 10" preserveAspectRatio="none" class="bar {'win' if win=='coord' else ''}">
          <rect x="0" y="0" width="{w_c:.2f}" height="10"/></svg>
        <b>{coord:,.1f}{(' ' + unit) if unit else ''}</b></div>
      <div class="row"><span class="k">Baseline</span>
        <svg viewBox="0 0 100 10" preserveAspectRatio="none" class="bar base {'win' if win=='base' else ''}">
          <rect x="0" y="0" width="{w_b:.2f}" height="10"/></svg>
        <b>{base:,.1f}{(' ' + unit) if unit else ''}</b></div>
    </figure>"""


def write_report(rows, args, path):
    coord = [r for r in rows if r["mode"] == "coordinated"]
    base = [r for r in rows if r["mode"] == "baseline"]
    charts = "".join(bar_chart(label, unit, aggregate(coord, key),
                               aggregate(base, key), better)
                     for key, label, better, unit in HEADLINE)

    def table(rowset, title):
        keys = ["seed", "tasks_completed", "avg_task_time", "p90_task_time",
                "collisions", "hard_stops", "deadlock_events", "gridlocks",
                "reroute_events", "avg_reassignment_latency", "total_distance"]
        head = "".join(f"<th>{k.replace('_', ' ')}</th>" for k in keys)
        body = "".join("<tr>" + "".join(f"<td>{r[k]}</td>" for k in keys) + "</tr>"
                       for r in rowset)
        return (f"<h3>{title}</h3><div class='scroll'><table>"
                f"<thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>")

    safety = ("PASS — no collision in any trial, in either mode"
              if not any(r["collisions"] for r in rows) else
              "FAIL — a collision occurred; see the per-trial table")

    path.write_text(f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AMR Fleet Benchmark</title><style>
:root{{color-scheme:dark light}}
body{{margin:0;padding:32px;background:#0a0d14;color:#e7edf8;
 font:14px/1.6 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
 font-variant-numeric:tabular-nums}}
main{{max-width:1000px;margin:0 auto}}
h1{{font-size:22px;margin:0 0 4px}} h2{{font-size:14px;text-transform:uppercase;
 letter-spacing:1.1px;color:#8b96ad;margin:34px 0 12px}}
h3{{font-size:13px;color:#8b96ad;margin:22px 0 8px;font-weight:600}}
p.sub{{color:#8b96ad;margin:0 0 8px}}
.safety{{background:#12241c;border:1px solid #23543c;color:#3ddc97;padding:11px 14px;
 border-radius:10px;font-weight:600;margin:16px 0 0}}
.safety.fail{{background:#2a1418;border-color:#5e2630;color:#ff6b6b}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px}}
.cmp{{margin:0;background:#111725;border:1px solid #212b3e;border-radius:12px;padding:13px 15px}}
.cmp figcaption{{font-weight:600;margin-bottom:9px;display:flex;align-items:baseline;gap:8px}}
.cmp em{{margin-left:auto;font-style:normal;font-size:11px;color:#5f6b82}}
.delta{{font-family:ui-monospace,monospace;font-size:12px}}
.delta.good{{color:#3ddc97}} .delta.bad{{color:#ff6b6b}}
.row{{display:grid;grid-template-columns:82px 1fr 96px;align-items:center;gap:10px;margin:5px 0}}
.k{{font-size:11px;color:#8b96ad}}
.row b{{font-family:ui-monospace,monospace;font-size:12px;text-align:right;font-weight:500}}
.bar{{height:10px;width:100%;display:block}}
.bar rect{{fill:#2f6f9e}} .bar.base rect{{fill:#7d5233}}
.bar.win rect{{fill:#4cc2ff}} .bar.base.win rect{{fill:#ffb020}}
.scroll{{overflow-x:auto;-webkit-overflow-scrolling:touch}}
table{{border-collapse:collapse;width:100%;min-width:760px;font-size:11.5px;
 font-family:ui-monospace,monospace}}
th,td{{padding:5px 7px;border-bottom:1px solid #1b2333;text-align:right}}
th{{color:#8b96ad;font-weight:600;text-align:right;white-space:nowrap}}
th:first-child,td:first-child{{text-align:left}}
footer{{margin-top:36px;color:#5f6b82;font-size:12px}}
</style></head><body><main>
<h1>AMR fleet coordination — baseline vs coordinated</h1>
<p class="sub">Scenario <b>{args.scenario}</b> · {args.trials} seeds ·
{args.ticks} ticks · {args.robots} robots · identical task list and faults in both arms.</p>
<div class="safety {'' if 'PASS' in safety else 'fail'}">Safety invariant: {safety}</div>

<h2>Headline metrics <span style="text-transform:none;letter-spacing:0">(mean over {args.trials} seeds)</span></h2>
<div class="grid">{charts}</div>

<h2>Per-trial detail</h2>
{table(coord, "Coordinated — the six documented algorithms")}
{table(base, "Baseline — independent A*, hard-stop, central dispatch")}

<footer>Generated by <code>run_benchmark.py</code>. Raw rows in
<code>results/benchmark.csv</code>.</footer>
</main></body></html>""", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=8)
    ap.add_argument("--ticks", type=int, default=500)
    ap.add_argument("--robots", type=int, default=8)
    ap.add_argument("--scenario", default="rush_hour_fixed",
                    choices=sorted(s["name"] for s in scenarios.catalogue()))
    args = ap.parse_args()

    RESULTS.mkdir(exist_ok=True)
    rows = []
    for seed in range(1, args.trials + 1):
        for mode in ("coordinated", "baseline"):
            row = run_trial(mode, seed, args.ticks, args.robots, args.scenario)
            rows.append(row)
            print(f"  seed {seed:>2}  {mode:<12} "
                  f"delivered={row['tasks_completed']:>3}  "
                  f"avg={row['avg_task_time']:>6.1f}  "
                  f"collisions={row['collisions']}  "
                  f"hard_stops={row['hard_stops']:>3}")

    csv_path = RESULTS / "benchmark.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    report = RESULTS / "report.html"
    write_report(rows, args, report)

    print("\n" + "=" * 62)
    for key, label, better, unit in HEADLINE:
        c = aggregate([r for r in rows if r["mode"] == "coordinated"], key)
        b = aggregate([r for r in rows if r["mode"] == "baseline"], key)
        mark = "+" if (c > b) == (better == "higher") and abs(c - b) > 1e-9 else " "
        print(f"{mark} {label:<32} coordinated {c:>9.2f}   baseline {b:>9.2f}")
    print("=" * 62)
    print(f"wrote {csv_path}\nwrote {report}")


if __name__ == "__main__":
    main()
