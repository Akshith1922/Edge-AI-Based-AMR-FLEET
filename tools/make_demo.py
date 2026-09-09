#!/usr/bin/env python3
"""
Turn a twin trace into a self-contained playback page.

    python3 tools/twin.py --robots 6 --scenario rush_hour --trace results/trace.json
    python3 tools/make_demo.py results/trace.json -o results/fleet_demo.html

The output is one HTML file with the warehouse map embedded as a data URI and
the whole trace inline, so it plays in any browser with no server, no network
and no build step -- which is what you want when the demo has to run on a
laptop in a room with someone else's projector.

Standard library only.
"""

import argparse
import base64
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "ros2_ws" / "src" / "warehouse_picker"

TEMPLATE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AMR Fleet Playback</title>
<style>
:root{--bg:#0e1117;--panel:#161a23;--line:#252b38;--fg:#e9edf5;--dim:#8a93a8;
      --ok:#3ddc97;--warn:#ffb020;--bad:#ff5c68}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.55 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{padding:16px 22px;border-bottom:1px solid var(--line)}
h1{font-size:17px;margin:0 0 4px;letter-spacing:.01em}
.sub{color:var(--dim);font-size:12.5px}
main{display:grid;grid-template-columns:minmax(300px,440px) 1fr;gap:18px;padding:18px;
     align-items:start}
@media(max-width:880px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:16px}
.card h2{font-size:11.5px;text-transform:uppercase;letter-spacing:.09em;
         color:var(--dim);margin:0 0 12px;font-weight:600}
#stage{position:relative;width:100%;aspect-ratio:__ASPECT__;background:#f7f7f4;
       border-radius:8px;overflow:hidden}
#stage img{width:100%;height:100%;display:block}
#stage svg{position:absolute;inset:0;width:100%;height:100%}
.controls{display:flex;gap:10px;align-items:center;margin-top:12px;flex-wrap:wrap}
button{background:#222836;color:var(--fg);border:1px solid var(--line);
       border-radius:7px;padding:7px 14px;font:inherit;cursor:pointer}
button:hover{background:#2b3242}
input[type=range]{flex:1;min-width:130px;accent-color:#4c9aff}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
th{text-align:left;font-size:11px;text-transform:uppercase;letter-spacing:.06em;
   color:var(--dim);font-weight:600;padding:4px 6px}
td{padding:7px 6px;border-top:1px solid var(--line);font-size:13px}
.bar{height:6px;background:#2a3040;border-radius:3px;overflow:hidden;width:56px;
     display:inline-block;vertical-align:middle;margin-right:6px}
.bar i{display:block;height:100%}
.pill{display:inline-block;padding:1px 8px;border-radius:99px;font-size:11px;
      border:1px solid var(--line);color:var(--dim)}
.why{font-size:12px;color:var(--dim)}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:12px}
.stat b{display:block;font-size:20px;font-variant-numeric:tabular-nums}
.stat span{color:var(--dim);font-size:11.5px}
</style></head><body>
<header>
  <h1>__TITLE__</h1>
  <div class="sub">__SUBTITLE__</div>
</header>
<main>
  <div class="card"><h2>Warehouse</h2>
    <div id="stage"><img src="__MAP__" alt="warehouse map"><svg id="ov"
      viewBox="0 0 __VW__ __VH__" preserveAspectRatio="none"></svg></div>
    <div class="controls">
      <button id="play">Pause</button>
      <input id="scrub" type="range" min="0" max="0" value="0">
      <span id="clock" class="sub">0.0 s</span>
      <button id="speed">1&times;</button>
    </div>
  </div>
  <div>
    <div class="card"><h2>Fleet</h2>
      <table><thead><tr><th>Robot</th><th>State</th><th>Battery</th>
        <th>Position</th></tr></thead><tbody id="rows"></tbody></table>
      <div id="why" style="margin-top:12px"></div>
    </div>
    <div class="card" style="margin-top:18px"><h2>Run</h2>
      <div class="stats" id="stats"></div>
    </div>
  </div>
</main>
<script>
const DATA = __DATA__;
const B = DATA.bounds, VW = __VW__, VH = __VH__;
const COLOURS = ["#4c9aff","#3ddc97","#ffb020","#ff5c68","#b388ff","#00c8d7",
                 "#ff8fab","#9bd66b"];
const sx = x => (x - B[0]) / (B[2] - B[0]) * VW;
const sy = y => (B[3] - y) / (B[3] - B[1]) * VH;
const frames = DATA.frames, ov = document.getElementById('ov');
let i = 0, playing = true, speed = 1, acc = 0;

document.getElementById('scrub').max = frames.length - 1;
document.getElementById('stats').innerHTML = Object.entries(DATA.stats)
  .map(([k,v]) => `<div class="stat"><b>${v}</b><span>${k}</span></div>`).join('');

function draw(){
  const f = frames[i]; if(!f) return;
  const parts = [];
  (DATA.stations||[]).forEach(s => parts.push(
    `<circle cx="${sx(s.x)}" cy="${sy(s.y)}" r="1.6" fill="#b9bcc4"/>`));
  f.r.forEach((r,n) => {
    const c = COLOURS[n % COLOURS.length];
    if (r.p && r.p.length > 1) parts.push(
      `<polyline points="${r.p.map(p=>sx(p[0])+','+sy(p[1])).join(' ')}"
       fill="none" stroke="${c}" stroke-width="1.6" stroke-opacity=".5"
       stroke-dasharray="4 3"/>`);
    const hx = sx(r.x) + Math.cos(-r.yaw)*8, hy = sy(r.y) + Math.sin(-r.yaw)*8;
    parts.push(`<line x1="${sx(r.x)}" y1="${sy(r.y)}" x2="${hx}" y2="${hy}"
      stroke="${c}" stroke-width="2.2"/>`);
    parts.push(`<circle cx="${sx(r.x)}" cy="${sy(r.y)}" r="5.5" fill="${c}"
      stroke="#0e1117" stroke-width="1.3"/>`);
    if (r.w) parts.push(`<circle cx="${sx(r.x)}" cy="${sy(r.y)}" r="9"
      fill="none" stroke="${c}" stroke-width="1" stroke-opacity=".6"/>`);
  });
  ov.innerHTML = parts.join('');

  document.getElementById('rows').innerHTML = f.r.map((r,n)=>{
    const c = COLOURS[n % COLOURS.length];
    const bc = r.b > 45 ? 'var(--ok)' : r.b > 20 ? 'var(--warn)' : 'var(--bad)';
    return `<tr><td><span style="color:${c}">&#9679;</span> ${r.id}</td>
      <td><span class="pill">${r.m}</span></td>
      <td><span class="bar"><i style="width:${Math.max(0,Math.min(100,r.b))}%;
        background:${bc}"></i></span>${r.b.toFixed(0)}%</td>
      <td>${r.x.toFixed(1)}, ${r.y.toFixed(1)}</td></tr>`;
  }).join('');
  document.getElementById('why').innerHTML = f.r.map((r,n)=>
    `<div class="why"><b style="color:${COLOURS[n%COLOURS.length]}">${r.id}</b>
     &middot; ${r.why || 'idle'}${r.w ? ' &middot; waiting on ' + r.w : ''}</div>`
  ).join('');
  document.getElementById('clock').textContent =
    `${f.t.toFixed(1)} s  |  ${f.done}/${DATA.task_count} delivered`;
  document.getElementById('scrub').value = i;
}

let last = performance.now();
function loop(now){
  const dt = (now - last) / 1000; last = now;
  if (playing) {
    acc += dt * speed;
    const step = DATA.frame_dt || 0.5;
    while (acc >= step) { acc -= step; i = (i + 1) % frames.length; }
    draw();
  }
  requestAnimationFrame(loop);
}
document.getElementById('play').onclick = e => {
  playing = !playing; e.target.textContent = playing ? 'Pause' : 'Play';
};
document.getElementById('speed').onclick = e => {
  speed = speed >= 8 ? 1 : speed * 2; e.target.innerHTML = speed + '&times;';
};
document.getElementById('scrub').oninput = e => {
  i = +e.target.value; playing = false;
  document.getElementById('play').textContent = 'Play'; draw();
};
draw(); requestAnimationFrame(loop);
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trace", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=ROOT / "results" / "fleet_demo.html")
    ap.add_argument("--map", type=Path, default=PKG / "maps" / "warehouse_map.png")
    ap.add_argument("--title", default="AMR Fleet — Warehouse Playback")
    args = ap.parse_args()

    trace = json.loads(args.trace.read_text())
    report = trace.get("report", {})
    frames = trace.get("frames", [])
    if not frames:
        raise SystemExit(f"{args.trace} has no frames; re-run twin.py with --trace")

    png = base64.b64encode(args.map.read_bytes()).decode("ascii")
    bounds = trace.get("bounds", [-15, -25, 15, 25])
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    vw, vh = 300, int(round(300 * height / width))

    frame_dt = (round(frames[1]["t"] - frames[0]["t"], 3) if len(frames) > 1 else 0.5)
    stats = {
        "tasks delivered": f"{report.get('completed', '?')}/{report.get('tasks', '?')}",
        "robot collisions": report.get("robot_collisions", "?"),
        "wall contacts": report.get("wall_contacts", "?"),
        "makespan": f"{report.get('makespan_s', '?')} s",
        "distance driven": f"{report.get('distance_m', '?')} m",
        "time yielding": f"{report.get('time_yielding_s', '?')} s",
    }
    baseline = trace.get("baseline")
    subtitle = (f"{report.get('robots')} Tugbots &middot; "
                f"{report.get('tasks')} pick-and-drop tasks &middot; "
                f"policy: {report.get('policy')}")
    if baseline:
        base_ms, coop_ms = baseline.get("makespan_s"), report.get("makespan_s")
        if base_ms and coop_ms:
            subtitle += (f" &middot; stop-and-wait control arm finished the same "
                         f"workload in {base_ms} s")

    payload = {
        "bounds": bounds,
        "frames": frames,
        "stations": trace.get("stations", []),
        "stats": stats,
        "frame_dt": frame_dt,
        "task_count": report.get("tasks", 0),
    }

    html = (TEMPLATE
            .replace("__DATA__", json.dumps(payload, separators=(",", ":")))
            .replace("__MAP__", "data:image/png;base64," + png)
            .replace("__ASPECT__", f"{width}/{height}")
            .replace("__VW__", str(vw)).replace("__VH__", str(vh))
            .replace("__TITLE__", args.title)
            .replace("__SUBTITLE__", subtitle))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(html)
    print(f"{args.out}  ({args.out.stat().st_size / 1024:.0f} KB, "
          f"{len(frames)} frames, {frame_dt}s apart)")


if __name__ == "__main__":
    main()
