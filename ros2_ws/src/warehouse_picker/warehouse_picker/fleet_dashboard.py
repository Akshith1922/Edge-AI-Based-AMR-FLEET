#!/usr/bin/env python3
"""
Fleet dashboard: a lightweight live monitor for the whole fleet.

Subscribes to the same `/fleet/mesh` broadcast the robots send each other --
it is a *listener on the mesh*, not a component of it, so it can be started,
stopped or run on three laptops at once without the fleet noticing.

Serves a single page on http://<host>:8080 showing every robot's position on
the warehouse map, its battery, its mode, what it is waiting for and why.

    ros2 run warehouse_picker dashboard --ros-args -p port:=8080

The previous version of this file subscribed to `/fleet/p2p_mesh` and expected
fields (`target_x`, `battery`) that the agent never sent on it, so it printed
"searching for heartbeats" forever regardless of what the fleet was doing. The
topic name and the payload schema now come from `protocol.py`, which is also
what the agents encode with, so the two cannot drift apart again.
"""

import json
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .protocol import FleetState

MESH_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=10)

STALE_S = 3.0


class Store:
    """Thread-safe snapshot of the fleet, shared between ROS and HTTP threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.robots = {}
        self.telemetry = {}
        self.started = time.time()
        self.tasks_done = 0
        self.events = []

    def update_mesh(self, msg, now):
        with self.lock:
            self.robots[msg["id"]] = dict(msg, seen=now)

    def update_telemetry(self, data, now):
        with self.lock:
            self.telemetry[data["id"]] = dict(data, seen=now)

    def note(self, text, now):
        with self.lock:
            self.events.append({"t": round(now - self.started, 1), "text": text})
            del self.events[:-40]

    def snapshot(self, now):
        with self.lock:
            robots = []
            for rid, state in sorted(self.robots.items()):
                tel = self.telemetry.get(rid, {})
                robots.append({
                    "id": rid,
                    "x": state["x"], "y": state["y"], "yaw": state["yaw"],
                    "v": state["v"], "battery": state["battery"],
                    "mode": state["mode"], "task": state.get("task"),
                    "waiting_for": state.get("waiting_for"),
                    "priority": state.get("priority", 1.0),
                    "online": (now - state["seen"]) < STALE_S,
                    "age": round(now - state["seen"], 2),
                    "reason": tel.get("reason", ""),
                    "path": tel.get("path", []),
                    "distance": tel.get("distance", 0.0),
                    "tasks_done": tel.get("tasks_done", 0),
                    "goal": state.get("goal"),
                })
            return {"uptime": round(now - self.started, 1), "robots": robots,
                    "events": list(self.events)}


class Handler(BaseHTTPRequestHandler):
    store = None
    page = b""
    map_png = b""

    def log_message(self, *_args):
        pass                      # the ROS logger is the one that matters

    def do_GET(self):
        if self.path.startswith("/api/state"):
            body = json.dumps(self.store.snapshot(time.time())).encode()
            self._send(200, "application/json", body)
        elif self.path.startswith("/map.png"):
            self._send(200, "image/png", self.map_png)
        else:
            self._send(200, "text/html; charset=utf-8", self.page)

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


class FleetDashboard(Node):
    def __init__(self):
        super().__init__("fleet_dashboard")
        self.declare_parameter("port", 8080)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("assets", "")

        assets = self.get_parameter("assets").value
        if not assets:
            from ament_index_python.packages import get_package_share_directory
            assets = get_package_share_directory("warehouse_picker")

        self.store = Store()
        self.create_subscription(String, "/fleet/mesh", self.on_mesh, MESH_QOS)
        self.create_subscription(String, "/fleet/telemetry", self.on_telemetry,
                                 MESH_QOS)

        Handler.store = self.store
        Handler.page = _page(assets).encode()
        map_png = Path(assets) / "maps" / "warehouse_map.png"
        Handler.map_png = map_png.read_bytes() if map_png.exists() else b""

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self.server = ThreadingHTTPServer((host, port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f"fleet dashboard -> http://{host}:{port}")

    def on_mesh(self, msg):
        parsed = FleetState.from_json(msg.data)
        if parsed:
            self.store.update_mesh(parsed, time.time())

    def on_telemetry(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict) and "id" in data:
            self.store.update_telemetry(data, time.time())


def _page(assets):
    bounds = [-15.0, -25.0, 15.0, 25.0]
    layout = Path(assets) / "config" / "warehouse_layout.json"
    if layout.exists():
        try:
            bounds = json.loads(layout.read_text())["bounds"]
        except (ValueError, KeyError):
            pass
    return DASHBOARD_HTML.replace("__BOUNDS__", json.dumps(bounds))


DASHBOARD_HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AMR Fleet Monitor</title>
<style>
:root{--bg:#10131a;--panel:#181c26;--line:#262c3a;--fg:#e8ecf4;--dim:#8b93a7;
      --ok:#3ddc97;--warn:#ffb020;--bad:#ff5c68;--accent:#4c9aff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.5 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{padding:14px 20px;border-bottom:1px solid var(--line);display:flex;
       align-items:baseline;gap:16px;flex-wrap:wrap}
h1{font-size:16px;margin:0;letter-spacing:.02em}
.sub{color:var(--dim);font-size:12px}
main{display:grid;grid-template-columns:minmax(320px,1fr) 420px;gap:16px;padding:16px}
@media(max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;
      padding:14px;min-width:0}
.card h2{font-size:12px;text-transform:uppercase;letter-spacing:.08em;
         color:var(--dim);margin:0 0 10px}
#stage{position:relative;width:100%;aspect-ratio:30/50;background:#f7f7f4;
       border-radius:6px;overflow:hidden}
#stage img{width:100%;height:100%;display:block;object-fit:fill}
#stage svg{position:absolute;inset:0;width:100%;height:100%}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
th{text-align:left;font-weight:500;color:var(--dim);font-size:11px;
   text-transform:uppercase;letter-spacing:.06em;padding:4px 6px}
td{padding:6px;border-top:1px solid var(--line);font-size:13px}
.bar{height:6px;background:#2a3040;border-radius:3px;overflow:hidden;min-width:54px}
.bar i{display:block;height:100%}
.pill{display:inline-block;padding:1px 7px;border-radius:99px;font-size:11px;
      border:1px solid var(--line)}
.off{opacity:.45}
.why{color:var(--dim);font-size:12px}
ul{list-style:none;margin:0;padding:0;max-height:150px;overflow:auto}
li{border-top:1px solid var(--line);padding:4px 0;font-size:12px;color:var(--dim)}
</style></head><body>
<header>
  <h1>AMR Fleet Monitor</h1>
  <span class="sub">decentralised mesh &middot; <span id="count">0</span> robots online
  &middot; uptime <span id="uptime">0</span>s</span>
</header>
<main>
  <div class="card"><h2>Warehouse</h2>
    <div id="stage"><img src="/map.png" alt="warehouse map"><svg id="ov"
      viewBox="0 0 300 500" preserveAspectRatio="none"></svg></div>
  </div>
  <div>
    <div class="card"><h2>Fleet</h2>
      <table><thead><tr><th>Robot</th><th>Mode</th><th>Battery</th>
      <th>Pos</th><th>Done</th></tr></thead><tbody id="rows"></tbody></table>
      <div id="why" style="margin-top:10px"></div>
    </div>
    <div class="card" style="margin-top:16px"><h2>Coordination</h2>
      <ul id="events"></ul></div>
  </div>
</main>
<script>
const B = __BOUNDS__;                       // [x0,y0,x1,y1] in metres
const COLOURS = ["#4c9aff","#3ddc97","#ffb020","#ff5c68","#b388ff","#00c8d7"];
const sx = x => (x - B[0]) / (B[2] - B[0]) * 300;
const sy = y => (B[3] - y) / (B[3] - B[1]) * 500;

async function tick(){
  let s; try { s = await (await fetch('/api/state')).json(); } catch(e){ return; }
  document.getElementById('uptime').textContent = s.uptime;
  document.getElementById('count').textContent = s.robots.filter(r=>r.online).length;

  const parts = [];
  s.robots.forEach((r,i) => {
    const c = COLOURS[i % COLOURS.length];
    if (r.path && r.path.length > 1) {
      parts.push(`<polyline points="${r.path.map(p=>sx(p[0])+','+sy(p[1])).join(' ')}"
        fill="none" stroke="${c}" stroke-width="2" stroke-opacity=".55"
        stroke-dasharray="5 4"/>`);
    }
    if (r.goal) parts.push(`<rect x="${sx(r.goal[0])-4}" y="${sy(r.goal[1])-4}"
      width="8" height="8" fill="none" stroke="${c}" stroke-width="1.6"/>`);
    const hx = sx(r.x) + Math.cos(-r.yaw)*9, hy = sy(r.y) + Math.sin(-r.yaw)*9;
    parts.push(`<line x1="${sx(r.x)}" y1="${sy(r.y)}" x2="${hx}" y2="${hy}"
      stroke="${c}" stroke-width="2.5"/>`);
    parts.push(`<circle cx="${sx(r.x)}" cy="${sy(r.y)}" r="6" fill="${c}"
      fill-opacity="${r.online?0.95:0.3}" stroke="#101319" stroke-width="1.5"/>`);
    parts.push(`<text x="${sx(r.x)+9}" y="${sy(r.y)-7}" font-size="11"
      fill="${c}">${r.id}</text>`);
  });
  document.getElementById('ov').innerHTML = parts.join('');

  document.getElementById('rows').innerHTML = s.robots.map((r,i)=>{
    const c = COLOURS[i % COLOURS.length];
    const bc = r.battery > 45 ? 'var(--ok)' : r.battery > 20 ? 'var(--warn)' : 'var(--bad)';
    return `<tr class="${r.online?'':'off'}">
      <td><span style="color:${c}">&#9679;</span> ${r.id}</td>
      <td><span class="pill">${r.mode}</span></td>
      <td><div class="bar"><i style="width:${Math.max(0,Math.min(100,r.battery))}%;
          background:${bc}"></i></div>${r.battery.toFixed(0)}%</td>
      <td>${r.x.toFixed(1)}, ${r.y.toFixed(1)}</td>
      <td>${r.tasks_done}</td></tr>`;
  }).join('');

  document.getElementById('why').innerHTML = s.robots.map(r =>
    `<div class="why"><b>${r.id}</b> &middot; ${r.reason || 'idle'}` +
    (r.waiting_for ? ` &middot; waiting on ${r.waiting_for}` : '') + `</div>`).join('');

  document.getElementById('events').innerHTML =
    s.events.slice().reverse().map(e=>`<li>${e.t}s &middot; ${e.text}</li>`).join('');
}
tick(); setInterval(tick, 400);
</script></body></html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = FleetDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
