"""
Dashboard server: a dependency-free HTTP front end for the simulation.

The simulation runs on its own thread behind a lock; the browser polls
``/api/state`` for a JSON snapshot and posts to ``/api/control`` to play,
pause, step, change speed, switch scenario or flip between coordinated and
baseline mode. Nothing beyond the Python standard library is required.
"""

import json
import mimetypes
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from amrsim import scenarios
from amrsim.config import DEFAULT
from amrsim.engine import ALGORITHMS, Simulation

STATIC = Path(__file__).parent / "static"


class SimRunner:
    """Owns the simulation and advances it on a background thread."""

    def __init__(self, mode="coordinated", scenario="rush_hour", seed=7,
                 robots=8, speed=6.0, cfg=DEFAULT):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.playing = False
        self.speed = speed
        self.mode = mode
        self.scenario = scenario
        self.seed = seed
        self.robots = robots
        self.sim = self._build()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _build(self):
        sim = Simulation(mode=self.mode, seed=self.seed, num_robots=self.robots,
                         cfg=self.cfg)
        scenarios.build(sim, self.scenario)
        return sim

    def _loop(self):
        next_tick = time.monotonic()
        while not self._stop.is_set():
            if not self.playing:
                time.sleep(0.02)
                next_tick = time.monotonic()
                continue
            period = 1.0 / max(0.5, self.speed)
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.02, next_tick - now))
                continue
            with self.lock:
                self.sim.step()
            next_tick += period
            if next_tick < now:            # never spiral if a tick ran long
                next_tick = now + period

    # ------------------------------------------------------------- controls
    def control(self, action, value=None):
        with self.lock:
            if action == "play":
                self.playing = True
            elif action == "pause":
                self.playing = False
            elif action == "step":
                self.playing = False
                self.sim.step()
            elif action == "speed":
                self.speed = max(0.5, min(60.0, float(value)))
            elif action == "mode":
                self.mode = "baseline" if self.mode == "coordinated" else "coordinated"
                self.sim = self._build()
            elif action == "scenario":
                self.scenario = value
                self.sim = self._build()
            elif action == "robots":
                self.robots = max(2, min(12, int(value)))
                self.sim = self._build()
            elif action == "seed":
                self.seed = int(value)
                self.sim = self._build()
            elif action == "reset":
                self.sim = self._build()
            elif action == "obstacle":
                cell = (int(value[0]), int(value[1]))
                if cell in self.sim.obstacles:
                    self.sim.remove_obstacle(cell)
                elif self.sim.warehouse.is_walkable(*cell):
                    self.sim.place_obstacle(cell)
            elif action == "fail":
                self.sim.silence_robot(int(value))
            elif action == "revive":
                self.sim.revive_robot(int(value))
            return self.state()

    def state(self, include_layout=False):
        data = self.sim.snapshot(include_layout=include_layout)
        data.update(playing=self.playing, speed=self.speed,
                    robot_count=self.robots)
        return data

    def snapshot(self, include_layout=False):
        with self.lock:
            return self.state(include_layout)


def make_handler(runner):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_):       # keep the console clean
            pass

        # ------------------------------------------------------------ helpers
        def _send(self, body, ctype="application/json", code=200):
            if isinstance(body, str):
                body = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(json.dumps(obj, separators=(",", ":")), code=code)

        # ---------------------------------------------------------------- GET
        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                path = "/index.html"
            if path == "/api/state":
                return self._json(runner.snapshot())
            if path == "/api/init":
                data = runner.snapshot(include_layout=True)
                data["scenarios"] = scenarios.catalogue()
                data["algorithms"] = ALGORITHMS
                return self._json(data)
            file = (STATIC / path.lstrip("/")).resolve()
            if file.is_file() and STATIC.resolve() in file.parents:
                ctype = mimetypes.guess_type(file.name)[0] or "application/octet-stream"
                return self._send(file.read_bytes(), ctype)
            return self._send("not found", "text/plain", 404)

        # --------------------------------------------------------------- POST
        def do_POST(self):
            if self.path != "/api/control":
                return self._send("not found", "text/plain", 404)
            length = int(self.headers.get("Content-Length", 0))
            try:
                payload = json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError:
                return self._json({"error": "bad json"}, 400)
            return self._json(runner.control(payload.get("action"), payload.get("value")))

    return Handler


def serve(host="127.0.0.1", port=8000, **kwargs):
    runner = SimRunner(**kwargs)
    server = ThreadingHTTPServer((host, port), make_handler(runner))
    return server, runner
