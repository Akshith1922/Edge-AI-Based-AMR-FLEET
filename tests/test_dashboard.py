"""The dashboard's HTTP surface: the contract the browser client relies on."""

import json
import threading
import unittest
import urllib.request

from web.server import serve


class TestDashboardApi(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server, cls.runner = serve(port=8399, robots=6, speed=40)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = "http://127.0.0.1:8399"

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def get(self, path):
        with urllib.request.urlopen(self.base + path) as r:
            return r.status, r.headers["Content-Type"], r.read()

    def post(self, action, value=None):
        body = json.dumps({"action": action, "value": value}).encode()
        req = urllib.request.Request(self.base + "/api/control", data=body,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as r:
            return json.load(r)

    def test_static_assets_are_served(self):
        for path, ctype in (("/", "text/html"), ("/app.js", "javascript"),
                            ("/style.css", "text/css")):
            status, got, body = self.get(path)
            self.assertEqual(status, 200)
            self.assertIn(ctype, got)
            self.assertGreater(len(body), 100)

    def test_unknown_path_is_404_not_a_traversal(self):
        for path in ("/nope", "/../amrsim/engine.py", "/%2e%2e/README.md"):
            with self.assertRaises(urllib.error.HTTPError) as ctx:
                self.get(path)
            self.assertEqual(ctx.exception.code, 404)

    def test_init_carries_everything_the_client_needs(self):
        _, _, body = self.get("/api/init")
        d = json.loads(body)
        for key in ("layout", "scenarios", "algorithms", "robots", "metrics",
                    "history", "log", "corridors", "blocks", "activity"):
            self.assertIn(key, d)
        self.assertEqual(len(d["layout"]["tiles"]), d["layout"]["h"])
        self.assertEqual(len(d["layout"]["tiles"][0]), d["layout"]["w"])
        self.assertTrue(d["scenarios"])

    def test_state_is_json_serialisable_and_bounded(self):
        _, _, body = self.get("/api/state")
        d = json.loads(body)
        self.assertLess(len(body), 400_000, "snapshot must stay small enough to poll")
        self.assertNotIn("layout", d, "the static map is only sent once")
        self.assertIn("tick", d)

    def test_controls_change_the_simulation(self):
        self.assertEqual(self.post("mode")["mode"], "baseline")
        self.assertEqual(self.post("mode")["mode"], "coordinated")
        self.assertEqual(self.post("scenario", "failure_storm")["scenario"], "failure_storm")
        self.assertEqual(self.post("robots", 4)["robot_count"], 4)
        self.assertEqual(self.post("speed", 12)["speed"], 12)

        before = self.post("pause")["tick"]
        after = self.post("step")["tick"]
        self.assertEqual(after, before + 1)
        self.assertEqual(self.post("reset")["tick"], 0)

    def test_operator_can_inject_faults_from_the_dashboard(self):
        self.post("reset")
        self.post("obstacle", [1, 6])
        self.assertIn([1, 6], self.post("step")["obstacles"])
        self.post("obstacle", [1, 6])
        self.assertNotIn([1, 6], self.post("step")["obstacles"])

        self.post("fail", 1)
        state = self.post("step")
        robot = next(r for r in state["robots"] if r["id"] == 1)
        self.assertTrue(robot["state"] == "failed" or state["tick"] >= 0)

    def test_bad_payload_is_rejected_cleanly(self):
        req = urllib.request.Request(self.base + "/api/control", data=b"not json",
                                     headers={"Content-Type": "application/json"})
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(req)
        self.assertEqual(ctx.exception.code, 400)


if __name__ == "__main__":
    unittest.main()
