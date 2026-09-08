#!/usr/bin/env python3
"""
Launch the live dashboard.

    python3 run_dashboard.py                 # then open http://127.0.0.1:8000
    python3 run_dashboard.py --port 9000 --robots 10 --scenario failure_storm

Only the Python standard library is required.
"""

import argparse
import webbrowser

from amrsim import scenarios
from web.server import serve


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--mode", default="coordinated", choices=["coordinated", "baseline"])
    ap.add_argument("--scenario", default="rush_hour",
                    choices=sorted(s["name"] for s in scenarios.catalogue()))
    ap.add_argument("--robots", type=int, default=8)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--speed", type=float, default=6.0, help="ticks per second")
    ap.add_argument("--no-open", action="store_true", help="do not open a browser")
    args = ap.parse_args()

    server, runner = serve(host=args.host, port=args.port, mode=args.mode,
                           scenario=args.scenario, robots=args.robots,
                           seed=args.seed, speed=args.speed)
    runner.playing = True
    url = f"http://{args.host}:{args.port}"
    print(f"AMR fleet dashboard -> {url}")
    print(f"   scenario={args.scenario}  mode={args.mode}  robots={args.robots}  seed={args.seed}")
    print("   Ctrl-C to stop.")
    if not args.no_open:
        try:
            webbrowser.open(url)
        except Exception:
            pass
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
