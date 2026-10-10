#!/usr/bin/env python3
"""
Start the whole simulated robot in one command: sim_robot.py (rover + ESP32) and sim_lidar.py (RPLidar),
both on the UDP bus, both using the SAME hidden world image. Ctrl+C stops both.

  python run_sim.py                          # world = cmap.png (next to this script, in ugv/, or the current folder)
  python run_sim.py --cmap ../maps/world.png --invert
  python run_sim.py --fix "Fixed RTK" --start-x -6 --start-y -6
  python run_sim.py --no-lidar               # robot only

Any option it doesn't recognise is passed on to sim_robot.py (e.g. --circle 2, --esp-hz 5).
The visualiser is separate: run it in another terminal  (python ../visualiser.py).

Layout: ugv/sims/run_sim.py
"""

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from common import config as cfg                     # noqa: E402
from common.protocol import selftest                 # noqa: E402


def find_world(explicit):
    if explicit:
        return explicit if os.path.exists(explicit) else None
    for p in (os.path.join(HERE, "cmap.png"), os.path.join(HERE, "..", "cmap.png"), os.path.join(os.getcwd(), "cmap.png")):
        if os.path.exists(p):
            return os.path.normpath(p)
    return None


def main():
    ap = argparse.ArgumentParser(description="Run the simulated robot + lidar", epilog="Other options go to sim_robot.py")
    ap.add_argument("--cmap", help="hidden world image (black = wall). Default: cmap.png next to this script, in ugv/, or cwd")
    ap.add_argument("--res", type=float, default=cfg.MAP_RES, help="metres per world pixel (default from common/config.py)")
    ap.add_argument("--invert", action="store_true", help="world image is black = free, white = wall")
    ap.add_argument("--no-lidar", action="store_true", help="don't start the lidar simulator")
    args, extra = ap.parse_known_args()

    if selftest() != 0:
        sys.exit("The UDP bus does not work on this machine, so nothing would be able to talk. See the message above.")

    world = find_world(args.cmap)
    if world is None:
        print("No world image found (looked for cmap.png). The robot will run without collisions and without a lidar; "
              "pass --cmap path\\to\\cmap.png.")
    if world:
        print(f"[run_sim] world image: {os.path.abspath(world)}" + (" (inverted)" if args.invert else ""), flush=True)
    py = sys.executable
    robot = [py, os.path.join(HERE, "sim_robot.py"), "--res", str(args.res)]
    if world:
        robot += ["--cmap", world]
    if args.invert:
        robot.append("--invert")
    procs = [("robot", subprocess.Popen(robot + extra))]
    if world and not args.no_lidar:
        lidar = [py, os.path.join(HERE, "sim_lidar.py"), "--map", world, "--mm-per-pixel", str(args.res * 1000.0)]
        if args.invert:
            lidar.append("--invert")
        procs.append(("lidar", subprocess.Popen(lidar)))

    try:
        while True:
            for name, p in procs:
                if p.poll() is not None:
                    print(f"[run_sim] {name} exited with code {p.returncode}; stopping the rest.")
                    raise KeyboardInterrupt
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for _, p in procs:
            if p.poll() is None:
                p.terminate()
        for _, p in procs:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()


if __name__ == "__main__":
    main()
