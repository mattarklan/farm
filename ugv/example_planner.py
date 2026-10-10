#!/usr/bin/env python3
"""
Example of an EXTERNAL planner: the smallest program that talks to the visualiser over the bus.
Replace the "plan" line with your Theta*.

  listens   pose  (where the robot is)      goal  (where you clicked)      lidar (scans, if you want them)
  publishes cmd_vel (drive the robot)       path  (so the visualiser can draw your plan)

Run the visualiser WITHOUT --builtin-nav, then:   python example_planner.py
This one has no obstacle avoidance: it drives in a straight line. Layout: ugv/example_planner.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import config as cfg                                # noqa: E402
from common.protocol import Publisher, Subscriber               # noqa: E402

sub, pub = Subscriber(["pose", "goal"]), Publisher()
pose = goal = None
print("waiting for pose + goal on the bus (click the map in the visualiser) ...")
while True:
    for m in sub.poll():
        if m["type"] == "pose":
            pose = m
        elif m["type"] == "goal":
            goal = (m["x"], m["y"]) if m["action"] == "go" else None
            if goal:
                print(f"goal {goal}")
            else:
                print("goal cancelled")
            if goal and pose:
                path = [[pose["x"], pose["y"]], [goal[0], goal[1]]]      # <- your planner goes here
                pub.send("path", points=path)
            else:
                pub.send("path", points=[])
    if goal and pose and time.time() - pose["ts"] < 1.0:
        dx, dy = goal[0] - pose["x"], goal[1] - pose["y"]
        dist = math.hypot(dx, dy)
        if dist < 0.25:
            pub.send("cmd_vel", linear=0.0, angular=0.0)
            pub.send("path", points=[])
            print("arrived")
            goal = None
        else:
            err = (math.atan2(dy, dx) - pose["yaw"] + math.pi) % (2 * math.pi) - math.pi
            w = max(-cfg.MAX_TURN, min(cfg.MAX_TURN, 2.0 * err))
            v = 0.0 if abs(err) > 0.8 else min(cfg.MAX_SPEED, 0.2 + dist) * math.cos(err)
            pub.send("cmd_vel", linear=v, angular=w)
    time.sleep(0.05)                                                   # 20 Hz (the robot stops if cmd_vel stops for 0.5 s)
