#!/usr/bin/env python3
"""Tell me what the world image looks like to the simulators.   python check_world.py cmap.png [--invert]
Layout: ugv/sims/check_world.py"""
import sys
import numpy as np
from PIL import Image

path = next((a for a in sys.argv[1:] if not a.startswith("-")), "cmap.png")
inv = "--invert" in sys.argv
im = Image.open(path)
import os
print(f"{os.path.abspath(path)}: mode {im.mode}, {im.size[0]}x{im.size[1]} px")
if im.mode in ("RGBA", "LA", "PA") or (im.mode == "P" and "transparency" in im.info):
    rgba = np.asarray(im.convert("RGBA"))
    print(f"  has transparency: {np.mean(rgba[..., 3] < 255) * 100:.0f}% of pixels are transparent (treated as free)")
    bg = Image.new("RGBA", im.size, (0, 0, 0, 255) if inv else (255, 255, 255, 255)); bg.alpha_composite(im.convert("RGBA")); im = bg
g = np.asarray(im.convert("L"))
wall = (g > 127) if inv else (g < 128)
print(f"  pixel values: min {g.min()}, max {g.max()}; dark (<128) {np.mean(g < 128) * 100:.0f}%, light {np.mean(g >= 128) * 100:.0f}%")
print(f"  with{'' if inv else 'out'} --invert, {wall.mean() * 100:.0f}% of the image is WALL")
h, w = g.shape
cy, cx = h // 2, w // 2
r = 20
print(f"  around the map centre (where the robot starts): {wall[cy - r:cy + r, cx - r:cx + r].mean() * 100:.0f}% wall, centre pixel = {g[cy, cx]}")
print("  OK" if wall.mean() < 0.5 and not wall[cy - r:cy + r, cx - r:cx + r].mean() > 0.5
      else "  PROBLEM: the robot would start inside a wall. Try the other setting of --invert.")
