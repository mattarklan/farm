#!/usr/bin/env python3
import time
import json
import mmap
import os
import math
import collections
import matplotlib.pyplot as plt

DISPLAY_RADIUS = 5.0  # Meters
MMAP_FILE = "rover_pos.dat"
MMAP_SIZE = 256

def latlon_to_xy(lat, lon, lat0, lon0):
    """Converts WGS84 Lat/Lon into local X/Y offset meters relative to an origin."""
    R = 6378137.0  # Earth radius in meters
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = dlon * math.cos(math.radians(lat0)) * R
    y = dlat * R
    return x, y

def read_shared_position():
    """Reads telemetry dict from the shared memory file."""
    if not os.path.exists(MMAP_FILE):
        return None
    try:
        with open(MMAP_FILE, "r+b") as f:
            mm = mmap.mmap(f.fileno(), MMAP_SIZE)
            raw = mm.read(MMAP_SIZE).strip()
            mm.close()
            if raw:
                return json.loads(raw.decode('utf-8'))
    except Exception:
        pass
    return None

def main():
    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7), facecolor='white')
    ax.set_facecolor('white')

    trail_line, = ax.plot([], [], 'b-', linewidth=2, label='Rover Path')
    rover_dot, = ax.plot([], [], 'ro', markersize=8, label='Current Position')
    origin_dot, = ax.plot([0], [0], 'k+', markersize=10, markeredgewidth=2, label='Start Reference')

    ax.set_title("Rover Local Movement (5m Radius)", fontsize=12, fontweight='bold')
    ax.set_xlabel("East / West (meters)")
    ax.set_ylabel("North / South (meters)")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_aspect('equal', adjustable='box')
    ax.legend(loc='upper right')

    origin_lat, origin_lon = None, None
    x_data = collections.deque(maxlen=2000)
    y_data = collections.deque(maxlen=2000)

    print("[Visualizer] Waiting for incoming shared memory position stream...")

    last_ts = 0.0

    while True:
        pos = read_shared_position()
        if pos and pos.get('ts', 0) != last_ts:
            last_ts = pos['ts']
            lat = pos['lat']
            lon = pos['lon']
            fix = pos.get('fix', 'Unknown')

            # Lock the origin on first valid coordinate read
            if origin_lat is None:
                origin_lat, origin_lon = lat, lon
                print(f"[Visualizer] Origin locked at: Lat {origin_lat:.7f}, Lon {origin_lon:.7f}")

            x, y = latlon_to_xy(lat, lon, origin_lat, origin_lon)
            x_data.append(x)
            y_data.append(y)

            trail_line.set_data(x_data, y_data)
            rover_dot.set_data([x], [y])

            # Color dot green for Fixed RTK, orange for Float, red for standard GPS
            color_map = {"Fixed RTK": "green", "Float RTK": "orange"}
            rover_dot.set_color(color_map.get(fix, "red"))

            ax.set_xlim(-DISPLAY_RADIUS, DISPLAY_RADIUS)
            ax.set_ylim(-DISPLAY_RADIUS, DISPLAY_RADIUS)

            fig.canvas.draw()
            fig.canvas.flush_events()

        time.sleep(0.1)  # Refresh display at 10 Hz

if __name__ == "__main__":
    main()