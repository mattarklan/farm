#!/usr/bin/env python3
"""
Simulated rover: writes rover_pos.dat in exactly the same format as rover_rtka.py
(256-byte, space-padded JSON), so your program thinks LoRa + LC29H are live.
No serial ports needed.

  python sim_rover.py                       # realistic startup: No Fix -> 3D -> DGNSS -> Float -> Fixed RTK
  python sim_rover.py --fix "Fixed RTK"    # lock to one fix state
  python sim_rover.py --lat -31.9505 --lon 115.8605 --speed 1.5
  python sim_rover.py --cycle              # keep cycling through fix states (test your handling)

Stop it with Ctrl+C. Run it INSTEAD of rover_rtka.py (not alongside it).

Layout: this file goes in ugv/sims/sim_rover.py and writes ugv/rtk/rover_pos.dat.
"""

import argparse
import json
import math
import mmap
import os
import random
import time

# This script lives in ugv/sims/, the shared file lives in ugv/rtk/ (resolved from the
# script's own location, so it works no matter where you run it from).
MMAP_FILE = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rtk", "rover_pos.dat")
)
MMAP_SIZE = 256

# (fix label, seconds to spend in it during startup, sats range, hdop range, corr_age range)
STAGES = [
    ("No Fix",    4,  (0, 4),   (99.0, 99.0), (0.0, 0.0)),
    ("3D Fix",    6,  (7, 11),  (1.2, 2.0),   (0.0, 0.0)),
    ("DGNSS",     6,  (10, 14), (0.8, 1.3),   (1.0, 3.0)),
    ("Float RTK", 10, (14, 20), (0.5, 0.9),   (0.5, 2.0)),
    ("Fixed RTK", 0,  (18, 26), (0.4, 0.7),   (0.3, 1.5)),   # 0 = stay forever
]
STAGE_BY_NAME = {s[0]: s for s in STAGES}


def publish(path, lat, lon, fix, sats, hdop, corr_age):
    payload = {
        "lat": lat,
        "lon": lon,
        "fix": fix,
        "sats": sats,
        "hdop": hdop,
        "corr_age": corr_age,
        "ts": time.time(),
    }
    data = json.dumps(payload).ljust(MMAP_SIZE).encode("utf-8")[:MMAP_SIZE]

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    if not os.path.exists(path) or os.path.getsize(path) < MMAP_SIZE:
        with open(path, "wb") as f:
            f.write(b" " * MMAP_SIZE)

    with open(path, "r+b") as f:
        mm = mmap.mmap(f.fileno(), MMAP_SIZE)
        mm.seek(0)
        mm.write(data)
        mm.close()


def main():
    ap = argparse.ArgumentParser(description="Spoof rover_pos.dat as if RTK rover were running")
    ap.add_argument("--file", default=MMAP_FILE, help="shared file path (default ../rtk/rover_pos.dat relative to this script)")
    ap.add_argument("--lat", type=float, default=-31.9505, help="start latitude")
    ap.add_argument("--lon", type=float, default=115.8605, help="start longitude")
    ap.add_argument("--hz", type=float, default=1.0, help="updates per second (default 1)")
    ap.add_argument("--speed", type=float, default=0.5,
                    help="speed in m/s (default 0.5; 0 = stationary, jitter only)")
    ap.add_argument("--radius", type=float, default=2.0,
                    help="drive a circle of this radius (m) around the start point (default 2; 0 = straight walk)")
    ap.add_argument("--heading", type=float, default=90.0, help="walk heading in degrees (0=N, 90=E)")
    ap.add_argument("--fix", choices=list(STAGE_BY_NAME), help="lock to a single fix state")
    ap.add_argument("--cycle", action="store_true", help="loop through all fix states forever")
    args = ap.parse_args()

    lat, lon = args.lat, args.lon
    dt = 1.0 / args.hz
    stage_idx = STAGES.index(STAGE_BY_NAME[args.fix]) if args.fix else 0
    stage_start = time.monotonic()
    heading = math.radians(args.heading)

    print(f"Simulating rover -> {args.file} @ {args.hz} Hz")
    print(f"Start: {lat:.7f}, {lon:.7f}  speed={args.speed} m/s")
    print("Ctrl+C to stop.\n")

    lora_packets = 0
    lora_bytes = 0

    try:
        while True:
            now = time.monotonic()
            name, dur, sat_r, hdop_r, age_r = STAGES[stage_idx]

            # advance fix stage (startup sequence or cycle) unless locked
            if not args.fix and dur and now - stage_start >= dur:
                stage_idx += 1
                if stage_idx >= len(STAGES):
                    stage_idx = 0 if args.cycle else len(STAGES) - 1
                stage_start = now
                name, dur, sat_r, hdop_r, age_r = STAGES[stage_idx]
                print(f"\n*** FIX CHANGED: {name} ***\n")
            elif args.cycle and not dur and now - stage_start >= 10:
                stage_idx = 0
                stage_start = now

            # position: walk + noise (noise shrinks as fix improves)
            noise_m = {"No Fix": 0, "3D Fix": 2.0, "DGNSS": 0.5, "Float RTK": 0.1, "Fixed RTK": 0.01}[name]
            if name == "No Fix":
                sats = random.randint(*sat_r)
                publish(args.file, 0.0, 0.0, name, sats, 99.0, 0.0)
                print(f"[GNSS] {name:<10} | sats={sats:02d} hdop=99.00 corr_age=0.0s | no position")
            else:
                if args.radius > 0:
                    # circle around the start point; angle comes from wall-clock time so
                    # sim_esp.py (same formula) stays in sync with this without talking to it
                    ang = (args.speed / args.radius) * time.time()
                    lat = args.lat + (args.radius * math.sin(ang)) / 111320.0
                    lon = args.lon + (args.radius * math.cos(ang)) / (111320.0 * math.cos(math.radians(args.lat)))
                else:
                    step = args.speed * dt
                    lat += (step * math.cos(heading)) / 111320.0
                    lon += (step * math.sin(heading)) / (111320.0 * math.cos(math.radians(lat)))
                jlat = lat + random.gauss(0, noise_m) / 111320.0
                jlon = lon + random.gauss(0, noise_m) / (111320.0 * math.cos(math.radians(lat)))

                sats = random.randint(*sat_r)
                hdop = round(random.uniform(*hdop_r), 2)
                corr_age = round(random.uniform(*age_r), 1)

                # fake LoRa traffic counters (~ one RTCM burst per update once corrections arrive)
                if name != "3D Fix":
                    lora_packets += 1
                    lora_bytes += random.randint(150, 400)

                publish(args.file, jlat, jlon, name, sats, hdop, corr_age)
                print(f"[GNSS] {name:<10} | sats={sats:02d} hdop={hdop:.2f} corr_age={corr_age:.1f}s | "
                      f"lat={jlat:.7f}, lon={jlon:.7f} | [LoRa] Packets={lora_packets} ({lora_bytes}B)")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nSimulator stopped.")


if __name__ == "__main__":
    main()
