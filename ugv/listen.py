#!/usr/bin/env python3
"""
Print what is being sent on a UDP port of the UGV bus.

  python listen.py 5008              # goals / poses / paths
  python listen.py 5005              # lidar scans (summarised)
  python listen.py 5006 5007         # several ports at once
  python listen.py                   # asks you which port
  python listen.py 5008 --type goal  # only messages of one type
  python listen.py 5005 --full       # print every lidar point too
  python listen.py 5008 --raw        # the JSON exactly as received, one line each
  python listen.py 5000 --unicast    # a plain (non-bus) UDP port, e.g. the simulator's ground-truth pose

Ports: 5005 lidar | 5006 esp + cmd_vel | 5007 rtk | 5008 pose + goal + path.  Ctrl+C to stop.
Layout: ugv/listen.py
"""
import argparse
import json
import os
import select
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import protocol as P                                   # noqa: E402

NAMES = {5005: "lidar", 5006: "esp / cmd_vel", 5007: "rtk", 5008: "pose / goal / path"}


def show(msg, full):
    body = {k: v for k, v in msg.items() if k not in ("type", "v")}
    if msg["type"] == "lidar" and not full:
        pts = msg["points"]
        near = min(d for _, d in pts) if pts else 0
        body = {"ts": msg["ts"], "points": f"{len(pts)} points, nearest {near:.0f} mm"}
    elif msg["type"] == "path" and not full and len(msg["points"]) > 6:
        body = {"ts": msg["ts"], "points": f"{len(msg['points'])} waypoints, first {msg['points'][0]}, last {msg['points'][-1]}"}
    return f"{msg['type']:8s} " + json.dumps(body)


def main():
    ap = argparse.ArgumentParser(description="Print UDP messages from a port of the UGV bus")
    ap.add_argument("ports", nargs="*", type=int, help="port(s) to listen on, e.g. 5008")
    ap.add_argument("--type", action="append", help="only show this message type (repeatable)")
    ap.add_argument("--raw", action="store_true", help="print the raw datagram text, no checking")
    ap.add_argument("--full", action="store_true", help="don't shorten lidar scans / long paths")
    ap.add_argument("--unicast", action="store_true", help="plain UDP on 127.0.0.1 instead of the bus (e.g. 5000)")
    args = ap.parse_args()

    ports = args.ports
    if not ports:
        for k, v in NAMES.items():
            print(f"  {k}  {v}")
        try:
            ports = [int(x) for x in input("Port(s) to listen on (e.g. 5008 or 5005 5007): ").replace(",", " ").split()]
        except (ValueError, EOFError):
            sys.exit("Please enter port numbers like 5008.")

    socks = {}
    for p in ports:
        try:
            if args.unicast:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind(("127.0.0.1", p))
                s.setblocking(False)
            else:
                s = P._listener_socket(p)
        except OSError as e:
            sys.exit(f"Could not listen on port {p}: {e}")
        socks[s] = p
    where = "127.0.0.1 (unicast)" if args.unicast else f"bus {P.GROUP} via {P.IFACE}"
    print(f"Listening on {', '.join(f'{p} ({NAMES.get(p, '?')})' for p in ports)}  [{where}]  Ctrl+C to stop\n")

    n = bad = 0
    t0 = time.time()
    try:
        while True:
            ready, _, _ = select.select(list(socks), [], [], 0.5)
            for s in ready:
                while True:
                    try:
                        data, addr = s.recvfrom(65535)
                    except (BlockingIOError, InterruptedError):
                        break
                    except ConnectionResetError:
                        break
                    stamp = time.strftime("%H:%M:%S")
                    if args.raw or args.unicast:
                        print(f"{stamp} :{socks[s]} {data.decode('utf-8', 'replace').strip()}")
                        n += 1
                        continue
                    try:
                        msg = P.decode(data)
                    except P.ProtocolError as e:
                        bad += 1
                        print(f"{stamp} :{socks[s]} NOT VALID ({e}): {data[:80]!r}")
                        continue
                    if args.type and msg["type"] not in args.type:
                        continue
                    n += 1
                    print(f"{stamp} :{socks[s]} {show(msg, args.full)}")
    except KeyboardInterrupt:
        dt = max(time.time() - t0, 1e-9)
        print(f"\n{n} messages in {dt:.0f} s ({n / dt:.1f}/s)" + (f", {bad} not valid" if bad else ""))


if __name__ == "__main__":
    main()
