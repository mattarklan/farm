#!/usr/bin/env python3
# RTCM3 over Waveshare SX1262 fixed-address mode (base transmitter).
#
# Same transport format as before (rover_rx / rover_rtk understand it):
#   A5 5A | ver | type | body_len(2) | body
#   SINGLE body: seq(2) | frag_index(1) | frag_total(1) | RTCM bytes
#   BATCH  body: count(1) | [frame_len(2) | complete RTCM frame]...
#
# What's new vs the old basetx.py:
#   * --types       only send the listed RTCM message types (default 1074 = GPS MSM4)
#   * --pos-interval  send 1005/1006 (base position) only every N seconds
#   * --max-age     observation frames that waited longer than this in the queue
#                   are DROPPED, never sent late. Stale corrections are worse than
#                   missing ones.
#   * status line shows LAT (how long frames waited before being sent). It should
#     stay well under 1 s. If it climbs, the link can't carry what you selected.
#
# Usage (same as before, e.g. piping from your LC29H reader):
#   python3 reader.py | python3 basetx.py --port /dev/ttyUSB1 --dst-addr 2
#   python3 reader.py | python3 basetx.py --port /dev/ttyUSB1 --dst-addr 2 --types 1074,1094

import argparse
import os
import sys
import threading
import time
from collections import deque

try:
    import serial
except ImportError:
    serial = None

UART_BAUD = 9600
CHANNEL = 65
MAX_APP_PAYLOAD = 64          # must match the module's packet length setting
MAGIC = b"\xA5\x5A"
VERSION = 1
TYPE_SINGLE = 1
TYPE_BATCH = 2
OUTER_HEADER = 6
SINGLE_META = 4
BATCH_META = 1
MAX_BODY = MAX_APP_PAYLOAD - OUTER_HEADER
MAX_SINGLE_DATA = MAX_BODY - SINGLE_META
MAX_RTCM = 1029


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def validate_rtcm(frame):
    if len(frame) < 6 or frame[0] != 0xD3:
        return False
    n = ((frame[1] & 0x03) << 8) | frame[2]
    if len(frame) != n + 6:
        return False
    return crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big")


def rtcm_type(frame):
    if len(frame) < 5:
        return -1
    return (frame[3] << 4) | (frame[4] >> 4)



def is_msm(t):
    return 1070 < t < 1130


def get_sync(frame):
    """MSM 'multiple message bit' (DF393): bit 54 of the message payload."""
    return (frame[9] >> 1) & 1


def set_sync(frame, flag):
    body = bytearray(frame[:-3])
    if flag:
        body[9] |= 0x02
    else:
        body[9] &= 0xFD
    body = bytes(body)
    return body + crc24q(body).to_bytes(3, "big")


def _bits(data, start, n):
    total = len(data) * 8
    return (int.from_bytes(data, "big") >> (total - start - n)) & ((1 << n) - 1)


def get_1005_ecef(frame):
    p = frame[3:-3]
    if len(p) < 19:
        return None
    out = []
    for start in (34, 74, 114):
        v = _bits(p, start, 38)
        if v & (1 << 37):
            v -= 1 << 38
        out.append(v * 0.0001)
    return tuple(out)


def set_1005_ecef(frame, x, y, z):
    p = frame[3:-3]
    total = len(p) * 8
    v = int.from_bytes(p, "big")
    for start, val in ((34, x), (74, y), (114, z)):
        field = round(val * 10000) & ((1 << 38) - 1)
        shift = total - start - 38
        v = (v & ~(((1 << 38) - 1) << shift)) | (field << shift)
    body = frame[:3] + v.to_bytes(len(p), "big")
    return body + crc24q(body).to_bytes(3, "big")


def plausible_ecef(pos):
    """Reject the BS's start-up placeholder (it reports the North Pole)."""
    x, y, z = pos
    return (x * x + y * y) ** 0.5 > 1.0e6 and 6.2e6 < (x * x + y * y + z * z) ** 0.5 < 6.5e6


def extract_rtcm(buf):
    out = []
    bad = 0
    while len(buf) >= 3:
        try:
            pos = buf.index(0xD3)
        except ValueError:
            buf.clear()
            break
        if pos:
            del buf[:pos]
        if len(buf) < 3:
            break
        if buf[1] & 0xFC:
            del buf[0]
            continue
        total = (((buf[1] & 0x03) << 8) | buf[2]) + 6
        if total > MAX_RTCM:
            del buf[0]
            continue
        if len(buf) < total:
            break
        candidate = bytes(buf[:total])
        if validate_rtcm(candidate):
            out.append(candidate)
            del buf[:total]
        else:
            bad += 1
            del buf[0]
    return out, bad


def make_packet(pkt_type, body):
    if len(body) > MAX_BODY:
        raise ValueError("transport body too large")
    return MAGIC + bytes([VERSION, pkt_type]) + len(body).to_bytes(2, "big") + body


def make_single(seq, index, total, data):
    return make_packet(TYPE_SINGLE, seq.to_bytes(2, "big") + bytes([index, total]) + data)


def make_batch(frames):
    body = bytearray([len(frames)])
    for frame in frames:
        body += len(frame).to_bytes(2, "big")
        body += frame
    return make_packet(TYPE_BATCH, bytes(body))


RAW = False


def frame_to_packets(frame, state):
    """One RTCM frame -> list of radio packets (a single batch, or fragments)."""
    if RAW:
        return [frame]                   # raw RTCM, no transport framing at all
    if BATCH_META + 2 + len(frame) <= MAX_BODY:
        return [make_batch([frame])]
    seq = state["seq"]
    state["seq"] = (seq + 1) & 0xFFFF
    total = (len(frame) + MAX_SINGLE_DATA - 1) // MAX_SINGLE_DATA
    if total > 255:
        raise ValueError("too many fragments")
    return [make_single(seq, i, total, frame[i * MAX_SINGLE_DATA:(i + 1) * MAX_SINGLE_DATA])
            for i in range(total)]


class Queue:
    """Queue of (enqueue_time, packets, perishable). Bounded: oldest perishable
    entries are discarded when it gets too deep."""

    def __init__(self, max_groups):
        self.q = deque()
        self.cv = threading.Condition()
        self.max_groups = max_groups
        self.max_depth = 0
        self.overflow_dropped = 0

    def put(self, packets, perishable):
        with self.cv:
            self.q.append((time.monotonic(), packets, perishable))
            while len(self.q) > self.max_groups:
                for i, item in enumerate(self.q):
                    if item[2]:
                        del self.q[i]
                        self.overflow_dropped += 1
                        break
                else:
                    break
            self.max_depth = max(self.max_depth, len(self.q))
            self.cv.notify()

    def get(self, timeout=0.1):
        with self.cv:
            if not self.q:
                self.cv.wait(timeout)
            return self.q.popleft() if self.q else None

    def depth(self):
        with self.cv:
            return len(self.q)


def send_fixed(ser, dst, channel, payload, check=True):
    if check and len(payload) > MAX_APP_PAYLOAD:
        raise ValueError(f"application packet {len(payload)} > {MAX_APP_PAYLOAD}")
    if dst is None:                      # transparent mode: no addressing bytes
        wire = payload
    else:
        wire = bytes([(dst >> 8) & 0xFF, dst & 0xFF, channel & 0xFF]) + payload
    ser.write(wire)
    ser.flush()
    return len(wire)


def main():
    global MAX_APP_PAYLOAD, MAX_BODY, MAX_SINGLE_DATA, RAW

    p = argparse.ArgumentParser(description="RTCM3 -> SX1262 base transmitter (with stale-drop)")
    p.add_argument("--port", required=True, help="SX1262 UART, e.g. /dev/ttyUSB1")
    p.add_argument("--dst-addr", type=int, default=None,
                   help="destination address (fixed mode). Omit with --transparent")
    p.add_argument("--transparent", action="store_true",
                   help="module is in transparent mode: no 3-byte address header")
    p.add_argument("--raw", action="store_true",
                   help="send raw RTCM bytes with NO transport headers (rover needs --raw too)")
    p.add_argument("--channel", type=int, default=CHANNEL)
    p.add_argument("--baud", type=int, default=UART_BAUD)
    p.add_argument("--delay", type=float, default=0.05,
                   help="delay after each radio packet (default 0.05 s)")
    p.add_argument("--types", default="1074",
                   help="comma list of RTCM types to send, or 'all' (default 1074)")
    p.add_argument("--pos-interval", type=float, default=10.0,
                   help="send 1005/1006 only every N seconds (default 10)")
    p.add_argument("--max-age", type=float, default=1.5,
                   help="drop observation frames that waited longer than this (default 1.5 s)")
    p.add_argument("--decimate", type=int, default=1,
                   help="send only every Nth observation epoch, but send each chosen epoch with ALL "
                        "selected types (default 1 = every epoch). Use with --max-age >= ~4.")
    p.add_argument("--hold-position", action="store_true",
                   help="rewrite every 1005 with the FIRST valid base position seen, so the broadcast "
                        "position never changes (a changing base position seems to reset the rover)")
    p.add_argument("--base-ecef", help="X,Y,Z in metres: broadcast exactly this base position")
    p.add_argument("--max-queue", type=int, default=8,
                   help="max frames waiting in the queue (default 8)")
    p.add_argument("--max-payload", type=int, default=MAX_APP_PAYLOAD,
                   help="radio packet payload size. MUST equal the packet length set on "
                        "BOTH modules (default 64)")
    args = p.parse_args()

    if serial is None:
        sys.exit("pyserial missing: pip install pyserial")
    RAW = args.raw
    if not args.transparent:
        if args.dst_addr is None or not 0 <= args.dst_addr <= 65535:
            sys.exit("--dst-addr 0..65535 is required unless --transparent")
    dst = None if args.transparent else args.dst_addr

    MAX_APP_PAYLOAD = args.max_payload
    MAX_BODY = MAX_APP_PAYLOAD - OUTER_HEADER
    MAX_SINGLE_DATA = MAX_BODY - SINGLE_META

    allowed = None if args.types.strip().lower() == "all" else \
        {int(x) for x in args.types.split(",") if x.strip()}

    ser = serial.Serial(args.port, args.baud, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    q = Queue(args.max_queue)
    state = dict(seq=0, source_frames=0, bad_source_crc=0, filtered=0, queued=0,
                 stale_dropped=0, packets_tx=0, lat_max=0.0)

    def worker():
        try:
            while True:
                item = q.get()
                if item is None:
                    continue
                t_enq, packets, perishable = item
                wait = time.monotonic() - t_enq
                if perishable and wait > args.max_age:
                    state["stale_dropped"] += 1
                    continue
                state["lat_max"] = max(state["lat_max"], wait)
                for pkt in packets:
                    send_fixed(ser, dst, args.channel, pkt, check=not RAW)
                    state["packets_tx"] += 1
                    if args.delay > 0:
                        time.sleep(args.delay)
        except Exception as e:
            sys.stderr.write(f"\n[TX ERROR] {e}\n")

    threading.Thread(target=worker, daemon=True).start()

    sys.stderr.write(
        "RTCM LoRa base TX\n"
        f"  LoRa UART: {args.port} @ {args.baud}\n"
        f"  destination: {'transparent' if dst is None else dst}, channel: {args.channel}\n"
        + ("  RAW mode: RTCM bytes sent as-is, no transport headers\n" if RAW else
           f"  packet payload: {MAX_APP_PAYLOAD} B, fragment data: {MAX_SINGLE_DATA} B\n") +
        f"  sending types: {'all' if allowed is None else sorted(allowed)} "
        f"| 1005 every {args.pos_interval:.0f}s | drop if waiting > {args.max_age}s"
        f" | every {args.decimate} epoch(s)\n\n"
    )

    ep = {"frames": [], "start": None}

    def flush_epoch():
        fr = ep["frames"]
        for i, f in enumerate(fr):
            want = 0 if i == len(fr) - 1 else 1     # last message sent = end of epoch
            if get_sync(f) != want:
                f = set_sync(f, want)
            q.put(frame_to_packets(f, state), perishable=True)
            state["queued"] += 1
        ep["frames"] = []
        ep["start"] = None

    raw = bytearray()
    held = None
    if args.base_ecef:
        held = tuple(float(v) for v in args.base_ecef.split(","))
        if len(held) != 3 or not plausible_ecef(held):
            sys.exit("--base-ecef needs three plausible ECEF values in metres")
    last_pos = None
    last_frame = None
    epoch_n = 0
    epoch_on = True
    last_status = time.monotonic()

    try:
        fd = sys.stdin.buffer.fileno()
        while True:
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            raw.extend(chunk)
            frames, bad = extract_rtcm(raw)
            state["bad_source_crc"] += bad
            now = time.monotonic()

            for frame in frames:
                state["source_frames"] += 1
                last_frame = frame
                t = rtcm_type(frame)
                if t == 1074:                      # GPS MSM4 marks the start of a new epoch
                    epoch_n += 1
                    epoch_on = ((epoch_n - 1) % max(args.decimate, 1) == 0)
                if t == 1005 and (args.hold_position or held is not None):
                    if held is None:
                        pos = get_1005_ecef(frame)
                        if pos and plausible_ecef(pos):
                            held = pos
                            sys.stderr.write(f"\n[HOLD] base position fixed at ECEF "
                                             f"{held[0]:.4f},{held[1]:.4f},{held[2]:.4f}\n")
                    if held is not None:
                        frame = set_1005_ecef(frame, *held)
                    else:
                        state["filtered"] += 1       # no valid position yet: don't send the placeholder
                        continue
                if t in (1005, 1006):
                    if last_pos is None or now - last_pos >= args.pos_interval:
                        last_pos = now
                        q.put(frame_to_packets(frame, state), perishable=False)
                        state["queued"] += 1
                    else:
                        state["filtered"] += 1
                elif is_msm(t):
                    if (allowed is None or t in allowed) and epoch_on:
                        ep["frames"].append(frame)
                        if ep["start"] is None:
                            ep["start"] = now
                    else:
                        state["filtered"] += 1
                    if get_sync(frame) == 0:        # BS marks the last message of its epoch
                        flush_epoch()
                elif allowed is None or t in allowed:   # non-MSM (ephemeris etc.)
                    q.put(frame_to_packets(frame, state), perishable=True)
                    state["queued"] += 1
                else:
                    state["filtered"] += 1

            if ep["frames"] and ep["start"] is not None and now - ep["start"] > 0.8:
                flush_epoch()                        # safety net if no end-of-epoch frame arrives

            if now - last_status >= 2:
                last = f"{rtcm_type(last_frame)}/{len(last_frame)}B" if last_frame else "-"
                sys.stderr.write(
                    f"\rSRC={state['source_frames']} QUEUED={state['queued']} "
                    f"FILT={state['filtered']} STALE_DROP={state['stale_dropped']} "
                    f"Q={q.depth()}/{q.max_depth} LAT={state['lat_max']:.2f}s "
                    f"TXPKT={state['packets_tx']} BADCRC={state['bad_source_crc']} "
                    f"last={last}   "
                )
                sys.stderr.flush()
                state["lat_max"] = 0.0
                last_status = now
    except KeyboardInterrupt:
        pass
    finally:
        sys.stderr.write("\nStopped.\n")
        ser.close()


if __name__ == "__main__":
    main()
