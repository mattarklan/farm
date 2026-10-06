#!/usr/bin/env python3
# RTCM3 over Waveshare SX1262 fixed-address mode.
# Transport packet:
#   A5 5A | ver | type | body_len(2) | body
#
# SINGLE body:
#   seq(2) | frag_index(1) | frag_total(1) | RTCM bytes
#
# BATCH body:
#   count(1) | [frame_len(2) | complete RTCM frame]...
#
# The explicit body_len is the important part: UART reads are a byte stream,
# so neither side assumes one serial.read() == one LoRa packet.

import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None

UART_BAUD = 9600
CHANNEL = 65
MAX_APP_PAYLOAD = 230
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
    expected = int.from_bytes(frame[-3:], "big")
    return crc24q(frame[:-3]) == expected

def rtcm_type(frame):
    if len(frame) < 5:
        return -1
    return (frame[3] << 4) | (frame[4] >> 4)

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
        # RTCM3's six reserved bits must be zero.
        if buf[1] & 0xFC:
            del buf[0]
            continue
        n = ((buf[1] & 0x03) << 8) | buf[2]
        total = n + 6
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
    body = seq.to_bytes(2, "big") + bytes([index, total]) + data
    return make_packet(TYPE_SINGLE, body)

def make_batch(frames):
    body = bytearray([len(frames)])
    for frame in frames:
        body += len(frame).to_bytes(2, "big")
        body += frame
    return make_packet(TYPE_BATCH, bytes(body))

def parse_transport(buf):
    """Streaming parser. Returns (records, discarded_runs).

    discarded_runs contains the actual bytes removed while resynchronizing,
    rather than only a count. This lets us see whether the loss is noise,
    partial transport packets, fixed-address metadata, etc.
    """
    records = []
    discarded_runs = []

    while True:
        if len(buf) < OUTER_HEADER:
            break

        pos = buf.find(MAGIC)
        if pos < 0:
            # Keep a trailing A5 because it could be the first byte of A5 5A
            # split across two UART reads.
            keep = 1 if buf and buf[-1] == MAGIC[0] else 0
            cut = len(buf) - keep
            if cut:
                discarded_runs.append(bytes(buf[:cut]))
                del buf[:cut]
            break

        if pos:
            discarded_runs.append(bytes(buf[:pos]))
            del buf[:pos]

        if len(buf) < OUTER_HEADER:
            break

        version = buf[2]
        pkt_type = buf[3]
        body_len = int.from_bytes(buf[4:6], "big")

        if version != VERSION or pkt_type not in (TYPE_SINGLE, TYPE_BATCH) or body_len > MAX_BODY:
            discarded_runs.append(bytes(buf[:1]))
            del buf[0]
            continue

        total = OUTER_HEADER + body_len
        if len(buf) < total:
            break

        records.append((pkt_type, bytes(buf[6:total])))
        del buf[:total]

    return records, discarded_runs


def hex_preview(data, limit=48):
    shown = data[:limit]
    text = " ".join(f"{b:02X}" for b in shown)
    if len(data) > limit:
        text += f" ... (+{len(data) - limit}B)"
    return text

class Reassembler:
    def __init__(self):
        self.sets = {}
    def add(self, body):
        if len(body) < SINGLE_META:
            return None, "short"
        seq = int.from_bytes(body[0:2], "big")
        idx, total = body[2], body[3]
        data = body[4:]
        if total == 0 or idx >= total:
            return None, "meta"
        entry = self.sets.get(seq)
        if entry is None or entry["total"] != total:
            entry = {"total": total, "parts": {}, "t": time.monotonic()}
            self.sets[seq] = entry
        entry["parts"][idx] = data

        # Expire old incomplete messages.
        now = time.monotonic()
        for old in list(self.sets):
            if old != seq and now - self.sets[old]["t"] > 5:
                del self.sets[old]

        if len(entry["parts"]) != total:
            return None, None
        try:
            frame = b"".join(entry["parts"][i] for i in range(total))
        except KeyError:
            return None, None
        del self.sets[seq]
        return frame, None

def unpack_batch(body):
    if not body:
        return None
    count = body[0]
    pos = 1
    frames = []
    for _ in range(count):
        if pos + 2 > len(body):
            return None
        n = int.from_bytes(body[pos:pos+2], "big")
        pos += 2
        if n < 6 or pos + n > len(body):
            return None
        frames.append(body[pos:pos+n])
        pos += n
    if pos != len(body):
        return None
    return frames

def main():
    p = argparse.ArgumentParser(description="Minimal SX1262 -> RTCM3 receiver/diagnostic")
    p.add_argument("--port", default="COM3")
    p.add_argument("--baud", type=int, default=UART_BAUD)
    p.add_argument("--output", help="optional file to write valid RTCM3 bytes")
    p.add_argument("--stdout-rtcm", action="store_true",
                   help="write valid RTCM binary to stdout (don't use for visual diagnostics)")
    args = p.parse_args()

    if serial is None:
        sys.exit("pyserial missing: pip install pyserial")

    ser = serial.Serial(args.port, args.baud, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    out = open(args.output, "ab", buffering=0) if args.output else None
    buf = bytearray()
    reasm = Reassembler()
    s = dict(raw=0, transport=0, discarded=0, batches=0, fragments=0,
             rtcm=0, rtcm_bytes=0, crc_bad=0, malformed=0)
    last = None
    start = last_status = time.monotonic()

    print("RTCM LoRa rover RX diagnostic", file=sys.stderr)
    print(f"  port: {args.port} @ {args.baud}", file=sys.stderr)
    print("  explicit-length transport framing + desync hex diagnostics", file=sys.stderr)
    print("  Ctrl+C to stop.\n", file=sys.stderr)

    def accept(frame, source):
        nonlocal last
        if not validate_rtcm(frame):
            s["crc_bad"] += 1
            print(f"[RTCM BAD CRC] source={source} len={len(frame)}", file=sys.stderr)
            return
        typ = rtcm_type(frame)
        s["rtcm"] += 1
        s["rtcm_bytes"] += len(frame)
        last = (typ, len(frame))
        print(f"[RTCM OK] type={typ:4d} frame={len(frame):4d}B source={source}",
              file=sys.stderr)
        if out:
            out.write(frame)
        if args.stdout_rtcm:
            sys.stdout.buffer.write(frame)
            sys.stdout.buffer.flush()

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                s["raw"] += len(chunk)
                buf.extend(chunk)
                records, discarded_runs = parse_transport(buf)

                for lost in discarded_runs:
                    s["discarded"] += len(lost)
                    print(
                        f"[DESYNC] discarded={len(lost)}B "
                        f"total={s['discarded']} hex={hex_preview(lost)}",
                        file=sys.stderr
                    )

                for typ, body in records:
                    s["transport"] += 1
                    if typ == TYPE_BATCH:
                        s["batches"] += 1
                        frames = unpack_batch(body)
                        count = body[0] if body else -1
                        print(
                            f"[PKT BATCH] body={len(body)}B frames={count}",
                            file=sys.stderr
                        )
                        if frames is None:
                            s["malformed"] += 1
                            print("[BAD TRANSPORT] malformed batch", file=sys.stderr)
                            continue
                        for frame in frames:
                            accept(frame, "batch")
                    else:
                        s["fragments"] += 1
                        if len(body) >= SINGLE_META:
                            seq = int.from_bytes(body[0:2], "big")
                            idx = body[2]
                            total = body[3]
                            data_len = len(body) - SINGLE_META
                            print(
                                f"[PKT FRAG] seq={seq} frag={idx + 1}/{total} "
                                f"data={data_len}B body={len(body)}B",
                                file=sys.stderr
                            )
                        else:
                            print(
                                f"[PKT FRAG] short body={len(body)}B "
                                f"hex={hex_preview(body)}",
                                file=sys.stderr
                            )
                        frame, err = reasm.add(body)
                        if err:
                            s["malformed"] += 1
                            print(f"[BAD TRANSPORT] fragment {err}", file=sys.stderr)
                        elif frame is not None:
                            accept(frame, "reassembled")

            now = time.monotonic()
            if now - last_status >= 2:
                elapsed = max(now - start, 0.001)
                last_s = f"{last[0]}/{last[1]}B" if last else "-"
                print(
                    f"[STATUS] RX={s['raw']}B avg={s['raw']/elapsed:.1f}B/s "
                    f"PKT={s['transport']} BATCH={s['batches']} FRAG={s['fragments']} "
                    f"RTCM={s['rtcm']} CRC_BAD={s['crc_bad']} "
                    f"MALFORMED={s['malformed']} DISCARD={s['discarded']} "
                    f"BUF={len(buf)} last={last_s}",
                    file=sys.stderr
                )
                last_status = now
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
    finally:
        if out:
            out.close()
        ser.close()

if __name__ == "__main__":
    main()
