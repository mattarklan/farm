#!/usr/bin/env python3
"""
LoRa (SX1262) -> RTCM3 -> LC29H DA rover, with live fix status. Standalone.

  python rover_rtk.py --list
  python rover_rtk.py --lora COM3 --gnss COM12
  python rover_rtk.py --lora COM3 --gnss COM12 --rtcm-port COM13

--gnss      port the LC29H sends NMEA (GGA) on
--rtcm-port port RTCM is written to (defaults to --gnss). If the LC29H shows up
            as two USB COM ports, the RTCM input is usually the OTHER one.
"""

import argparse
import math
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

# ---------------- transport / RTCM (from rover_rx.py) ----------------
UART_BAUD = 9600
GNSS_BAUD = 115200
GPS_UTC_LEAP = 18  # GPS time is ahead of UTC by this many seconds
MAX_APP_PAYLOAD = 230
MAGIC = b"\xA5\x5A"
VERSION = 1
TYPE_SINGLE = 1
TYPE_BATCH = 2
OUTER_HEADER = 6
SINGLE_META = 4
MAX_BODY = MAX_APP_PAYLOAD - OUTER_HEADER

FIX_STATUS_MAP = {
    '0': 'No Fix / Invalid',
    '1': 'Autonomous',
    '2': 'DGPS (Differential)',
    '4': 'Fixed RTK',
    '5': 'Float RTK',
}


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


def parse_transport(buf):
    records = []
    discarded = 0
    while len(buf) >= OUTER_HEADER:
        pos = buf.find(MAGIC)
        if pos < 0:
            keep = 1 if buf and buf[-1] == MAGIC[0] else 0
            cut = len(buf) - keep
            if cut:
                discarded += cut
                del buf[:cut]
            break
        if pos:
            discarded += pos
            del buf[:pos]
        if len(buf) < OUTER_HEADER:
            break
        version, pkt_type = buf[2], buf[3]
        body_len = int.from_bytes(buf[4:6], "big")
        if version != VERSION or pkt_type not in (TYPE_SINGLE, TYPE_BATCH) or body_len > MAX_BODY:
            discarded += 1
            del buf[0]
            continue
        total = OUTER_HEADER + body_len
        if len(buf) < total:
            break
        records.append((pkt_type, bytes(buf[6:total])))
        del buf[:total]
    return records, discarded


class Reassembler:
    def __init__(self):
        self.sets = {}

    def add(self, body):
        if len(body) < SINGLE_META:
            return None
        seq = int.from_bytes(body[0:2], "big")
        idx, total = body[2], body[3]
        data = body[4:]
        if total == 0 or idx >= total:
            return None
        entry = self.sets.get(seq)
        if entry is None or entry["total"] != total:
            entry = {"total": total, "parts": {}, "t": time.monotonic()}
            self.sets[seq] = entry
        entry["parts"][idx] = data
        now = time.monotonic()
        for old in list(self.sets):
            if old != seq and now - self.sets[old]["t"] > 5:
                del self.sets[old]
        if len(entry["parts"]) != total:
            return None
        try:
            frame = b"".join(entry["parts"][i] for i in range(total))
        except KeyError:
            return None
        del self.sets[seq]
        return frame


def unpack_batch(body):
    if not body:
        return None
    count = body[0]
    pos = 1
    frames = []
    for _ in range(count):
        if pos + 2 > len(body):
            return None
        n = int.from_bytes(body[pos:pos + 2], "big")
        pos += 2
        if n < 6 or pos + n > len(body):
            return None
        frames.append(body[pos:pos + n])
        pos += n
    if pos != len(body):
        return None
    return frames


def bits(data, start, n):
    total = len(data) * 8
    return (int.from_bytes(data, "big") >> (total - start - n)) & ((1 << n) - 1)


def signed(v, n):
    return v - (1 << n) if v & (1 << (n - 1)) else v


def decode_1005(frame):
    """Return base ECEF (m) from a 1005 frame, or None."""
    p = frame[3:-3]
    if len(p) < 19:
        return None
    x = signed(bits(p, 34, 38), 38) * 0.0001
    y = signed(bits(p, 74, 38), 38) * 0.0001
    z = signed(bits(p, 114, 38), 38) * 0.0001
    return x, y, z


def ecef_to_llh(x, y, z):
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = f * (2 - f)
    lon = math.atan2(y, x)
    pr = math.hypot(x, y)
    lat = math.atan2(z, pr * (1 - e2))
    h = 0.0
    for _ in range(6):
        nn = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = pr / math.cos(lat) - nn if abs(math.cos(lat)) > 1e-9 else abs(z) - nn * (1 - e2)
        lat = math.atan2(z, pr * (1 - e2 * nn / (nn + h)))
    return math.degrees(lat), math.degrees(lon), h


def nmea_deg(val, hemi):
    if not val:
        return None
    dot = val.index('.')
    deg = float(val[:dot - 2]) + float(val[dot - 2:]) / 60
    return -deg if hemi in ('S', 'W') else deg


def dist_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dl = math.radians(lon2 - lon1)
    h = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


def extract_rtcm(buf):
    out = []
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
        if len(buf) < total:
            break
        cand = bytes(buf[:total])
        if validate_rtcm(cand):
            out.append(cand)
            del buf[:total]
        else:
            del buf[0]
    return out


def msm4_quality(frame):
    """Decode C/N0 and lock-time indicators from an MSM4 message. Returns (cnrs, locks) or None."""
    p = frame[3:-3]
    if len(p) < 24:
        return None
    nsat = bin(bits(p, 73, 64)).count('1')
    nsig = bin(bits(p, 137, 32)).count('1')
    mask_len = nsat * nsig
    if nsat == 0 or nsig == 0 or 169 + mask_len > len(p) * 8:
        return None
    n = bin(bits(p, 169, mask_len)).count('1')            # number of satellite/signal cells
    start = 169 + mask_len + nsat * 18                    # after per-satellite data (8 + 10 bits)
    if n == 0 or start + n * 48 > len(p) * 8:
        return None
    lock0 = start + n * 37                                # after fine pseudorange (15) + phase (22)
    cnr0 = lock0 + n * 5                                  # after lock time (4) + half-cycle (1)
    cnrs = [bits(p, cnr0 + 6 * i, 6) for i in range(n)]
    locks = [bits(p, lock0 + 4 * i, 4) for i in range(n)]
    return [c for c in cnrs if c > 0], locks


def parse_gsv(line, store):
    f = line[1:].split('*')[0].split(',')
    sig = None
    if len(f) > 4 and (len(f) - 4) % 4 == 1:
        sig = f[-1]
        f = f[:-1]
    now = time.monotonic()
    for i in range(4, len(f) - 3, 4):
        prn, snr = f[i], f[i + 3]
        if prn and snr:
            try:
                store[(prn, sig)] = (float(snr), now)
            except ValueError:
                pass


def recent_avg(store, window=3.0):
    now = time.monotonic()
    v = [x for x, t in store.values() if now - t <= window and x > 0]
    return (sum(v) / len(v), len(v)) if v else None


def recent_by_sig(store, window=3.0):
    """Average rover C/N0 per NMEA signal id."""
    now = time.monotonic()
    groups = {}
    for (prn, sig), (x, t) in store.items():
        if now - t <= window and x > 0:
            groups.setdefault(sig or "?", []).append(x)
    return {k: (sum(v) / len(v), len(v)) for k, v in groups.items()}


# ---------------- NMEA ----------------
def nmea_ok(line):
    if not line.startswith('$') or '*' not in line:
        return False
    body, _, cs = line[1:].partition('*')
    try:
        expected = int(cs[:2], 16)
    except ValueError:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return calc == expected


def parse_gga(line):
    f = line.split(',')
    if len(f) < 14:
        return None
    return {'quality': f[6], 'sats': f[7] or '?', 'hdop': f[8] or '?', 'age': f[13] or '-',
            'lat': nmea_deg(f[2], f[3]), 'lon': nmea_deg(f[4], f[5]),
            'utc': (int(f[1][0:2]) * 3600 + int(f[1][2:4]) * 60 + float(f[1][4:])) if len(f[1]) >= 6 else None}


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
    for p in ports:
        print(f"{p.device:8} {p.description}")


def main():
    p = argparse.ArgumentParser(description="LoRa RTCM -> LC29H rover + fix status")
    p.add_argument("--list", action="store_true", help="list COM ports and exit")
    p.add_argument("--lora", default="COM3", help="SX1262 serial port")
    p.add_argument("--lora-baud", type=int, default=UART_BAUD)
    p.add_argument("--gnss", default="COM12", help="LC29H port that outputs NMEA")
    p.add_argument("--gnss-baud", type=int, default=GNSS_BAUD)
    p.add_argument("--rtcm-port", help="port to write RTCM to (default: same as --gnss)")
    p.add_argument("--rtcm-baud", type=int, default=GNSS_BAUD)
    p.add_argument("--direct", help="read RTCM straight from this port (e.g. the BS by USB) instead of LoRa")
    p.add_argument("--direct-baud", type=int, default=115200)
    p.add_argument("--raw", action="store_true",
                   help="LoRa carries raw RTCM (base used --raw): no transport framing")
    p.add_argument("--verbose", action="store_true", help="log every RTCM frame")
    args = p.parse_args()

    if args.list:
        list_ports()
        return

    lora = None if args.direct else serial.Serial(args.lora, args.lora_baud, timeout=0)
    direct = serial.Serial(args.direct, args.direct_baud, timeout=0) if args.direct else None
    direct_buf = bytearray()
    gnss = serial.Serial(args.gnss, args.gnss_baud, timeout=0, write_timeout=1)
    if args.rtcm_port and args.rtcm_port.upper() != args.gnss.upper():
        rtcm_out = serial.Serial(args.rtcm_port, args.rtcm_baud, timeout=0, write_timeout=1)
    else:
        rtcm_out = gnss
    time.sleep(0.3)
    if lora:
        lora.reset_input_buffer()
    if direct:
        direct.reset_input_buffer()
    gnss.reset_input_buffer()

    if direct:
        print(f"RTCM src   : {args.direct} @ {args.direct_baud} (direct, no LoRa)")
    else:
        print(f"LoRa       : {args.lora} @ {args.lora_baud}")
    print(f"NMEA in    : {args.gnss} @ {args.gnss_baud}")
    print(f"RTCM out   : {rtcm_out.port} @ {rtcm_out.baudrate}")
    print("Ctrl+C to stop.\n")

    lora_buf = bytearray()
    nmea_buf = bytearray()
    reasm = Reassembler()
    rtcm_count = rtcm_bytes = crc_bad = discarded = data_in = 0
    last_type = None
    gga = None
    last_quality = None
    last_status = last_detail = t0 = time.monotonic()
    type_counts = {}
    type_bytes = {}
    info = {'sync': {}}
    gsv = {}

    def forward(frame, source):
        nonlocal rtcm_count, rtcm_bytes, crc_bad, last_type
        if not validate_rtcm(frame):
            crc_bad += 1
            return
        rtcm_out.write(frame)
        rtcm_count += 1
        rtcm_bytes += len(frame)
        last_type = rtcm_type(frame)
        type_counts[last_type] = type_counts.get(last_type, 0) + 1
        type_bytes[last_type] = type_bytes.get(last_type, 0) + len(frame)

        if last_type == 1005:
            info['base'] = decode_1005(frame)
        elif last_type in (1074, 1084, 1094, 1124):
            qual = msm4_quality(frame)
            if qual and qual[0]:
                cn, lk = qual
                info[f'q_{last_type}'] = (sum(cn) / len(cn), min(cn), sum(1 for x in lk if x < 10), len(lk))
            if len(frame) > 24:
                info[f'sats_{last_type}'] = bin(bits(frame[3:-3], 73, 64)).count('1')

            if last_type == 1074:
                tow = bits(frame[3:-3], 24, 30)
                if 'tow' in info:
                    info['gap'] = ((tow - info['tow']) % 604800000) / 1000.0
                info['tow'] = tow
                if gga and gga.get('utc') is not None and 'gga_mono' in info:
                    u = gga['utc'] + (time.monotonic() - info['gga_mono'])
                    tod = (tow / 1000.0) % 86400
                    info['age'] = (u + GPS_UTC_LEAP - tod + 43200) % 86400 - 43200
                    info['age_t'] = time.monotonic()

            elif last_type == 1084 and gga and gga.get('utc') is not None and 'gga_mono' in info:
                tod = bits(frame[3:-3], 27, 27) / 1000.0
                u = gga['utc'] + (time.monotonic() - info['gga_mono'])
                info['age'] = (u + 10800 - tod + 43200) % 86400 - 43200
                info['age_t'] = time.monotonic()

        if 1070 < last_type < 1130 and len(frame) > 9:
            info['sync'][last_type] = (frame[9] >> 1) & 1     # MSM 'more messages follow' flag

        if args.verbose:
            print(f"[RTCM -> LC29H] type={last_type} {len(frame)}B ({source})")

    try:
        while True:
            busy = False

            if direct:
                n = direct.in_waiting
                if n:
                    busy = True
                    direct_buf.extend(direct.read(n))
                    for fr in extract_rtcm(direct_buf):
                        forward(fr, "direct")

            n = lora.in_waiting if lora else 0
            if n:
                busy = True
                lora_buf.extend(lora.read(n))
                if args.raw:
                    data_in += n
                    for fr in extract_rtcm(lora_buf):
                        forward(fr, "raw")
                    discarded = data_in - rtcm_bytes - len(lora_buf)
                    records = []
                else:
                    records, lost = parse_transport(lora_buf)
                    discarded += lost
                for typ, body in records:
                    if typ == TYPE_BATCH:
                        frames = unpack_batch(body)
                        for fr in frames or []:
                            forward(fr, "batch")
                    else:
                        frame = reasm.add(body)
                        if frame is not None:
                            forward(frame, "reassembled")

            n = gnss.in_waiting
            if n:
                busy = True
                nmea_buf.extend(gnss.read(n))
                while b'\n' in nmea_buf:
                    raw, _, rest = bytes(nmea_buf).partition(b'\n')
                    nmea_buf = bytearray(rest)
                    line = raw.decode('ascii', errors='ignore').strip()
                    if any(line.startswith(p) for p in ('$GPGSV', '$GLGSV', '$GAGSV', '$GBGSV', '$GNGSV')) and nmea_ok(line):
                        parse_gsv(line, gsv)
                    if 'GGA' in line[:7] and nmea_ok(line):
                        g = parse_gga(line)
                        if g:
                            gga = g
                            info['gga_mono'] = time.monotonic()
                            if g['quality'] != last_quality:
                                last_quality = g['quality']
                                name = FIX_STATUS_MAP.get(g['quality'], f"Other ({g['quality']})")
                                print(f"\n*** FIX CHANGED: {name} ***\n")
                if len(nmea_buf) > 4096:
                    nmea_buf.clear()

            now = time.monotonic()
            if now - last_status >= 2:
                last_status = now
                if gga:
                    name = FIX_STATUS_MAP.get(gga['quality'], f"Other ({gga['quality']})")
                    stale = now - info.get('gga_mono', now)
                    if stale > 3:
                        gps = f"NO GGA FOR {stale:.0f}s (last: {name}) - NMEA from LC29H stalled?"
                    else:
                        gps = f"{name:<20} sats={gga['sats']:>2} hdop={gga['hdop']} corr_age={gga['age']}s"
                else:
                    gps = "no GGA yet (check LC29H port/baud)"
                print(f"[GNSS] {gps} | [LoRa] RTCM={rtcm_count} ({rtcm_bytes}B) "
                      f"last_type={last_type} crc_bad={crc_bad} discard={discarded}B")

            if now - last_detail >= 5:
                last_detail = now
                counts = " ".join(f"{k}:{v}x{type_bytes[k] // v}B" for k, v in sorted(type_counts.items()))
                line = f"[BASE] count x avg-size {counts} | avg rate={rtcm_bytes / max(now - t0, 1):.0f}B/s"
                if info.get('base'):
                    blat, blon, bh = ecef_to_llh(*info['base'])
                    line += f" | base={blat:.7f},{blon:.7f} h={bh:.1f}m"
                    line += f" ecef={info['base'][0]:.4f},{info['base'][1]:.4f},{info['base'][2]:.4f}"
                    if gga and gga.get('lat') is not None and gga['lat'] != 0:
                        d = dist_m(blat, blon, gga['lat'], gga['lon'])
                        line += f" | base-rover dist={d:.1f}m"
                if 'gap' in info:
                    line += f" | last GPS(1074) epoch gap={info['gap']:.1f}s"
                
                # Report active constellation stats
                msm_sats = []
                for typ, name in ((1074, 'GPS'), (1084, 'GLO'), (1094, 'GAL'), (1124, 'BDS')):
                    if f'sats_{typ}' in info:
                        msm_sats.append(f"{name}:{info[f'sats_{typ}']}")
                if msm_sats:
                    line += " | base tracks " + " ".join(msm_sats)

                ra = recent_avg(gsv)
                if ra:
                    line += f" | ROVER C/N0 avg={ra[0]:.0f} dBHz ({ra[1]} sigs)"
                    bands = recent_by_sig(gsv)
                    if len(bands) > 1:
                        line += " [" + ", ".join(f"sig{k}: {v[0]:.0f} dBHz x{v[1]}" for k, v in sorted(bands.items())) + "]"
                if info['sync']:
                    line += " | MORE-FOLLOW-flag " + " ".join(f"{k}:{v}" for k, v in sorted(info['sync'].items()))
                if 'age' in info:
                    line += f" | BASE DATA AGE ~{info['age'] + (now - info['age_t']):.0f}s"
                print(line)

            if not busy:
                time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as e:
        print(f"\nSerial error: {e}")
    finally:
        if lora:
            lora.close()
        if direct:
            direct.close()
        gnss.close()
        if rtcm_out is not gnss:
            rtcm_out.close()


if __name__ == "__main__":
    main()