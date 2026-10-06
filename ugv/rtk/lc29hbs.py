#!/usr/bin/env python3
"""
lc29h_bs_config.py

Configures a Quectel LC29H(BS) GPS/RTK HAT as an RTK base station,
entirely over serial -- no QGNSS, no Windows required. Works the same
on a Raspberry Pi (/dev/ttyS0, jumper on B) or over USB on any OS
(/dev/ttyUSBx or COMx, jumper on A).

Command bodies (PQTM.../PAIR... names, IDs, and field layouts) are
transcribed from Quectel's own protocol documents:
    Quectel_LC29H(BS)_GNSS_Protocol_Specification_V1.0.pdf
    Quectel_LC29H&LC79H_Series_GNSS_Protocol_Specification_V1.1.pdf
NOTE ON VERIFICATION: this environment doesn't have those PDFs available
to check against, so the exact command names/IDs (e.g. PAIR432/433/434/
435/436/437, PQTMCFGSVIN, PQTMCFGMSGRATE) have NOT been independently
re-verified here -- they're transcribed as originally written and should
be treated as unconfirmed until you've tested each one against a live
module and checked the response. Two things ARE independently verified
in this file: the checksum algorithm itself (see the test block at the
bottom -- it's checked against a standard, vendor-independent NMEA
checksum example, since we don't have Quectel's own worked examples to
test against), and the $PQTMSVINSTATUS field-offset parsing in
_parse_svinstatus(), which was hand-checked against the worked example
quoted in that function's own docstring.

Default baud rate for this module is 115200 (different from the LoRa
HAT, which talks to the host at 9600 -- don't mix these up).

USAGE EXAMPLES

Check it's alive (reads firmware version, no config changes):
    python3 lc29h_bs_config.py --port /dev/ttyUSB0 version

Quick bench-test survey-in (120s, loose 5m target -- NOT for production):
    python3 lc29h_bs_config.py --port /dev/ttyUSB0 survey-in --duration 120 --accuracy 5

Check survey-in progress/result:
    python3 lc29h_bs_config.py --port /dev/ttyUSB0 survey-in --get

Set a fixed known position instead of surveying (ECEF X/Y/Z in metres):
    python3 lc29h_bs_config.py --port /dev/ttyUSB0 fixed --ecef -2472446.4619 4828304.1363 3343730.2653

Trim the RTCM output the way we discussed (MSM4, no ephemeris relay, ARP on):
    python3 lc29h_bs_config.py --port /dev/ttyUSB0 rtcm --mode msm4
    python3 lc29h_bs_config.py --port /dev/ttyUSB0 rtcm --ephemeris off
    python3 lc29h_bs_config.py --port /dev/ttyUSB0 rtcm --arp on
    python3 lc29h_bs_config.py --port /dev/ttyUSB0 rtcm --status   (reads all three back)

Make it permanent (survives power-off) and restart cleanly, no unplugging needed:
    python3 lc29h_bs_config.py --port /dev/ttyUSB0 save

Run the built-in self-test (checksum + field-parsing, no hardware needed):
    python3 lc29h_bs_config.py --selftest
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None


DEFAULT_BAUD = 115200


# =====================================================================
# NMEA-style checksum: 8-bit XOR of everything between $ and *.
# Verified in the test block at the bottom against a standard,
# vendor-independent NMEA checksum example (this environment doesn't
# have Quectel's own worked examples to test against, so this confirms
# the algorithm itself is a correct implementation of the standard NMEA
# checksum -- it does NOT confirm any Quectel-specific command syntax).
# =====================================================================
def checksum(body: str) -> str:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return format(c, "02X")


def build_sentence(body: str) -> bytes:
    return f"${body}*{checksum(body)}\r\n".encode("ascii")


# =====================================================================
# Low-level send/receive. The module replies with one or more NMEA-style
# lines; we read for a short window and return everything we got, since
# some commands (PAIR) return an intermediate $PAIR001 ack line THEN a
# separate result line.
# =====================================================================
def _is_valid_sentence(line: str) -> bool:
    """The base station also streams continuous binary RTCM3 output on this
    same UART once running. That binary data can coincidentally contain
    \\r\\n byte pairs, which would otherwise get misread as a spurious ASCII
    line. Guard against that: only accept lines that look like a real NMEA
    sentence (start with $, contain a checksum) AND whose checksum actually
    matches -- anything else is treated as noise and dropped."""
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cksum = line[1:].rpartition("*")
    return len(cksum) >= 2 and checksum(body) == cksum[:2].upper()


def send_command(ser, body: str, timeout: float = 1.5, quiet_period: float = 0.15) -> list:
    """Sends a command and collects valid response lines.

    Previously this always waited out the FULL `timeout` window on every
    call, even after the module had already replied -- with no early exit.
    That meant a command sequence like plain 'rtcm --status' (three separate
    send_command calls, for mode/ARP/ephemeris) always took the full
    3 x 1.5s = 4.5s no matter how fast the module actually answered, which
    is what caused the several-second delay observed before a command
    showed as sent.

    Now: once at least one valid response line has been received, the wait
    ends after `quiet_period` seconds of no further incoming bytes, rather
    than continuing to burn the rest of `timeout`. If the module never
    responds at all, `timeout` still applies as the maximum wait -- this
    only shortens the *common* case where a response already arrived."""
    ser.reset_input_buffer()
    sentence = build_sentence(body)
    ser.write(sentence)
    ser.flush()

    lines = []
    buf = b""
    deadline = time.time() + timeout
    last_activity = None

    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            last_activity = time.time()
            buf += chunk
            while b"\r\n" in buf:
                raw, buf = buf.split(b"\r\n", 1)
                if raw:
                    line = raw.decode("ascii", errors="replace")
                    if _is_valid_sentence(line):
                        lines.append(line)
                    # else: silently discarded -- almost certainly a stray
                    # \r\n found inside interleaved binary RTCM3 output,
                    # not a real response.
        else:
            # Once we've received at least one valid line and the module
            # has been quiet for `quiet_period`, stop waiting early rather
            # than burning the rest of `timeout`.
            if lines and last_activity is not None and (time.time() - last_activity) >= quiet_period:
                break
            time.sleep(0.02)
    return lines


def check_result(lines: list, expect_startswith: str) -> bool:
    return any(line.startswith(expect_startswith) for line in lines)


# =====================================================================
# PQTM commands (LC29H(BS)-specific)
# UNVERIFIED (see module docstring): command names/IDs below are
# transcribed from Quectel's spec but not re-checked against it here.
# =====================================================================
def get_version(ser):
    lines = send_command(ser, "PQTMVERNO")
    for line in lines:
        print(line)
    return lines


def survey_in_set(ser, min_dur: int, acc_limit: float, mode: int = 1, ecef=(0.0, 0.0, 0.0)):
    """mode=1: survey-in (weighted-average over min_dur seconds, target acc_limit metres).
    mode=2: fixed (use ecef directly, ignoring min_dur/acc_limit -- see fixed_position_set).

    Floats are formatted to fixed decimal notation explicitly rather than
    relying on Python's default str(float) -- an unformatted value like
    0.00001 renders as '1e-05' in scientific notation, which this module's
    plain-decimal ASCII parser won't accept. ECEF coordinates in normal use
    are always large (millions of metres) so this wouldn't trigger from
    realistic inputs, but it costs nothing to guarantee it can't happen."""
    body = (f"PQTMCFGSVIN,W,{mode},{min_dur},{acc_limit:.4f},"
            f"{ecef[0]:.4f},{ecef[1]:.4f},{ecef[2]:.4f}")
    lines = send_command(ser, body)
    ok = check_result(lines, "$PQTMCFGSVIN,OK")
    for line in lines:
        print(line)
    return ok


def survey_in_get(ser):
    """WARNING: this only echoes back the CONFIGURED parameters (mode, duration,
    accuracy target, and whatever ECEF you set) -- it is NOT a live survey-in
    readout. In survey-in mode you configure ECEF as 0,0,0, so this will show
    0,0,0 forever, no matter how long you wait. Use survey_status_watch() instead
    to see actual live progress."""
    lines = send_command(ser, "PQTMCFGSVIN,R")
    for line in lines:
        print(line)
    return lines


def survey_status_set_rate(ser, enable: bool):
    """Turns the live $PQTMSVINSTATUS output on/off. Must be enabled before you
    can see real survey-in progress -- it's not on by default."""
    lines = send_command(ser, f"PQTMCFGMSGRATE,W,PQTMSVINSTATUS,{1 if enable else 0},1")
    for line in lines:
        print(line)
    return check_result(lines, "$PQTMCFGMSGRATE,OK")


def _parse_svinstatus(line: str):
    """Field layout per Quectel's spec (2.3.9 PQTMSVINSTATUS): after the sentence
    name itself (index 0), fields are MsgVer,TOW,Valid,Res0,Res1,Obs,CfgDur,
    MeanX,MeanY,MeanZ,MeanAcc -- so Valid is index 3, and the mean position/
    accuracy are always the last 4 fields regardless of how many reserved
    fields precede them. VERIFIED: hand-checked field-by-field against the
    spec's own worked example (also exercised in the --selftest block):
    $PQTMSVINSTATUS,1,2241,1,,01,538,43200,-2472436.0802,4828383.0026,3343698.4839,9.5"""
    body = line[1:].split("*", 1)[0]
    fields = body.split(",")
    if len(fields) < 8 or not fields[0].startswith("PQTMSVINSTATUS"):
        return None
    try:
        valid = int(fields[3])
        mean_x, mean_y, mean_z, mean_acc = (float(x) for x in fields[-4:])
    except ValueError:
        return None
    return {"valid": valid, "mean_x": mean_x, "mean_y": mean_y,
            "mean_z": mean_z, "mean_acc": mean_acc}


def survey_status_watch(ser, duration: float):
    """Enables live survey-in status output, watches it for `duration` seconds,
    and prints a clean decoded summary of the latest reading -- this is the
    actual way to check whether survey-in is progressing or has converged."""
    survey_status_set_rate(ser, True)
    print(f"\nWatching for {duration}s (Valid: 0=invalid, 1=in-progress, 2=converged)...\n")

    ser.reset_input_buffer()
    buf = b""
    latest = None
    deadline = time.time() + duration
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            while b"\r\n" in buf:
                raw, buf = buf.split(b"\r\n", 1)
                if raw:
                    line = raw.decode("ascii", errors="replace")
                    parsed = _parse_svinstatus(line)
                    if parsed:
                        latest = parsed
                        status = {0: "invalid", 1: "in-progress", 2: "CONVERGED"}.get(
                            parsed["valid"], f"unknown({parsed['valid']})")
                        print(f"  status={status:12s} "
                              f"ECEF=({parsed['mean_x']:.4f}, {parsed['mean_y']:.4f}, {parsed['mean_z']:.4f})  "
                              f"accuracy={parsed['mean_acc']:.3f}m")
        else:
            time.sleep(0.02)

    survey_status_set_rate(ser, False)
    if latest is None:
        print("\nNo $PQTMSVINSTATUS lines seen at all -- either survey-in isn't "
              "configured/running, or this message isn't supported in your firmware. "
              "Try 'monitor' to check raw output, and confirm 'survey-in' was run first.")
    elif latest["valid"] == 2:
        print(f"\nConverged. Final position: ECEF ({latest['mean_x']:.4f}, "
              f"{latest['mean_y']:.4f}, {latest['mean_z']:.4f}), accuracy {latest['mean_acc']:.3f}m.")
        print("Use this with 'fixed --ecef X Y Z' to lock it in permanently, or just leave "
              "survey-in running if you're happy with it converging fresh each boot.")
    else:
        print(f"\nStill in-progress (accuracy target not yet reached). Current best estimate: "
              f"accuracy={latest['mean_acc']:.3f}m. Let it keep running.")
    return latest


def fixed_position_set(ser, ecef_x: float, ecef_y: float, ecef_z: float):
    return survey_in_set(ser, min_dur=0, acc_limit=0, mode=2, ecef=(ecef_x, ecef_y, ecef_z))


def save_settings(ser):
    lines = send_command(ser, "PQTMSAVEPAR")
    ok = check_result(lines, "$PQTMSAVEPAR,OK")
    for line in lines:
        print(line)
    return ok


# =====================================================================
# PAIR commands (RTCM output control)
# UNVERIFIED (see module docstring): command IDs below are transcribed
# from Quectel's spec but not re-checked against it here.
# =====================================================================
def rtcm_set_mode(ser, mode: int):
    """mode: -1=disable RTCM entirely, 0=MSM4 (recommended), 1=MSM7"""
    lines = send_command(ser, f"PAIR432,{mode}")
    for line in lines:
        print(line)
    return check_result(lines, "$PAIR001,432,0")


def rtcm_get_mode(ser):
    lines = send_command(ser, "PAIR433")
    for line in lines:
        print(line)
    return lines


def rtcm_set_arp(ser, enable: bool):
    """Enable/disable RTCM message 1005 (station position)."""
    lines = send_command(ser, f"PAIR434,{1 if enable else 0}")
    for line in lines:
        print(line)
    return check_result(lines, "$PAIR001,434,0")


def rtcm_get_arp(ser):
    lines = send_command(ser, "PAIR435")
    for line in lines:
        print(line)
    return lines


def rtcm_set_ephemeris(ser, enable: bool):
    """Enable/disable relaying ephemeris (1019/1020/1042/1044/1046) in the RTCM stream.
    Your rover already gets ephemeris directly from satellites it tracks itself --
    disabling this is the single biggest easy trim for your LoRa bandwidth."""
    lines = send_command(ser, f"PAIR436,{1 if enable else 0}")
    for line in lines:
        print(line)
    return check_result(lines, "$PAIR001,436,0")


def rtcm_get_ephemeris(ser):
    lines = send_command(ser, "PAIR437")
    for line in lines:
        print(line)
    return lines


def set_constellations(ser, mask: int):
    """PAIR066 -- UNVERIFIED, and doubly so right now: this was changed from a
    six-field positional layout (one 0/1 per constellation) to a single
    bitmask integer on request, but neither layout has actually been checked
    against Quectel's spec PDFs or live hardware in this environment -- the
    bit allocation below (including bit 5 for NavIC/IRNSS) has not been
    confirmed against any source. Quectel's spec also explicitly marks
    PAIR066 unsupported on the LC29H(BA/CA/DA/EA) rover variants, and the
    LC29H(BS)-specific spec document doesn't mention it at all, so whether
    this command exists on BS firmware -- in ANY field layout -- is itself
    unconfirmed. Do not trust this against real hardware until you've sent
    it and checked the result with --get.

    Bit allocation (1 = enabled, 0 = disabled), as given, unverified:
      Bit 0 (1):   GPS
      Bit 1 (2):   GLONASS
      Bit 2 (4):   Galileo
      Bit 3 (8):   BDS (BeiDou)
      Bit 4 (16):  QZSS
      Bit 5 (32):  NavIC / IRNSS (if supported)
    """
    body = f"PAIR066,{mask}"
    lines = send_command(ser, body)
    for line in lines:
        print(line)
    return check_result(lines, "$PAIR001,066,0")


def get_constellations(ser):
    lines = send_command(ser, "PAIR067")
    for line in lines:
        print(line)
    return lines


# =====================================================================
# Soft restart -- no physical unplugging needed. This is the documented
# sequence Quectel uses elsewhere in their own spec for changes that
# require a restart: lock sleep, power off the GNSS subsystem, do your
# thing, then power back on.
#
# CAUTION: this interrupts live corrections output for the duration of
# the restart. If a rover is actively relying on this base station's
# RTCM stream in the field, it will lose corrections during this window
# -- fine while bench-testing, worth planning around once deployed.
# =====================================================================
def soft_restart(ser):
    print("NOTE: this will briefly interrupt RTCM output. Any rover currently "
          "using this base station's corrections will lose them until it "
          "comes back up.")
    send_command(ser, "PAIR382,1")   # lock sleep so it stays responsive
    send_command(ser, "PAIR003")     # power off GNSS subsystem
    time.sleep(0.5)
    send_command(ser, "PAIR002")     # power back on
    print("Soft restart complete.")


# =====================================================================
# Self-test: verifies what CAN be independently verified without
# Quectel's own spec PDFs or live hardware -- the checksum algorithm
# (against a standard, vendor-independent NMEA example) and the
# $PQTMSVINSTATUS field-parsing (against the worked example quoted in
# _parse_svinstatus's docstring). This does NOT verify any
# Quectel-specific command syntax (PQTM.../PAIR... names/IDs) -- those
# remain unconfirmed until tested against real hardware.
# =====================================================================
def run_selftest() -> bool:
    all_ok = True

    known_body = ("GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,"
                  "1.2,18.893,M,-25.669,M,2.0,0031")
    known_checksum = "4F"
    computed = checksum(known_body)
    ok = computed == known_checksum
    all_ok &= ok
    print(f"[{'PASS' if ok else 'FAIL'}] checksum() vs standard NMEA example: "
          f"computed={computed} expected={known_checksum}")

    test_sentence = build_sentence("PQTMVERNO").decode("ascii").strip("\r\n")
    ok = _is_valid_sentence(test_sentence)
    all_ok &= ok
    print(f"[{'PASS' if ok else 'FAIL'}] build_sentence() round-trips through "
          f"_is_valid_sentence(): {test_sentence!r}")

    example = ("$PQTMSVINSTATUS,1,2241,1,,01,538,43200,"
               "-2472436.0802,4828383.0026,3343698.4839,9.5*00")
    parsed = _parse_svinstatus(example)
    expected = {"valid": 1, "mean_x": -2472436.0802, "mean_y": 4828383.0026,
                "mean_z": 3343698.4839, "mean_acc": 9.5}
    ok = parsed == expected
    all_ok &= ok
    print(f"[{'PASS' if ok else 'FAIL'}] _parse_svinstatus() vs worked example: "
          f"{parsed}")

    print()
    if all_ok:
        print("All self-tests passed. Note: this confirms the checksum "
              "algorithm and status-parsing logic only -- it does NOT confirm "
              "Quectel-specific command syntax (PQTM.../PAIR... names/IDs), "
              "which can only be verified against real hardware or the spec "
              "PDFs directly.")
    else:
        print("SELF-TEST FAILURE -- do not trust this script against real "
              "hardware until this is fixed.")
    return all_ok


def stream_rtcm_bytes(ser, disable_ephemeris: bool = True, keep_every_n: int = 1, window_seconds: float = 1.0):
    if disable_ephemeris:
        sys.stderr.write("Disabling ephemeris relay before streaming (use --keep-ephemeris to skip this)...\n")
        ok = rtcm_set_ephemeris(ser, False)
        if not ok:
            sys.stderr.write("WARNING: could not confirm ephemeris was disabled -- check the response "
                              "above. Continuing to stream anyway, but your link may be carrying more "
                              "data than necessary.\n")

    if keep_every_n > 1:
        sys.stderr.write(
            f"Decimating output: forwarding 1 out of every {keep_every_n} "
            f"{window_seconds:.1f}s windows (whole windows only, never split mid-epoch).\n"
        )

    sys.stderr.write("Streaming binary RTCM3 to stdout (Ctrl+C to stop)...\n")
    ser.reset_input_buffer()

    rtcm_count = 0
    bytes_forwarded = 0
    dropped_frames = 0
    windows_forwarded = 0
    windows_skipped = 0

    current_window_index = None
    forward_current_window = True

    try:
        while True:
            header = ser.read(1)
            if not header:
                continue

            if header != b"\xd3":
                continue

            length_bytes = ser.read(2)
            if len(length_bytes) < 2:
                dropped_frames += 1
                continue

            msg_len = ((length_bytes[0] & 0x03) << 8) | length_bytes[1]
            payload_and_crc = ser.read(msg_len + 3)

            if len(payload_and_crc) < (msg_len + 3):
                dropped_frames += 1
                continue

            frame = header + length_bytes + payload_and_crc

            if keep_every_n > 1:
                window_index = int(time.time() / window_seconds)
                if window_index != current_window_index:
                    current_window_index = window_index
                    forward_current_window = (window_index % keep_every_n == 0)
                    if forward_current_window:
                        windows_forwarded += 1
                    else:
                        windows_skipped += 1

                if not forward_current_window:
                    continue

            sys.stdout.buffer.write(frame)
            sys.stdout.buffer.flush()

            rtcm_count += 1
            bytes_forwarded += len(frame)

            drop_info = f" (Dropped: {dropped_frames})" if dropped_frames > 0 else ""
            window_info = (
                f" (windows: {windows_forwarded} sent / {windows_skipped} skipped)"
                if keep_every_n > 1 else ""
            )
            sys.stderr.write(
                f"\rStreamed {rtcm_count} RTCM frames ({bytes_forwarded} bytes){drop_info}{window_info}"
            )
            sys.stderr.flush()

    except KeyboardInterrupt:
        sys.stderr.write("\nStopped RTCM streaming.\n")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="Serial port, e.g. /dev/ttyUSB0, /dev/ttyS0, or COM5")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Default {DEFAULT_BAUD}")
    p.add_argument("--selftest", action="store_true",
                    help="Run the built-in self-test (checksum + field-parsing) and exit. "
                         "No hardware/--port needed.")

    sub = p.add_subparsers(dest="action", required=False)

    sub.add_parser("version", help="Query firmware version (good first smoke test)")

    sv = sub.add_parser("survey-in", help="Start or check survey-in")
    sv.add_argument("--duration", type=int, default=3600, help="Minimum observation seconds (default 3600)")
    sv.add_argument("--accuracy", type=float, default=1.0, help="Target 3D accuracy in metres (default 1.0)")
    sv.add_argument("--get", action="store_true",
                     help="Read back CONFIGURED parameters only -- NOT live progress. In survey-in "
                          "mode this always shows ECEF 0,0,0 by design (that's the placeholder you "
                          "configured it with). Use --status instead to see real progress.")
    sv.add_argument("--status", action="store_true",
                     help="Watch live survey-in progress via $PQTMSVINSTATUS -- this is what "
                          "actually shows convergence, not --get.")
    sv.add_argument("--status-duration", type=float, default=15.0,
                     help="Seconds to watch when using --status (default 15)")

    fx = sub.add_parser("fixed", help="Set a known fixed ECEF position instead of surveying")
    fx.add_argument("--ecef", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"),
                     help="WGS84 ECEF X Y Z in metres")

    rt = sub.add_parser("rtcm", help="Configure what the RTCM stream contains")
    rt.add_argument("--mode", choices=["off", "msm4", "msm7"], help="RTCM output mode")
    rt.add_argument("--arp", choices=["on", "off"], help="Station position message (1005)")
    rt.add_argument("--ephemeris", choices=["on", "off"], help="Ephemeris relay messages")
    rt.add_argument("--status", action="store_true", help="Read back current settings")

    cs = sub.add_parser("constellations", help="Try to restrict tracked constellations via bitmask "
                                                 "(unconfirmed on BS, unconfirmed bit layout -- see --help below)")
    cs.add_argument("--enable", type=int, metavar="MASK",
                     help="Bitmask of constellations to enable (unverified layout -- see "
                          "set_constellations() docstring): 1=GPS 2=GLONASS 4=Galileo 8=BDS "
                          "16=QZSS 32=NavIC. E.g. 31 for GPS+GLO+GAL+BDS+QZSS, 13 for GPS+GAL+BDS.")
    cs.add_argument("--get", action="store_true", help="Read back current setting")

    sr = sub.add_parser("stream-rtcm", help="Stream live RTCM3 binary frames to stdout, for piping into "
                                        "lora_node.py (e.g. 'stream-rtcm | python3 lora_node.py "
                                        "--port ... --dst-addr ... --channel ...'). Disables ephemeris "
                                        "relay by default before streaming, since that's usually the "
                                        "biggest easy trim for a bandwidth-constrained LoRa link.")
    sr.add_argument("--keep-ephemeris", action="store_true",
                     help="Don't touch the ephemeris relay setting before streaming -- leave whatever "
                          "is currently configured as-is.")
    sr.add_argument("--keep-every-n", type=int, default=1,
                     help="Forward only 1 out of every N time windows, dropping the rest whole "
                          "(never splits a single epoch's message set). E.g. --keep-every-n 2 "
                          "roughly halves the output rate by skipping every other window. "
                          "Default 1 = no decimation.")
    sr.add_argument("--window-seconds", type=float, default=1.0,
                     help="Size of each window in seconds when using --keep-every-n (default 1.0, "
                          "matching a typical 1Hz base station output rate). Tune this if the skip "
                          "pattern doesn't line up with your base's actual output cadence.")

    sub.add_parser("save", help="Save current config to flash (persists across power-off) and soft-restart")
    sub.add_parser("restart", help="Soft restart only, no config changes")

    mon = sub.add_parser("monitor", help="Dump raw serial output for a few seconds -- use this to check "
                                          "whether the module has a GNSS fix (look for a $..GGA line and "
                                          "check its fix-quality/satellite-count fields) before trusting "
                                          "survey-in results. CAUTION: base-station firmware sometimes "
                                          "doesn't emit standard GGA sentences by default -- if you see "
                                          "nothing GGA-shaped, that does NOT necessarily mean 'no fix'; "
                                          "use 'survey-in --status' as the more reliable BS-specific check.")
    mon.add_argument("--duration", type=float, default=10.0, help="Seconds to listen (default 10)")

    return p.parse_args()


def main():
    args = parse_args()

    if args.selftest:
        ok = run_selftest()
        sys.exit(0 if ok else 1)

    if not args.port or not args.action:
        print("Error: --port and an action are required (unless using --selftest).")
        sys.exit(1)

    if serial is None:
        print("pyserial not installed. Run: pip install pyserial")
        sys.exit(1)

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.3)

    try:
        if args.action == "version":
            get_version(ser)

        elif args.action == "survey-in":
            if args.status:
                survey_status_watch(ser, args.status_duration)
            elif args.get:
                print("(Note: this shows CONFIGURED parameters only, not live progress -- "
                      "0,0,0 here is expected and normal while in survey-in mode. Use "
                      "--status to see real convergence.)\n")
                survey_in_get(ser)
            else:
                ok = survey_in_set(ser, min_dur=args.duration, acc_limit=args.accuracy)
                print("Survey-in configured." if ok else "FAILED -- check response above.")
                print("Check progress with: survey-in --status (not --get)")

        elif args.action == "fixed":
            ok = fixed_position_set(ser, *args.ecef)
            print("Fixed position set." if ok else "FAILED -- check response above.")

        elif args.action == "rtcm":
            if args.mode is not None:
                mode_val = {"off": -1, "msm4": 0, "msm7": 1}[args.mode]
                rtcm_set_mode(ser, mode_val)
            if args.arp is not None:
                rtcm_set_arp(ser, args.arp == "on")
            if args.ephemeris is not None:
                rtcm_set_ephemeris(ser, args.ephemeris == "on")
            if args.status or not any([args.mode, args.arp, args.ephemeris]):
                print("--- current RTCM settings ---")
                rtcm_get_mode(ser)
                rtcm_get_arp(ser)
                rtcm_get_ephemeris(ser)

        elif args.action == "constellations":
            if args.get:
                get_constellations(ser)
            elif args.enable is not None:
                ok = set_constellations(ser, args.enable)
                print("Sent -- verify with 'constellations --get'. Neither PAIR066's support on BS "
                      "nor this bitmask layout is confirmed; treat this result as provisional." if ok
                      else "FAILED or unsupported -- check response above.")
            else:
                print("Specify --enable MASK or --get")

        elif args.action == "stream-rtcm":
            stream_rtcm_bytes(
                ser,
                disable_ephemeris=not args.keep_ephemeris,
                keep_every_n=args.keep_every_n,
                window_seconds=args.window_seconds,
            )

        elif args.action == "save":
            ok = save_settings(ser)
            if ok:
                print("Saved to flash. Restarting so it takes effect...")
                soft_restart(ser)
            else:
                print("Save FAILED -- check response above.")

        elif args.action == "restart":
            soft_restart(ser)

        elif args.action == "monitor":
            print(f"Listening for {args.duration}s -- look for a line starting with $..GGA "
                  f"(e.g. $GNGGA or $GPGGA). Its 6th comma-separated field is fix quality "
                  f"(0 = no fix -- this is your problem if you see it) and the 7th is satellite "
                  f"count in use.\n"
                  f"CAUTION: base-station firmware sometimes doesn't emit GGA by default -- if "
                  f"nothing GGA-shaped shows up, that does NOT necessarily mean no fix. Use "
                  f"'survey-in --status' for a more reliable BS-specific check.\n")
            buf = b""
            deadline = time.time() + args.duration
            while time.time() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    buf += chunk
                    while b"\r\n" in buf:
                        raw, buf = buf.split(b"\r\n", 1)
                        if raw:
                            print(raw.decode("ascii", errors="replace"))
                else:
                    time.sleep(0.02)

    finally:
        ser.close()


if __name__ == "__main__":
    main()