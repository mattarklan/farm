#!/usr/bin/env python3
"""
UGV message protocol + bus.  The single source of truth for "what does an RTK / encoder / IMU /
velocity-command / lidar message look like".

Anything that PRODUCES data (real hardware bridges, simulators) builds messages with encode() or
Publisher.send(), which refuses to send a malformed message. Anything that CONSUMES data (the
visualiser, your own program) reads them with Subscriber.poll(), which refuses to hand you a
malformed one. So both sides always know what to expect.

Transport: UDP multicast on this machine (loopback). Every program joins the same group, so ANY
number of programs can listen to the same port, and programs can both talk and listen on one port
(that is how encoders/IMU and velocity commands share 5006). Messages are one JSON object per UDP
datagram.

    port 5005  lidar    scans                                  lidar   -> everyone
    port 5006  esp      encoders + IMU                         robot   -> everyone
               cmd_vel  velocity command                       anyone  -> robot
    port 5007  rtk      GNSS position + fix status             rover   -> everyone
    port 5008  pose     robot pose in the map frame            visualiser -> planner
               goal     click-to-drive target / cancel         visualiser -> planner
               path     the planner's path, for display        planner -> visualiser

Frames and units (everywhere): SI units, degrees only where the field name says so. The local map
frame is x = east (m), y = north (m), yaw = 0 facing east, counter-clockwise positive.

Run it:
    python protocol.py --selftest       check the bus works on this machine
    python protocol.py --listen         print every message on the bus (a "wire monitor")
    python protocol.py --doc            print the full protocol description (PROTOCOL.md)
    python protocol.py --example rtk    print a valid example message

Environment:  UGV_IFACE (default 127.0.0.1) = interface address to use; set it to this machine's
LAN address to run producers and consumers on different computers.  UGV_GROUP = multicast group.

Layout: ugv/common/protocol.py
"""

import argparse
import copy
import json
import math
import os
import socket
import struct
import sys
import time

GROUP = os.environ.get("UGV_GROUP", "239.255.42.42")
IFACE = os.environ.get("UGV_IFACE", "127.0.0.1")
PORT_LIDAR, PORT_ESP, PORT_RTK, PORT_NAV = 5005, 5006, 5007, 5008
VERSION = 1
MAX_DATAGRAM = 60000

FIX_NAMES = ("No Fix", "3D Fix", "DGNSS", "Float RTK", "Fixed RTK", "Unknown")
FIX_RANK = {"No Fix": 0, "Unknown": 0, "3D Fix": 1, "DGNSS": 2, "Float RTK": 3, "Fixed RTK": 4}


class ProtocolError(ValueError):
    """A message does not match the protocol."""


# ----------------------------------------------------------------------------------------------
# Schema
# ----------------------------------------------------------------------------------------------
class Field:
    """One field of a message. `name` may be dotted ("enc.left_ticks") for nested objects."""

    def __init__(self, name, kind, unit="", doc="", lo=None, hi=None, choices=None, optional=False, n=None):
        self.name, self.kind, self.unit, self.doc = name, kind, unit, doc
        self.lo, self.hi, self.choices, self.optional, self.n = lo, hi, choices, optional, n

    def type_text(self):
        t = {"int": "integer", "float": "number", "str": "string",
             "floats": f"array of {self.n} numbers", "points": "array of [angle_deg, dist_mm] pairs",
             "xys": "array of [x, y] pairs"}[self.kind]
        return t

    def range_text(self):
        if self.choices:
            return " / ".join(f'"{c}"' for c in self.choices)
        if self.lo is not None and self.hi is not None:
            return f"{self.lo} .. {self.hi}"
        if self.lo is not None:
            return f">= {self.lo}"
        if self.hi is not None:
            return f"<= {self.hi}"
        return ""


class Message:
    def __init__(self, name, port, producer, consumers, rate, doc, fields, example, notes=()):
        self.name, self.port, self.producer, self.consumers = name, port, producer, consumers
        self.rate, self.doc, self.fields, self.example, self.notes = rate, doc, fields, example, notes


MESSAGES = {}


def _register(m):
    MESSAGES[m.name] = m


_register(Message(
    "rtk", PORT_RTK,
    producer="the rover (rover_rtka.py on hardware, sim_robot.py in simulation)",
    consumers="visualiser, your program",
    rate="1 - 10 Hz (one message per GGA sentence)",
    doc="Position and fix quality from the RTK GNSS receiver (LC29H).",
    fields=[
        Field("lat", "float", "degrees", "WGS84 latitude, north positive", -90, 90),
        Field("lon", "float", "degrees", "WGS84 longitude, east positive", -180, 180),
        Field("fix", "str", "", "fix quality, from the GGA quality flag", choices=FIX_NAMES),
        Field("sats", "int", "count", "satellites used in the solution", 0, 99),
        Field("hdop", "float", "", "horizontal dilution of precision; 99.0 when unknown", 0, 99.99),
        Field("corr_age", "float", "s", "age of the RTK correction data; 0.0 when there is none", 0, None),
    ],
    example={"type": "rtk", "v": 1, "ts": 1760000000.123, "lat": -31.9505, "lon": 115.8605,
             "fix": "Fixed RTK", "sats": 22, "hdop": 0.55, "corr_age": 0.8},
    notes=[
        'When `fix` is "No Fix", `lat` and `lon` are 0.0 and MUST be ignored by consumers.',
        '`fix` ranks, worst to best: No Fix / Unknown < 3D Fix < DGNSS < Float RTK < Fixed RTK. '
        'Only Float RTK and Fixed RTK are centimetre-level; consumers that map or navigate should require one of them.',
        '`ts` is the wall-clock time the position was measured (when the GGA sentence was parsed).',
    ]))

_register(Message(
    "esp", PORT_ESP,
    producer="the motor controller bridge (ESP32 on hardware, sim_robot.py in simulation)",
    consumers="visualiser, your program",
    rate="1 - 50 Hz",
    doc="Wheel encoders and IMU from the ESP32.",
    fields=[
        Field("seq", "int", "", "increments by 1 per message; consumers use it to spot new data and dropouts", 0, None),
        Field("enc.left_ticks", "int", "ticks", "CUMULATIVE signed encoder count, left wheel (not a delta)"),
        Field("enc.right_ticks", "int", "ticks", "CUMULATIVE signed encoder count, right wheel (not a delta)"),
        Field("enc.left_vel", "float", "ticks/s", "left wheel speed, forward positive"),
        Field("enc.right_vel", "float", "ticks/s", "right wheel speed, forward positive"),
        Field("imu.gyro", "floats", "rad/s", "angular rate [x, y, z]; z = yaw rate, counter-clockwise positive", n=3),
        Field("imu.accel", "floats", "m/s^2", "acceleration [x, y, z] in the body frame; ~[0, 0, +9.81] when level and still", n=3),
        Field("imu.quat", "floats", "", "orientation quaternion [w, x, y, z]; yaw 0 = facing east. Omit if the IMU does not fuse one", n=4, optional=True),
    ],
    example={"type": "esp", "v": 1, "ts": 1760000000.456, "seq": 128,
             "enc": {"left_ticks": 4660, "right_ticks": 5452, "left_vel": 920.5, "right_vel": 1080.2},
             "imu": {"gyro": [0.001, -0.002, 0.256], "accel": [-0.03, 0.11, 9.81], "quat": [0.82, 0.0, 0.0, 0.57]}},
    notes=[
        "Tick counts are cumulative. A consumer gets the distance travelled by subtracting the previous count "
        "(a dropped message then loses nothing). If your counter is 32-bit it can wrap; handle that on the consumer side.",
        "Body frame: x forward, y left, z up.",
    ]))

_register(Message(
    "cmd_vel", PORT_ESP,
    producer="the visualiser (arrow keys / click-to-drive), your program",
    consumers="the robot (sim_robot.py, or the ESP32 bridge). The visualiser also listens, to animate its goal robot.",
    rate="repeat at >= 5 Hz (the visualiser uses 20 Hz) while the robot should move",
    doc="Velocity command for the robot.",
    fields=[
        Field("linear", "float", "m/s", "forward speed, forward positive", None, None),
        Field("angular", "float", "rad/s", "turn rate, counter-clockwise (left) positive"),
    ],
    example={"type": "cmd_vel", "v": 1, "ts": 1760000000.789, "linear": 0.5, "angular": 0.1},
    notes=[
        "WATCHDOG: the robot stops if it has not received a cmd_vel for 0.5 s. Keep sending while it should move; "
        "send one {linear: 0, angular: 0} when you stop.",
        "The robot clamps/accelerates at its own limits (see common/config.py).",
        "Shares port 5006 with `esp`; tell them apart with `type`. Receivers just ignore types they don't want.",
    ]))

_register(Message(
    "lidar", PORT_LIDAR,
    producer="the lidar publisher (RPLidar A2 on hardware, sim_lidar.py in simulation)",
    consumers="visualiser, your program",
    rate="~10 Hz",
    doc="One full 360 degree lidar scan.",
    fields=[
        Field("points", "points", "[deg, mm]", "list of [angle_deg, dist_mm]. angle: 0..360, clockwise from the robot's front. "
              "dist: 0 < d <= 12000 mm. Beams with no return are left out"),
    ],
    example={"type": "lidar", "v": 1, "ts": 1760000000.9,
             "points": [[0.0, 2310.5], [1.0, 2305.2], [2.0, 2299.9]]},
    notes=[
        "The lidar sits at the GNSS antenna position and turns with the robot.",
        "A beam that hits nothing is simply absent. The visualiser treats a gap of more than 3 degrees as open space "
        "(free out to --free-range, default 6 m) and a completely empty scan as an open field, so send the scan even if it has no points.",
        "`ts` is the time the scan was completed. (The older payload used the key `timestamp` and had no `type`; "
        "decode() still accepts that.)",
    ]))

_register(Message(
    "pose", PORT_NAV,
    producer="the visualiser (its best estimate of where the robot is right now)",
    consumers="your path planner / navigation program",
    rate="10 Hz while there is a usable GNSS fix",
    doc="Robot pose in the local map frame (GNSS + IMU, carried forward to `ts` with the commanded motion). "
        "Lets an external planner skip doing its own lat/lon conversion.",
    fields=[
        Field("x", "float", "m", "east of the map origin (common/config.py ORIGIN_LAT/LON)"),
        Field("y", "float", "m", "north of the map origin"),
        Field("yaw", "float", "rad", "heading, 0 = east, counter-clockwise positive"),
        Field("fix", "str", "", "GNSS fix quality behind this pose", choices=FIX_NAMES),
    ],
    example={"type": "pose", "v": 1, "ts": 1760000000.5, "x": 4.97, "y": 0.05, "yaw": -0.8, "fix": "Fixed RTK"},
    notes=["Not published when there is no fix or the GNSS data is stale."]))

_register(Message(
    "goal", PORT_NAV,
    producer="the visualiser (click on the map = go, right-click / teleop = cancel), or any program",
    consumers="your path planner / navigation program (the visualiser's built-in planner when run with --builtin-nav)",
    rate="once per click",
    doc="Where the user wants the robot to drive to, or a request to stop going there.",
    fields=[
        Field("action", "str", "", '"go" = drive to x, y; "cancel" = abandon the current goal', choices=("go", "cancel")),
        Field("x", "float", "m", "east of the map origin (ignored for cancel)", optional=True),
        Field("y", "float", "m", "north of the map origin (ignored for cancel)", optional=True),
    ],
    example={"type": "goal", "v": 1, "ts": 1760000000.5, "action": "go", "x": 5.0, "y": 0.0},
    notes=["On action 'go', x and y are required.",
           "The visualiser sends 'cancel' when you take over with the arrow keys, so a planner knows to stop sending cmd_vel."]))

_register(Message(
    "path", PORT_NAV,
    producer="your path planner",
    consumers="the visualiser (draws it), anyone else",
    rate="whenever the plan changes",
    doc="The path the planner intends to follow, in the local map frame, for display.",
    fields=[
        Field("points", "xys", "m", "waypoints [[x, y], ...], start to goal. An empty list clears the displayed path"),
    ],
    example={"type": "path", "v": 1, "ts": 1760000000.5, "points": [[0.0, 0.0], [2.0, 1.5], [5.0, 0.0]]},
    notes=["The visualiser only draws this; it never drives from it. Drive the robot with cmd_vel."]))

# the fields every message carries
COMMON = [
    Field("type", "str", "", "which message this is", choices=tuple(MESSAGES)),
    Field("v", "int", "", "protocol version (currently 1); may be omitted, which means 1", 1, None, optional=True),
    Field("ts", "float", "s (unix)", "wall-clock time the data was MEASURED (time.time() on the producer)", 0, None),
]


# ----------------------------------------------------------------------------------------------
# Validation / codec
# ----------------------------------------------------------------------------------------------
def _is_num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _get(d, path):
    for k in path.split("."):
        if not isinstance(d, dict) or k not in d:
            raise KeyError(path)
        d = d[k]
    return d


def _set(d, path, value):
    keys = path.split(".")
    for k in keys[:-1]:
        d = d[k]
    d[keys[-1]] = value


def _coerce(f, v):
    where = f"field '{f.name}'"
    if f.kind == "int":
        if isinstance(v, float) and v.is_integer():
            v = int(v)
        if isinstance(v, bool) or not isinstance(v, int):
            raise ProtocolError(f"{where} must be an integer, got {v!r}")
    elif f.kind == "float":
        if not _is_num(v):
            raise ProtocolError(f"{where} must be a finite number, got {v!r}")
        v = float(v)
    elif f.kind == "str":
        if not isinstance(v, str):
            raise ProtocolError(f"{where} must be a string, got {v!r}")
    elif f.kind == "floats":
        if not isinstance(v, (list, tuple)) or len(v) != f.n or not all(_is_num(x) for x in v):
            raise ProtocolError(f"{where} must be a list of {f.n} finite numbers, got {v!r}")
        return [float(x) for x in v]
    elif f.kind == "xys":
        if not isinstance(v, (list, tuple)):
            raise ProtocolError(f"{where} must be a list of [x, y]")
        out = []
        for p in v:
            if not isinstance(p, (list, tuple)) or len(p) != 2 or not _is_num(p[0]) or not _is_num(p[1]):
                raise ProtocolError(f"{where} has a bad entry {p!r}; expected [x, y]")
            out.append([float(p[0]), float(p[1])])
        return out
    elif f.kind == "points":
        if not isinstance(v, (list, tuple)):
            raise ProtocolError(f"{where} must be a list of [angle_deg, dist_mm]")
        out = []
        for p in v:
            if not isinstance(p, (list, tuple)) or len(p) != 2 or not _is_num(p[0]) or not _is_num(p[1]):
                raise ProtocolError(f"{where} has a bad entry {p!r}; expected [angle_deg, dist_mm]")
            if not (0.0 <= p[0] <= 360.0) or not (0.0 < p[1] <= 20000.0):
                raise ProtocolError(f"{where} entry {p!r} out of range (angle 0..360, 0 < dist <= 20000 mm)")
            out.append([float(p[0]), float(p[1])])
        return out
    if f.choices is not None and v not in f.choices:
        raise ProtocolError(f"{where} must be one of {list(f.choices)}, got {v!r}")
    if f.lo is not None and v < f.lo:
        raise ProtocolError(f"{where} = {v} is below the minimum {f.lo}")
    if f.hi is not None and v > f.hi:
        raise ProtocolError(f"{where} = {v} is above the maximum {f.hi}")
    return v


def validate(msg):
    """Check a message dict against the schema. Returns a normalised copy, or raises ProtocolError."""
    if not isinstance(msg, dict):
        raise ProtocolError("a message must be a JSON object")
    msg = copy.deepcopy(msg)
    mtype = msg.get("type")
    if mtype is None and "points" in msg:            # legacy lidar payload
        mtype = msg["type"] = "lidar"
    if mtype not in MESSAGES:
        raise ProtocolError(f"unknown message type {mtype!r}; known: {list(MESSAGES)}")
    if "ts" not in msg and "timestamp" in msg:       # legacy lidar payload
        msg["ts"] = msg.pop("timestamp")
    msg.setdefault("v", VERSION)
    for f in COMMON:
        try:
            _set(msg, f.name, _coerce(f, _get(msg, f.name)))
        except KeyError:
            if not f.optional:
                raise ProtocolError(f"missing required field '{f.name}'")
    if msg["v"] > VERSION:
        raise ProtocolError(f"message is protocol version {msg['v']}, this code understands up to {VERSION}")
    for f in MESSAGES[mtype].fields:
        try:
            value = _get(msg, f.name)
        except KeyError:
            if f.optional:
                continue
            raise ProtocolError(f"{mtype}: missing required field '{f.name}'")
        _set(msg, f.name, _coerce(f, value))
    if mtype == "goal" and msg["action"] == "go" and ("x" not in msg or "y" not in msg):
        raise ProtocolError("goal: action 'go' needs x and y")
    return msg


def encode(mtype, ts=None, **fields):
    """Build a validated message and return it as bytes, ready to send. Raises ProtocolError if wrong."""
    if mtype not in MESSAGES:
        raise ProtocolError(f"unknown message type {mtype!r}; known: {list(MESSAGES)}")
    msg = {"type": mtype, "v": VERSION, "ts": time.time() if ts is None else ts}
    msg.update(fields)
    data = json.dumps(validate(msg), separators=(",", ":")).encode("utf-8")
    if len(data) > MAX_DATAGRAM:
        raise ProtocolError(f"{mtype} message is {len(data)} bytes; limit is {MAX_DATAGRAM}")
    return data


def decode(data):
    """Parse + validate one datagram. Returns the message dict (always has type, v, ts) or raises ProtocolError."""
    try:
        msg = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        raise ProtocolError("not valid UTF-8 JSON")
    return validate(msg)


# ----------------------------------------------------------------------------------------------
# Bus (UDP multicast)
# ----------------------------------------------------------------------------------------------
def _sender_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(IFACE))
    return s


def _listener_socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    s.bind(("", port))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                 struct.pack("4s4s", socket.inet_aton(GROUP), socket.inet_aton(IFACE)))
    s.setblocking(False)
    return s


class Publisher:
    """Send validated messages onto the bus.   pub = Publisher();  pub.send("cmd_vel", linear=0.5, angular=0.0)"""

    def __init__(self):
        self.sock = _sender_socket()

    def send(self, mtype, ts=None, **fields):
        data = encode(mtype, ts=ts, **fields)
        self.sock.sendto(data, (GROUP, MESSAGES[mtype].port))
        return data

    def close(self):
        self.sock.close()


class Subscriber:
    """Receive validated messages from the bus.

        sub = Subscriber(["rtk", "esp"])        # or Subscriber() for everything
        for msg in sub.poll():                   # returns immediately; [] when nothing new
            ...
    Malformed datagrams are dropped and counted in sub.bad (latest reason in sub.last_error)."""

    def __init__(self, types=None):
        types = list(types) if types else list(MESSAGES)
        for t in types:
            if t not in MESSAGES:
                raise ProtocolError(f"unknown message type {t!r}")
        self.types = set(types)
        self.socks = [_listener_socket(p) for p in sorted({MESSAGES[t].port for t in types})]
        self.bad = 0
        self.last_error = ""

    def poll(self, max_msgs=1000):
        out = []
        for s in self.socks:
            for _ in range(max_msgs):
                try:
                    data, _addr = s.recvfrom(65535)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:                     # e.g. WSAECONNRESET on Windows
                    continue
                try:
                    m = decode(data)
                except ProtocolError as e:
                    self.bad += 1
                    self.last_error = str(e)
                    continue
                if m["type"] in self.types:
                    out.append(m)
        out.sort(key=lambda m: m["ts"])
        return out

    def close(self):
        for s in self.socks:
            s.close()


# ----------------------------------------------------------------------------------------------
# Tools: --selftest / --listen / --doc / --example
# ----------------------------------------------------------------------------------------------
def selftest():
    print(f"bus: group {GROUP}, interface {IFACE}, ports {PORT_LIDAR}/{PORT_ESP}/{PORT_RTK}/{PORT_NAV}")
    try:
        a, b, pub = Subscriber(["cmd_vel"]), Subscriber(["cmd_vel", "esp"]), Publisher()
    except OSError as e:
        print(f"FAILED to open the bus: {e}\n  - is another program using the port exclusively?"
              f"\n  - try UGV_IFACE=<this machine's LAN address>")
        return 1
    pub.send("cmd_vel", linear=0.1, angular=0.0)
    deadline = time.time() + 1.5
    got_a = got_b = []
    while time.time() < deadline and not (got_a and got_b):
        got_a = got_a or a.poll()
        got_b = got_b or b.poll()
        time.sleep(0.02)
    try:
        encode("rtk", lat=0, lon=0, fix="Bogus", sats=1, hdop=1, corr_age=0)
        print("FAILED: a bad message was accepted")
        return 1
    except ProtocolError:
        pass
    if got_a and got_b:
        print("OK: two listeners on the same port both received the message, and bad messages are rejected.")
        return 0
    print("FAILED: the message was not delivered to both listeners.\n"
          "  - check the firewall allows UDP multicast on this interface (Windows: allow Python on private networks)\n"
          "  - a VPN or virtual adapter may be hijacking the route: set UGV_IFACE to your real LAN address")
    return 1


def listen(types=None):
    sub = Subscriber(types)
    counts, last_print = {}, time.time()
    print(f"listening on {GROUP} via {IFACE} for {sorted(sub.types)} ... Ctrl+C to stop")
    try:
        while True:
            for m in sub.poll():
                counts[m["type"]] = counts.get(m["type"], 0) + 1
                body = {k: v for k, v in m.items() if k not in ("type", "v")}
                if m["type"] == "lidar":
                    body = {"ts": m["ts"], "points": f"{len(m['points'])} points"}
                print(f"{m['type']:8} {json.dumps(body)[:200]}")
            if time.time() - last_print > 5:
                last_print = time.time()
                print(f"-- 5 s totals: {counts}   malformed dropped: {sub.bad} {sub.last_error}")
                counts = {}
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


def describe():
    """The protocol as Markdown (this is what PROTOCOL.md is generated from, so it can't go stale)."""
    L = []
    w = L.append
    w("# UGV message protocol\n")
    w("_Generated from `common/protocol.py` (`python protocol.py --doc`). Do not edit by hand; edit the schema in the code._\n")
    w("## How it works\n")
    w("Every program joins one UDP multicast group on this machine and sends/receives **one JSON object per datagram**. "
      "Any number of programs can listen to the same port, and one program can both send and listen on it. "
      "Producers (hardware bridges, simulators) and consumers (visualiser, your program) never talk to each other "
      "directly, so a simulated robot and a real one are interchangeable.\n")
    w(f"- Group `{GROUP}`, interface `{IFACE}` (override with the `UGV_GROUP` / `UGV_IFACE` environment variables; "
      "use your LAN address in `UGV_IFACE` to run on separate computers).")
    w("- Test the bus: `python protocol.py --selftest`.  Watch all traffic: `python protocol.py --listen`.\n")
    w("| Port | Message | Direction |")
    w("|---|---|---|")
    for m in MESSAGES.values():
        w(f"| {m.port} | `{m.name}` | {m.producer} -> {m.consumers} |")
    w("")
    w("## Conventions\n")
    w("- Units are SI. Angles are radians unless the field name ends in `_deg`.")
    w("- Local map frame: **x = east (m), y = north (m)**, yaw 0 = facing east, **counter-clockwise positive**. "
      "Latitude/longitude are converted to this frame around the map origin set in `common/config.py`.")
    w("- Every message has `type`, `v` (protocol version, 1) and `ts` (unix time, seconds, when the data was **measured**). "
      "All programs must share a clock (same computer, or time-synced computers).")
    w("- Consumers ignore fields they don't know (so the protocol can grow) and drop messages that fail validation.")
    w("- Rates are guidelines; consumers must cope with a message arriving late or not at all, and use `ts`/`seq`, not arrival order.\n")
    for m in MESSAGES.values():
        w(f"## `{m.name}`  (port {m.port})\n")
        w(m.doc + "\n")
        w(f"- **Sent by:** {m.producer}")
        w(f"- **Used by:** {m.consumers}")
        w(f"- **Rate:** {m.rate}\n")
        w("| Field | Type | Unit | Allowed | Meaning |")
        w("|---|---|---|---|---|")
        for f in COMMON[:1] + COMMON[2:] + m.fields:
            if f.name == "type":
                allowed, meaning = f'`"{m.name}"`', f.doc
            else:
                allowed, meaning = (f.range_text() or "").replace("|", "/"), f.doc
            opt = " *(optional)*" if f.optional else ""
            w(f"| `{f.name}`{opt} | {f.type_text()} | {f.unit} | {allowed} | {meaning} |")
        w("\nExample:\n")
        w("```json")
        w(json.dumps(m.example, indent=2))
        w("```\n")
        for n in m.notes:
            w(f"- {n}")
        w("")
    w("## Sim only: ground-truth pose (not part of the bus)\n")
    w("`sim_robot.py` sends the robot's TRUE pose to `sim_lidar.py` as plain unicast UDP to `127.0.0.1:5000`, "
      'as `{"x": mm, "y": mm, "heading": deg}` in the lidar simulator\'s image frame (millimetres from the top-left of the '
      "world image, y down, heading clockwise). The lidar simulator needs it to know where to ray-cast from. "
      "Nothing else uses it, and the robot's own map/navigation code never sees it; it only learns about the world "
      "from the lidar scans.\n")
    w("## Using it from Python\n")
    w("```python")
    w("from common.protocol import Publisher, Subscriber")
    w("")
    w("sub = Subscriber([\"rtk\", \"esp\", \"lidar\"])")
    w("pub = Publisher()")
    w("while True:")
    w("    for msg in sub.poll():            # already validated")
    w("        if msg[\"type\"] == \"rtk\" and msg[\"fix\"] == \"Fixed RTK\":")
    w("            print(msg[\"lat\"], msg[\"lon\"])")
    w("    pub.send(\"cmd_vel\", linear=0.3, angular=0.0)")
    w("```\n")
    w("## Using it from anything else\n")
    w(f"Send the JSON as a UDP datagram to `{GROUP}:<port>` (set the multicast interface to `{IFACE}`), or listen on the "
      "port after joining the group. There is nothing Python-specific in the wire format.")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser(description="UGV message protocol tools")
    ap.add_argument("--selftest", action="store_true", help="check the bus works on this machine")
    ap.add_argument("--listen", action="store_true", help="print every message on the bus")
    ap.add_argument("--types", nargs="+", choices=list(MESSAGES), help="with --listen: only these types")
    ap.add_argument("--doc", action="store_true", help="print the protocol description as Markdown")
    ap.add_argument("--example", choices=list(MESSAGES), help="print a valid example message")
    args = ap.parse_args()
    if args.doc:
        sys.stdout.write(describe())
    elif args.example:
        print(json.dumps(MESSAGES[args.example].example, indent=2))
    elif args.listen:
        listen(args.types)
    else:
        sys.exit(selftest())


if __name__ == "__main__":
    main()
