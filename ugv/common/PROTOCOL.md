# UGV message protocol

_Generated from `common/protocol.py` (`python protocol.py --doc`). Do not edit by hand; edit the schema in the code._

## How it works

Every program joins one UDP multicast group on this machine and sends/receives **one JSON object per datagram**. Any number of programs can listen to the same port, and one program can both send and listen on it. Producers (hardware bridges, simulators) and consumers (visualiser, your program) never talk to each other directly, so a simulated robot and a real one are interchangeable.

- Group `239.255.42.42`, interface `127.0.0.1` (override with the `UGV_GROUP` / `UGV_IFACE` environment variables; use your LAN address in `UGV_IFACE` to run on separate computers).
- Test the bus: `python protocol.py --selftest`.  Watch all traffic: `python protocol.py --listen`.

| Port | Message | Direction |
|---|---|---|
| 5007 | `rtk` | the rover (rover_rtka.py on hardware, sim_robot.py in simulation) -> visualiser, your program |
| 5006 | `esp` | the motor controller bridge (ESP32 on hardware, sim_robot.py in simulation) -> visualiser, your program |
| 5006 | `cmd_vel` | the visualiser (arrow keys / click-to-drive), your program -> the robot (sim_robot.py, or the ESP32 bridge). The visualiser also listens, to animate its goal robot. |
| 5005 | `lidar` | the lidar publisher (RPLidar A2 on hardware, sim_lidar.py in simulation) -> visualiser, your program |
| 5008 | `pose` | the visualiser (its best estimate of where the robot is right now) -> your path planner / navigation program |
| 5008 | `goal` | the visualiser (click on the map = go, right-click / teleop = cancel), or any program -> your path planner / navigation program (the visualiser's built-in planner when run with --builtin-nav) |
| 5008 | `path` | your path planner -> the visualiser (draws it), anyone else |

## Conventions

- Units are SI. Angles are radians unless the field name ends in `_deg`.
- Local map frame: **x = east (m), y = north (m)**, yaw 0 = facing east, **counter-clockwise positive**. Latitude/longitude are converted to this frame around the map origin set in `common/config.py`.
- Every message has `type`, `v` (protocol version, 1) and `ts` (unix time, seconds, when the data was **measured**). All programs must share a clock (same computer, or time-synced computers).
- Consumers ignore fields they don't know (so the protocol can grow) and drop messages that fail validation.
- Rates are guidelines; consumers must cope with a message arriving late or not at all, and use `ts`/`seq`, not arrival order.

## `rtk`  (port 5007)

Position and fix quality from the RTK GNSS receiver (LC29H).

- **Sent by:** the rover (rover_rtka.py on hardware, sim_robot.py in simulation)
- **Used by:** visualiser, your program
- **Rate:** 1 - 10 Hz (one message per GGA sentence)

| Field | Type | Unit | Allowed | Meaning |
|---|---|---|---|---|
| `type` | string |  | `"rtk"` | which message this is |
| `ts` | number | s (unix) | >= 0 | wall-clock time the data was MEASURED (time.time() on the producer) |
| `lat` | number | degrees | -90 .. 90 | WGS84 latitude, north positive |
| `lon` | number | degrees | -180 .. 180 | WGS84 longitude, east positive |
| `fix` | string |  | "No Fix" / "3D Fix" / "DGNSS" / "Float RTK" / "Fixed RTK" / "Unknown" | fix quality, from the GGA quality flag |
| `sats` | integer | count | 0 .. 99 | satellites used in the solution |
| `hdop` | number |  | 0 .. 99.99 | horizontal dilution of precision; 99.0 when unknown |
| `corr_age` | number | s | >= 0 | age of the RTK correction data; 0.0 when there is none |

Example:

```json
{
  "type": "rtk",
  "v": 1,
  "ts": 1760000000.123,
  "lat": -31.9505,
  "lon": 115.8605,
  "fix": "Fixed RTK",
  "sats": 22,
  "hdop": 0.55,
  "corr_age": 0.8
}
```

- When `fix` is "No Fix", `lat` and `lon` are 0.0 and MUST be ignored by consumers.
- `fix` ranks, worst to best: No Fix / Unknown < 3D Fix < DGNSS < Float RTK < Fixed RTK. Only Float RTK and Fixed RTK are centimetre-level; consumers that map or navigate should require one of them.
- `ts` is the wall-clock time the position was measured (when the GGA sentence was parsed).

## `esp`  (port 5006)

Wheel encoders and IMU from the ESP32.

- **Sent by:** the motor controller bridge (ESP32 on hardware, sim_robot.py in simulation)
- **Used by:** visualiser, your program
- **Rate:** 1 - 50 Hz

| Field | Type | Unit | Allowed | Meaning |
|---|---|---|---|---|
| `type` | string |  | `"esp"` | which message this is |
| `ts` | number | s (unix) | >= 0 | wall-clock time the data was MEASURED (time.time() on the producer) |
| `seq` | integer |  | >= 0 | increments by 1 per message; consumers use it to spot new data and dropouts |
| `enc.left_ticks` | integer | ticks |  | CUMULATIVE signed encoder count, left wheel (not a delta) |
| `enc.right_ticks` | integer | ticks |  | CUMULATIVE signed encoder count, right wheel (not a delta) |
| `enc.left_vel` | number | ticks/s |  | left wheel speed, forward positive |
| `enc.right_vel` | number | ticks/s |  | right wheel speed, forward positive |
| `imu.gyro` | array of 3 numbers | rad/s |  | angular rate [x, y, z]; z = yaw rate, counter-clockwise positive |
| `imu.accel` | array of 3 numbers | m/s^2 |  | acceleration [x, y, z] in the body frame; ~[0, 0, +9.81] when level and still |
| `imu.quat` *(optional)* | array of 4 numbers |  |  | orientation quaternion [w, x, y, z]; yaw 0 = facing east. Omit if the IMU does not fuse one |

Example:

```json
{
  "type": "esp",
  "v": 1,
  "ts": 1760000000.456,
  "seq": 128,
  "enc": {
    "left_ticks": 4660,
    "right_ticks": 5452,
    "left_vel": 920.5,
    "right_vel": 1080.2
  },
  "imu": {
    "gyro": [
      0.001,
      -0.002,
      0.256
    ],
    "accel": [
      -0.03,
      0.11,
      9.81
    ],
    "quat": [
      0.82,
      0.0,
      0.0,
      0.57
    ]
  }
}
```

- Tick counts are cumulative. A consumer gets the distance travelled by subtracting the previous count (a dropped message then loses nothing). If your counter is 32-bit it can wrap; handle that on the consumer side.
- Body frame: x forward, y left, z up.

## `cmd_vel`  (port 5006)

Velocity command for the robot.

- **Sent by:** the visualiser (arrow keys / click-to-drive), your program
- **Used by:** the robot (sim_robot.py, or the ESP32 bridge). The visualiser also listens, to animate its goal robot.
- **Rate:** repeat at >= 5 Hz (the visualiser uses 20 Hz) while the robot should move

| Field | Type | Unit | Allowed | Meaning |
|---|---|---|---|---|
| `type` | string |  | `"cmd_vel"` | which message this is |
| `ts` | number | s (unix) | >= 0 | wall-clock time the data was MEASURED (time.time() on the producer) |
| `linear` | number | m/s |  | forward speed, forward positive |
| `angular` | number | rad/s |  | turn rate, counter-clockwise (left) positive |

Example:

```json
{
  "type": "cmd_vel",
  "v": 1,
  "ts": 1760000000.789,
  "linear": 0.5,
  "angular": 0.1
}
```

- WATCHDOG: the robot stops if it has not received a cmd_vel for 0.5 s. Keep sending while it should move; send one {linear: 0, angular: 0} when you stop.
- The robot clamps/accelerates at its own limits (see common/config.py).
- Shares port 5006 with `esp`; tell them apart with `type`. Receivers just ignore types they don't want.

## `lidar`  (port 5005)

One full 360 degree lidar scan.

- **Sent by:** the lidar publisher (RPLidar A2 on hardware, sim_lidar.py in simulation)
- **Used by:** visualiser, your program
- **Rate:** ~10 Hz

| Field | Type | Unit | Allowed | Meaning |
|---|---|---|---|---|
| `type` | string |  | `"lidar"` | which message this is |
| `ts` | number | s (unix) | >= 0 | wall-clock time the data was MEASURED (time.time() on the producer) |
| `points` | array of [angle_deg, dist_mm] pairs | [deg, mm] |  | list of [angle_deg, dist_mm]. angle: 0..360, clockwise from the robot's front. dist: 0 < d <= 12000 mm. Beams with no return are left out |

Example:

```json
{
  "type": "lidar",
  "v": 1,
  "ts": 1760000000.9,
  "points": [
    [
      0.0,
      2310.5
    ],
    [
      1.0,
      2305.2
    ],
    [
      2.0,
      2299.9
    ]
  ]
}
```

- The lidar sits at the GNSS antenna position and turns with the robot.
- A beam that hits nothing is simply absent. The visualiser treats a gap of more than 3 degrees as open space (free out to --free-range, default 6 m) and a completely empty scan as an open field, so send the scan even if it has no points.
- `ts` is the time the scan was completed. (The older payload used the key `timestamp` and had no `type`; decode() still accepts that.)

## `pose`  (port 5008)

Robot pose in the local map frame (GNSS + IMU, carried forward to `ts` with the commanded motion). Lets an external planner skip doing its own lat/lon conversion.

- **Sent by:** the visualiser (its best estimate of where the robot is right now)
- **Used by:** your path planner / navigation program
- **Rate:** 10 Hz while there is a usable GNSS fix

| Field | Type | Unit | Allowed | Meaning |
|---|---|---|---|---|
| `type` | string |  | `"pose"` | which message this is |
| `ts` | number | s (unix) | >= 0 | wall-clock time the data was MEASURED (time.time() on the producer) |
| `x` | number | m |  | east of the map origin (common/config.py ORIGIN_LAT/LON) |
| `y` | number | m |  | north of the map origin |
| `yaw` | number | rad |  | heading, 0 = east, counter-clockwise positive |
| `fix` | string |  | "No Fix" / "3D Fix" / "DGNSS" / "Float RTK" / "Fixed RTK" / "Unknown" | GNSS fix quality behind this pose |

Example:

```json
{
  "type": "pose",
  "v": 1,
  "ts": 1760000000.5,
  "x": 4.97,
  "y": 0.05,
  "yaw": -0.8,
  "fix": "Fixed RTK"
}
```

- Not published when there is no fix or the GNSS data is stale.

## `goal`  (port 5008)

Where the user wants the robot to drive to, or a request to stop going there.

- **Sent by:** the visualiser (click on the map = go, right-click / teleop = cancel), or any program
- **Used by:** your path planner / navigation program (the visualiser's built-in planner when run with --builtin-nav)
- **Rate:** once per click

| Field | Type | Unit | Allowed | Meaning |
|---|---|---|---|---|
| `type` | string |  | `"goal"` | which message this is |
| `ts` | number | s (unix) | >= 0 | wall-clock time the data was MEASURED (time.time() on the producer) |
| `action` | string |  | "go" / "cancel" | "go" = drive to x, y; "cancel" = abandon the current goal |
| `x` *(optional)* | number | m |  | east of the map origin (ignored for cancel) |
| `y` *(optional)* | number | m |  | north of the map origin (ignored for cancel) |

Example:

```json
{
  "type": "goal",
  "v": 1,
  "ts": 1760000000.5,
  "action": "go",
  "x": 5.0,
  "y": 0.0
}
```

- On action 'go', x and y are required.
- The visualiser sends 'cancel' when you take over with the arrow keys, so a planner knows to stop sending cmd_vel.

## `path`  (port 5008)

The path the planner intends to follow, in the local map frame, for display.

- **Sent by:** your path planner
- **Used by:** the visualiser (draws it), anyone else
- **Rate:** whenever the plan changes

| Field | Type | Unit | Allowed | Meaning |
|---|---|---|---|---|
| `type` | string |  | `"path"` | which message this is |
| `ts` | number | s (unix) | >= 0 | wall-clock time the data was MEASURED (time.time() on the producer) |
| `points` | array of [x, y] pairs | m |  | waypoints [[x, y], ...], start to goal. An empty list clears the displayed path |

Example:

```json
{
  "type": "path",
  "v": 1,
  "ts": 1760000000.5,
  "points": [
    [
      0.0,
      0.0
    ],
    [
      2.0,
      1.5
    ],
    [
      5.0,
      0.0
    ]
  ]
}
```

- The visualiser only draws this; it never drives from it. Drive the robot with cmd_vel.

## Sim only: ground-truth pose (not part of the bus)

`sim_robot.py` sends the robot's TRUE pose to `sim_lidar.py` as plain unicast UDP to `127.0.0.1:5000`, as `{"x": mm, "y": mm, "heading": deg}` in the lidar simulator's image frame (millimetres from the top-left of the world image, y down, heading clockwise). The lidar simulator needs it to know where to ray-cast from. Nothing else uses it, and the robot's own map/navigation code never sees it; it only learns about the world from the lidar scans.

## Using it from Python

```python
from common.protocol import Publisher, Subscriber

sub = Subscriber(["rtk", "esp", "lidar"])
pub = Publisher()
while True:
    for msg in sub.poll():            # already validated
        if msg["type"] == "rtk" and msg["fix"] == "Fixed RTK":
            print(msg["lat"], msg["lon"])
    pub.send("cmd_vel", linear=0.3, angular=0.0)
```

## Using it from anything else

Send the JSON as a UDP datagram to `239.255.42.42:<port>` (set the multicast interface to `127.0.0.1`), or listen on the port after joining the group. There is nothing Python-specific in the wire format.
