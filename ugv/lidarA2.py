import json
import socket
import time
from rplidar import RPLidar, RPLidarException

# Protocol configuration from PROTOCOL.md
MCAST_GROUP = "239.255.42.42"
MCAST_PORT = 5005  # Port for lidar messages
MCAST_IFACE = "127.0.0.1"  # Use your LAN IP if running across multiple machines[cite: 1]

# Adjust your serial port path (e.g., '/dev/ttyUSB0' on Linux or 'COM3' on Windows)
SERIAL_PORT = "COM3"


def main():
  # Setup UDP socket for multicast transmission
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
  sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
  sock.setsockopt(
      socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(MCAST_IFACE)
  )

  print(f"Connecting to RPLidar A2 on {SERIAL_PORT}...")
  lidar = RPLidar(SERIAL_PORT, baudrate=115200)

  try:
    print("Starting lidar motor...")
    lidar.start_motor()
    print("Streaming scan data to multicast group...")

    # iter_scans yields a list of measurements for a full revolution: (quality, angle_deg, distance_mm)
    for scan in lidar.iter_scans(max_buf_meas=20000):
      ts = time.time()
      points = []

      for _, angle, dist in scan:
        # Protocol validation: 0 < d <= 12000 mm; beams with no return are left out[cite: 1]
        if 0 < dist <= 12000:
          points.append([round(float(angle), 2), round(float(dist), 1)])

      # Construct JSON payload matching the 'lidar' message schema[cite: 1]
      message = {"type": "lidar", "v": 1, "ts": ts, "points": points}

      payload = json.dumps(message)
      sock.sendto(payload.encode("utf-8"), (MCAST_GROUP, MCAST_PORT))

  except KeyboardInterrupt:
    print("\nStopping publisher...")
  except RPLidarException as e:
    print(f"RPLidar hardware error: {e}")
  finally:
    print("Shutting down lidar and closing socket...")
    try:
      lidar.stop()
      lidar.stop_motor()
      lidar.disconnect()
    except Exception:
      pass
    sock.close()


if __name__ == "__main__":
  main()