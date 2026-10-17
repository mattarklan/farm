import argparse
import math
import os  # ADDED
import sys  # ADDED
import time
import json
import socket
import threading
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))  # ADDED
from common.protocol import Publisher  # ADDED

# Your LidarPublisher, unchanged except for small additions marked "# ADDED":
#   - `invert_map` option (so a black = free world image can be used with --invert)
#   - publishes on the UDP bus (port 5005, format in common/protocol.py) when udp_ip is None (the default);
#     pass an IP to get your old plain-UDP behaviour
#   - the pose listener no longer dies on one bad packet
#   - a command-line interface in the __main__ block so run_sim.py can launch it
# Layout: ugv/sims/sim_lidar.py


class LidarPublisher:
    """
    Continuous UDP Publisher for RPLidar A2 (Hardware or Simulation).
    Publishes scan frames to a dedicated UDP port.
    """
    def __init__(
        self,
        port="SIM",
        map_path=None,
        udp_ip=None,  # ADDED: None = publish on the UDP bus (port 5005). An IP string = old plain unicast
        udp_port=5005,
        pose_listen_port=5000,  # Optional: receive live pose updates over UDP in SIM mode
        mm_per_pixel=50.0,
        scan_rate_hz=10.0,
        samples_per_scan=600,
        noise_std_mm=10.0,
        invert_map=False,  # ADDED
    ):
        self.port = port
        self.is_sim = (str(port).upper() == "SIM") or (map_path is not None and port is None)
        self.udp_ip = udp_ip
        self.udp_port = udp_port

        # Dedicated UDP socket for publishing LiDAR scans
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pub = Publisher() if udp_ip is None else None  # ADDED

        if self.is_sim:
            if not map_path:
                raise ValueError("Simulation mode requires `map_path` pointing to a B/W image.")
            
            # Load map (Grayscale: <128 = Obstacle, >=128 = Free Space)
            img = Image.open(map_path)
            if img.mode in ("RGBA", "LA", "PA") or (img.mode == "P" and "transparency" in img.info):  # ADDED: transparent = free
                img = img.convert("RGBA")
                bg = Image.new("RGBA", img.size, (0, 0, 0, 255) if invert_map else (255, 255, 255, 255))
                bg.alpha_composite(img)
                img = bg
            img = img.convert("L")
            self.map_data = np.array(img)
            if invert_map:  # ADDED
                self.map_data = 255 - self.map_data
            self._inverted = bool(invert_map)  # ADDED
            self.height, self.width = self.map_data.shape
            dark = float((self.map_data < 128).mean())          # ADDED: sanity check of the world image
            if dark > 0.5:
                print(f"[LiDAR Pub] *** WARNING: {dark * 100:.0f}% of {map_path} is dark = obstacle. If black means FREE in "
                      f"your image, run with --invert.", flush=True)
            
            self.mm_per_pixel = float(mm_per_pixel)
            self.scan_delay = 1.0 / scan_rate_hz
            self.samples_per_scan = samples_per_scan
            self.noise_std_mm = noise_std_mm
            
            # RPLidar A2 Specs
            self.min_range_mm = 200.0   # 0.2m min range
            self.max_range_mm = 12000.0 # 12.0m max range
            
            # Default rover pose in world space (millimeters)
            self.rover_x_mm = (self.width * self.mm_per_pixel) / 2.0
            self.rover_y_mm = (self.height * self.mm_per_pixel) / 2.0
            self.rover_heading_deg = 0.0

            # Optional: Start background listener for live pose updates from encoder script
            if pose_listen_port:
                self._start_pose_listener(pose_listen_port)
            
            print(f"[LiDAR Pub] SIMULATION mode | Map: {os.path.abspath(map_path)} {"(inverted) " if invert_map else ""}{self.width}x{self.height} px | Streaming to {udp_ip}:{udp_port}")
        else:
            from rplidar import RPLidar
            print(f"[LiDAR Pub] HARDWARE connected on {port} | Streaming to {udp_ip}:{udp_port}")
            self.hw_lidar = RPLidar(port, baudrate=115200)

    def set_pose(self, x_mm, y_mm, heading_deg=0.0):
        """Manually update simulated rover position/heading in world coordinates."""
        if self.is_sim:
            self.rover_x_mm = x_mm
            self.rover_y_mm = y_mm
            self.rover_heading_deg = heading_deg

    def _start_pose_listener(self, listen_port):
        """Background thread to receive live rover position from pose/encoder script."""
        def listen():
            pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            pose_sock.bind(("127.0.0.1", listen_port))
            while True:
                try:
                    data, _ = pose_sock.recvfrom(1024)
                    pose = json.loads(data.decode('utf-8'))
                    self.set_pose(pose['x'], pose['y'], pose.get('heading', 0.0))
                    # ADDED: sim_robot says which world settings it uses; follow them so both sims see the SAME world
                    if "invert" in pose and bool(pose["invert"]) != self._inverted:
                        self.map_data = 255 - self.map_data
                        self._inverted = bool(pose["invert"])
                        print(f"[LiDAR Pub] sim_robot uses the world {'INVERTED' if self._inverted else 'NOT inverted'}; "
                              f"switching to match (start sim_lidar with the same --invert next time).", flush=True)
                    if "mm_per_pixel" in pose and abs(float(pose["mm_per_pixel"]) - self.mm_per_pixel) > 1e-6:
                        self.mm_per_pixel = float(pose["mm_per_pixel"])
                        print(f"[LiDAR Pub] sim_robot uses {self.mm_per_pixel:g} mm per pixel; switching to match.", flush=True)
                except Exception:
                    continue  # ADDED (was `break`, which stopped pose updates for good after one bad packet)

        t = threading.Thread(target=listen, daemon=True)
        t.start()

    def _raycast(self, angle_deg):
        """2D vector raycast against image grid."""
        rad = math.radians(self.rover_heading_deg + angle_deg)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        
        step_mm = self.mm_per_pixel / 2.0
        dist_mm = self.min_range_mm

        while dist_mm <= self.max_range_mm:
            px_mm = self.rover_x_mm + dist_mm * cos_a
            py_mm = self.rover_y_mm + dist_mm * sin_a

            gx = int(px_mm / self.mm_per_pixel)
            gy = int(py_mm / self.mm_per_pixel)

            # Outside map bounds = free space per requirement
            if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
                dist_mm += step_mm
                continue

            # Black pixel (<128) = obstacle hit
            if self.map_data[gy, gx] < 128:
                noisy_dist = dist_mm + np.random.normal(0, self.noise_std_mm)
                return max(self.min_range_mm, min(self.max_range_mm, noisy_dist))

            dist_mm += step_mm

        return 0.0  # Clear space / out of max range

    def _capture_scan(self):
        """Generates a single scan array."""
        if not self.is_sim:
            for scan_data in self.hw_lidar.iter_scans():
                return [(round(angle, 1), round(dist, 1)) for _, angle, dist in scan_data if dist > 0]
        else:
            start_time = time.time()
            scan_points = []
            angles = np.linspace(0, 360, self.samples_per_scan, endpoint=False)

            for angle in angles:
                dist_mm = self._raycast(angle)
                if dist_mm > 0:
                    scan_points.append((round(angle, 1), round(dist_mm, 1)))

            # ADDED: say so if the lidar is boxed in by "wall" on all sides (almost always a wrong --invert / image problem)
            if scan_points and time.time() - getattr(self, "_warned", 0) > 5:
                near = sum(1 for _, d in scan_points if d <= self.min_range_mm + 15) / len(scan_points)
                if near > 0.6:
                    self._warned = time.time()
                    gx, gy = int(self.rover_x_mm / self.mm_per_pixel), int(self.rover_y_mm / self.mm_per_pixel)
                    val = int(self.map_data[gy, gx]) if 0 <= gx < self.width and 0 <= gy < self.height else "outside the image"
                    print(f"[LiDAR Pub] *** {near * 100:.0f}% of rays hit a wall at {self.min_range_mm:.0f} mm. Robot is at pixel "
                          f"({gx}, {gy}), pixel value {val} (below 128 = wall). Fix the world image or add/remove --invert.", flush=True)

            # Maintain 10 Hz rate timing
            elapsed = time.time() - start_time
            if elapsed < self.scan_delay:
                time.sleep(self.scan_delay - elapsed)

            return scan_points

    def start_publishing(self):
        """Starts the infinite continuous streaming loop."""
        print(f"[LiDAR Pub] Publisher active. Broad-casting frames on port {self.udp_port}...")
        try:
            while True:
                points = self._capture_scan()
                
                if self.pub is not None:  # ADDED: validated message on the bus
                    self.pub.send("lidar", points=[[a, d] for a, d in points])
                    continue

                # Payload format: JSON containing timestamp and list of [angle_deg, dist_mm]
                payload = json.dumps({
                    "timestamp": time.time(),
                    "points": points
                }).encode('utf-8')

                self.sock.sendto(payload, (self.udp_ip, self.udp_port))
        except KeyboardInterrupt:
            print("\n[LiDAR Pub] Stopping publisher...")
        finally:
            self.disconnect()

    def disconnect(self):
        self.sock.close()
        if self.pub is not None:  # ADDED
            self.pub.close()
        if not self.is_sim and hasattr(self, 'hw_lidar'):
            try:
                self.hw_lidar.stop()
                self.hw_lidar.stop_motor()
                self.hw_lidar.disconnect()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Direct Script Execution  (ADDED: command-line options; defaults match your old constants)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="LiDAR UDP publisher (simulated or RPLidar hardware)")
    ap.add_argument("--port", default="SIM", help='"SIM" for simulation (default) or a COM port like COM5')
    ap.add_argument("--map", default="obstacle_map.png", help="B/W map image for SIM mode (black = obstacle)")
    ap.add_argument("--invert", action="store_true", help="map is black = free, white = obstacle")
    ap.add_argument("--udp-ip", default=None, help="send plain unicast UDP to this IP instead of using the bus")
    ap.add_argument("--udp-port", type=int, default=5005, help="port for --udp-ip mode (the bus always uses 5005)")
    ap.add_argument("--pose-port", type=int, default=5000, help="port the robot pose is received on (SIM)")
    ap.add_argument("--mm-per-pixel", type=float, default=50.0)
    ap.add_argument("--scan-hz", type=float, default=10.0)
    args = ap.parse_args()

    use_sim = args.port.upper() == "SIM"
    publisher = LidarPublisher(
        port=args.port,
        map_path=args.map if use_sim else None,
        udp_ip=args.udp_ip,
        udp_port=args.udp_port,
        pose_listen_port=args.pose_port,
        mm_per_pixel=args.mm_per_pixel,
        scan_rate_hz=args.scan_hz,
        invert_map=args.invert,
    )

    # Runs continuously until Ctrl+C
    publisher.start_publishing()
