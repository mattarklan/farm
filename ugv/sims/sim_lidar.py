import argparse
import math
import os
import sys
import time
import json
import socket
import threading
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common.protocol import Publisher


class LidarPublisher:
    """
    Continuous UDP Publisher for RPLidar A2 (Hardware or Simulation).
    Publishes scan frames to a dedicated UDP port.
    """
    def __init__(
        self,
        port="SIM",
        map_path=None,
        udp_ip=None,
        udp_port=5005,
        pose_listen_port=5000,
        mm_per_pixel=50.0,
        scan_rate_hz=10.0,
        samples_per_scan=180,  # Optimized to 180 for low CPU usage
        noise_std_mm=10.0,
        invert_map=False,
    ):
        self.port = port
        self.is_sim = (str(port).upper() == "SIM") or (map_path is not None and port is None)
        self.udp_ip = udp_ip
        self.udp_port = udp_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pub = Publisher() if udp_ip is None else None

        if self.is_sim:
            if not map_path:
                raise ValueError("Simulation mode requires `map_path` pointing to a B/W image.")
            
            img = Image.open(map_path)
            if img.mode in ("RGBA", "LA", "PA") or (img.mode == "P" and "transparency" in img.info):
                img = img.convert("RGBA")
                bg = Image.new("RGBA", img.size, (0, 0, 0, 255) if invert_map else (255, 255, 255, 255))
                bg.alpha_composite(img)
                img = bg
            img = img.convert("L")
            self.map_data = np.array(img)
            if invert_map:
                self.map_data = 255 - self.map_data
            self._inverted = bool(invert_map)
            self.height, self.width = self.map_data.shape
            
            self.mm_per_pixel = float(mm_per_pixel)
            self.scan_delay = 1.0 / scan_rate_hz
            self.samples_per_scan = samples_per_scan
            self.noise_std_mm = noise_std_mm
            
            self.min_range_mm = 200.0
            self.max_range_mm = 12000.0
            
            self.rover_x_mm = (self.width * self.mm_per_pixel) / 2.0
            self.rover_y_mm = (self.height * self.mm_per_pixel) / 2.0
            self.rover_heading_deg = 0.0

            if pose_listen_port:
                self._start_pose_listener(pose_listen_port)
            
            print(f"[LiDAR Pub] SIMULATION mode | Map: {os.path.abspath(map_path)} {'(inverted) ' if invert_map else ''}{self.width}x{self.height} px | Samples: {samples_per_scan}")
        else:
            from rplidar import RPLidar
            print(f"[LiDAR Pub] HARDWARE connected on {port}")
            self.hw_lidar = RPLidar(port, baudrate=115200)

    def set_pose(self, x_mm, y_mm, heading_deg=0.0):
        if self.is_sim:
            self.rover_x_mm = x_mm
            self.rover_y_mm = y_mm
            self.rover_heading_deg = heading_deg

    def _start_pose_listener(self, listen_port):
        def listen():
            pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            pose_sock.bind(("127.0.0.1", listen_port))
            while True:
                try:
                    data, _ = pose_sock.recvfrom(1024)
                    pose = json.loads(data.decode('utf-8'))
                    self.set_pose(pose['x'], pose['y'], pose.get('heading', 0.0))
                    if "invert" in pose and bool(pose["invert"]) != self._inverted:
                        self.map_data = 255 - self.map_data
                        self._inverted = bool(pose["invert"])
                    if "mm_per_pixel" in pose and abs(float(pose["mm_per_pixel"]) - self.mm_per_pixel) > 1e-6:
                        self.mm_per_pixel = float(pose["mm_per_pixel"])
                except Exception:
                    continue

        t = threading.Thread(target=listen, daemon=True)
        t.start()

    def _raycast(self, angle_deg):
        rad = math.radians(self.rover_heading_deg + angle_deg)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        
        # Step by a full pixel instead of half a pixel to slash CPU overhead
        step_mm = self.mm_per_pixel
        dist_mm = self.min_range_mm

        while dist_mm <= self.max_range_mm:
            px_mm = self.rover_x_mm + dist_mm * cos_a
            py_mm = self.rover_y_mm + dist_mm * sin_a

            gx = int(px_mm / self.mm_per_pixel)
            gy = int(py_mm / self.mm_per_pixel)

            if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
                dist_mm += step_mm
                continue

            if self.map_data[gy, gx] < 128:
                noisy_dist = dist_mm + np.random.normal(0, self.noise_std_mm)
                return max(self.min_range_mm, min(self.max_range_mm, noisy_dist))

            dist_mm += step_mm

        return 0.0

    def _capture_scan(self):
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

            elapsed = time.time() - start_time
            if elapsed < self.scan_delay:
                time.sleep(self.scan_delay - elapsed)

            return scan_points

    def start_publishing(self):
        print(f"[LiDAR Pub] Publisher active on port {self.udp_port}...")
        try:
            while True:
                points = self._capture_scan()
                if self.pub is not None:
                    self.pub.send("lidar", points=[[a, d] for a, d in points])
                    continue

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
        if self.pub is not None:
            self.pub.close()
        if not self.is_sim and hasattr(self, 'hw_lidar'):
            try:
                self.hw_lidar.stop()
                self.hw_lidar.stop_motor()
                self.hw_lidar.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="LiDAR UDP publisher")
    ap.add_argument("--port", default="SIM")
    ap.add_argument("--map", default="obstacle_map.png")
    ap.add_argument("--invert", action="store_true")
    ap.add_argument("--udp-ip", default=None)
    ap.add_argument("--udp-port", type=int, default=5005)
    ap.add_argument("--pose-port", type=int, default=5000)
    ap.add_argument("--mm-per-pixel", type=float, default=50.0)
    ap.add_argument("--scan-hz", type=float, default=10.0)
    ap.add_argument("--samples", type=int, default=180, help="samples per scan (default 180)")
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
        samples_per_scan=args.samples,
        invert_map=args.invert,
    )
    publisher.start_publishing()