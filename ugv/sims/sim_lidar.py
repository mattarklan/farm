import math
import time
import json
import socket
import threading
import numpy as np
from PIL import Image


class LidarPublisher:
    """
    Continuous UDP Publisher for RPLidar A2 (Hardware or Simulation).
    Publishes scan frames to a dedicated UDP port.
    """
    def __init__(
        self,
        port="SIM",
        map_path=None,
        udp_ip="127.0.0.1",
        udp_port=5005,
        pose_listen_port=5000,  # Optional: receive live pose updates over UDP in SIM mode
        mm_per_pixel=50.0,
        scan_rate_hz=10.0,
        samples_per_scan=360,
        noise_std_mm=10.0,
    ):
        self.port = port
        self.is_sim = (str(port).upper() == "SIM") or (map_path is not None and port is None)
        self.udp_ip = udp_ip
        self.udp_port = udp_port

        # Dedicated UDP socket for publishing LiDAR scans
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if self.is_sim:
            if not map_path:
                raise ValueError("Simulation mode requires `map_path` pointing to a B/W image.")
            
            # Load map (Grayscale: <128 = Obstacle, >=128 = Free Space)
            img = Image.open(map_path).convert("L")
            self.map_data = np.array(img)
            self.height, self.width = self.map_data.shape
            
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
            
            print(f"[LiDAR Pub] SIMULATION mode | Map: {self.width}x{self.height} px | Streaming to {udp_ip}:{udp_port}")
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
                except Exception:
                    break

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
        if not self.is_sim and hasattr(self, 'hw_lidar'):
            try:
                self.hw_lidar.stop()
                self.hw_lidar.stop_motor()
                self.hw_lidar.disconnect()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Direct Script Execution
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    USE_SIMULATION = True
    PORT = "SIM" if USE_SIMULATION else "COM5"
    MAP_FILE = "obstacle_map.png" if USE_SIMULATION else None

    publisher = LidarPublisher(
        port=PORT,
        map_path=MAP_FILE,
        udp_ip="127.0.0.1",
        udp_port=5005,  # LiDAR dedicated port
        mm_per_pixel=50.0
    )
    
    # Runs continuously until Ctrl+C
    publisher.start_publishing()