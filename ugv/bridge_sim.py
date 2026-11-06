#!/usr/bin/env python3
"""
Integrated simulation & Web/Phone WebSocket bridge.
Runs a lightweight built-in rover simulation (pose, lidar, esp, rtk)
and streams it over WebSocket port 8765 so your browser/phone visualizer works instantly.
"""

import asyncio
import json
import math
import websockets

CONNECTED_CLIENTS = set()

# Simulated Robot State
robot = {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0,
    "speed": 0.5,
    "seq": 0
}

async def ws_handler(websocket):
    CONNECTED_CLIENTS.add(websocket)
    print(f"[WS] Client connected from {websocket.remote_address}")
    try:
        async for message in websocket:
            data = json.loads(message)
            # Handle incoming click-to-drive goals from browser/phone
            if data.get("type") == "goal":
                print(f"[GOAL] Received target from web UI: x={data.get('x'):.2f}, y={data.get('y'):.2f}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CONNECTED_CLIENTS.remove(websocket)
        print("[WS] Client disconnected")

async def simulation_loop():
    while True:
        await asyncio.sleep(0.05) # 20 Hz update rate
        
        # Drive in a smooth circle / lawnmower pattern for testing
        robot["seq"] += 1
        robot["yaw"] += 0.02
        robot["x"] += robot["speed"] * math.cos(robot["yaw"]) * 0.05
        robot["y"] += robot["speed"] * math.sin(robot["yaw"]) * 0.05

        # Generate synthetic 360-degree Lidar points (every 5 degrees)
        lidar_points = []
        for deg in range(0, 360, 5):
            dist_mm = 3000 + 500 * math.sin(math.radians(deg * 4 + robot["seq"]))
            lidar_points.append([float(deg), float(dist_mm)])

        # Construct bus data packets matching index.html handlers
        packets = [
            json.dumps({"type": "rtk", "fix": "Fixed RTK", "sats": 16, "hdop": 0.7, "lat": -31.9505, "lon": 115.8605}),
            json.dumps({"type": "esp", "seq": robot["seq"], "enc": {"left_vel": 0.5 * 1000, "right_vel": 0.5 * 1000}, "imu": {"gyro": [0, 0, 0.02]}}),
            json.dumps({"type": "pose", "x": robot["x"], "y": robot["y"], "yaw": robot["yaw"]}),
            json.dumps({"type": "lidar", "points": lidar_points})
        ]

        if CONNECTED_CLIENTS:
            websockets.broadcast(CONNECTED_CLIENTS, packets[0])
            websockets.broadcast(CONNECTED_CLIENTS, packets[1])
            websockets.broadcast(CONNECTED_CLIENTS, packets[2])
            websockets.broadcast(CONNECTED_CLIENTS, packets[3])

async def main():
    async with websockets.serve(ws_handler, "0.0.0.0", 8765):
        print("[Server] WebSocket Simulation Bridge running on ws://0.0.0.0:8765")
        print("[Server] Open index.html in your browser and click 'Connect'!")
        await simulation_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Server] Shut down.")