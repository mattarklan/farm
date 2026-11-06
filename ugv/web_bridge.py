#!/usr/bin/env python3
"""
Web visualizer bridge server with cmap.png loading and UDP bus bridging.
"""

import asyncio
import json
import os
import websockets
from PIL import Image
import numpy as np
from common.protocol import Subscriber, Publisher

CONNECTED_CLIENTS = set()
pub = None
map_payload_cache = None

def load_cmap_as_base64():
    global map_payload_cache
    # Search for cmap.png in current and common paths
    paths = ["cmap.png", "sims/cmap.png", os.path.join(os.path.dirname(__file__), "cmap.png")]
    cmap_path = next((p for p in paths if os.path.exists(p)), None)
    
    if not cmap_path:
        print("[MAP] Warning: cmap.png not found. Running without static background map.")
        return None

    try:
        im = Image.open(cmap_path).convert("L")
        # Invert if necessary so walls are black (0) and free space is white (255)
        arr = np.asarray(im, dtype=np.uint8)
        
        # Convert to RGB png bytes or base64 data url
        import io
        buffered = io.BytesIO()
        im.save(buffered, format="PNG")
        import base64
        img_str = base64.b64encode(buffered.getvalue()).decode("utf-8")
        
        h, w = arr.shape
        res = 0.05 # standard resolution from config
        map_payload_cache = {
            "type": "map_image",
            "width": w,
            "height": h,
            "resolution": res,
            "width_m": w * res,
            "height_m": h * res,
            "data_url": f"data:image/png;base64,{img_str}"
        }
        print(f"[MAP] Successfully loaded static map from {cmap_path} ({w}x{h} px)")
    except Exception as e:
        print(f"[MAP ERROR] Failed to load cmap.png: {e}")

async def ws_handler(websocket):
    CONNECTED_CLIENTS.add(websocket)
    print(f"[WS] Client connected from {websocket.remote_address}")
    
    # If static map is loaded, send it immediately to the newly connected client
    if map_payload_cache:
        await websocket.send(json.dumps(map_payload_cache))
        
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                msg_type = data.get("type")
                if msg_type in ("goal", "cmd_vel"):
                    fields = {k: v for k, v in data.items() if k != "type"}
                    pub.send(msg_type, **fields)
                    print(f"[UDP Published] {msg_type}: {fields}")
            except Exception as e:
                print(f"[WS Command Error]: {e}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CONNECTED_CLIENTS.remove(websocket)
        print("[WS] Client disconnected")

async def bus_bridge_loop():
    sub = Subscriber(["rtk", "esp", "lidar", "cmd_vel", "path", "goal", "pose"])
    print("[UDP Bridge] Subscribed to UGV protocol bus via common.protocol")
    
    while True:
        try:
            messages = sub.poll()
            if messages and CONNECTED_CLIENTS:
                for msg in messages:
                    payload = json.dumps(msg)
                    websockets.broadcast(CONNECTED_CLIENTS, payload)
            await asyncio.sleep(0.01)
        except Exception as e:
            print(f"[Bridge Error]: {e}")
            await asyncio.sleep(0.1)

async def main():
    global pub
    load_cmap_as_base64()
    pub = Publisher()
    async with websockets.serve(ws_handler, "0.0.0.0", 8765):
        print("[Server] WebSocket bridge running on ws://0.0.0.0:8765")
        await bus_bridge_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        if pub:
            pub.close()
        print("\n[Server] Shutting down bridge.")