"""
Simplified WebSocket client for drone simulator.
Uses a separate thread for websocket communication to avoid blocking cv2.
"""

import json
import time
import asyncio
import base64
import threading
import cv2
import numpy as np
import websockets


# Default simulator preset - exact copy from local_dev_interceptor_auto_lock
DEFAULT_PRESET = {
    'time_of_day': 'night',
    'vision_mode': 'thermal',
    'drone_type': 'winged',
    'wind_strength': 0.5,
    'targets': {
        'drone': {'count': 0},
        'plane': {
            'count': 1,
            'height_range': [300, 350],
            'distance_range': [0, 10],
            'speed': 40,
            'evasive': True,
        },
        'vehicle': {'count': 0},
        'fennek': {'count': 0},
    },
}


class DroneState:
    """Drone state - simulates real sensors (no GPS)."""
    def __init__(self):
        # IMU - attitude (degrees) and acceleration (m/s²)
        self.imu = {
            'roll': 0, 'pitch': 0, 'yaw': 0,
            'ax': 0, 'ay': 0, 'az': 9.81
        }
        # Barometer - altitude only (no lat/lon)
        self.barometer = {'altitude': 0}

    def update(self, data):
        orientation = data['orientation']
        self.imu['roll'] = orientation['roll']
        self.imu['pitch'] = orientation['pitch']
        self.imu['yaw'] = orientation['yaw']

        velocity = data['velocity']
        self.imu['ax'] = velocity['vx'] * 0.1
        self.imu['ay'] = velocity['vy'] * 0.1
        self.imu['az'] = 9.81 + velocity['vz'] * 0.1

        self.barometer['altitude'] = data['position']['altitude']


class SimClient:
    """WebSocket client for drone simulator."""

    def __init__(self, url="ws://localhost:8080"):
        self.url = url
        self.ws = None
        self.connected = False
        self.running = True
        self.state = DroneState()
        self.latest_frame = None
        self._frame_count = 0

        # Asyncio loop runs in separate thread
        self._loop = None
        self._thread = None

    def connect(self):
        """Connect to simulator (blocking, starts background thread)."""
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        # Wait for connection
        event = threading.Event()
        self._loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self._connect_async(event))
        )
        event.wait(timeout=5)
        return self.connected

    def _run_loop(self):
        """Run asyncio event loop in thread."""
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    async def _connect_async(self, event):
        """Async connection."""
        try:
            self.ws = await websockets.connect(self.url)
            self.connected = True
            print(f"Connected to {self.url}")
            asyncio.create_task(self._receive_loop())
        except Exception as e:
            print(f"Connection failed: {e}")
        event.set()

    def start_camera(self, rate=30, width=640, height=480, preset=None):
        """Start camera stream with optional simulator preset."""
        if not self.connected:
            return
        if preset is None:
            preset = DEFAULT_PRESET
        self._loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self._start_camera_async(rate, width, height, preset))
        )

    async def _start_camera_async(self, rate, width, height, preset):
        data = {
            "cameraId": "camera1",
            "active": True,
            "rate": rate,
            "quality": 0.8,
            "width": width,
            "height": height,
            "fov": 120.0,  # From calculate_cesium_fov([715,715], (640,512)) - kurbas-512
            "angle": 90,   # Camera looks up (interceptor config)
            **preset
        }
        msg = {
            "type": "camera_stream_multi",
            "data": data,
            "sender": "client",
            "timestamp": int(time.time() * 1000)
        }
        msg_str = json.dumps(msg)
        print(f"Sending message ({len(msg_str)} bytes):")
        print(f"  targets: {data.get('targets')}")
        await self.ws.send(msg_str)
        print(f"Camera config sent: {width}x{height} @ {rate}fps, fov={data['fov']}, angle={data['angle']}")

    def reset(self):
        """Reset the simulator."""
        if not self.connected:
            return
        self._loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self._reset_async())
        )

    async def _reset_async(self):
        msg = {
            "type": "command",
            "data": {
                "command": "reset"
            },
            "sender": "client",
            "timestamp": int(time.time() * 1000)
        }
        await self.ws.send(json.dumps(msg))
        print("Simulator reset")

    def set_position(self, latitude, longitude, heading=0):
        """Set drone position and heading."""
        if not self.connected:
            return
        self._loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self._set_position_async(latitude, longitude, heading))
        )

    async def _set_position_async(self, latitude, longitude, heading):
        msg = {
            "type": "command",
            "data": {
                "command": "setPosition",
                "params": {
                    "latitude": latitude,
                    "longitude": longitude,
                    "altutude": 0,
                    "roll": 0,
                    "pitch": 0,
                    "yaw": heading
                }
            },
            "sender": "client",
            "timestamp": int(time.time() * 1000)
        }
        await self.ws.send(json.dumps(msg))
        print(f"Position set to: lat={latitude}, lon={longitude}, heading={heading}")

    def send_control(self, roll=0, pitch=0, yaw=0, throttle=0):
        """Send control commands (non-blocking). Values should be -1 to 1."""
        if not self.connected:
            return
        self._loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(
                self._send_control_async(roll, pitch, yaw, throttle)
            )
        )

    async def _send_control_async(self, roll, pitch, yaw, throttle):
        msg = {
            "type": "control",
            "data": {
                "roll": max(-1, min(1, roll)),
                "pitch": max(-1, min(1, pitch)),
                "yaw": max(-1, min(1, yaw)),
                "throttle": max(-1, min(1, throttle))
            },
            "sender": "client",
            "timestamp": int(time.time() * 1000)
        }
        await self.ws.send(json.dumps(msg))

    async def _receive_loop(self):
        """Receive messages from simulator."""
        try:
            while self.running and self.connected:
                message = await self.ws.recv()
                data = json.loads(message)

                if data["type"] == "camera_frame_multi":
                    frame_data = data["data"]
                    if frame_data.get("cameraId") == "camera1":
                        # Decode frame
                        b64 = frame_data.get("data", "")
                        if b64:
                            padding = len(b64) % 4
                            if padding:
                                b64 += '=' * (4 - padding)
                            img_bytes = base64.b64decode(b64)
                            arr = np.frombuffer(img_bytes, np.uint8)
                            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                            if img is not None:
                                self.latest_frame = img
                                self._frame_count += 1
                                if self._frame_count == 1:
                                    print(f"First frame received: {img.shape[1]}x{img.shape[0]}")

                        # Update state
                        if "droneState" in frame_data:
                            self.state.update(frame_data["droneState"])

        except websockets.exceptions.ConnectionClosed:
            print("Connection closed")
            self.connected = False
        except Exception as e:
            print(f"Receive error: {e}")
            self.connected = False

    def disconnect(self):
        """Disconnect from simulator."""
        self.running = False
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=1)
