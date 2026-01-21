import numpy as np
import time
from scipy.spatial.transform import Rotation

class ProNavController:
    def __init__(self, n=4.5, filter_alpha=0.25):
        self.n = n 
        self.filter_alpha = filter_alpha
        self.fov_deg = 120.0 
        self.last_los_world = None
        self.filtered_los_rate = np.zeros(3)
        self.last_time = None

    def on_start(self):
        self.last_los_world = None
        self.filtered_los_rate = np.zeros(3)
        self.last_time = time.time()

    def on_frame(self, frame, target, state):
        now = time.time()
        dt = now - self.last_time if self.last_time else 1.0/60.0
        self.last_time = now

        if target is None:
            return {'roll': 0, 'pitch': 0, 'yaw': 0, 'throttle': 0.3}

        h, w = frame.shape[:2]
        f = w / (2 * np.tan(np.radians(self.fov_deg / 2)))
        v_cam = np.array([(target['x'] - w/2)/f, (target['y'] - h/2)/f, 1.0])
        v_cam /= np.linalg.norm(v_cam)

        # Map to Drone Body (Camera looking UP 90 deg)
        v_drone = np.array([v_cam[1], v_cam[0], -v_cam[2]])

        r = Rotation.from_euler('zyx', [state.imu['yaw'], state.imu['pitch'], state.imu['roll']], degrees=True)
        los_world = r.apply(v_drone)

        if self.last_los_world is not None:
            cross = np.cross(self.last_los_world, los_world)
            los_rate_raw = cross / dt
            
            self.filtered_los_rate = (self.filter_alpha * los_rate_raw +
                                     (1 - self.filter_alpha) * self.filtered_los_rate)

            los_rate_body = r.inv().apply(self.filtered_los_rate)

            # Sensitivity multiplier at 0.04 to allow for the sharper terminal turn
            pitch_cmd = np.degrees(los_rate_body[1]) * self.n * 0.04
            roll_cmd = -np.degrees(los_rate_body[0]) * self.n * 0.04
            
            pitch_cmd = np.clip(pitch_cmd, -1.0, 1.0)
            roll_cmd = np.clip(roll_cmd, -1.0, 1.0)
        else:
            pitch_cmd, roll_cmd = 0, 0

        self.last_los_world = los_world
        return {'roll': roll_cmd, 'pitch': pitch_cmd}
