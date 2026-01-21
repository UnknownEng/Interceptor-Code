"""
Visual overlay for debugging.
"""

import cv2
import time


class Display:
    """Handles visual overlay drawing."""

    def __init__(self):
        self.last_time = time.time()
        self.fps = 0

    def draw(self, frame, target, state, control):
        """Draw overlay on frame.

        Args:
            frame: BGR numpy array
            target: dict with 'x', 'y', 'brightness' or None
            state: DroneState
            control: dict with 'roll', 'pitch', 'yaw', 'throttle'

        Returns:
            frame with overlay drawn
        """
        out = frame.copy()
        h, w = out.shape[:2]

        # Calculate FPS
        now = time.time()
        dt = now - self.last_time
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
        self.last_time = now

        # Draw crosshair at center
        cx, cy = w // 2, h // 2
        cv2.line(out, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 1)
        cv2.line(out, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 1)

        # Draw target
        if target:
            tx, ty = int(target['x']), int(target['y'])
            cv2.circle(out, (tx, ty), 15, (0, 0, 255), 2)
            cv2.putText(out, f"({tx}, {ty})", (tx + 20, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Draw info panel
        y = 20
        color = (255, 255, 255)
        cv2.putText(out, f"FPS: {self.fps:.0f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        y += 25
        cv2.putText(out, f"R:{state.imu['roll']:+.1f} P:{state.imu['pitch']:+.1f} Y:{state.imu['yaw']:+.1f}",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        y += 25
        cv2.putText(out, f"Alt: {state.barometer['altitude']:.1f}m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Draw control outputs
        y += 25
        cv2.putText(out, f"Ctrl: R={control['roll']:+.2f} P={control['pitch']:+.2f}",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        return out
