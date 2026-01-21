#!/usr/bin/env python3
import sys, time, cv2, numpy as np
sys.stdout.reconfigure(line_buffering=True)

from sim_client import SimClient
from detector import find_brightest_point, reset_tracking
from controller import ProNavController
from display import Display

# ---------------- CONFIG ----------------
BRIGHTNESS_THRESHOLD = 12  
BASE_THROTTLE = 0.65       
NAV_GAIN = 4.5             # Base gain for stable approach
# ----------------------------------------

def main():
    client = SimClient(url="ws://localhost:8080")
    if not client.connect(): return

    client.set_position(latitude=52.52, longitude=13.405)
    time.sleep(0.5)

    # Force Camera to 120-degree mode (Ensure sim_client.py "fov" is 120.0)
    client.start_camera(rate=30, width=640, height=512)
    time.sleep(1.0) 

    controller = ProNavController(n=NAV_GAIN, filter_alpha=0.25)
    controller.on_start() 
    
    display = Display()
    lock_count = 0

    try:
        while True:
            frame = client.latest_frame
            if frame is None: continue

            x, y, brightness = find_brightest_point(frame, client.state)

            if brightness >= BRIGHTNESS_THRESHOLD:
                lock_count += 1
                target = {'x': x, 'y': y, 'brightness': brightness} if lock_count > 3 else None
            else:
                target = None
                lock_count = 0
                reset_tracking()

            # --- TERMINAL INTERCEPT OVERRIDE ---
            if target:
                # Calculate the estimated distance based on thermal signature
                dist = (715 * 1.5) / max(1, target.get('brightness', 1))
                
                # TRIGGER AT THE 8.5m THRESHOLD
                if dist <= 8.5:
                    print(f"-> INTERCEPTED")
                    controller.n = 8.0   # Maximum turn aggression
                    throttle_cmd = 1.0   # Full motor power to close the gap
                elif dist < 20.0:
                    controller.n = 6.0   # Increase lead angle pursuit
                    throttle_cmd = 0.95 
                    print(f" Tring to intercept")
                else:
                    controller.n = NAV_GAIN # Standard approach gain
                    # Maintain high power for the climb to target altitude
                    throttle_cmd = 0.85 if client.state.barometer['altitude'] < 480 else BASE_THROTTLE
            else:
                throttle_cmd = BASE_THROTTLE

            control = controller.on_frame(frame, target, client.state)

            client.send_control(
                roll=control.get('roll', 0.0),
                pitch=control.get('pitch', 0.0), 
                yaw=0.0,
                throttle=throttle_cmd
            )

            out = display.draw(frame, target, client.state, control)
            cv2.imshow("AEROMAVERICKS - TERMINAL INTERCEPT", out)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        client.disconnect()
        cv2.destroyAllWindows()

if __name__ == "__main__": main()
