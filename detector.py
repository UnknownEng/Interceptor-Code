import cv2
import numpy as np

_last_pos = None
_lock_frames = 0
_search_size = 400 

def find_brightest_point(frame, state=None):
    global _last_pos, _lock_frames
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Dynamic Horizon Masking
    mask = np.zeros((h, w), dtype=np.uint8)
    pitch = state.imu['pitch'] if state else 0
    horizon_y = int(h * 0.5 + (pitch * 1.1)) 
    mask[0:max(5, min(h-5, horizon_y)), :] = 255
    gray = cv2.bitwise_and(gray, mask)

    search_area = gray
    off_x, off_y = 0, 0
    if _last_pos is not None and _lock_frames > 5:
        x0, y0 = max(0, int(_last_pos[0]-200)), max(0, int(_last_pos[1]-200))
        x1, y1 = min(w, int(_last_pos[0]+200)), min(h, int(_last_pos[1]+200))
        search_area = gray[y0:y1, x0:x1]
        off_x, off_y = x0, y0

    search_area = cv2.GaussianBlur(search_area, (9, 9), 0)
    _, max_val, _, _ = cv2.minMaxLoc(search_area)
    
    if max_val >= 12:
        _, thresh = cv2.threshold(search_area, max_val - 2, 255, cv2.THRESH_BINARY)
        M = cv2.moments(thresh)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"]) + off_x
            cy = int(M["m01"] / M["m00"]) + off_y
            
            # Reset if target hits extreme edges of 120 FOV
            if cx < 10 or cx > (w - 10) or cy < 10 or cy > (h - 10):
                reset_tracking()
                return None, None, 0
            
            _last_pos = (cx, cy)
            _lock_frames += 1
            return cx, cy, max_val

    reset_tracking()
    return None, None, 0

def reset_tracking():
    global _last_pos, _lock_frames
    _last_pos = None
    _lock_frames = 0
