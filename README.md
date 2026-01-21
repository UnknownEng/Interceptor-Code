# Drone Intercept Challenge

**Goal**: Implement a guidance algorithm that reliably hits a moving target (plane's motor).

This is NOT a tracking problem - you must predict where the target will be and intercept it, not just follow it.

Vehicle reference: [Quadcopter with wings](https://www.thingiverse.com/thing:4236457)

## Quick Start
1. Open simulator: https://sim.dremian.com/simulator.html (activation code: `dremian`)
2. In a separate terminal, start the websocket proxy:
```bash
python websocket_proxy.py
```
3. In another terminal, run the main script:
```bash
pip install -r requirements.txt
python run.py
```

## Your Task

Implement `on_frame()` in `controller.py`:

```python
def on_frame(self, frame, target, state):
    # target: {'x', 'y', 'brightness'} or None - from detector / tracker, could be noisy, might need smoothing
    # state.imu: {'roll', 'pitch', 'yaw', 'ax', 'ay', 'az'}
    # state.barometer: {'altitude'}
    # NOTE: no GSP, no LIDAR
    # CAMERA Matrix is known (focal distance, etc),
    # you can use camera params to calc Line of Sight vector for proportional navigation
    return {'roll': 0, 'pitch': 0, 'yaw': 0, 'throttle': 0}  # -1 to 1
```

## Vehicle Specs (from simulator)
| Parameter | Value |
|-----------|-------|
| Type | Quadcopter + wings (hybrid) |
| Mass | 4.0 kg |
| Max Thrust | 80 N (~2:1 thrust-to-weight) |
| Max Speed | ~40 m/s |
| Camera | 640x512 thermal, 48° FOV, f=715px |
| Camera Mount | Looking UP (90°) |

See `sim_client.py` for full camera parameters. Focal length: `f = width / (2 * tan(fov/2))`

## Target
- Plane flying at 500-600m altitude
- Speed: ~40 m/s (144 km/h)
- Evasive maneuvers enabled
- Appears as brightest point in thermal view
- Frame center: (320, 256)

## Possible Approaches

1. **Proportional Navigation (PN)** - Classic missile guidance
   - Command acceleration perpendicular to LOS
   - `a_cmd = N * V_c * LOS_rate` where N ≈ 3-5

2. **Augmented PN** - Accounts for target acceleration

3. **Model Predictive Control (MPC)** - Optimize trajectory over horizon

4. **Reinforcement Learning** - Train policy to maximize hit probability

5. **Your own idea/approach** - feel free to come up with any other idea or combination

Press 'q' to quit.
