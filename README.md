# SpaceMouse Teleop for RM75-B

Incremental Cartesian teleoperation of the **RealMan RM75-B** 7-DOF arm using a **3Dconnexion SpaceMouse**, with a ZhiXing 90D gripper controlled via Modbus RTU.

## File Structure

```
spacemouse_teleop/
├── config.py               # All tunable parameters (IP, scales, axis mapping, …)
├── spacemouse_input.py     # Non-blocking SpaceMouse reader (libspnav + ctypes)
├── spacemouse_teleop.py    # Main control loop (50 Hz Cartesian streaming)
├── rm75b.py                # RM75-B hardware interface (arm + gripper)
└── libspnav.so.0.4         # libspnav shared library (bundled)
```

## Dependencies

### System

- **spacenavd** — userspace SpaceMouse daemon

  ```bash
  sudo apt install spacenavd
  sudo systemctl enable --now spacenavd
  ```

### Python

- **numpy**
- **Robotic-Arm** — RealMan official Python SDK

  ```bash
  pip install numpy Robotic-Arm
  ```

## Quick Start

1. Plug in the SpaceMouse and confirm `spacenavd` is running:

   ```bash
   systemctl is-active spacenavd
   ```

2. Connect the robot arm over Ethernet and set the IP in `config.py`:

   ```python
   ROBOT_IP = "192.168.5.74"
   ```

3. Run:

   ```bash
   python3 spacemouse_teleop.py
   # or override IP at runtime:
   python3 spacemouse_teleop.py --ip 192.168.5.74 --port 8080
   ```

4. Press **Ctrl+C** to stop — the arm will slow-stop and the gripper will open.

## Controls

| Input | Action |
|-------|--------|
| Translate SpaceMouse | Move end-effector (X / Y / Z) |
| Rotate SpaceMouse | Rotate end-effector (configurable axes) |
| Button 0 (hold) | Close gripper |
| Button 1 (hold) | Open gripper |

## Configuration (`config.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ROBOT_IP` | `192.168.5.74` | Robot TCP/IP address |
| `CONTROL_RATE_HZ` | `50` | Control loop frequency |
| `DEADZONE` | `40` | SpaceMouse raw axis dead-zone |
| `TRANSLATION_SCALE` | `0.0004/350` | Full deflection → m/cycle |
| `ROTATION_SCALE` | `0.004/350` | Full deflection → rad/cycle |
| `AXIS_MAP` | `[2,0,1,5,3,4]` | Remap SpaceMouse axes to robot axes |
| `AXIS_SIGNS` | `[1,-1,1,-1,1,1]` | Flip axis directions |
| `AXIS_ENABLE` | `[1,1,1,0,0,1]` | Enable/disable individual axes |
| `WORKSPACE_MIN/MAX` | `±0.5 m, 0~0.7 m` | Cartesian workspace clamp |
| `GRIPPER_MODE` | `incremental` | `binary` (one-shot) or `incremental` (hold) |

## Testing SpaceMouse Input Only

```bash
python3 spacemouse_input.py
```

Prints live axis values and button events without connecting to the robot.
