"""SpaceMouse teleoperation configuration for RM75B."""

import os

_HERE = os.path.dirname(os.path.abspath(__file__))

# --- Robot connection ---
ROBOT_IP = "192.168.5.74"
ROBOT_PORT = 8080

# --- Control loop ---
CONTROL_RATE_HZ = 50

# --- libspnav ---
LIBSPNAV_PATH = os.path.join(_HERE, "libspnav.so.0.4")

# --- SpaceMouse input processing ---
DEADZONE = 40                            # raw axis dead-zone (full range ~+-350)
TRANSLATION_SCALE = 0.0004 / 350.0       # full deflection -> 0.0004 m/cycle (0.02 m/s @ 50Hz)
ROTATION_SCALE = 0.004 / 350.0           # full deflection -> 0.005 rad/cycle (0.25 rad/s @ 50Hz)

MAX_TRANSLATION_PER_CYCLE = 0.001        # m, hard clamp per cycle
MAX_ROTATION_PER_CYCLE = 0.01            # rad, hard clamp per cycle

EMA_ALPHA = 1.0                          # exponential moving average smoothing

# --- Axis mapping: SpaceMouse -> robot [x, y, z, rx, ry, rz] ---
AXIS_SIGNS = [1, -1, 1, -1, 1, 1]       # flip signs to match intuitive directions
AXIS_MAP = [2, 0, 1, 5, 3, 4]           # reorder axes if needed
AXIS_ENABLE = [1, 1, 1, 0, 0, 1]        # 各轴开关: [x, y, z, rx, ry, rz], 0=屏蔽 1=启用

# --- Workspace limits (meters, in robot base frame) ---
WORKSPACE_MIN = [-0.5, -0.5, 0.0]
WORKSPACE_MAX = [0.5, 0.5, 0.7]

# --- Gripper ---
GRIPPER_OPEN_POS = 1000
GRIPPER_CLOSE_POS = 0
GRIPPER_MODE = "incremental"         # "binary" = 按一下全开/全闭; "incremental" = 按住持续开合
GRIPPER_INCREMENT = 20               # incremental 模式下每周期变化量 (0~1000 范围)
