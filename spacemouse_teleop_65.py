"""SpaceMouse teleoperation for RM65B — incremental Cartesian control via rm_movep_canfd."""

import sys
import argparse
import signal
import time

import numpy as np

import config as cfg
from spacemouse_input import SpaceMouseReader
from rm65b import RM65BInterface


class SpaceMouseTeleop:
    """Incremental Cartesian teleop: SpaceMouse -> rm_movep_canfd -> RM65B."""

    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port

        self.mouse = None
        self.arm = None

        self.target_pose = None          # [x, y, z, rx, ry, rz] (m / rad)
        self._smoothed = np.zeros(6)     # EMA state
        self._consecutive_fails = 0
        self._gripper_pos = float(cfg.GRIPPER_OPEN_POS)  # 当前夹爪位置 (0~1000)

    # ------ setup / teardown ------

    def setup(self):
        # 1. SpaceMouse
        self.mouse = SpaceMouseReader()
        self.mouse.open()
        self.mouse.start()

        # 2. Robot arm + gripper
        self.arm = RM65BInterface(self.ip, self.port, enable_gripper=True)

        # 3. Read current pose as starting point
        ret, state = self.arm.arm.rm_get_current_arm_state()
        if ret != 0:
            raise RuntimeError(f"Failed to read arm state (ret={ret})")
        # state["pose"] = [x, y, z, rx, ry, rz] (m / rad)
        # RM65 控制链路固定按 6 维姿态运行，这里只保留前 6 维。
        self.target_pose = np.array(state["pose"][:6], dtype=float)
        if self.target_pose.shape[0] != 6:
            raise RuntimeError(f"Invalid pose dimension: got {self.target_pose.shape[0]}, expected 6")
        print(f"Start pose: {np.round(self.target_pose, 4).tolist()}")

    def teardown(self, slow_stop: bool = True):
        """Clean shutdown: stop arm, open gripper, disconnect."""
        if self.arm is not None:
            if slow_stop:
                self.arm.arm.rm_set_arm_slow_stop()
            self.arm.set_gripper_position(cfg.GRIPPER_OPEN_POS)
            time.sleep(0.3)
            self.arm.close()
        if self.mouse is not None:
            self.mouse.stop()
        print("Shutdown complete.")

    # ------ control helpers ------

    def _compute_delta(self, raw_axes):
        """Apply axis mapping, sign flip, EMA smoothing, scaling, and clamping."""
        mapped = np.array([raw_axes[cfg.AXIS_MAP[i]] * cfg.AXIS_SIGNS[i] for i in range(6)],
                          dtype=float)

        # EMA smoothing
        self._smoothed = cfg.EMA_ALPHA * mapped + (1.0 - cfg.EMA_ALPHA) * self._smoothed

        # Scale to physical units
        delta = np.zeros(6)
        delta[:3] = self._smoothed[:3] * cfg.TRANSLATION_SCALE
        delta[3:] = self._smoothed[3:] * cfg.ROTATION_SCALE

        # Per-axis hard clamp
        delta[:3] = np.clip(delta[:3], -cfg.MAX_TRANSLATION_PER_CYCLE, cfg.MAX_TRANSLATION_PER_CYCLE)
        delta[3:] = np.clip(delta[3:], -cfg.MAX_ROTATION_PER_CYCLE, cfg.MAX_ROTATION_PER_CYCLE)

        # Axis enable mask
        delta *= np.array(cfg.AXIS_ENABLE, dtype=float)

        return delta

    def _clamp_workspace(self):
        """Clamp target_pose xyz to workspace boundaries."""
        for i in range(3):
            self.target_pose[i] = np.clip(self.target_pose[i],
                                          cfg.WORKSPACE_MIN[i], cfg.WORKSPACE_MAX[i])

    def _handle_buttons(self):
        """Process button events for gripper control."""
        if cfg.GRIPPER_MODE == "binary":
            # 按一下立即到位
            for bnum, pressed in self.mouse.pop_button_events():
                if not pressed:
                    continue
                if bnum == 0:
                    self._gripper_pos = float(cfg.GRIPPER_CLOSE_POS)
                    print("Gripper CLOSE")
                elif bnum == 1:
                    self._gripper_pos = float(cfg.GRIPPER_OPEN_POS)
                    print("Gripper OPEN")
                self.arm.set_gripper_position(self._gripper_pos)
        else:
            # incremental: 按住持续变化，松开停止
            self.mouse.pop_button_events()  # 消费事件队列，保持按钮状态更新
            if self.mouse.get_button(0):
                self._gripper_pos = max(cfg.GRIPPER_CLOSE_POS,
                                        self._gripper_pos - cfg.GRIPPER_INCREMENT)
                self.arm.set_gripper_position(self._gripper_pos)
            elif self.mouse.get_button(1):
                self._gripper_pos = min(cfg.GRIPPER_OPEN_POS,
                                        self._gripper_pos + cfg.GRIPPER_INCREMENT)
                self.arm.set_gripper_position(self._gripper_pos)

    # ------ main loop ------

    def run(self):
        """50 Hz control loop."""
        dt = 1.0 / cfg.CONTROL_RATE_HZ
        print(f"Running at {cfg.CONTROL_RATE_HZ} Hz.  Ctrl+C to stop.\n")

        while True:
            t0 = time.monotonic()

            # 1. Read SpaceMouse
            raw = self.mouse.get_axes()

            # 2. Compute incremental delta
            delta = self._compute_delta(raw)

            # 3. Accumulate target pose
            self.target_pose += delta

            # 4. Workspace clamp
            self._clamp_workspace()

            # 5. Send to robot
            ret = self.arm.arm.rm_movep_canfd(self.target_pose.tolist(), follow=True)
            if ret != 0:
                self._consecutive_fails += 1
                if self._consecutive_fails >= 10:
                    print(f"[ERROR] {self._consecutive_fails} consecutive movep failures — emergency stop!")
                    self.arm.arm.rm_set_arm_stop()
                    break
            else:
                self._consecutive_fails = 0

            # 6. Gripper buttons
            self._handle_buttons()

            # 7. Timing
            elapsed = time.monotonic() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


# ---------- entry point ----------

def main():
    parser = argparse.ArgumentParser(description="SpaceMouse teleop for RM65B")
    parser.add_argument("--ip", default=cfg.ROBOT_IP, help="Robot IP address")
    parser.add_argument("--port", type=int, default=cfg.ROBOT_PORT, help="Robot TCP port")
    args = parser.parse_args()

    teleop = SpaceMouseTeleop(args.ip, args.port)

    def _sigint(sig, frame):
        print("\nCtrl+C received — stopping...")
        teleop.teardown(slow_stop=True)
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    teleop.setup()
    teleop.run()


if __name__ == "__main__":
    main()
