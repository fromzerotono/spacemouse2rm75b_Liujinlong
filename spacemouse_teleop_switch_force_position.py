"""SpaceMouse teleoperation for RM75B via rm_force_position_move_pose."""

import sys
import argparse
import signal
import time

import numpy as np

import config_switch_force_position as cfg
from spacemouse_input import SpaceMouseReader
from rm75b import RM75BInterface
from class_switch import USBRelayController 


class SpaceMouseTeleop(USBRelayController):
    """Incremental Cartesian teleop: SpaceMouse -> rm_force_position_move_pose."""

    def __init__(self, ip: str, port: int):
        super().__init__(
            serial_port=cfg.SERIAL_PORT,
            baud_rate=cfg.SERIAL_BAUD_RATE,
            timeout=cfg.SERIAL_TIMEOUT,
        )

        self.ip = ip
        self.port = port

        self.mouse = None
        self.arm = None

        self.target_pose = None          # [x, y, z, rx, ry, rz] (m / rad)
        self._smoothed = np.zeros(6)     # EMA state
        self._consecutive_fails = 0
        self._gripper_pos = float(cfg.GRIPPER_OPEN_POS)  # 当前夹爪位置 (0~1000)
        self._relay_ready = False
        self._loop_idx = 0
        self._force_payload_warned = False

    # ------ setup / teardown ------

    def setup(self):
        # 1. SpaceMouse
        self.mouse = SpaceMouseReader()
        self.mouse.open()
        self.mouse.start()

        # 2. Robot arm + gripper
        self.arm = RM75BInterface(self.ip, self.port, enable_gripper=(cfg.GRIPPER_MODE != "Switching"))
        self.arm.arm.rm_start_force_position_move()
        if not hasattr(self.arm.arm, "rm_force_position_move_pose"):
            raise RuntimeError("SDK missing rm_force_position_move_pose, please upgrade robot SDK.")

        if cfg.GRIPPER_MODE == "Switching":
            try:
                self.connect()
                self._relay_ready = True
            except Exception as e:
                self._relay_ready = False
                print(f"[WARN] Relay connect failed: {e}")

        # 3. Read current pose as starting point
        ret, state = self.arm.arm.rm_get_current_arm_state()
        if ret != 0:
            raise RuntimeError(f"Failed to read arm state (ret={ret})")
        # state["pose"] = [x, y, z, rx, ry, rz] (m / rad)
        self.target_pose = np.array(state["pose"], dtype=float)
        print(f"Start pose: {np.round(self.target_pose, 4).tolist()}")

    def teardown(self, slow_stop: bool = True):
        """Clean shutdown: stop arm, open gripper, disconnect."""
        if self.arm is not None:
            try:
                self.arm.arm.rm_stop_force_position_move()
            except Exception:
                pass
            if slow_stop:
                self.arm.arm.rm_set_arm_slow_stop()
            if cfg.GRIPPER_MODE != "Switching":
                self.arm.set_gripper_position(cfg.GRIPPER_OPEN_POS)
                time.sleep(0.3)
            self.arm.close()
        if cfg.GRIPPER_MODE == "Switching":
            try:
                self.disconnect()
            except Exception as e:
                print(f"[WARN] Relay disconnect failed: {e}")
        if self.mouse is not None:
            self.mouse.stop()
        print("Shutdown complete.")

    # ------ control helpers ------

    def _compute_delta(self, raw_axes):
        """Apply axis mapping, sign flip, EMA smcoothing, scaling, and clamping."""
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
        elif cfg.GRIPPER_MODE == "incremental":
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
        elif cfg.GRIPPER_MODE == "Switching":
            # if not self._relay_ready:
            #     return
            for bnum, pressed in self.mouse.pop_button_events():
                if not pressed:
                    continue
                try:
                    if bnum == 0:
                        self.open_relay(cfg.RELAY_CHANNEL)
                        print("Relay CLOSE (magnet on)")
                    elif bnum == 1:
                        self.close_relay(cfg.RELAY_CHANNEL)
                        print("Relay OPEN (magnet off)")
                except Exception as e:
                    print(f"[WARN] Relay command failed: {e}")

    def _read_force_wrench(self):
        """Read and return [Fx, Fy, Fz, Mx, My, Mz]. Returns None on failure."""
        try:
            result = self.arm.arm.rm_get_force_data()
        except Exception as e:
            print(f"[WARN] rm_get_force_data failed: {e}")
            return None

        ret = 0
        payload = result
        if isinstance(result, tuple) and len(result) == 2 and isinstance(result[0], (int, np.integer)):
            ret, payload = result
        if ret != 0:
            print(f"[WARN] rm_get_force_data returned {ret}")
            return None

        if isinstance(payload, dict):
            if all(k in payload for k in ("Fx", "Fy", "Fz", "Mx", "My", "Mz")):
                return np.array([
                    payload["Fx"], payload["Fy"], payload["Fz"],
                    payload["Mx"], payload["My"], payload["Mz"],
                ], dtype=float)

            # RM SDK (v1.1.4+) returns rm_force_data_t as dict with these keys.
            for key in ("zero_force_data", "work_zero_force_data", "tool_zero_force_data", "force_data", "force", "data"):
                if key in payload and payload[key] is not None:
                    payload = payload[key]
                    break
            else:
                if not self._force_payload_warned:
                    self._force_payload_warned = True
                    print(f"[WARN] Unknown force payload keys: {list(payload.keys())}")
                payload = None
        if payload is None:
            return None

        try:
            force6 = np.array(payload[:6], dtype=float)
            if force6.shape[0] < 6:
                print(f"[WARN] Invalid force payload length: {force6.shape[0]}")
                return None
        except Exception as e:
            print(f"[WARN] Force payload parse failed: {e}")
            return None

        return force6

    # ------ main loop ------

    def run(self):
        """Force-position streaming loop."""
        dt = 1.0 / cfg.CONTROL_RATE_HZ
        axis_names = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
        dir_idx = int(cfg.FORCE_TASK_DIR)
        if not (0 <= dir_idx < 6):
            raise ValueError(f"Invalid FORCE_TASK_DIR={dir_idx}, expected 0~5")

        print(
            f"Running at {cfg.CONTROL_RATE_HZ} Hz with rm_force_position_move_pose.\n"
            f"force axis={axis_names[dir_idx]}, target={cfg.FORCE_TARGET:.3f}, "
            f"threshold=+/-{cfg.FORCE_THRESHOLD:.3f}, follow={bool(cfg.FORCE_TASK_FOLLOW)}\n"
            "Ctrl+C to stop.\n"
        )

        while True:
            t0 = time.monotonic()
            self._loop_idx += 1

            # 1. Read SpaceMouse
            raw = self.mouse.get_axes()


            # 2. Compute incremental delta
            delta = self._compute_delta(raw)

            # 3. Accumulate target pose
            self.target_pose += delta
            print("1",self.target_pose)
            # 4. Workspace clamp
            self._clamp_workspace()

            # 5. Send force-position command to robot
            pose_cmd = self.target_pose.tolist()
            ret = self.arm.arm.rm_force_position_move_pose(
                pose_cmd,
                int(cfg.FORCE_TASK_SENSOR),
                int(cfg.FORCE_TASK_MODE),
                dir_idx,
                float(cfg.FORCE_TARGET),
                bool(cfg.FORCE_TASK_FOLLOW),
            )
            if ret != 0:
                self._consecutive_fails += 1
                if self._consecutive_fails >= int(cfg.MAX_CONSECUTIVE_FORCE_CMD_FAILS):
                    print(
                        f"[ERROR] {self._consecutive_fails} consecutive "
                        "rm_force_position_move_pose failures — emergency stop!"
                    )
                    self.arm.arm.rm_set_arm_stop()
                    break
            else:
                self._consecutive_fails = 0

            # 6. Read force feedback + real-time debug info
            force6 = self._read_force_wrench()
            measured = None
            reached = "N/A"
            if force6 is not None:
                measured = float(force6[dir_idx])
                reached = "Y" if abs(measured - float(cfg.FORCE_TARGET)) <= float(cfg.FORCE_THRESHOLD) else "N"

            should_print = (
                self._loop_idx % int(cfg.FORCE_DEBUG_PRINT_EVERY) == 0
                or ret != 0
            )
            if should_print:
                if measured is None:
                    print(
                        "[FORCE] "
                        f"ret={ret} fails={self._consecutive_fails} "
                        f"axis={axis_names[dir_idx]} target={float(cfg.FORCE_TARGET):+7.3f} "
                        "measured=N/A reached=N/A "
                        f"pose=[{pose_cmd[0]:+6.3f}, {pose_cmd[1]:+6.3f}, {pose_cmd[2]:+6.3f}, "
                        f"{pose_cmd[3]:+6.3f}, {pose_cmd[4]:+6.3f}, {pose_cmd[5]:+6.3f}]",
                        end="\r",
                        flush=True,
                    )
                else:
                    print(
                        "[FORCE] "
                        f"ret={ret} fails={self._consecutive_fails} "
                        f"axis={axis_names[dir_idx]} target={float(cfg.FORCE_TARGET):+7.3f} "
                        f"measured={measured:+7.3f} th={float(cfg.FORCE_THRESHOLD):.3f} reached={reached} "
                        f"pose=[{pose_cmd[0]:+6.3f}, {pose_cmd[1]:+6.3f}, {pose_cmd[2]:+6.3f}, "
                        f"{pose_cmd[3]:+6.3f}, {pose_cmd[4]:+6.3f}, {pose_cmd[5]:+6.3f}]",
                        end="\r",
                        flush=True,
                    )

            # 7. Gripper buttons
            self._handle_buttons()

            # 8. Timing
            elapsed = time.monotonic() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


# ---------- entry point ----------

def main():
    parser = argparse.ArgumentParser(description="SpaceMouse teleop for RM75B")
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
