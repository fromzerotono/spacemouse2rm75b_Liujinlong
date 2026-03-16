"""
Velocity-mode motion recorder for RM75B.

What it does:
- Reads SpaceMouse axes
- Streams Cartesian velocity via rm_movev_canfd
- Logs command velocity + target-integrated pose + actual pose to CSV
- Auto-generates a PNG summary figure after saving

Usage:
  python3 record_motionteststop_velocity.py --ip 192.168.5.105
  python3 record_motionteststop_velocity.py --out outputs/velocity_run.csv --hz 100
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np

import config as cfg
from rm75b import RM75BInterface
from spacemouse_input import SpaceMouseReader


class ArmPoller(threading.Thread):
    def __init__(self, arm_interface, hz=200):
        super().__init__(daemon=True)
        self._arm = arm_interface
        self._interval = 1.0 / hz
        self._lock = threading.Lock()
        self._pose = None
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            ret, state = self._arm.arm.rm_get_current_arm_state()
            if ret == 0:
                with self._lock:
                    self._pose = np.array(state["pose"], dtype=float)
            time.sleep(self._interval)

    def get_pose(self):
        with self._lock:
            return self._pose.copy() if self._pose is not None else None

    def stop(self):
        self._stop.set()


def _ensure_output_path(path):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)


def _build_velocity(raw_axes, smoothed):
    mapped = np.array([raw_axes[cfg.AXIS_MAP[i]] * cfg.AXIS_SIGNS[i] for i in range(6)], dtype=float)
    smoothed = cfg.EMA_ALPHA * mapped + (1.0 - cfg.EMA_ALPHA) * smoothed

    vel = np.zeros(6)
    vel[:3] = smoothed[:3] * cfg.TRANSLATION_VEL_SCALE
    vel[3:] = smoothed[3:] * cfg.ROTATION_VEL_SCALE

    vel[:3] = np.clip(vel[:3], -cfg.MAX_LINEAR_VEL, cfg.MAX_LINEAR_VEL)
    vel[3:] = np.clip(vel[3:], -cfg.MAX_ANGULAR_VEL, cfg.MAX_ANGULAR_VEL)

    vel *= np.array(cfg.AXIS_ENABLE, dtype=float)
    return vel, smoothed


def _clamp_workspace(target_pose):
    for i in range(3):
        target_pose[i] = np.clip(target_pose[i], cfg.WORKSPACE_MIN[i], cfg.WORKSPACE_MAX[i])


def _save_csv(path, rows):
    _ensure_output_path(path)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "t",
            "inp_mag",
            "cmd_vx", "cmd_vy", "cmd_vz",
            "cmd_wx", "cmd_wy", "cmd_wz",
            "tgt_x", "tgt_y", "tgt_z",
            "tgt_rx", "tgt_ry", "tgt_rz",
            "act_x", "act_y", "act_z",
            "act_rx", "act_ry", "act_rz",
        ])
        writer.writerows(rows)


def _auto_plot(csv_path):
    cmd = [sys.executable, "plot_motion.py", csv_path, "--no-show"]
    try:
        subprocess.run(cmd, check=True)
    except Exception as e:
        print(f"[WARN] Failed to auto-plot: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=cfg.ROBOT_IP)
    parser.add_argument("--port", type=int, default=cfg.ROBOT_PORT)
    parser.add_argument("--hz", type=int, default=100, help="Control loop rate in Hz")
    parser.add_argument(
        "--out",
        default=os.path.join(cfg.OUTPUT_DIR, "motion_velocity.csv"),
        help="Output CSV path",
    )
    args = parser.parse_args()

    mouse = SpaceMouseReader()
    poller = None
    arm = None
    records = []

    try:
        mouse.open()
        mouse.start()

        arm = RM75BInterface(args.ip, args.port, enable_gripper=False)

        if not hasattr(arm.arm, "rm_set_movev_canfd_init") or not hasattr(arm.arm, "rm_movev_canfd"):
            raise RuntimeError("SDK does not expose rm_set_movev_canfd_init/rm_movev_canfd")

        dt = 1.0 / float(args.hz)
        init_ret = arm.arm.rm_set_movev_canfd_init(
            int(cfg.MOVEV_AVOID_SINGULARITY),
            int(cfg.MOVEV_FRAME_TYPE),
            float(dt),
        )
        if init_ret != 0:
            raise RuntimeError(f"rm_set_movev_canfd_init failed (ret={init_ret})")

        ret, state = arm.arm.rm_get_current_arm_state()
        if ret != 0:
            raise RuntimeError("Failed to read initial pose")

        target_pose = np.array(state["pose"], dtype=float)
        print(f"Start pose: {np.round(target_pose, 4).tolist()}")
        print(
            f"Velocity mode init: dt={dt:.4f}s, frame_type={cfg.MOVEV_FRAME_TYPE}, "
            f"avoid_singularity={cfg.MOVEV_AVOID_SINGULARITY}"
        )

        poller = ArmPoller(arm, hz=cfg.ARM_STATE_POLL_HZ)
        poller.start()
        time.sleep(0.2)

        t_start = time.perf_counter()
        smoothed = np.zeros(6)
        consecutive_fails = 0
        print(f"Recording velocity mode at {args.hz} Hz ... Press Ctrl+C to stop.\n")

        stopping = {"flag": False}

        def _stop_handler(*_):
            stopping["flag"] = True

        signal.signal(signal.SIGINT, _stop_handler)

        while not stopping["flag"]:
            loop_t0 = time.monotonic()
            now = time.perf_counter() - t_start

            raw = mouse.get_axes()
            inp_mag = max(abs(v) for v in raw)

            if inp_mag < cfg.DEADZONE:
                vel_cmd = np.zeros(6)
            else:
                vel_cmd, smoothed = _build_velocity(raw, smoothed)

            target_pose += vel_cmd * dt
            _clamp_workspace(target_pose)

            ret = arm.arm.rm_movev_canfd(
                vel_cmd.tolist(),
                False,
                int(cfg.MOVEV_TRAJECTORY_MODE),
                int(cfg.MOVEV_RADIO),
            )
            if ret != 0:
                consecutive_fails += 1
                if consecutive_fails >= 10:
                    print("[ERROR] 10 consecutive send failures - emergency stop")
                    arm.arm.rm_set_arm_stop()
                    break
            else:
                consecutive_fails = 0

            actual = poller.get_pose()
            if actual is not None:
                records.append([
                    round(now, 5),
                    inp_mag,
                    *[round(v, 6) for v in vel_cmd],
                    *[round(v, 6) for v in target_pose],
                    *[round(v, 6) for v in actual],
                ])
                err_mm = np.linalg.norm(target_pose[:3] - actual[:3]) * 1000
                speed_mm = np.linalg.norm(vel_cmd[:3]) * 1000
                print(
                    f"\r  t={now:6.2f}s  input={inp_mag:4.0f}  v={speed_mm:6.2f}mm/s  "
                    f"error={err_mm:5.2f}mm  frames={len(records):5d}",
                    end="",
                    flush=True,
                )

            elapsed = time.monotonic() - loop_t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        # Stop motion cleanly by sending zero velocity once before shutdown.
        if arm is not None:
            try:
                arm.arm.rm_movev_canfd([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], False,
                                       int(cfg.MOVEV_TRAJECTORY_MODE), int(cfg.MOVEV_RADIO))
            except Exception:
                pass

    finally:
        print(f"\nStopping. Saving {len(records)} frames to {args.out} ...")
        _save_csv(args.out, records)
        print("Saved CSV.")
        _auto_plot(args.out)

        if poller is not None:
            poller.stop()
        if mouse is not None:
            try:
                mouse.stop()
            except Exception:
                pass
        if arm is not None:
            arm.close()


if __name__ == "__main__":
    main()
