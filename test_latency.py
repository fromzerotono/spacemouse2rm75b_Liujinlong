"""
端到端延迟测试：在完整控制循环中测量两种延迟

  延迟A（开始响应）: SpaceMouse 从静止→有输入  →  臂位姿开始变化
  延迟B（停止响应）: SpaceMouse 从有输入→静止  →  臂位姿停止变化

脚本本身就是一个完整的 teleop 控制循环（与 spacemouse_teleop.py 相同逻辑），
同时挂载状态机来检测触发时刻和臂的响应时刻。

用法：
  python test_latency.py [--ip 192.168.5.105] [--port 8080] [--samples 5]

  运行后随意推动/松开 SpaceMouse，脚本自动计时，积累到 --samples 次后打印统计。
"""

import argparse
import signal
import sys
import time
import numpy as np

import config as cfg
from spacemouse_input import SpaceMouseReader
from rm75b import RM75BInterface

# ---------- 可调参数 ----------
CONTROL_HZ       = 50       # 控制循环频率（与 teleop 保持一致）
POLL_HZ          = 200      # 读取臂状态的频率（独立线程）

INPUT_THRESHOLD  = 80       # SpaceMouse 原始值：超过=有输入，低于=静止
POSE_MOVE_THR_M  = 0.00002    # 位置变化 > 2mm  → 判定"臂开始动"
POSE_STOP_THR_M  = 0.00002   # 位置变化 < 0.5mm → 判定"臂停下了"（连续 N 次）
POSE_STOP_COUNT  = 3        # 连续 N 次低于阈值才算停稳

SETTLE_TIME      = 1.0      # 触发后至少等多久才进入下一次测量 (s)
# ------------------------------

import threading

class ArmPoller(threading.Thread):
    """独立线程高频读取臂位姿，供主循环查询。"""
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
                pose = np.array(state["pose"], dtype=float)
                with self._lock:
                    self._pose = pose
            time.sleep(self._interval)

    def get_pose(self):
        with self._lock:
            return self._pose.copy() if self._pose is not None else None

    def stop(self):
        self._stop.set()


class LatencyTester:
    def __init__(self, ip, port, n_samples):
        self.ip = ip
        self.port = port
        self.n_samples = n_samples

        self.mouse = None
        self.arm = None
        self.poller = None
        self.target_pose = None
        self._smoothed = np.zeros(6)
        self._consecutive_fails = 0

        # 测量结果
        self.results_start = []   # 延迟A：输入开始 → 臂开始动
        self.results_stop  = []   # 延迟B：输入停止 → 臂停下来

        # 状态机
        # IDLE       : 等待输入出现
        # WAIT_MOVE  : 已捕获 T0_start，等待臂开始动
        # ACTIVE     : 臂在动，等待输入消失
        # WAIT_STOP  : 已捕获 T0_stop，等待臂停下
        # SETTLE     : 等待稳定后重新开始
        self._state = "IDLE"
        self._t0 = None
        self._base_pose = None
        self._last_pose = None
        self._stop_count = 0
        self._settle_until = 0.0

    def setup(self):
        self.mouse = SpaceMouseReader()
        self.mouse.open()
        self.mouse.start()

        self.arm = RM75BInterface(self.ip, self.port, enable_gripper=False)

        ret, state = self.arm.arm.rm_get_current_arm_state()
        if ret != 0:
            raise RuntimeError(f"无法读取初始位姿 (ret={ret})")
        self.target_pose = np.array(state["pose"], dtype=float)
        print(f"起始位姿: {np.round(self.target_pose, 4).tolist()}")

        self.poller = ArmPoller(self.arm, hz=POLL_HZ)
        self.poller.start()
        time.sleep(0.2)  # 等 poller 读到第一帧

        # 位移显示：以起始位姿为原点
        self._disp_ref = self.target_pose.copy()
        self._disp_prev = self.target_pose.copy()

    def teardown(self):
        if self.arm:
            self.arm.arm.rm_set_arm_slow_stop()
        if self.poller:
            self.poller.stop()
        if self.mouse:
            self.mouse.stop()
        if self.arm:
            self.arm.close()
        print("关闭完成。")

    # ----- control helpers (与 teleop 相同) -----

    def _compute_delta(self, raw_axes):
        mapped = np.array([raw_axes[cfg.AXIS_MAP[i]] * cfg.AXIS_SIGNS[i] for i in range(6)],
                          dtype=float)
        self._smoothed = cfg.EMA_ALPHA * mapped + (1.0 - cfg.EMA_ALPHA) * self._smoothed
        delta = np.zeros(6)
        delta[:3] = self._smoothed[:3] * cfg.TRANSLATION_SCALE
        delta[3:] = self._smoothed[3:] * cfg.ROTATION_SCALE
        delta[:3] = np.clip(delta[:3], -cfg.MAX_TRANSLATION_PER_CYCLE, cfg.MAX_TRANSLATION_PER_CYCLE)
        delta[3:] = np.clip(delta[3:], -cfg.MAX_ROTATION_PER_CYCLE, cfg.MAX_ROTATION_PER_CYCLE)
        delta *= np.array(cfg.AXIS_ENABLE, dtype=float)
        return delta

    def _clamp_workspace(self):
        for i in range(3):
            self.target_pose[i] = np.clip(self.target_pose[i],
                                          cfg.WORKSPACE_MIN[i], cfg.WORKSPACE_MAX[i])

    # ----- 状态机 -----

    def _is_input_active(self, axes) -> bool:
        return max(abs(v) for v in axes) >= INPUT_THRESHOLD

    def _pose_delta_m(self, a, b) -> float:
        return float(np.linalg.norm(a[:3] - b[:3]))

    def _tick_state_machine(self, axes):
        cur_pose = self.poller.get_pose()
        if cur_pose is None:
            return

        active = self._is_input_active(axes)

        if self._state == "IDLE":
            if time.monotonic() < self._settle_until:
                return
            if active:
                self._t0 = time.perf_counter()
                self._base_pose = cur_pose.copy()
                self._state = "WAIT_MOVE"
                print(f"\n[A] T0 捕获：检测到输入 {axes}")

        elif self._state == "WAIT_MOVE":
            dp = self._pose_delta_m(cur_pose, self._base_pose)
            if dp > POSE_MOVE_THR_M:
                lat = (time.perf_counter() - self._t0) * 1000
                self.results_start.append(lat)
                n = len(self.results_start)
                print(f"[A] T1 捕获：臂位移 {dp*1000:.2f}mm  →  开始延迟 = {lat:.1f} ms  "
                      f"({n}/{self.n_samples})")
                self._state = "ACTIVE"
            elif not active:
                # 输入消失得太快，臂还没动就放开了，直接跳到等停止
                self._t0 = time.perf_counter()
                self._last_pose = cur_pose.copy()
                self._stop_count = 0
                self._state = "WAIT_STOP"
                print(f"\n[B] T0 捕获：输入消失（臂未及响应）")

        elif self._state == "ACTIVE":
            if not active:
                self._t0 = time.perf_counter()
                self._last_pose = cur_pose.copy()
                self._stop_count = 0
                self._state = "WAIT_STOP"
                print(f"\n[B] T0 捕获：输入停止")

        elif self._state == "WAIT_STOP":
            if self._last_pose is not None:
                dp = self._pose_delta_m(cur_pose, self._last_pose)
                if dp < POSE_STOP_THR_M:
                    self._stop_count += 1
                else:
                    self._stop_count = 0
                self._last_pose = cur_pose.copy()

                if self._stop_count >= POSE_STOP_COUNT:
                    lat = (time.perf_counter() - self._t0) * 1000
                    self.results_stop.append(lat)
                    n = len(self.results_stop)
                    print(f"[B] T1 捕获：臂停稳  →  停止延迟 = {lat:.1f} ms  "
                          f"({n}/{self.n_samples})")
                    self._settle_until = time.monotonic() + SETTLE_TIME
                    self._state = "IDLE"

            if active:
                # 臂还没停又有输入，重新跑
                self._state = "ACTIVE"

    # ----- 主循环 -----

    def run(self):
        dt = 1.0 / CONTROL_HZ
        total_needed = self.n_samples
        print(f"\n开始测量，目标 {total_needed} 组。随意推动/松开 SpaceMouse...\n")
        print("  延迟A = 输入出现  → 臂开始动")
        print("  延迟B = 输入消失  → 臂停下来\n")

        while (len(self.results_start) < total_needed or
               len(self.results_stop) < total_needed):
            t0 = time.monotonic()

            raw = self.mouse.get_axes()

            # 控制循环：正常发指令（臂才会动）
            delta = self._compute_delta(raw)
            self.target_pose += delta
            self._clamp_workspace()
            ret = self.arm.arm.rm_movep_canfd(self.target_pose.tolist(), follow=False)
            if ret != 0:
                self._consecutive_fails += 1
                if self._consecutive_fails >= 10:
                    print("[ERROR] 连续发送失败，紧急停止")
                    self.arm.arm.rm_set_arm_stop()
                    break
            else:
                self._consecutive_fails = 0

            # 状态机：检测延迟
            self._tick_state_machine(raw)

            # 实时位移显示
            cur = self.poller.get_pose()
            if cur is not None:
                total_mm = self._pose_delta_m(cur, self._disp_ref) * 1000
                step_mm  = self._pose_delta_m(cur, self._disp_prev) * 1000
                x, y, z  = (cur[:3] - self._disp_ref[:3]) * 1000
                print(f"\r  总位移: {total_mm:6.3f}mm  步进: {step_mm:5.3f}mm  "
                      f"[Δx={x:+6.2f} Δy={y:+6.2f} Δz={z:+6.2f}]mm  "
                      f"状态:{self._state:<10}", end="", flush=True)
                self._disp_prev = cur.copy()

            elapsed = time.monotonic() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def print_summary(self):
        print("\n" + "=" * 50)
        print("测量结果汇总")
        print("=" * 50)

        for label, data in [("A 开始响应（输入→臂动）", self.results_start),
                             ("B 停止响应（松开→臂停）", self.results_stop)]:
            print(f"\n  {label}:")
            if data:
                arr = np.array(data)
                print(f"    样本数: {len(arr)}")
                print(f"    最小值: {arr.min():.1f} ms")
                print(f"    最大值: {arr.max():.1f} ms")
                print(f"    平均值: {arr.mean():.1f} ms")
                print(f"    中位数: {np.median(arr):.1f} ms")
                print(f"    标准差: {arr.std():.1f} ms")
                print(f"    各次:   {[round(r,1) for r in data]}")
            else:
                print("    无有效数据")
        print("=" * 50)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip",      default=cfg.ROBOT_IP)
    parser.add_argument("--port",    type=int, default=cfg.ROBOT_PORT)
    parser.add_argument("--samples", type=int, default=5, help="每种延迟的测量次数")
    args = parser.parse_args()

    tester = LatencyTester(args.ip, args.port, args.samples)

    def _sigint(*_):
        print("\n中断...")
        tester.print_summary()
        tester.teardown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    tester.setup()
    try:
        tester.run()
    finally:
        tester.print_summary()
        tester.teardown()


if __name__ == "__main__":
    main()
