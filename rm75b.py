# -*- coding: utf-8 -*-
"""
用于 RealMan RM75-B 7自由度机械臂和智行 90D 夹爪的硬件接口。

通过 TCP/IP 控制机械臂，通过机械臂末端的 RS485 接口使用 Modbus RTU 协议控制夹爪。
"""

import sys
import os
import math
import time
from typing import List, Union

import numpy as np

# 优先尝试从 pip 安装的库导入，如果失败则回退到本地源码
try:
    from Robotic_Arm.rm_robot_interface import (
        RoboticArm,
        rm_thread_mode_e,
        rm_peripheral_read_write_params_t,
    )
except ImportError:
    # 如果找不到库，则尝试从本地相对路径添加 SDK 源码
    _local_sdk = os.path.join(os.path.dirname(__file__), "../../../../repos/RMDemo_ModbusRTU-ZhiXing/src")
    if os.path.isdir(_local_sdk):
        sys.path.insert(0, _local_sdk)
    from Robotic_Arm.rm_robot_interface import (
        RoboticArm,
        rm_thread_mode_e,
        rm_peripheral_read_write_params_t,
    )

# --- 常量定义 ---

# 单位转换
DEG_TO_RAD = math.pi / 180.0  # 角度转弧度
RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度

# 智行 90D 夹爪的 Modbus RTU 常量
_MODBUS_PORT = 1            # 对应机械臂末端工具RS485接口
_MODBUS_BAUDRATE = 115200   # 波特率
_MODBUS_TIMEOUT = 2         # 超时时间，单位: 100ms
_GRIPPER_DEVICE_ADDR = 1    # 夹爪的 Modbus 从站地址

# 夹爪寄存器地址
_GRIPPER_ENABLE_REG = 256   # 使能寄存器 (写入 1 进行使能)
_GRIPPER_POS_HIGH_REG = 258 # 32位目标位置高16位
_GRIPPER_POS_LOW_REG = 259  # 32位目标位置低16位 (范围 0~1000)
_GRIPPER_SPEED_REG = 260    # 速度寄存器 (范围 0~100%)
_GRIPPER_TRIGGER_REG = 264  # 触发运动寄存器 (写入 1 触发)

# 夹爪控制参数
_GRIPPER_CHANGE_THRESHOLD = 10   # 夹爪位置变化阈值，小于此值不发送指令 (总行程 0~1000)
_GRIPPER_MIN_INTERVAL = 0.05     # 两次 Modbus 写入的最小时间间隔 (秒)


class RM75BInterface:
    """
    RealMan RM75-B 机械臂硬件接口类。
    通过 TCP/IP 与机械臂通信，并通过其末端 RS485 接口上的 Modbus RTU 控制智行 90D 夹爪。
    """

    def __init__(self, ip: str = "192.168.5.46", port: int = 8080, enable_gripper: bool = True):
        """
        初始化机械臂和夹爪接口。

        Args:
            ip (str): 机械臂的 IP 地址。
            port (int): 机械臂的端口号。
            enable_gripper (bool): 是否启用夹爪。
        """
        self.ip = ip
        self.port = port
        self.enable_gripper = enable_gripper
        self._last_gripper_pos = -9999  # 上一次的夹爪位置，初始化为-9999以强制首次写入
        self._last_gripper_time = 0.0   # 上一次写入夹爪指令的时间

        # 初始化机械臂 SDK
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        # 创建机械臂连接句柄
        handle = self.arm.rm_create_robot_arm(ip, port)
        if handle.id == -1:
            raise RuntimeError(f"无法连接到机械臂 {ip}:{port}")
        print(f"成功连接到 RM75-B 机械臂 (地址: {ip}:{port}, 句柄 ID: {handle.id})")

        # 如果启用夹爪，则进行初始化
        if enable_gripper:
            self._init_gripper()

    def _init_gripper(self):
        """配置末端 RS485 的 Modbus RTU 模式并使能智行 90D 夹爪。"""
        # 设置 Modbus 模式
        ret = self.arm.rm_set_modbus_mode(_MODBUS_PORT, _MODBUS_BAUDRATE, _MODBUS_TIMEOUT)
        if ret != 0:
            print(f"[警告] rm_set_modbus_mode 调用失败，返回码: {ret}")
        time.sleep(0.5)  # 等待设置生效

        # 使能夹爪：向寄存器 _GRIPPER_ENABLE_REG (256) 写入 1
        params = rm_peripheral_read_write_params_t(_GRIPPER_DEVICE_ADDR, _GRIPPER_ENABLE_REG, 1)
        ret = self.arm.rm_write_single_register(params, 1)
        if ret != 0:
            print(f"[警告] 夹爪使能失败，返回码: {ret}")
        time.sleep(1.0)  # 等待夹爪使能完成
        print("智行 90D 夹爪已通过 Modbus RTU 使能。")

    def get_joint_positions(self) -> np.ndarray:
        """
        获取当前所有关节的位置。

        Returns:
            np.ndarray: 包含7个关节位置的NumPy数组，单位为弧度。
        """
        ret, degrees = self.arm.rm_get_joint_degree()
        if ret != 0:
            print(f"[警告] rm_get_joint_degree 调用失败，返回码: {ret}")
            return np.zeros(7)
        # API返回8个值，前7个为关节角度
        return np.array(degrees[:7]) * DEG_TO_RAD

    def get_joint_velocities(self) -> np.ndarray:
        """
        获取当前所有关节的速度。
        注意：RM 机械臂的基础 API 不提供关节速度反馈，此函数始终返回零。

        Returns:
            np.ndarray: 包含7个关节速度的NumPy数组，始终为零。
        """
        return np.zeros(7)

    def set_joint_positions(self, positions: Union[List[float], np.ndarray]):
        """
        设置目标关节位置。
        使用 `rm_movej_canfd` 并设置 `follow=False` (低跟随模式)，适用于50Hz的实时控制。

        Args:
            positions (Union[List[float], np.ndarray]): 包含7个目标关节位置的列表或数组，单位为弧度。
        """
        # 将弧度转换为角度
        degrees = [float(p * RAD_TO_DEG) for p in positions]
        ret = self.arm.rm_movej_canfd(degrees, follow=False)
        if ret != 0:
            print(f"[警告] rm_movej_canfd 调用失败，返回码: {ret}")

    def _write_gripper_reg(self, addr: int, value: int):
        """
        向夹爪写入一个16位的 Modbus 寄存器。

        Args:
            addr (int): 寄存器地址。
            value (int): 要写入的16位值。
        """
        params = rm_peripheral_read_write_params_t(_GRIPPER_DEVICE_ADDR, addr, 1)
        self.arm.rm_write_single_register(params, value)

    def set_gripper_position(self, position: float):
        """
        设置夹爪的目标位置。

        此函数会将32位的位置数据拆分为两个16位寄存器写入，然后触发运动。
        为了避免 Modbus 总线拥堵，函数内置了频率限制和变化阈值。

        Args:
            position (float): 目标位置，范围 0~1000 (0=闭合, 1000=完全张开)。
        """
        if not self.enable_gripper:
            return

        # 将输入值约束在 0-1000 范围内并取整
        pos_int = int(max(0, min(1000, round(position))))

        # 如果位置变化小于阈值，则不发送指令
        if abs(pos_int - self._last_gripper_pos) < _GRIPPER_CHANGE_THRESHOLD:
            return

        # 如果距离上次写入时间过短，则不发送指令
        now = time.time()
        if now - self._last_gripper_time < _GRIPPER_MIN_INTERVAL:
            return

        # 将32位位置数据拆分为高16位和低16位，并分别写入寄存器
        self._write_gripper_reg(_GRIPPER_POS_HIGH_REG, (pos_int >> 16) & 0xFFFF)
        self._write_gripper_reg(_GRIPPER_POS_LOW_REG, pos_int & 0xFFFF)
        # 写入触发寄存器以启动运动
        self._write_gripper_reg(_GRIPPER_TRIGGER_REG, 1)

        # 更新上一次的位置和时间
        self._last_gripper_pos = pos_int
        self._last_gripper_time = now

    def go_home(self, joints_deg: List[float] = None):
        """
        以20%的速度阻塞式地移动到原点或指定位置。

        Args:
            joints_deg (List[float], optional): 目标关节角度列表 (单位: 度)。
                                                如果为 None，则移动到全零位置。
        """
        if joints_deg is None:
            joints_deg = [0.0] * 7
        # 调用阻塞式 movej 接口
        ret = self.arm.rm_movej(list(joints_deg), v=20, r=0, connect=0, block=1)
        if ret != 0:
            print(f"[警告] rm_movej (go_home) 调用失败，返回码: {ret}")

    def close(self):
        """关闭 Modbus 端口并断开与机械臂的连接。"""
        if self.enable_gripper:
            try:
                self.arm.rm_close_modbus_mode(_MODBUS_PORT)
            except Exception as e:
                print(f"[警告] 关闭 Modbus 端口时发生错误: {e}")
        try:
            self.arm.rm_delete_robot_arm()
        except Exception as e:
            print(f"[警告] 断开机械臂连接时发生错误: {e}")
        print(f"已断开与 RM75-B 机械臂 ({self.ip}:{self.port}) 的连接。")
