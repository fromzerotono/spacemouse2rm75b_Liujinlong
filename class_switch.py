import serial
import time

class USBRelayController:
    """
    USB 继电器控制器类
    用于与其他工程集成和控制串口继电器
    """
    
    def __init__(self, serial_port="/dev/ttyUSB0", baud_rate=9600, timeout=0.5):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_conn = None

    def connect(self):
        """打开串口连接"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout)
            # 等待串口初始化稳定
            time.sleep(0.1)
        return self.serial_conn

    def disconnect(self):
        """关闭串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

    def send_command(self, cmd_hex_str):
        """发送 16 进制指令字符串"""
        if not self.serial_conn or not self.serial_conn.is_open:
            raise ConnectionError("串口未连接，请先调用 connect() 方法。")
        d = bytes.fromhex(cmd_hex_str)
        self.serial_conn.write(d)
        print("发送命令：{0}".format(cmd_hex_str))

    def set_relay(self, channel=1, on=True):
        """基础控制通道状态：on=True 打开，on=False 关闭"""
        op = 0x01 if on else 0x00
        check = 0xA0 + channel + op
        cmd = "A0 {0:02X} {1:02X} {2:02X}".format(channel, op, check)
        self.send_command(cmd)

    def open_relay(self, channel=1):
        """留给外部的高级接口：打开指定通道的继电器"""
        self.set_relay(channel, on=True)

    def close_relay(self, channel=1):
        """留给外部的高级接口：关闭指定通道的继电器"""
        self.set_relay(channel, on=False)
