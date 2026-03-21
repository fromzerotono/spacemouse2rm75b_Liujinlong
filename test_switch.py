from class_switch import USBRelayController 
import time 

# 测试1
m = USBRelayController()
m.connect()
m.open_relay(1)  # 打开第1通道
time.sleep(3)
m.close_relay(1)  # 关闭第1通道
time.sleep(3)
m.open_relay(1)  # 打开第1通道
time.sleep(3)
m.close_relay(1)  # 关闭第1通道
m.disconnect()   # 断开连接
print("Relay CLOSE (magnet on)")

# # 测试2

# m = USBRelayController()
# m.connect()
# time.sleep(1)  # 等待1秒钟
# m.close_relay(1)  # 关闭第1通道
# time.sleep(3)
# m.disconnect()   # 断开连接
# print("Relay CLOSE (magnet on)")

