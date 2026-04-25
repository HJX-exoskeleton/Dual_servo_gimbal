import time

import sys
import os
# ================= 动态环境变量注入 =================
# 获取当前脚本所在目录 (.../Python/Servo_control)
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取项目根目录 (.../Python)
project_root = os.path.dirname(current_dir)
# 精准定位 SDK 所在的绝对路径 (.../Python/STservo_sdk)
sdk_path = os.path.join(project_root, 'STservo_sdk')
# 将【项目根目录】和【SDK 内部目录】都强行加入环境变量！
if project_root not in sys.path:
    sys.path.append(project_root)
if sdk_path not in sys.path:
    sys.path.append(sdk_path) # <--- 绝杀！加上这句，官方底层库想怎么互相调用都可以了

from STservo_sdk import *  # 微雪舵机库*


# ================= 1. 参数配置 =================
BAUDRATE = 115200
# DEVICENAME = 'COM6'
DEVICENAME = '/dev/ttyUSB0'
ID_PAN = 1
ID_TILT = 2

# 预设的中位角度
PAN_CENTER_DEG = 180.0
TILT_CENTER_DEG = 180.0


def angle_to_pos(angle):
    return int((angle / 360.0) * 4095.0)


# ================= 2. 初始化硬件 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

print("正在唤醒 ESP32...")
time.sleep(1)


# ================= 3. 执行复位逻辑 =================
def home_servos():
    print("\n" + "=" * 40)
    print("🏠 正在执行云台复位程序...")
    print("=" * 40)

    # 1. 确保力矩开启，准备运动
    scs.unLockEprom(ID_PAN)
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 1)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 1)

    # 2. 计算目标位置
    tgt_pan = angle_to_pos(PAN_CENTER_DEG)
    tgt_tilt = angle_to_pos(TILT_CENTER_DEG)

    # 3. 发送复位指令 (使用较低的速度 1500 和平滑的加速度 40)
    # 这一步必须先清除之前的购物车缓存，防止指令冲突
    scs.groupSyncWrite.clearParam()
    scs.SyncWritePosEx(ID_PAN, tgt_pan, 1500, 40)
    scs.SyncWritePosEx(ID_TILT, tgt_tilt, 1500, 40)
    scs.groupSyncWrite.txPacket()

    print(f"指令已下发 -> Pan: {PAN_CENTER_DEG}°, Tilt: {TILT_CENTER_DEG}°")

    # 4. 轮询检查是否到达目标位置附近
    while True:
        p1, _, _, _ = scs.ReadPosSpeed(ID_PAN)
        p2, _, _, _ = scs.ReadPosSpeed(ID_TILT)

        # 计算当前位置与目标的偏差
        diff1 = abs(p1 - tgt_pan)
        diff2 = abs(p2 - tgt_tilt)

        print(f"复位进度 -> ID1偏差: {diff1} | ID2偏差: {diff2}", end='\r')

        # 如果两个舵机的偏差都小于 10 个单位 (约 0.8度)，认为已到位
        if diff1 < 10 and diff2 < 10:
            print("\n✅ 云台已回到初始化中位。")
            break
        time.sleep(0.1)

    # 5. 任务完成，释放力矩（可选：如果需要云台保持姿态则不执行这一步）
    print("进入待机状态：释放力矩。")
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)


# ================= 4. 运行 =================
try:
    home_servos()
finally:
    portHandler.closePort()
    print("程序结束。")
