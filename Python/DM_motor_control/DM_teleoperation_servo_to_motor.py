import time
import math
import serial
from DM_CAN import *  # 达妙电机官方库

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

from STservo_sdk import *  # 微雪舵机库


# ================= 1. 硬件硬要求配置 =================
# --- 从端 (Slave): 达妙电机配置 ---
# DM_PORT = 'COM9'
DM_PORT = '/dev/ttyACM0'
DM_BAUD = 921600
Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)

# --- 主端 (Master): 舵机云台配置 ---
# SERVO_PORT = 'COM6'
SERVO_PORT = '/dev/ttyUSB0'
SERVO_BAUDRATE = 115200  # <-- 添加这一行定义
ID_PAN = 1
ID_TILT = 2

# --- 映射参数 ---
SERVO_MID = 2048  # 舵机 180度 中位
# 舵机单位(0-4095)转弧度系数: (360/4095) * (pi/180)
UNIT_TO_RAD = (360.0 / 4095.0) * (math.pi / 180.0)

# ================= 2. 硬件初始化 =================
# 初始化达妙串口与控制器
serial_device = serial.Serial(DM_PORT, DM_BAUD, timeout=0.5)
dm_ctrl = MotorControl(serial_device)
dm_ctrl.addMotor(Motor1)
dm_ctrl.addMotor(Motor2)

# 初始化舵机串口与对象
portHandler = PortHandler(SERVO_PORT)
scs = sts(portHandler)

if not portHandler.openPort() or not portHandler.setBaudRate(SERVO_BAUDRATE):
    print("❌ 舵机串口打开失败")
    quit()

# 等待 ESP32 重启并进入透传
print("正在唤醒系统...")
time.sleep(1)

# --- 模式设定 ---
# 主端：释放力矩，进入“手随”示教模式
scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)

# 从端：切换至位置-速度模式并使能
# dm_ctrl.switchControlMode(Motor1, Control_Type.POS_VEL)
# dm_ctrl.switchControlMode(Motor2, Control_Type.POS_VEL)
# --- 达妙电机初始化修改 ---
# 将 POS_VEL 改为 Torque_Pos
if dm_ctrl.switchControlMode(Motor1, Control_Type.Torque_Pos):
    print("Motor 1 switch to Torque_Pos success")
if dm_ctrl.switchControlMode(Motor2, Control_Type.Torque_Pos):
    print("Motor 2 switch to Torque_Pos success")

dm_ctrl.enable(Motor1)
dm_ctrl.enable(Motor2)

# ================= 3. 实时遥操作循环 =================
print("\n" + "="*50)
print("🎮 实时遥操作 Demo 已启动")
print(f"主端: {SERVO_PORT} (舵机云台) -> 从端: {DM_PORT} (达妙电机)")
print("操作: 手动摆动云台，达妙电机将同步跟随。")
print("="*50 + "\n")

try:
    while True:
        start_time = time.time()

        # 1. 读取主端位置 (Feedback)
        p1_raw, _, res1, _ = scs.ReadPosSpeed(ID_PAN)
        p2_raw, _, res2, _ = scs.ReadPosSpeed(ID_TILT)

        if res1 == COMM_SUCCESS and res2 == COMM_SUCCESS:
            # 2. 关节空间映射 (Joint Space Mapping)
            # 以 180 度为 0 弧度，计算相对偏移
            rad_pan = (p1_raw - SERVO_MID) * UNIT_TO_RAD
            rad_tilt = (p2_raw - SERVO_MID) * UNIT_TO_RAD

            # 3. 下发从端指令 (Command)
            # dm_ctrl.control_Pos_Vel(Motor1, rad_1, 500)  # 控制电机1
            # dm_ctrl.control_Pos_Vel(Motor2, rad_2, 500)  # 控制电机2
            dm_ctrl.control_pos_force(Motor1, -rad_pan, 5000, 500)  # 控制电机1
            dm_ctrl.control_pos_force(Motor2, rad_tilt, 5000, 500)  # 控制电机2

            # print(f"Tracking -> Pan Rad: {-rad_pan:6.2f} | Tilt Rad: {rad_tilt:6.2f}", end='\r')

        # 4. 频率控制：锁定 50Hz (20ms/周期)
        # 过高频率会导致两个串口争抢 CPU 时间片，50Hz 是最稳的
        elapsed = time.time() - start_time
        if elapsed < 0.02:
            time.sleep(0.02 - elapsed)

except KeyboardInterrupt:
    print("\n\n🛑 收到退出信号")

finally:
    # 安全下电流程
    print("正在关闭系统...")
    dm_ctrl.disable(Motor1)
    dm_ctrl.disable(Motor2)
    serial_device.close()
    portHandler.closePort()
    print("✅ 所有设备已安全释放")
