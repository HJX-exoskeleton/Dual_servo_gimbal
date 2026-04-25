import time
import math
import serial
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from DM_CAN import *   # 达妙电机官方库

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

# ================= 1. 硬件配置 =================
# DM_PORT = 'COM9'
DM_PORT = '/dev/ttyACM0'
DM_BAUD = 921600
# SERVO_PORT = 'COM6'
SERVO_PORT = '/dev/ttyUSB0'
SERVO_BAUDRATE = 115200

ID_PAN = 1
ID_TILT = 2
SERVO_MID = 2048
UNIT_TO_RAD = (360.0 / 4095.0) * (math.pi / 180.0)
RAD_TO_DEG = 180.0 / math.pi

# ================= 2. 数据缓存 =================
MAX_POINTS = 100
t_data = deque(maxlen=MAX_POINTS)
# 合并存储：Pan轴(1号)主从，Tilt轴(2号)主从
pan_m_deg = deque(maxlen=MAX_POINTS)
pan_s_deg = deque(maxlen=MAX_POINTS)
tilt_m_deg = deque(maxlen=MAX_POINTS)
tilt_s_deg = deque(maxlen=MAX_POINTS)

is_running = True

# ================= 3. 硬件初始化 =================
# --- 达妙电机 ---
serial_device = serial.Serial(DM_PORT, DM_BAUD, timeout=0.01)
dm_ctrl = MotorControl(serial_device)
Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
dm_ctrl.addMotor(Motor1)
dm_ctrl.addMotor(Motor2)

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

# --- 飞特舵机 ---
portHandler = PortHandler(SERVO_PORT)
scs = sts(portHandler)
portHandler.openPort()
portHandler.setBaudRate(SERVO_BAUDRATE)

print("系统初始化中...")
time.sleep(1)
scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)


# ================= 4. 修正后的控制线程 =================
def control_loop():
    global is_running
    frame_idx = 0

    while is_running:
        loop_start = time.time()

        # 1. 读取舵机位置
        p1_raw, _, res1, _ = scs.ReadPosSpeed(ID_PAN)  # 舵机1
        p2_raw, _, res2, _ = scs.ReadPosSpeed(ID_TILT)  # 舵机2

        if res1 == COMM_SUCCESS and res2 == COMM_SUCCESS:
            # 2. 映射逻辑修正：
            # 舵机1的数据 对应 电机1 (rad_1)
            # 舵机2的数据 对应 电机2 (rad_2)
            rad_1 = (p1_raw - SERVO_MID) * UNIT_TO_RAD
            rad_2 = (p2_raw - SERVO_MID) * UNIT_TO_RAD

            # 3. 下发指令 (确保 ID 对应正确)
            # dm_ctrl.control_Pos_Vel(Motor1, rad_1, 500)  # 控制电机1
            # dm_ctrl.control_Pos_Vel(Motor2, rad_2, 500)  # 控制电机2
            dm_ctrl.control_pos_force(Motor1, -rad_1, 5000, 500)  # 控制电机1
            dm_ctrl.control_pos_force(Motor2, rad_2, 5000, 500)  # 控制电机2

            # 防止 CAN 堵塞的代码
            for _ in range(10):
                if hasattr(dm_ctrl, 'receive_reply'):
                    dm_ctrl.receive_reply()

            # 获取解析后的角度 (度)
            s1_real_deg = Motor1.getPosition() * RAD_TO_DEG
            s2_real_deg = Motor2.getPosition() * RAD_TO_DEG

            m1_real_deg = -1 * (p1_raw - SERVO_MID) * (360.0 / 4095.0)
            m2_real_deg = (p2_raw - SERVO_MID) * (360.0 / 4095.0)

            # 数据入队
            frame_idx += 1
            t_data.append(frame_idx)
            pan_m_deg.append(m1_real_deg)
            pan_s_deg.append(s1_real_deg)
            tilt_m_deg.append(m2_real_deg)
            tilt_s_deg.append(s2_real_deg)

        time.sleep(0.01)


t = threading.Thread(target=control_loop)
t.daemon = True
t.start()

# ================= 5. 合并绘图逻辑 =================
plt.style.use('dark_background')
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
fig.canvas.manager.set_window_title('主从同步：[ID1 -> ID1] & [ID2 -> ID2]')

# --- 子图 1: Pan (ID 1) ---
line_pan_m, = ax1.plot([], [], 'cyan', linestyle='--', label='Master Servo 1', alpha=0.7)
line_pan_s, = ax1.plot([], [], 'cyan', linestyle='-', label='Slave Motor 1', linewidth=2)
ax1.set_ylim(-180, 180)
ax1.set_ylabel('Angle (Deg)')
ax1.set_title('Joint 1 (Pan) Mapping')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.2)

# --- 子图 2: Tilt (ID 2) ---
line_tilt_m, = ax2.plot([], [], 'magenta', linestyle='--', label='Master Servo 2', alpha=0.7)
line_tilt_s, = ax2.plot([], [], 'magenta', linestyle='-', label='Slave Motor 2', linewidth=2)
ax2.set_ylim(-180, 180)
ax2.set_ylabel('Angle (Deg)')
ax2.set_xlabel('Time Frames')
ax2.set_title('Joint 2 (Tilt) Mapping')
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.2)


def update(frame):
    if len(t_data) < 2: return line_pan_m, line_pan_s, line_tilt_m, line_tilt_s

    line_pan_m.set_data(t_data, pan_m_deg)
    line_pan_s.set_data(t_data, pan_s_deg)
    line_tilt_m.set_data(t_data, tilt_m_deg)
    line_tilt_s.set_data(t_data, tilt_s_deg)

    ax1.set_xlim(t_data[0], t_data[-1] + 5)
    return line_pan_m, line_pan_s, line_tilt_m, line_tilt_s


ani = FuncAnimation(fig, update, interval=40, blit=True)
plt.tight_layout()

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    is_running = False
    dm_ctrl.disable(Motor1)
    dm_ctrl.disable(Motor2)
    serial_device.close()
    portHandler.closePort()