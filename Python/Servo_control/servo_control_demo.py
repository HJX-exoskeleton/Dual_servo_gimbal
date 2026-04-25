import time
import math
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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


# ================= 1. 硬件参数与安全限位 =================
BAUDRATE = 115200
# DEVICENAME = 'COM6'
DEVICENAME = '/dev/ttyUSB0'
ID_PAN = 1
ID_TILT = 2


def angle_to_pos(angle):
    return int((angle / 360.0) * 4095.0)


def pos_to_angle(pos):
    return (pos / 4095.0) * 360.0


def clamp_pos(val, min_val, max_val):
    return max(min_val, min(val, max_val))


PAN_MIN = angle_to_pos(90)
PAN_MAX = angle_to_pos(270)
TILT_MIN = angle_to_pos(85)
TILT_MAX = angle_to_pos(225)

# 云台中位点与摆动幅度
PAN_CENTER_DEG = 180.0
TILT_CENTER_DEG = 180.0
PAN_AMPLITUDE = 60.0
TILT_AMPLITUDE = 30.0
TIME_SCALE = 1.5

# ================= 2. 线程通信与数据缓存 =================
is_running = True  # 全局运行标志
MAX_POINTS = 100  # 屏幕显示的滑动窗口大小

# 数据队列
t_data = deque(maxlen=MAX_POINTS)
pan_tgt_data = deque(maxlen=MAX_POINTS)
pan_real_data = deque(maxlen=MAX_POINTS)
tilt_tgt_data = deque(maxlen=MAX_POINTS)
tilt_real_data = deque(maxlen=MAX_POINTS)

# ================= 3. 硬件初始化 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("串口打开失败！")
    quit()

print("等待 ESP32 开机...")
time.sleep(2.5)

scs.unLockEprom(ID_PAN)
scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 1)
scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 1)

print("正在复位到中心点...")
scs.groupSyncWrite.clearParam()
scs.SyncWritePosEx(ID_PAN, angle_to_pos(PAN_CENTER_DEG), 2000, 50)
scs.SyncWritePosEx(ID_TILT, angle_to_pos(TILT_CENTER_DEG), 2000, 50)
scs.groupSyncWrite.txPacket()
time.sleep(1.0)


# ================= 4. 后台控制与读取线程 =================
def robot_control_thread():
    global is_running
    start_time = time.time()
    frame_count = 0

    print("🤖 控制线程已启动，正在下发高频指令...")

    while is_running:
        t = (time.time() - start_time) * TIME_SCALE
        frame_count += 1

        # 1. 计算目标 (Target)
        target_pan_deg = PAN_CENTER_DEG + PAN_AMPLITUDE * math.sin(t)
        target_tilt_deg = TILT_CENTER_DEG + TILT_AMPLITUDE * math.sin(2 * t)

        tgt_pan_pos = clamp_pos(angle_to_pos(target_pan_deg), PAN_MIN, PAN_MAX)
        tgt_tilt_pos = clamp_pos(angle_to_pos(target_tilt_deg), TILT_MIN, TILT_MAX)

        # 2. 下发指令 (Write)
        scs.groupSyncWrite.clearParam()
        scs.SyncWritePosEx(ID_PAN, tgt_pan_pos, 0, 0)
        scs.SyncWritePosEx(ID_TILT, tgt_tilt_pos, 0, 0)
        scs.groupSyncWrite.txPacket()

        # 3. 立即读取真实位置 (Read)
        # 注意：读和写都在同一个线程，完美避免了串口冲突
        real_pan_pos, _, res_p, _ = scs.ReadPosSpeed(ID_PAN)
        real_tilt_pos, _, res_t, _ = scs.ReadPosSpeed(ID_TILT)

        # 4. 数据压栈 (供前台画图使用)
        t_data.append(frame_count)
        pan_tgt_data.append(target_pan_deg)
        tilt_tgt_data.append(target_tilt_deg)

        # 如果读取成功，转换真实角度；如果失败，暂用目标角度顶替以免断线
        pan_real_data.append(pos_to_angle(real_pan_pos) if res_p == COMM_SUCCESS else target_pan_deg)
        tilt_real_data.append(pos_to_angle(real_tilt_pos) if res_t == COMM_SUCCESS else target_tilt_deg)

        time.sleep(0.02)  # 保持 50Hz 的完美控制频率


# 启动后台线程
thread = threading.Thread(target=robot_control_thread)
thread.daemon = True  # 主程序退出时，线程也会强制结束
thread.start()

# ================= 5. 前台 Matplotlib 绘图 =================
plt.style.use('fast')
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
fig.canvas.manager.set_window_title('云台 8 字形轨迹追踪分析')

# 图 1: 底座 Pan
line_p_tgt, = ax1.plot([], [], 'r--', label='Target Pan', linewidth=1.5, alpha=0.7)
line_p_real, = ax1.plot([], [], 'r-', label='Real Pan', linewidth=2)
ax1.set_ylim(90, 270)
ax1.set_ylabel('Pan Angle (°)')
ax1.legend(loc='upper right')
ax1.grid(True, linestyle=':')

# 图 2: 俯仰 Tilt
line_t_tgt, = ax2.plot([], [], 'b--', label='Target Tilt', linewidth=1.5, alpha=0.7)
line_t_real, = ax2.plot([], [], 'b-', label='Real Tilt', linewidth=2)
ax2.set_ylim(85, 225)
ax2.set_xlabel('Time (frames)')
ax2.set_ylabel('Tilt Angle (°)')
ax2.legend(loc='upper right')
ax2.grid(True, linestyle=':')


def update_plot(frame):
    # 更新四条线的坐标数据
    line_p_tgt.set_data(t_data, pan_tgt_data)
    line_p_real.set_data(t_data, pan_real_data)
    line_t_tgt.set_data(t_data, tilt_tgt_data)
    line_t_real.set_data(t_data, tilt_real_data)

    if len(t_data) > 0:
        current_frame = t_data[-1]
        if current_frame > MAX_POINTS:
            ax1.set_xlim(current_frame - MAX_POINTS, current_frame)
        else:
            ax1.set_xlim(0, MAX_POINTS)

    return line_p_tgt, line_p_real, line_t_tgt, line_t_real


# ================= 6. 启动与清理 =================
try:
    print("📈 绘图界面已加载，关闭绘图窗口即可退出程序。")
    # interval=50 表示每 50 毫秒刷新一次 UI
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()  # 这是一句阻塞代码，窗口不关，程序就一直停在这里

except KeyboardInterrupt:
    pass
finally:
    # 窗口关闭后，通知后台线程停止，并安全释放硬件
    print("\n🛑 正在执行安全退出程序...")
    is_running = False
    thread.join(timeout=1.0)  # 等待后台线程安全结束

    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("程序已完美结束。")
