import time
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

from STservo_sdk import *  # 微雪舵机库*


# ================= 1. 基础硬件参数配置 =================
BAUDRATE = 115200  # 必须与 ESP32 的透传降速波特率一致
# DEVICENAME = 'COM6'
DEVICENAME = '/dev/ttyUSB0'
ID_1 = 1
ID_2 = 2

SERVO_DIGITAL_RANGE = 4095.0
SERVO_ANGLE_RANGE = 360.0

# ================= 2. 图表数据缓存 (双端队列) =================
MAX_POINTS = 100  # 屏幕上最多显示 100 个数据点
t_data = deque(maxlen=MAX_POINTS)

# 位置缓存
y1_pos = deque(maxlen=MAX_POINTS)
y2_pos = deque(maxlen=MAX_POINTS)

# 速度缓存 (新增)
y1_spd = deque(maxlen=MAX_POINTS)
y2_spd = deque(maxlen=MAX_POINTS)

# ================= 3. 硬件初始化 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("串口打开失败，请检查连线或端口号占用！")
    quit()

print("等待 ESP32 开机...")
time.sleep(2.5)
scs.unLockEprom(ID_1)
scs.write1ByteTxRx(ID_1, STS_TORQUE_ENABLE, 0)  # 释放力矩，方便手动扭动测试
scs.write1ByteTxRx(ID_2, STS_TORQUE_ENABLE, 0)
print("硬件就绪！现在请用手转动舵机...")

# ================= 4. 初始化 Matplotlib 双屏图表 =================
plt.style.use('fast')
# 创建 2 行 1 列的图表，共享 X 轴
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
fig.canvas.manager.set_window_title('微雪总线舵机实时遥测系统 (位置+速度)')

# --- 图表 1: 位置 (Angle) ---
line1_pos, = ax1.plot([], [], label=f'Pos ID:{ID_1}', color='#FF5722', linewidth=2)
line2_pos, = ax1.plot([], [], label=f'Pos ID:{ID_2}', color='#03A9F4', linewidth=2)
ax1.set_ylim(0, 360)
ax1.set_ylabel('Angle (Degree)')
ax1.set_title('Real-time Servo Status')
ax1.legend(loc='upper right')
ax1.grid(True, linestyle='--', alpha=0.6)

# --- 图表 2: 速度 (Speed) ---
# 使用虚线区分速度曲线
line1_spd, = ax2.plot([], [], label=f'Spd ID:{ID_1}', color='#FF5722', linewidth=2, linestyle='--')
line2_spd, = ax2.plot([], [], label=f'Spd ID:{ID_2}', color='#03A9F4', linewidth=2, linestyle='--')
# ST3215 满载原始速度最高大概在 3400 左右，正负代表顺逆时针
ax2.set_ylim(-4000, 4000)
ax2.set_xlabel('Time (frames)')
ax2.set_ylabel('Raw Speed (steps/s)')
ax2.legend(loc='upper right')
ax2.grid(True, linestyle='--', alpha=0.6)

# 添加一条穿过 0 的水平参考线，方便看舵机是否静止
ax2.axhline(0, color='black', linewidth=1, alpha=0.3)

start_time = time.time()
frame_count = 0


# ================= 5. 核心更新函数 =================
def update_plot(frame):
    global frame_count

    # --- 抓取 1 号舵机数据 ---
    pos_1, speed_1, res_1, _ = scs.ReadPosSpeed(ID_1)
    if res_1 == COMM_SUCCESS:
        angle_1 = (pos_1 / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE
        spd_1 = speed_1
    else:
        # 掉线保护：维持上一个值
        angle_1 = y1_pos[-1] if len(y1_pos) > 0 else 0
        spd_1 = y1_spd[-1] if len(y1_spd) > 0 else 0

    # --- 抓取 2 号舵机数据 ---
    pos_2, speed_2, res_2, _ = scs.ReadPosSpeed(ID_2)
    if res_2 == COMM_SUCCESS:
        angle_2 = (pos_2 / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE
        spd_2 = speed_2
    else:
        angle_2 = y2_pos[-1] if len(y2_pos) > 0 else 0
        spd_2 = y2_spd[-1] if len(y2_spd) > 0 else 0

    # 数据压栈
    frame_count += 1
    t_data.append(frame_count)
    y1_pos.append(angle_1)
    y2_pos.append(angle_2)
    y1_spd.append(spd_1)
    y2_spd.append(spd_2)

    # 刷新线条数据
    line1_pos.set_data(t_data, y1_pos)
    line2_pos.set_data(t_data, y2_pos)
    line1_spd.set_data(t_data, y1_spd)
    line2_spd.set_data(t_data, y2_spd)

    # 动态滑动 X 轴 (两个图表都要滑)
    if frame_count > MAX_POINTS:
        ax1.set_xlim(frame_count - MAX_POINTS, frame_count)
        ax2.set_xlim(frame_count - MAX_POINTS, frame_count)
    else:
        ax1.set_xlim(0, MAX_POINTS)
        ax2.set_xlim(0, MAX_POINTS)

    return line1_pos, line2_pos, line1_spd, line2_spd


# ================= 6. 启动动画 =================
try:
    # 50ms 刷新一次
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()  # 自动调整上下图表的间距，防止重叠
    plt.show()

except Exception as e:
    print(f"出现异常: {e}")
finally:
    portHandler.closePort()
    print("绘图窗口已关闭，串口已安全释放。")
