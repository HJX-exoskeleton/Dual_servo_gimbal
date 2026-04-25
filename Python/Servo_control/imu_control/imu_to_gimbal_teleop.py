import time
import math
import sys
import os
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ================= 动态环境变量注入 =================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
project_root = os.path.dirname(parent_dir)
sdk_path = os.path.join(project_root, 'STservo_sdk')

if project_root not in sys.path: sys.path.append(project_root)
if sdk_path not in sys.path: sys.path.append(sdk_path)

# 导入底层库
from STservo_sdk import *
from YbImuLib import YbImuSerial

# ================= 1. 硬件配置 =================
IMU_PORT = '/dev/ttyUSB1'
SERVO_PORT = '/dev/ttyUSB0'
SERVO_BAUD = 115200

# ================= 1. 硬件配置 =================
ID_PAN = 1
ID_TILT = 2
SERVO_MID = 2048
DEG_TO_SERVO_UNIT = 4095.0 / 360.0
SERVO_UNIT_TO_DEG = 360.0 / 4095.0

# --- ⚠️ 核心手感调校参数 ---
PAN_DIR = -1  # 修正了云台水平方向反转
TILT_DIR = 1
PAN_SCALE = 1
TILT_SCALE = 1.0
EMA_ALPHA = 0.5  # 建议尝试 0.4 到 0.6 之间的值

# ================= 新增：机械限位换算 =================
def angle_to_pos(angle_deg):
    """将物理度数 (0-360) 转换为舵机内部单位 (0-4095)"""
    return int(angle_deg * DEG_TO_SERVO_UNIT)

# 严格按照物理极限设置安全死区
PAN_SAFE_MIN = angle_to_pos(90)   # 对应位置 1023
PAN_SAFE_MAX = angle_to_pos(270)  # 对应位置 3071
TILT_SAFE_MIN = angle_to_pos(85)  # 对应位置 966
TILT_SAFE_MAX = angle_to_pos(225) # 对应位置 2559

# ================= 2. 数据缓存 (可视化) =================
MAX_POINTS = 50
t_data = deque(maxlen=MAX_POINTS)
# IMU 目标角度 (经过归零和缩放后的基准指令)
imu_pan_tgt = deque(maxlen=MAX_POINTS)
imu_tilt_tgt = deque(maxlen=MAX_POINTS)
# 舵机真实角度反馈
servo_pan_real = deque(maxlen=MAX_POINTS)
servo_tilt_real = deque(maxlen=MAX_POINTS)

is_running = True


# ================= 3. 辅助函数 =================
def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))


def normalize_angle(angle):
    """将角度强制归一化到 -180 到 180 之间，防止越界抽搐"""
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle


# ================= 4. 控制线程 =================
def control_loop(imu, scs):
    global is_running

    # --- A. 传感器动态归零 (Calibration) ---
    print("\n" + "=" * 50)
    print("⚠️  请将 IMU 放置在您认为的【正前方初始位置】并保持静止！")
    print("正在进行零点校准 (Tare)...")
    for i in range(3, 0, -1):
        print(f" {i} ...")
        time.sleep(1)

    # 读取初始姿态作为基准零点
    base_roll, base_pitch, base_yaw = imu.get_imu_attitude_data(ToAngle=True)
    if base_pitch is None or base_yaw is None:
        print("❌ 无法读取 IMU 数据，程序终止。")
        is_running = False
        return

    print(f"✅ 校准完成！初始基准: Yaw={base_yaw:.1f}°, Pitch={base_pitch:.1f}°")
    print("🚀 开始动态体感追踪 (请观察弹出的波形图)...")
    print("=" * 50 + "\n")

    # 滤波器状态初始化为中位
    filtered_pan_pos = SERVO_MID
    filtered_tilt_pos = SERVO_MID
    frame_idx = 0

    while is_running:
        loop_start = time.time()

        # 1. 读取 IMU 当前姿态
        roll, pitch, yaw = imu.get_imu_attitude_data(ToAngle=True)

        # 2. 读取舵机真实位置 (用于可视化)
        p1_raw, _, res1, _ = scs.ReadPosSpeed(ID_PAN)
        p2_raw, _, res2, _ = scs.ReadPosSpeed(ID_TILT)

        if pitch is not None and yaw is not None and res1 == COMM_SUCCESS:
            # 3. 计算相对变化量 (Delta) 并做归一化防跳变
            delta_yaw = normalize_angle(yaw - base_yaw)
            delta_pitch = normalize_angle(pitch - base_pitch)

            # 4. 映射到舵机单位 (加入方向和缩放比例 Scale)
            raw_pan_pos = SERVO_MID + (delta_yaw * DEG_TO_SERVO_UNIT * PAN_DIR * PAN_SCALE)
            raw_tilt_pos = SERVO_MID + (delta_pitch * DEG_TO_SERVO_UNIT * TILT_DIR * TILT_SCALE)

            # 5. 安全限位
            safe_pan = clamp(raw_pan_pos, PAN_SAFE_MIN, PAN_SAFE_MAX)
            safe_tilt = clamp(raw_tilt_pos, TILT_SAFE_MIN, TILT_SAFE_MAX)

            # 6. EMA 防抖滤波
            filtered_pan_pos = EMA_ALPHA * safe_pan + (1 - EMA_ALPHA) * filtered_pan_pos
            filtered_tilt_pos = EMA_ALPHA * safe_tilt + (1 - EMA_ALPHA) * filtered_tilt_pos

            # 7. 下发指令
            cmd_pan = int(filtered_pan_pos)
            cmd_tilt = int(filtered_tilt_pos)
            scs.groupSyncWrite.clearParam()
            scs.SyncWritePosEx(ID_PAN, cmd_pan, 0, 0)
            scs.SyncWritePosEx(ID_TILT, cmd_tilt, 0, 0)
            scs.groupSyncWrite.txPacket()

            # 8. 可视化数据收集 (统一换算回度数进行对比)
            # IMU 的目标度数
            target_pan_deg = (filtered_pan_pos - SERVO_MID) * SERVO_UNIT_TO_DEG
            target_tilt_deg = (filtered_tilt_pos - SERVO_MID) * SERVO_UNIT_TO_DEG
            # 舵机的真实反馈度数
            real_pan_deg = (p1_raw - SERVO_MID) * SERVO_UNIT_TO_DEG
            real_tilt_deg = (p2_raw - SERVO_MID) * SERVO_UNIT_TO_DEG

            frame_idx += 1
            t_data.append(frame_idx)
            imu_pan_tgt.append(target_pan_deg)
            servo_pan_real.append(real_pan_deg)
            imu_tilt_tgt.append(target_tilt_deg)
            servo_tilt_real.append(real_tilt_deg)

            print(f"Delta_Y:{delta_yaw:5.1f}° | Tgt P:{cmd_pan} | Real P:{p1_raw}  ", end='\r')

        # 锁定约 50Hz 更新率 0.02
        # 锁定约 100Hz 更新率 (让指令下发更密集) 0.01
        elapsed = time.time() - loop_start
        if elapsed < 0.01:
            time.sleep(0.01 - elapsed)


# ================= 5. 主程序与 UI =================
def main():
    global is_running
    print("系统初始化中...")

    # 硬件初始化
    imu = YbImuSerial(IMU_PORT, debug=False)
    imu.create_receive_threading()

    portHandler = PortHandler(SERVO_PORT)
    scs = sts(portHandler)
    if not portHandler.openPort() or not portHandler.setBaudRate(SERVO_BAUD):
        print("❌ 舵机串口打开失败！")
        return
    time.sleep(2.0)

    # 激活云台并回正
    scs.unLockEprom(ID_PAN)
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 1)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 1)
    scs.SyncWritePosEx(ID_PAN, SERVO_MID, 1500, 50)
    scs.SyncWritePosEx(ID_TILT, SERVO_MID, 1500, 50)
    scs.groupSyncWrite.txPacket()
    time.sleep(2.0)  # 等待回正

    # 开启控制线程
    t = threading.Thread(target=control_loop, args=(imu, scs))
    t.daemon = True
    t.start()

    # --- UI 绘图配置 ---
    plt.style.use('dark_background')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('IMU 体感云台实时遥测')

    # Pan 轴
    line_p_tgt, = ax1.plot([], [], 'cyan', linestyle='--', label='IMU Target (Filtered)', alpha=0.7)
    line_p_real, = ax1.plot([], [], 'cyan', linestyle='-', label='Servo Real Pos', linewidth=2)
    ax1.set_ylim(-180, 180)
    ax1.set_ylabel('Pan Angle (Deg)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Tilt 轴
    line_t_tgt, = ax2.plot([], [], 'magenta', linestyle='--', label='IMU Target (Filtered)', alpha=0.7)
    line_t_real, = ax2.plot([], [], 'magenta', linestyle='-', label='Servo Real Pos', linewidth=2)
    ax2.set_ylim(-180, 180)
    ax2.set_ylabel('Tilt Angle (Deg)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    def update(frame):
        if len(t_data) < 2: return line_p_tgt, line_p_real, line_t_tgt, line_t_real

        line_p_tgt.set_data(t_data, imu_pan_tgt)
        line_p_real.set_data(t_data, servo_pan_real)
        line_t_tgt.set_data(t_data, imu_tilt_tgt)
        line_t_real.set_data(t_data, servo_tilt_real)

        ax1.set_xlim(t_data[0], t_data[-1] + 5)
        return line_p_tgt, line_p_real, line_t_tgt, line_t_real

    ani = FuncAnimation(fig, update, interval=40, blit=True)
    plt.tight_layout()

    try:
        plt.show()  # 阻塞主线程显示图表
    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        print("\n正在安全下电...")
        scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
        scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)
        portHandler.closePort()
        print("✨ 设备已安全释放。")


if __name__ == '__main__':
    main()