import time
import math
import serial
import h5py
import os
import argparse
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ================= 动态环境变量注入 (解决导入问题) =================
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sdk_path = os.path.join(project_root, 'STservo_sdk')
if project_root not in sys.path:
    sys.path.append(project_root)
if sdk_path not in sys.path:
    sys.path.append(sdk_path)

from DM_CAN import *  # 达妙电机官方库


# ================= 硬件配置与常量 =================
DM_PORT = '/dev/ttyACM0'  # Windows 请改回 'COM9'
DM_BAUD = 921600
RAD_TO_DEG = 180.0 / math.pi

# ================= 数据缓存 =================
MAX_POINTS = 150  # 屏幕上显示的帧数 (1.5秒的视野)
t_data = deque(maxlen=MAX_POINTS)
# Pan 轴 (Motor 1)
pan_target_deg = deque(maxlen=MAX_POINTS)
pan_real_deg = deque(maxlen=MAX_POINTS)
# Tilt 轴 (Motor 2)
tilt_target_deg = deque(maxlen=MAX_POINTS)
tilt_real_deg = deque(maxlen=MAX_POINTS)

is_running = True
playback_finished = False


def main():
    global is_running, playback_finished
    parser = argparse.ArgumentParser(description='带实时可视化的高频数据重播系统')
    parser.add_argument('--task_name', type=str, default='gimbal_tracking', help='任务名称')
    parser.add_argument('--dataset_dir', type=str, default='./data/', help='数据集目录')
    parser.add_argument('--episode_idx', type=int, default=0, help='要重播的回合编号')
    args = parser.parse_args()

    # --- 1. 加载 HDF5 数据 ---
    file_path = os.path.join(args.dataset_dir, args.task_name, f'episode_{args.episode_idx}.hdf5')
    if not os.path.exists(file_path):
        print(f"❌ 找不到数据文件: {file_path}")
        return

    print(f"📂 正在加载录像: {file_path} ...")
    with h5py.File(file_path, 'r') as root:
        action_data = root['/action'][:]  # 这是我们之前存的 [-rad_1, rad_2]

    num_frames = len(action_data)
    print(f"✅ 加载成功！共 {num_frames} 帧。")

    # --- 2. 初始化达妙电机 ---
    print("🔌 正在连接达妙电机...")
    serial_device = serial.Serial(DM_PORT, DM_BAUD, timeout=0.01)
    dm_ctrl = MotorControl(serial_device)
    Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
    Motor2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
    dm_ctrl.addMotor(Motor1)
    dm_ctrl.addMotor(Motor2)

    dm_ctrl.switchControlMode(Motor1, Control_Type.Torque_Pos)
    dm_ctrl.switchControlMode(Motor2, Control_Type.Torque_Pos)
    dm_ctrl.enable(Motor1)
    dm_ctrl.enable(Motor2)

    # --- 3. 柔性归位 ---
    print("\n🛡️ 正在柔性归位至起跑点，请勿触碰云台...")
    start_act1, start_act2 = action_data[0]
    dm_ctrl.control_pos_force(Motor1, start_act1, 1500, 500)
    dm_ctrl.control_pos_force(Motor2, start_act2, 1500, 500)
    time.sleep(2.0)

    for i in range(3, 0, -1):
        print(f"▶️ 回放将在 {i} 秒后开始...")
        time.sleep(1)
    print("🚀 开始回放！(请观察弹出的波形图)")

    # --- 4. 后台 100Hz 回放线程 ---
    def replay_loop():
        global is_running, playback_finished
        start_time = time.time()

        for i in range(num_frames):
            if not is_running:
                break

            loop_start = time.time()
            act_1, act_2 = action_data[i]

            # 高刚性回放指令
            dm_ctrl.control_pos_force(Motor1, act_1, 5000, 500)
            dm_ctrl.control_pos_force(Motor2, act_2, 5000, 500)

            # 强制刷新缓存
            for _ in range(10):
                if hasattr(dm_ctrl, 'receive_reply'):
                    dm_ctrl.receive_reply()

            # 获取真实反馈并转为角度
            real_1 = Motor1.getPosition() * RAD_TO_DEG
            real_2 = Motor2.getPosition() * RAD_TO_DEG
            tgt_1 = act_1 * RAD_TO_DEG
            tgt_2 = act_2 * RAD_TO_DEG

            # 压入绘图数据栈
            t_data.append(i)
            pan_target_deg.append(tgt_1)
            pan_real_deg.append(real_1)
            tilt_target_deg.append(tgt_2)
            tilt_real_deg.append(real_2)

            # 锁频 100Hz
            elapsed = time.time() - loop_start
            if elapsed < 0.01:
                time.sleep(0.01 - elapsed)

        playback_finished = True
        print("\n🏁 回放任务完成！可以关闭绘图窗口以安全退出。")

    t = threading.Thread(target=replay_loop)
    t.daemon = True
    t.start()

    # --- 5. 前台 UI 绘图 ---
    plt.style.use('dark_background')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('达妙电机 100Hz 回放追踪监控')

    # Pan 轴 (Motor 1)
    line_pan_tgt, = ax1.plot([], [], 'cyan', linestyle='--', label='Target (HDF5 Action)', alpha=0.7)
    line_pan_real, = ax1.plot([], [], 'cyan', linestyle='-', label='Real (Motor Pos)', linewidth=2)
    ax1.set_ylim(-180, 180)
    ax1.set_ylabel('Angle (Deg)')
    ax1.set_title('Joint 1 (Pan) Tracking')
    ax1.legend(loc='upper right', fontsize='small')
    ax1.grid(True, alpha=0.2)

    # Tilt 轴 (Motor 2)
    line_tilt_tgt, = ax2.plot([], [], 'magenta', linestyle='--', label='Target (HDF5 Action)', alpha=0.7)
    line_tilt_real, = ax2.plot([], [], 'magenta', linestyle='-', label='Real (Motor Pos)', linewidth=2)
    ax2.set_ylim(-180, 180)
    ax2.set_ylabel('Angle (Deg)')
    ax2.set_xlabel('Frames')
    ax2.set_title('Joint 2 (Tilt) Tracking')
    ax2.legend(loc='upper right', fontsize='small')
    ax2.grid(True, alpha=0.2)

    def update(frame):
        if len(t_data) < 2:
            return line_pan_tgt, line_pan_real, line_tilt_tgt, line_tilt_real

        line_pan_tgt.set_data(t_data, pan_target_deg)
        line_pan_real.set_data(t_data, pan_real_deg)
        line_tilt_tgt.set_data(t_data, tilt_target_deg)
        line_tilt_real.set_data(t_data, tilt_real_deg)

        ax1.set_xlim(t_data[0], t_data[-1] + 5)
        return line_pan_tgt, line_pan_real, line_tilt_tgt, line_tilt_real

    ani = FuncAnimation(fig, update, interval=40, blit=True)
    plt.tight_layout()

    try:
        plt.show()  # 此处会阻塞主线程，直到手动关掉窗口
    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        print("\n正在安全下电...")
        dm_ctrl.disable(Motor1)
        dm_ctrl.disable(Motor2)
        serial_device.close()
        print("✨ 设备已安全释放。")


if __name__ == '__main__':
    main()

# python replay_episodes.py --task_name gimbal_tracking --dataset_dir /media/hjx/PSSD/hjx_ws/Dual_servo_gimbal/data/ --episode_idx 0
