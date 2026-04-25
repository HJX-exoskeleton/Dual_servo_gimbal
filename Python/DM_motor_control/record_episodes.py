import time
import math
import serial
import h5py
import os
import argparse
from tqdm import tqdm
import numpy as np

# ================= 动态环境变量注入 (解决 SDK 导入问题) =================
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sdk_path = os.path.join(project_root, 'STservo_sdk')
if project_root not in sys.path:
    sys.path.append(project_root)
if sdk_path not in sys.path:
    sys.path.append(sdk_path)

from STservo_sdk import *  # 微雪舵机库
from DM_CAN import *  # 达妙电机官方库


# ================= 1. ALOHA 规范数据收集器 =================
class AlohaDataCollector:
    def __init__(self):
        self.data_dict = {
            '/time': [],
            '/action': [],  # [pan_target, tilt_target] (Master 指令)
            '/observations/qpos': [],  # [pan_real, tilt_real] (Slave 真实位置)
            '/observations/qvel': [],  # [pan_vel, tilt_vel] (Slave 真实速度)
        }

    def append_step(self, t, action, qpos, qvel):
        self.data_dict['/time'].append(t)
        self.data_dict['/action'].append(action)
        self.data_dict['/observations/qpos'].append(qpos)
        self.data_dict['/observations/qvel'].append(qvel)

    def save_to_hdf5(self, dataset_dir, task_name, episode_idx):
        task_dir = os.path.join(dataset_dir, task_name)
        os.makedirs(task_dir, exist_ok=True)
        file_path = os.path.join(task_dir, f'episode_{episode_idx}.hdf5')

        print(f"\n🔄 正在将示教数据写入 {file_path} ...")
        t0 = time.time()

        with h5py.File(file_path, 'w') as root:
            for key, value_list in self.data_dict.items():
                if len(value_list) == 0:
                    continue
                data_np = np.array(value_list, dtype=np.float32)
                root.create_dataset(key, data=data_np, compression='gzip')

        print(f"✅ 保存成功！耗时: {time.time() - t0:.2f}s | 总帧数: {len(self.data_dict['/time'])}")


# ================= 2. 硬件与参数配置 =================
DM_PORT = '/dev/ttyACM0'
DM_BAUD = 921600
SERVO_PORT = '/dev/ttyUSB0'
SERVO_BAUDRATE = 115200

ID_PAN = 1
ID_TILT = 2
SERVO_MID = 2048
UNIT_TO_RAD = (360.0 / 4095.0) * (math.pi / 180.0)


def main():
    parser = argparse.ArgumentParser(description='云台双轴遥操作数据采集 (ALOHA Format)')
    parser.add_argument('--task_name', type=str, default='gimbal_tracking', help='任务名称')
    parser.add_argument('--dataset_dir', type=str, default='./data/', help='数据集目录')
    parser.add_argument('--episode_len', type=int, default=1000, help='录制步数(100Hz下1000步为10秒)')
    parser.add_argument('--episode_idx', type=int, default=0, help='回合编号')
    args = parser.parse_args()

    # --- 硬件初始化 ---
    print("🔌 初始化达妙电机...")
    serial_device = serial.Serial(DM_PORT, DM_BAUD, timeout=0.01)
    dm_ctrl = MotorControl(serial_device)
    Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
    Motor2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)  # 保持隔离的 Master ID
    dm_ctrl.addMotor(Motor1)
    dm_ctrl.addMotor(Motor2)

    if dm_ctrl.switchControlMode(Motor1, Control_Type.Torque_Pos):
        print("✅ Motor 1 Torque_Pos ready")
    if dm_ctrl.switchControlMode(Motor2, Control_Type.Torque_Pos):
        print("✅ Motor 2 Torque_Pos ready")
    dm_ctrl.enable(Motor1)
    dm_ctrl.enable(Motor2)

    print("🔌 初始化飞特舵机...")
    portHandler = PortHandler(SERVO_PORT)
    scs = sts(portHandler)
    portHandler.openPort()
    portHandler.setBaudRate(SERVO_BAUDRATE)
    time.sleep(2.5)

    # 释放主端力矩
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)

    collector = AlohaDataCollector()

    # --- 倒计时准备 ---
    print("\n" + "=" * 50)
    print(f"🎥 准备录制 | 任务: {args.task_name} | 编号: episode_{args.episode_idx} | 时长: {args.episode_len / 100.0}s")
    print("⚠️  请握住舵机云台准备...")
    for i in range(3, 0, -1):
        print(f" {i} ...")
        time.sleep(1)
    print("🔴 开始采集！(Recording...)")

    start_time = time.time()

    try:
        # 使用 tqdm 显示 100Hz 采集进度
        for step in tqdm(range(args.episode_len), desc="Data Collection", unit="step", colour="green"):
            loop_start = time.time()
            current_t = loop_start - start_time

            # 1. 采集主端指令 (Master)
            p1_raw, _, res1, _ = scs.ReadPosSpeed(ID_PAN)
            p2_raw, _, res2, _ = scs.ReadPosSpeed(ID_TILT)

            if res1 == COMM_SUCCESS and res2 == COMM_SUCCESS:
                rad_1 = (p1_raw - SERVO_MID) * UNIT_TO_RAD
                rad_2 = (p2_raw - SERVO_MID) * UNIT_TO_RAD

                # 构建 Action (包含你之前的符号修正)
                action = [-rad_1, rad_2]

                # 2. 下发从端指令 (Slave)
                dm_ctrl.control_pos_force(Motor1, action[0], 5000, 500)
                dm_ctrl.control_pos_force(Motor2, action[1], 5000, 500)

                # 3. 强制清理缓冲区并获取观测值
                for _ in range(10):
                    if hasattr(dm_ctrl, 'receive_reply'):
                        dm_ctrl.receive_reply()

                qpos = [Motor1.getPosition(), Motor2.getPosition()]
                qvel = [Motor1.getVelocity(), Motor2.getVelocity()]

                # 4. 压入数据集
                collector.append_step(current_t, action, qpos, qvel)

            # 5. 精准锁频 100Hz (10ms)
            elapsed = time.time() - loop_start
            if elapsed < 0.01:
                time.sleep(0.01 - elapsed)

    except KeyboardInterrupt:
        print("\n\n🛑 检测到中断，提前保存当前数据...")

    # --- 保存与释放 ---
    collector.save_to_hdf5(args.dataset_dir, args.task_name, args.episode_idx)

    dm_ctrl.disable(Motor1)
    dm_ctrl.disable(Motor2)
    serial_device.close()
    portHandler.closePort()
    print("✨ 设备已安全释放。")


if __name__ == '__main__':
    main()

# python record_episodes.py --task_name gimbal_tracking --dataset_dir /media/hjx/PSSD/hjx_ws/Dual_servo_gimbal/data/ --episode_idx 0
