import time
import h5py
import numpy as np
import os
import argparse
from tqdm import tqdm  # 进度条神器

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


# ================= 1. ALOHA 规范化数据收集器 =================
class AlohaDataCollector:
    def __init__(self, camera_names=[]):
        self.camera_names = camera_names
        self.data_dict = {
            '/time': [],
            '/observations/qpos': [],  # 关节当前真实位置
            '/observations/qvel': [],  # 关节当前真实速度
            '/action/target_pos': [],  # 目标动作指令 (示教时就是真实位置)
        }
        for cam_name in self.camera_names:
            self.data_dict[f'/observations/images/{cam_name}'] = []

    def append_step_data(self, current_time, qpos, qvel, action):
        self.data_dict['/time'].append(current_time)
        self.data_dict['/observations/qpos'].append(qpos)
        self.data_dict['/observations/qvel'].append(qvel)
        self.data_dict['/action/target_pos'].append(action)

    def save_to_hdf5(self, dataset_dir, task_name, episode_idx):
        # 自动创建多级目录
        task_dir = os.path.join(dataset_dir, task_name)
        if not os.path.exists(task_dir):
            os.makedirs(task_dir)

        file_path = os.path.join(task_dir, f'episode_{episode_idx}.hdf5')

        print(f"\n🔄 正在将数据打包压缩写入 {file_path} ...")
        t0 = time.time()

        with h5py.File(file_path, 'w') as root:
            for key, value_list in self.data_dict.items():
                if len(value_list) == 0:
                    continue
                data_np = np.array(value_list, dtype=np.float32)
                root.create_dataset(key, data=data_np, compression='gzip')

        print(f"✅ 保存成功！耗时: {time.time() - t0:.2f}s | 总帧数: {len(self.data_dict['/time'])}")


# ================= 2. 主程序与命令行解析 =================
def main():
    # --- A. 设置 argparse ---
    parser = argparse.ArgumentParser(description='云台手动示教数据录制系统 (ALOHA 规范)')
    parser.add_argument('--task_name', action='store', type=str, default='control_test',
                        help='任务名称 (将作为子文件夹名)')
    parser.add_argument('--dataset_dir', action='store', type=str, default='./data/', help='数据集保存的根目录')
    parser.add_argument('--episode_len', action='store', type=int, default=800,
                        help='最大录制步数 (默认800步，50Hz下约16秒)')
    parser.add_argument('--episode_idx', action='store', type=int, default=0,
                        help='当前录制的回合编号 (如 episode_0.hdf5)')
    parser.add_argument('--port', action='store', type=str, default='COM6', help='串口号')
    args = parser.parse_args()

    # --- B. 初始化硬件 ---
    BAUDRATE = 115200
    ID_PAN = 1
    ID_TILT = 2

    portHandler = PortHandler(args.port)
    scs = sts(portHandler)

    if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
        print(f"❌ 串口 {args.port} 打开失败！")
        return

    print("🔌 正在等待 ESP32 开机...")
    time.sleep(2.5)

    # 释放力矩，进入示教模式 (让人手可以转动)
    scs.unLockEprom(ID_PAN)
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)

    # --- C. 录制准备与倒计时 ---
    print("\n" + "=" * 50)
    print(f"🎥 录制任务: {args.task_name} | 回合: {args.episode_idx} | 总步数: {args.episode_len}")
    print("=" * 50)
    print("⚠️  请将双手放在云台上准备，力矩已释放。")

    for i in range(3, 0, -1):
        print(f"录制将在 {i} 秒后开始...")
        time.sleep(1)

    print("🔴 正在录制 (Recording...) 请开始动作！")

    # --- D. 核心录制循环 ---
    collector = AlohaDataCollector()
    start_time = time.time()

    # 容错缓存 (防止单帧串口丢包导致数据断裂)
    last_qpos = [2048, 2048]
    last_qvel = [0, 0]

    try:
        # 使用 tqdm 包装 range，瞬间获得极具极客范的进度条！
        for step in tqdm(range(args.episode_len), desc="数据采集进度", unit="帧", ncols=80, colour='green'):

            loop_start = time.time()
            current_time = loop_start - start_time

            # 读取物理状态
            pos_1, spd_1, res_1, _ = scs.ReadPosSpeed(ID_PAN)
            pos_2, spd_2, res_2, _ = scs.ReadPosSpeed(ID_TILT)

            # 容错处理：如果读取成功则更新，否则使用上一帧的数据
            if res_1 == COMM_SUCCESS and res_2 == COMM_SUCCESS:
                last_qpos = [pos_1, pos_2]
                last_qvel = [spd_1, spd_2]

            # 在示教模式中，人类当前的动作就是 Target Action
            action = last_qpos

            # 极速压入内存
            collector.append_step_data(current_time, last_qpos, last_qvel, action)

            # --- 精准锁频 (50Hz = 20ms/帧) ---
            # 计算刚才读串口花掉的时间，剩下的时间用 sleep 补齐
            time_spent = time.time() - loop_start
            time_left = 0.02 - time_spent
            if time_left > 0:
                time.sleep(time_left)

    except KeyboardInterrupt:
        print("\n\n🛑 检测到人为中断，录制提前结束！将保存已录制的数据。")

    # --- E. 落地为安：写入 HDF5 ---
    collector.save_to_hdf5(args.dataset_dir, args.task_name, args.episode_idx)

    portHandler.closePort()
    print("🎉 串口已关闭，采集任务结束。")


if __name__ == '__main__':
    main()

# python servo_record_data_hdf5.py --port /dev/ttyUSB0 --task_name "control_test" --dataset_dir "/media/hjx/PSSD/hjx_ws/Dual_servo_gimbal/data" --episode_len 500 --episode_idx 0

