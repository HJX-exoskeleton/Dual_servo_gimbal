import time
import h5py
import os
import argparse
from tqdm import tqdm

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


def main():
    # --- A. 设置 argparse (与录制脚本保持一致) ---
    parser = argparse.ArgumentParser(description='云台 HDF5 数据回放系统')
    parser.add_argument('--task_name', action='store', type=str, default='control_test', help='任务名称')
    parser.add_argument('--dataset_dir', action='store', type=str, default='./data/', help='数据集目录')
    parser.add_argument('--episode_idx', action='store', type=int, default=0, help='要回放的回合编号')
    parser.add_argument('--port', action='store', type=str, default='COM6', help='串口号')
    args = parser.parse_args()

    # --- B. 加载 HDF5 数据 ---
    file_path = os.path.join(args.dataset_dir, args.task_name, f'episode_{args.episode_idx}.hdf5')
    if not os.path.exists(file_path):
        print(f"❌ 找不到数据文件: {file_path}")
        return

    print(f"📂 正在加载数据: {file_path} ...")
    with h5py.File(file_path, 'r') as root:
        # 提取目标动作数组 (Nx2 矩阵)
        action_data = root['/action/target_pos'][:]
        # 你也可以提取真实位置用于比对: qpos_data = root['/observations/qpos'][:]

    num_frames = len(action_data)
    print(f"✅ 加载成功！共包含 {num_frames} 帧动作数据 (约 {num_frames / 50.0:.1f} 秒)。")

    # --- C. 初始化硬件 ---
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

    # 开启力矩，准备运动
    scs.unLockEprom(ID_PAN)
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 1)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 1)

    # --- D. 硬件保护：平滑移动到起始帧 (Smooth Homing) ---
    print("\n" + "=" * 50)
    print("🛡️ 正在缓慢移动至录像的起始位置...")
    start_pan, start_tilt = action_data[0]

    scs.groupSyncWrite.clearParam()
    # 使用 1500 的缓速和 40 的加速度，防止瞬间甩尾
    scs.SyncWritePosEx(ID_PAN, int(start_pan), 1500, 40)
    scs.SyncWritePosEx(ID_TILT, int(start_tilt), 1500, 40)
    scs.groupSyncWrite.txPacket()

    # 给它 2 秒钟的时间慢慢就位
    time.sleep(2.0)
    print("=" * 50)

    # --- E. 倒计时准备 ---
    print("⚠️  请让开云台活动范围！")
    for i in range(3, 0, -1):
        print(f"回放将在 {i} 秒后开始...")
        time.sleep(1)

    print("▶️ 开始回放 (Playing...)")

    # --- F. 核心回放循环 ---
    try:
        # 使用 tqdm 显示回放进度条
        for i in tqdm(range(num_frames), desc="数据回放进度", unit="帧", ncols=80, colour='cyan'):
            loop_start = time.time()

            # 取出当前帧的目标位置
            tgt_pan, tgt_tilt = action_data[i]

            # 高频流式下发，速度和加速度设为 0 (完全依赖录像本身的平滑度)
            scs.groupSyncWrite.clearParam()
            scs.SyncWritePosEx(ID_PAN, int(tgt_pan), 0, 0)
            scs.SyncWritePosEx(ID_TILT, int(tgt_tilt), 0, 0)
            scs.groupSyncWrite.txPacket()

            # --- 精准锁频 (50Hz = 20ms/帧) ---
            time_spent = time.time() - loop_start
            time_left = 0.02 - time_spent
            if time_left > 0:
                time.sleep(time_left)

    except KeyboardInterrupt:
        print("\n\n🛑 检测到人为中断，回放提前停止！")

    # --- G. 落地为安：释放力矩 ---
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("🎉 串口已关闭，回放结束。")


if __name__ == '__main__':
    main()

# python servo_replay_data_hdf5.py --port /dev/ttyUSB0 --task_name "control_test" --dataset_dir "/media/hjx/PSSD/hjx_ws/Dual_servo_gimbal/data" --episode_idx 0

