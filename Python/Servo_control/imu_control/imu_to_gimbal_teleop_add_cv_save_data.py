import time
import math
import sys
import os
import threading
from collections import deque
import cv2
import h5py
import numpy as np
import argparse
from tqdm import tqdm

# ================= 动态环境变量注入 =================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
project_root = os.path.dirname(parent_dir)
sdk_path = os.path.join(project_root, 'STservo_sdk')

if project_root not in sys.path: sys.path.append(project_root)
if sdk_path not in sys.path: sys.path.append(sdk_path)

from STservo_sdk import *
from YbImuLib import YbImuSerial

# ================= 1. 硬件配置 =================
IMU_PORT = '/dev/ttyUSB1'
SERVO_PORT = '/dev/ttyUSB0'
SERVO_BAUD = 115200
CAMERA_ID = 2  # 外接相机索引

ID_PAN = 1
ID_TILT = 2
SERVO_MID = 2048
DEG_TO_SERVO_UNIT = 4095.0 / 360.0
SERVO_UNIT_TO_DEG = 360.0 / 4095.0

# --- 核心手感调校参数 ---
PAN_DIR = -1
TILT_DIR = 1
PAN_SCALE = 1
TILT_SCALE = 1.0
EMA_ALPHA = 0.5


def angle_to_pos(angle_deg):
    return int(angle_deg * DEG_TO_SERVO_UNIT)


PAN_SAFE_MIN = angle_to_pos(90)
PAN_SAFE_MAX = angle_to_pos(270)
TILT_SAFE_MIN = angle_to_pos(85)
TILT_SAFE_MAX = angle_to_pos(225)


# ================= 2. ALOHA 规范数据收集器 =================
class VLADataCollector:
    def __init__(self):
        self.data_dict = {
            '/time': [],
            '/action': [],  # [pan_cmd, tilt_cmd] (模型输出的动作)
            '/observations/qpos': [],  # [pan_real, tilt_real] (当前真实姿态)
            '/observations/images/cam_high': []  # 对应的第一视角图像
        }

    def append_step(self, t, action, qpos, img):
        self.data_dict['/time'].append(t)
        self.data_dict['/action'].append(action)
        self.data_dict['/observations/qpos'].append(qpos)
        # 将图像从 BGR 转为 RGB，更符合深度学习标准
        if img is not None:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.data_dict['/observations/images/cam_high'].append(img_rgb)

    def save_to_hdf5(self, dataset_dir, task_name, episode_idx):
        task_dir = os.path.join(dataset_dir, task_name)
        os.makedirs(task_dir, exist_ok=True)
        file_path = os.path.join(task_dir, f'episode_{episode_idx}.hdf5')

        print(f"\n🔄 正在将多模态数据写入 {file_path} ... (可能需要几秒钟)")
        t0 = time.time()

        with h5py.File(file_path, 'w') as root:
            # 保存非图像数据
            for key in ['/time', '/action', '/observations/qpos']:
                if len(self.data_dict[key]) == 0: continue
                data_np = np.array(self.data_dict[key], dtype=np.float32)
                root.create_dataset(key, data=data_np, compression='gzip')

            # 保存图像数据 (使用 uint8 节省空间，并启用 gzip 压缩)
            if len(self.data_dict['/observations/images/cam_high']) > 0:
                imgs_np = np.array(self.data_dict['/observations/images/cam_high'], dtype=np.uint8)
                root.create_dataset('/observations/images/cam_high', data=imgs_np, compression='gzip')

        print(f"✅ 保存成功！耗时: {time.time() - t0:.2f}s | 总帧数: {len(self.data_dict['/time'])}")


# ================= 3. 异步相机读取线程 =================
class AsyncCamera:
    def __init__(self, src=2, width=640, height=480):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.ret, self.frame = self.cap.read()
        self.is_running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        while self.is_running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame
            time.sleep(0.01)  # 防止 CPU 占用过高

    def read(self):
        with self.lock:
            if self.frame is None:
                return None
            # 缩小图像分辨率到 320x240，大幅减少 HDF5 存储压力，且足够模型训练
            resized_frame = cv2.resize(self.frame, (320, 240))
            return resized_frame

    def stop(self):
        self.is_running = False
        self.cap.release()


# ================= 4. 辅助函数 =================
def clamp(val, min_val, max_val): return max(min_val, min(val, max_val))


def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle


# ================= 5. 主程序 =================
def main():
    parser = argparse.ArgumentParser(description='多模态体感数据采集系统 (VLA Format)')
    parser.add_argument('--task_name', type=str, default='head_tracking', help='任务名称')
    parser.add_argument('--dataset_dir', type=str, default='./vla_data/', help='数据集目录')
    parser.add_argument('--episode_len', type=int, default=1000, help='录制步数(100Hz下1000步=10秒)')
    parser.add_argument('--episode_idx', type=int, default=0, help='回合编号')
    args = parser.parse_args()

    print("🔌 系统初始化中...")
    # 1. 硬件初始化
    imu = YbImuSerial(IMU_PORT, debug=False)
    imu.create_receive_threading()

    portHandler = PortHandler(SERVO_PORT)
    scs = sts(portHandler)
    if not portHandler.openPort() or not portHandler.setBaudRate(SERVO_BAUD):
        print("❌ 舵机串口打开失败！")
        return

    print("📷 启动异步相机线程...")
    cam = AsyncCamera(src=CAMERA_ID)
    collector = VLADataCollector()
    time.sleep(2.0)

    # 2. 舵机回正锁定
    scs.unLockEprom(ID_PAN)
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 1)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 1)
    scs.groupSyncWrite.clearParam()
    scs.SyncWritePosEx(ID_PAN, SERVO_MID, 1500, 50)
    scs.SyncWritePosEx(ID_TILT, SERVO_MID, 1500, 50)
    scs.groupSyncWrite.txPacket()
    time.sleep(1.5)

    # 3. IMU 归零校准
    print("\n" + "=" * 50)
    print("⚠️ 请将 IMU 放置在【正前方初始位置】并保持静止！")
    for i in range(3, 0, -1):
        print(f" {i} ...")
        time.sleep(1)
    base_roll, base_pitch, base_yaw = imu.get_imu_attitude_data(ToAngle=True)
    print(f"✅ 校准完成！Yaw={base_yaw:.1f}°, Pitch={base_pitch:.1f}°")

    print(f"\n🎥 准备录制 | 任务: {args.task_name} | 编号: {args.episode_idx} | 时长: {args.episode_len / 100.0}s")
    for i in range(3, 0, -1):
        print(f" 录制倒计时 {i} ...")
        time.sleep(1)
    print("🔴 开始采集！(在弹出窗口按 'q' 可提前结束)")

    filtered_pan_pos = SERVO_MID
    filtered_tilt_pos = SERVO_MID
    start_time = time.time()

    try:
        # 使用 tqdm 进度条显示采集进度
        for step in tqdm(range(args.episode_len), desc="Data Collection", unit="step", colour="green"):
            loop_start = time.time()
            current_t = loop_start - start_time

            # A. 姿态解算
            roll, pitch, yaw = imu.get_imu_attitude_data(ToAngle=True)
            if pitch is not None and yaw is not None:
                delta_yaw = normalize_angle(yaw - base_yaw)
                delta_pitch = normalize_angle(pitch - base_pitch)

                raw_pan_pos = SERVO_MID + (delta_yaw * DEG_TO_SERVO_UNIT * PAN_DIR * PAN_SCALE)
                raw_tilt_pos = SERVO_MID + (delta_pitch * DEG_TO_SERVO_UNIT * TILT_DIR * TILT_SCALE)

                safe_pan = clamp(raw_pan_pos, PAN_SAFE_MIN, PAN_SAFE_MAX)
                safe_tilt = clamp(raw_tilt_pos, TILT_SAFE_MIN, TILT_SAFE_MAX)

                filtered_pan_pos = EMA_ALPHA * safe_pan + (1 - EMA_ALPHA) * filtered_pan_pos
                filtered_tilt_pos = EMA_ALPHA * safe_tilt + (1 - EMA_ALPHA) * filtered_tilt_pos

                cmd_pan = int(filtered_pan_pos)
                cmd_tilt = int(filtered_tilt_pos)

                # B. 下发物理指令
                scs.groupSyncWrite.clearParam()
                scs.SyncWritePosEx(ID_PAN, cmd_pan, 0, 0)
                scs.SyncWritePosEx(ID_TILT, cmd_tilt, 0, 0)
                scs.groupSyncWrite.txPacket()

                # C. 读取真实状态与图像
                p1_raw, _, _, _ = scs.ReadPosSpeed(ID_PAN)
                p2_raw, _, _, _ = scs.ReadPosSpeed(ID_TILT)
                latest_img = cam.read()

                # 转为统一的弧度制 (便于深度学习模型学习)
                action_rad = [(cmd_pan - SERVO_MID) * SERVO_UNIT_TO_DEG * (math.pi / 180),
                              (cmd_tilt - SERVO_MID) * SERVO_UNIT_TO_DEG * (math.pi / 180)]
                qpos_rad = [(p1_raw - SERVO_MID) * SERVO_UNIT_TO_DEG * (math.pi / 180),
                            (p2_raw - SERVO_MID) * SERVO_UNIT_TO_DEG * (math.pi / 180)]

                # D. 压入数据集
                collector.append_step(current_t, action_rad, qpos_rad, latest_img)

                # ================= UI 降频显示逻辑 =================
                # 每 3 帧（约 33Hz）刷新一次画面，防止拖垮 100Hz 物理控制循环
                if step % 3 == 0 and latest_img is not None:
                    display_img = latest_img.copy()
                    cv2.putText(display_img, f"REC | Step: {step}/{args.episode_len}", (10, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    cv2.imshow("VLA Data Collection", display_img)

                    # 允许按 'q' 键提前结束录制
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("\n\n🛑 接收到 Q 键指令，提前终止采集...")
                        break
                # =================================================

            # E. 严格锁频 100Hz (10ms)
            elapsed = time.time() - loop_start
            if elapsed < 0.01:
                time.sleep(0.01 - elapsed)

    except KeyboardInterrupt:
        print("\n\n🛑 检测到人为中断，正在提前保存数据...")

    # ================= 结尾安全清理 =================
    # 保存文件并释放资源
    collector.save_to_hdf5(args.dataset_dir, args.task_name, args.episode_idx)

    print("下电与释放...")
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)
    portHandler.closePort()
    cam.stop()
    cv2.destroyAllWindows()  # 确保释放弹出的 OpenCV 窗口
    print("✨ 采集任务安全结束。")


if __name__ == '__main__':
    main()

# python imu_vla_data_collector.py --task_name head_tracking --dataset_dir /media/hjx/PSSD/hjx_ws/Dual_servo_gimbal/data/ --episode_len 1000 --episode_idx 0
