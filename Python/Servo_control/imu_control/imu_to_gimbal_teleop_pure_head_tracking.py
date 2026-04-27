import time
import sys
import os
import threading
import cv2

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
CAMERA_ID = 2  # 外接相机

ID_PAN = 1
ID_TILT = 2
SERVO_MID = 2048
DEG_TO_SERVO_UNIT = 4095.0 / 360.0

# --- 核心手感调校参数 ---
PAN_DIR = -1
TILT_DIR = 1
PAN_SCALE = 1.0
TILT_SCALE = 1.0
EMA_ALPHA = 0.5  # 0.4~0.6 之间，越大越跟手，越小越平滑


def angle_to_pos(angle_deg):
    return int(angle_deg * DEG_TO_SERVO_UNIT)


PAN_SAFE_MIN = angle_to_pos(90)
PAN_SAFE_MAX = angle_to_pos(270)
TILT_SAFE_MIN = angle_to_pos(85)
TILT_SAFE_MAX = angle_to_pos(225)


# ================= 2. 异步相机读取线程 =================
# 彻底分离相机读取与舵机控制，保证 100Hz 物理顺滑度
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
            time.sleep(0.01)

    def read(self):
        with self.lock:
            if self.frame is None: return None
            return self.frame.copy()

    def stop(self):
        self.is_running = False
        self.cap.release()


# ================= 3. 辅助函数 =================
def clamp(val, min_val, max_val): return max(min_val, min(val, max_val))


def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle


# ================= 4. 主程序 =================
def main():
    print("🔌 系统初始化中...")

    # 1. 硬件初始化
    imu = YbImuSerial(IMU_PORT, debug=False)
    imu.create_receive_threading()

    portHandler = PortHandler(SERVO_PORT)
    scs = sts(portHandler)
    if not portHandler.openPort() or not portHandler.setBaudRate(SERVO_BAUD):
        print("❌ 舵机串口打开失败！")
        return

    print("📷 启动异步相机流...")
    cam = AsyncCamera(src=CAMERA_ID)
    time.sleep(2.0)  # 等待设备预热

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
    print(f"✅ 校准完成！基准 Yaw={base_yaw:.1f}°, Pitch={base_pitch:.1f}°")
    print("=" * 50)
    print("🚀 FPV 头追模式已激活！(按 'q' 退出)")

    filtered_pan_pos = SERVO_MID
    filtered_tilt_pos = SERVO_MID

    # 用于 UI 降频的帧计数器
    frame_counter = 0

    try:
        while True:
            loop_start = time.time()

            # --- A. 姿态解算 ---
            roll, pitch, yaw = imu.get_imu_attitude_data(ToAngle=True)
            delta_yaw = 0
            delta_pitch = 0

            if pitch is not None and yaw is not None:
                delta_yaw = normalize_angle(yaw - base_yaw)
                delta_pitch = normalize_angle(pitch - base_pitch)

                raw_pan_pos = SERVO_MID + (delta_yaw * DEG_TO_SERVO_UNIT * PAN_DIR * PAN_SCALE)
                raw_tilt_pos = SERVO_MID + (delta_pitch * DEG_TO_SERVO_UNIT * TILT_DIR * TILT_SCALE)

                safe_pan = clamp(raw_pan_pos, PAN_SAFE_MIN, PAN_SAFE_MAX)
                safe_tilt = clamp(raw_tilt_pos, TILT_SAFE_MIN, TILT_SAFE_MAX)

                filtered_pan_pos = EMA_ALPHA * safe_pan + (1 - EMA_ALPHA) * filtered_pan_pos
                filtered_tilt_pos = EMA_ALPHA * safe_tilt + (1 - EMA_ALPHA) * filtered_tilt_pos

                # --- B. 下发物理指令 ---
                scs.groupSyncWrite.clearParam()
                scs.SyncWritePosEx(ID_PAN, int(filtered_pan_pos), 0, 0)
                scs.SyncWritePosEx(ID_TILT, int(filtered_tilt_pos), 0, 0)
                scs.groupSyncWrite.txPacket()

            # --- C. UI 降频渲染 (约 33Hz 刷新画面) ---
            frame_counter += 1
            if frame_counter % 3 == 0:
                img = cam.read()
                if img is not None:
                    h, w = img.shape[:2]
                    cx, cy = w // 2, h // 2

                    # 绘制战斗机 HUD 风格准星
                    cv2.line(img, (cx - 30, cy), (cx + 30, cy), (0, 255, 0), 1)
                    cv2.line(img, (cx, cy - 30), (cx, cy + 30), (0, 255, 0), 1)
                    cv2.circle(img, (cx, cy), 15, (0, 255, 0), 1)

                    # 绘制数据遥测板
                    cv2.rectangle(img, (10, 10), (220, 100), (0, 0, 0), cv2.FILLED)
                    cv2.putText(img, "FPV HEAD TRACKING", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(img, f"YAW:   {delta_yaw:6.1f} deg", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 1)
                    cv2.putText(img, f"PITCH: {delta_pitch:6.1f} deg", (15, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 1)

                    cv2.imshow("Pure Head Tracking", img)

            # 按键检测放在 33Hz 的刷新里即可
            if frame_counter % 3 == 0:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n🛑 退出 FPV 模式...")
                    break

            # --- D. 严格锁频 100Hz (10ms) ---
            elapsed = time.time() - loop_start
            if elapsed < 0.01:
                time.sleep(0.01 - elapsed)

    except KeyboardInterrupt:
        print("\n\n🛑 检测到强制中断...")

    # ================= 结尾安全清理 =================
    print("下电与释放...")
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)
    portHandler.closePort()
    cam.stop()
    cv2.destroyAllWindows()
    print("✨ 设备已安全释放。")


if __name__ == '__main__':
    main()
