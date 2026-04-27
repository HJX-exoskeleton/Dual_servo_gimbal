import time
import sys
import os
import cv2
import mediapipe as mp

# ================= 动态环境变量注入 =================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
project_root = os.path.dirname(parent_dir)
sdk_path = os.path.join(project_root, 'STservo_sdk')

if project_root not in sys.path: sys.path.append(project_root)
if sdk_path not in sys.path: sys.path.append(sdk_path)

# 导入底层库
from STservo_sdk import *

# ================= 1. 硬件参数配置 =================
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'  # Windows 请改为 'COMx'
CAMERA_ID = 2  # 你的外接相机索引

ID_PAN = 1
ID_TILT = 2

# 预设的中位角度
PAN_CENTER_DEG = 180.0
TILT_CENTER_DEG = 180.0

# --- ⚠️ 视觉伺服核心参数 (PID 比例控制) ---
# 灵敏度系数 (Kp)：像素误差转化为角度的比例。太大容易抖动/过冲，太小跟不上
KP_PAN = 0.02    # 水平方向也可以稍微降一点，求稳
KP_TILT = 0.018  # 俯仰轴因为有重力影响，Kp 必须比水平轴小很多！

# 舵机方向反转配置 (如果发现云台往人脸反方向跑，把对应的 1 改成 -1)
PAN_DIR = -1
TILT_DIR = 1

# 视觉死区 (Deadzone)：人脸在画面中心的这个像素范围内时，云台不动作，防止神经质般的高频微小抖动
DEADZONE = 25

# 安全限位 (复用你之前的极限数据，防止拉扯相机线材)
PAN_MIN_DEG = 90.0
PAN_MAX_DEG = 270.0
TILT_MIN_DEG = 85.0
TILT_MAX_DEG = 225.0


def angle_to_pos(angle):
    return int((angle / 360.0) * 4095.0)


def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))


# ================= 2. 硬件初始化 =================
print("🔌 正在连接云台舵机...")
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)
if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("❌ 舵机串口打开失败！")
    sys.exit()


def home_servos():
    print("🏠 正在执行云台复位程序...")
    scs.unLockEprom(ID_PAN)
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 1)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 1)

    tgt_pan = angle_to_pos(PAN_CENTER_DEG)
    tgt_tilt = angle_to_pos(TILT_CENTER_DEG)

    scs.groupSyncWrite.clearParam()
    scs.SyncWritePosEx(ID_PAN, tgt_pan, 1500, 40)
    scs.SyncWritePosEx(ID_TILT, tgt_tilt, 1500, 40)
    scs.groupSyncWrite.txPacket()
    time.sleep(1.5)  # 给一点时间回正
    print("✅ 云台已回到初始化中位。")


# ================= 3. 视觉追踪主循环 =================
def tracking_loop():
    print("📷 正在打开相机...")
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 画面中心点 (基准点)
    FRAME_W, FRAME_H = 640, 480
    CENTER_X, CENTER_Y = FRAME_W // 2, FRAME_H // 2

    # 初始化 MediaPipe 人脸检测
    mp_face_detection = mp.solutions.face_detection
    face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.7)

    # 记录云台当前的期望角度
    current_pan_deg = PAN_CENTER_DEG
    current_tilt_deg = TILT_CENTER_DEG

    print("\n" + "=" * 40)
    print("🚀 AI 人脸追踪已启动！")
    print("站在镜头前走动，按 'q' 键安全退出...")
    print("=" * 40)

    pTime = 0

    while True:
        success, img = cap.read()
        if not success:
            print("❌ 无法读取相机画面")
            break

        # 镜像翻转画面 (让你感觉像在照镜子，更直观)
        img = cv2.flip(img, 1)
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = face_detection.process(imgRGB)

        # 在画面中心画一个十字瞄准星
        cv2.drawMarker(img, (CENTER_X, CENTER_Y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

        if results.detections:
            # 只取检测到的第一张脸
            detection = results.detections[0]
            bboxC = detection.location_data.relative_bounding_box

            # 计算人脸中心在画面中的像素坐标
            cx = int((bboxC.xmin + bboxC.width / 2) * FRAME_W)
            cy = int((bboxC.ymin + bboxC.height / 2) * FRAME_H)

            # 画出人脸位置
            cv2.circle(img, (cx, cy), 5, (0, 0, 255), cv2.FILLED)
            cv2.rectangle(img,
                          (int(bboxC.xmin * FRAME_W), int(bboxC.ymin * FRAME_H)),
                          (int((bboxC.xmin + bboxC.width) * FRAME_W), int((bboxC.ymin + bboxC.height) * FRAME_H)),
                          (255, 0, 255), 2)

            # --- 核心逻辑：视觉误差计算 ---
            # 误差 = 目标位置 - 画面中心
            error_x = cx - CENTER_X
            error_y = cy - CENTER_Y

            # --- 核心逻辑：死区与比例控制 (P Control) ---
            update_pan = False
            update_tilt = False

            if abs(error_x) > DEADZONE:
                # 角度 = 当前角度 + (像素误差 * 灵敏度 * 方向)
                current_pan_deg += error_x * KP_PAN * PAN_DIR
                current_pan_deg = clamp(current_pan_deg, PAN_MIN_DEG, PAN_MAX_DEG)
                update_pan = True

            if abs(error_y) > DEADZONE:
                current_tilt_deg += error_y * KP_TILT * TILT_DIR
                current_tilt_deg = clamp(current_tilt_deg, TILT_MIN_DEG, TILT_MAX_DEG)
                update_tilt = True

            # --- 核心逻辑：执行动作 ---
            if update_pan or update_tilt:
                pos_pan = angle_to_pos(current_pan_deg)
                pos_tilt = angle_to_pos(current_tilt_deg)

                # 下发指令 (速度设为 0，完全由高频循环和步长控制平滑度)
                scs.groupSyncWrite.clearParam()
                scs.SyncWritePosEx(ID_PAN, pos_pan, 0, 0)
                scs.SyncWritePosEx(ID_TILT, pos_tilt, 0, 0)
                scs.groupSyncWrite.txPacket()

                # 在画面上显示追踪状态
                cv2.putText(img, "TRACKING", (CENTER_X - 60, CENTER_Y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255),
                            2)

        # 计算并显示 FPS
        cTime = time.time()
        fps = 1 / (cTime - pTime) if (cTime - pTime) > 0 else 0
        pTime = cTime
        cv2.putText(img, f'FPS: {int(fps)}', (10, 30), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

        # 显示画面
        cv2.imshow("AI Face Tracking Gimbal", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()


# ================= 4. 执行流程 =================
try:
    home_servos()
    tracking_loop()
finally:
    print("\n进入待机状态：释放力矩...")
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("✨ 程序结束，设备安全释放。")