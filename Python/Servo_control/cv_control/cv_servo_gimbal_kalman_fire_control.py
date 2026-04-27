import time
import sys
import os
import cv2
import mediapipe as mp
import numpy as np

# ================= 动态环境变量注入 =================
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
project_root = os.path.dirname(parent_dir)
sdk_path = os.path.join(project_root, 'STservo_sdk')

if project_root not in sys.path: sys.path.append(project_root)
if sdk_path not in sys.path: sys.path.append(sdk_path)

from STservo_sdk import *

# ================= 1. 硬件参数配置 =================
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'  # Windows 请改为 'COMx'
CAMERA_ID = 2

ID_PAN = 1
ID_TILT = 2

PAN_CENTER_DEG = 180.0
TILT_CENTER_DEG = 180.0

# 经过卡尔曼平滑后，我们可以稍微提高 Kp，并且大幅度减小死区！

# 稍微提高一点 P，引入 D 来刹车
KP_PAN = 0.015
KD_PAN = 0.05     # 新增：微分项，用于消除左右抖动
KP_TILT = 0.01
KD_TILT = 0.05    # 新增：微分项，用于消除上下点头

PAN_DIR = -1
TILT_DIR = 1
DEADZONE = 15  # 死区从 25 降到 10，因为高频噪声被滤除了

PAN_MIN_DEG = 90.0
PAN_MAX_DEG = 270.0
TILT_MIN_DEG = 85.0
TILT_MAX_DEG = 225.0


def angle_to_pos(angle): return int((angle / 360.0) * 4095.0)


def clamp(val, min_val, max_val): return max(min_val, min(val, max_val))


# ================= 2. 卡尔曼滤波器 (恒速模型) =================
class KalmanTracker:
    def __init__(self):
        # 状态变量: 4维 [x, y, vx, vy] | 观测变量: 2维 [x, y]
        self.kf = cv2.KalmanFilter(4, 2)

        # 测量矩阵 (H): 我们只能测量位置 (x, y)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                              [0, 1, 0, 0]], np.float32)

        # 状态转移矩阵 (A): x = x + vx*dt, y = y + vy*dt (假设 dt=1)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                             [0, 1, 0, 1],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]], np.float32)

        # 过程噪声 (Q): 决定系统信任预测模型的程度。数值越小越平滑但滞后
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03

        # 测量噪声 (R): 决定系统信任传感器观测的程度。数值越大越能滤除传感器抖动
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 30.0

        self.last_prediction = np.zeros((2, 1), np.float32)

    def predict(self):
        """预测下一帧状态 (甚至在丢失目标时也能盲猜)"""
        pred = self.kf.predict()
        self.last_prediction = pred
        return pred[0], pred[1], pred[2], pred[3]  # x, y, vx, vy

    def correct(self, x, y):
        """用实际观测值纠正滤波器"""
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        self.kf.correct(measurement)


# ================= 3. 硬件初始化 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)
if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("❌ 舵机串口打开失败！")
    sys.exit()


def home_servos():
    print("🏠 火控系统预热启动...")
    scs.unLockEprom(ID_PAN)
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 1)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 1)
    scs.groupSyncWrite.clearParam()
    scs.SyncWritePosEx(ID_PAN, angle_to_pos(PAN_CENTER_DEG), 1500, 40)
    scs.SyncWritePosEx(ID_TILT, angle_to_pos(TILT_CENTER_DEG), 1500, 40)
    scs.groupSyncWrite.txPacket()
    time.sleep(1.5)


# ================= 4. 火控追踪主循环 =================
def fire_control_loop():
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    FRAME_W, FRAME_H = 640, 480
    CENTER_X, CENTER_Y = FRAME_W // 2, FRAME_H // 2

    mp_face_detection = mp.solutions.face_detection
    face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.6)

    # 实例化卡尔曼跟踪器
    tracker = KalmanTracker()

    current_pan_deg = PAN_CENTER_DEG
    current_tilt_deg = TILT_CENTER_DEG

    # 追踪历史轨迹用于画图
    history_points = []

    print("\n" + "=" * 40)
    print("🎯 卡尔曼雷达火控已锁定！")
    print("尝试用手短暂遮挡脸部，或者快速移动，观察绿色十字星的惯性预测！")
    print("=" * 40)

    # 新增：用于记录上一帧误差的变量
    last_error_x = 0
    last_error_y = 0

    while True:
        success, img = cap.read()
        if not success: break
        img = cv2.flip(img, 1)
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        results = face_detection.process(imgRGB)

        # 1. 第一步：卡尔曼【先预测】(无论看没看到脸)
        pred_x, pred_y, vel_x, vel_y = tracker.predict()
        px, py = int(pred_x), int(pred_y)

        # 获取真实观测值
        observed = False
        if results.detections:
            bboxC = results.detections[0].location_data.relative_bounding_box
            raw_cx = int((bboxC.xmin + bboxC.width / 2) * FRAME_W)
            raw_cy = int((bboxC.ymin + bboxC.height / 2) * FRAME_H)

            # 画出原始摄像头的观测值 (红色散点/框，表示含有噪声)
            cv2.rectangle(img,
                          (int(bboxC.xmin * FRAME_W), int(bboxC.ymin * FRAME_H)),
                          (int((bboxC.xmin + bboxC.width) * FRAME_W), int((bboxC.ymin + bboxC.height) * FRAME_H)),
                          (0, 0, 255), 1)
            cv2.circle(img, (raw_cx, raw_cy), 4, (0, 0, 255), cv2.FILLED)

            # 2. 第二步：卡尔曼【后修正】
            tracker.correct(raw_cx, raw_cy)
            observed = True

        # --- 核心：弹道预测与火控提前量 ---
        # 我们不追踪当下的脸，而是追踪脸在 N 帧之后的位置 (Lead Aiming)
        LEAD_FRAMES = 0.0  # 提前量系数，移动越快，瞄得越靠前
        target_x = int(pred_x + vel_x * LEAD_FRAMES)
        target_y = int(pred_y + vel_y * LEAD_FRAMES)

        # 可视化：绿色十字星代表经过滤波后的稳定估计点
        cv2.drawMarker(img, (px, py), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        # 可视化：黄色连线代表速度矢量和未来预测落点
        cv2.line(img, (px, py), (target_x, target_y), (0, 255, 255), 2)
        cv2.circle(img, (target_x, target_y), 5, (0, 255, 255), 2)

        # 画准星
        cv2.drawMarker(img, (CENTER_X, CENTER_Y), (255, 255, 255), cv2.MARKER_CROSS, 30, 1)

        # ================== 执行动作 (以目标预测点为基准) ==================
        error_x = target_x - CENTER_X
        error_y = target_y - CENTER_Y

        # --- 核心：PD 控制器实现 ---
        # 计算微分（即误差的变化率）
        d_error_x = error_x - last_error_x
        d_error_y = error_y - last_error_y

        update_gimbal = False

        # 水平轴 PD 控制
        if abs(error_x) > DEADZONE:
            # 公式：Δ角度 = (P * 误差 + D * 误差变化) * 方向
            current_pan_deg += (error_x * KP_PAN + d_error_x * KD_PAN) * PAN_DIR
            current_pan_deg = clamp(current_pan_deg, PAN_MIN_DEG, PAN_MAX_DEG)
            update_gimbal = True

        # 俯仰轴 PD 控制
        if abs(error_y) > DEADZONE:
            current_tilt_deg += (error_y * KP_TILT + d_error_y * KD_TILT) * TILT_DIR
            current_tilt_deg = clamp(current_tilt_deg, TILT_MIN_DEG, TILT_MAX_DEG)
            update_gimbal = True

        # 存下当前误差，供下一帧计算 D 使用
        last_error_x = error_x
        last_error_y = error_y

        if update_gimbal:
            scs.groupSyncWrite.clearParam()
            scs.SyncWritePosEx(ID_PAN, angle_to_pos(current_pan_deg), 0, 0)
            scs.SyncWritePosEx(ID_TILT, angle_to_pos(current_tilt_deg), 0, 0)
            scs.groupSyncWrite.txPacket()

        # UI 文字
        status_txt = "LOCKED" if observed else "COASTING (BLIND PREDICT)"
        status_col = (0, 255, 0) if observed else (0, 165, 255)
        cv2.putText(img, status_txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_col, 2)
        cv2.putText(img, f"Vx: {vel_x[0]:.1f} Vy: {vel_y[0]:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 0), 2)

        cv2.imshow("Kalman Fire Control Gimbal", img)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()


# ================= 5. 执行流程 =================
try:
    home_servos()
    fire_control_loop()
finally:
    print("\n雷达关机，释放力矩...")
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("✨ 设备已安全释放。")
