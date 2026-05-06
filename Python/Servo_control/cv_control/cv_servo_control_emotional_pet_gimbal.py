import time
import sys
import os
import cv2
import math
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
CAMERA_ID = 0  # 2

ID_PAN = 1
ID_TILT = 2

PAN_CENTER_DEG = 180.0
TILT_CENTER_DEG = 180.0

# 复用你调优好的超棒参数！
KP_PAN = 0.02
KP_TILT = 0.018
PAN_DIR = -1
TILT_DIR = 1
DEADZONE = 25

PAN_MIN_DEG = 90.0
PAN_MAX_DEG = 270.0
TILT_MIN_DEG = 85.0
TILT_MAX_DEG = 225.0


# ================= 2. 辅助函数与手势识别 =================
def angle_to_pos(angle):
    return int((angle / 360.0) * 4095.0)


def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))


def detect_gesture(hand_landmarks):
    """极其轻量级的手势判断逻辑"""
    lm = hand_landmarks.landmark
    # 统计伸直的手指数量 (食指、中指、无名指、小拇指)
    fingers_up = 0
    for tip in [8, 12, 16, 20]:
        if lm[tip].y < lm[tip - 2].y:  # 指尖在关节上方(Y值更小)
            fingers_up += 1

    # 判断大拇指是否向上 (大拇指尖端Y < 大拇指根部Y 且其他手指弯曲)
    thumb_up = (lm[4].y < lm[3].y) and (lm[4].y < lm[5].y)

    if fingers_up >= 4:
        return "STOP"  # 张开手掌 (五指伸直)
    elif fingers_up == 0 and thumb_up:
        return "GOOD"  # 竖大拇指 (四指弯曲，大拇指向上)

    return "NONE"


# ================= 3. 硬件初始化 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)
if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("❌ 舵机串口打开失败！")
    sys.exit()


def home_servos():
    print("🏠 唤醒电子宠物...")
    scs.unLockEprom(ID_PAN)
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 1)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 1)
    scs.groupSyncWrite.clearParam()
    scs.SyncWritePosEx(ID_PAN, angle_to_pos(PAN_CENTER_DEG), 1500, 40)
    scs.SyncWritePosEx(ID_TILT, angle_to_pos(TILT_CENTER_DEG), 1500, 40)
    scs.groupSyncWrite.txPacket()
    time.sleep(1.5)


# ================= 4. 宠物大脑主循环 =================
def pet_loop():
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    FRAME_W, FRAME_H = 640, 480
    CENTER_X, CENTER_Y = FRAME_W // 2, FRAME_H // 2

    # 初始化视觉模型 (同时开启人脸和手部)
    mp_face = mp.solutions.face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.7)
    mp_hands = mp.solutions.hands.Hands(model_complexity=0, max_num_hands=1, min_detection_confidence=0.7)

    # 机器人内部记忆变量
    current_pan_deg = PAN_CENTER_DEG
    current_tilt_deg = TILT_CENTER_DEG

    # --- 宠物状态机变量 ---
    # 状态: "TRACKING" (正常追踪), "NODDING" (开心点头), "SHAKING" (拒绝摇头)
    emotion_state = "TRACKING"
    emotion_start_time = 0
    EMOTION_DURATION = 1.5  # 情绪动作持续 1.5 秒

    print("\n" + "=" * 40)
    print("🤖 电子宠物已上线！")
    print("尝试对它【竖大拇指】或【张开五指】吧！")
    print("=" * 40)

    while True:
        success, img = cap.read()
        if not success: break
        img = cv2.flip(img, 1)  # 镜像
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # 1. 获取感知数据
        face_results = mp_face.process(imgRGB)
        hand_results = mp_hands.process(imgRGB)

        # 2. 状态机：如果处于"正常追踪"状态，才允许被新手势打断
        if emotion_state == "TRACKING":
            gesture = "NONE"
            if hand_results.multi_hand_landmarks:
                for handLms in hand_results.multi_hand_landmarks:
                    mp.solutions.drawing_utils.draw_landmarks(img, handLms, mp.solutions.hands.HAND_CONNECTIONS)
                    gesture = detect_gesture(handLms)

            # 触发情绪转化！
            if gesture == "GOOD":
                emotion_state = "NODDING"
                emotion_start_time = time.time()
                print("触发情绪：开心！(点头)")
            elif gesture == "STOP":
                emotion_state = "SHAKING"
                emotion_start_time = time.time()
                print("触发情绪：害怕/拒绝！(摇头)")

        # ================== 动作执行层 ==================
        final_pan_deg = current_pan_deg
        final_tilt_deg = current_tilt_deg
        ui_text = "STATE: TRACKING"
        ui_color = (0, 255, 0)

        # 状态 A：正常追踪人脸
        if emotion_state == "TRACKING":
            if face_results.detections:
                bboxC = face_results.detections[0].location_data.relative_bounding_box
                cx = int((bboxC.xmin + bboxC.width / 2) * FRAME_W)
                cy = int((bboxC.ymin + bboxC.height / 2) * FRAME_H)
                cv2.circle(img, (cx, cy), 5, (0, 0, 255), cv2.FILLED)

                error_x = cx - CENTER_X
                error_y = cy - CENTER_Y

                if abs(error_x) > DEADZONE:
                    current_pan_deg += error_x * KP_PAN * PAN_DIR
                    current_pan_deg = clamp(current_pan_deg, PAN_MIN_DEG, PAN_MAX_DEG)
                if abs(error_y) > DEADZONE:
                    current_tilt_deg += error_y * KP_TILT * TILT_DIR
                    current_tilt_deg = clamp(current_tilt_deg, TILT_MIN_DEG, TILT_MAX_DEG)

            final_pan_deg = current_pan_deg
            final_tilt_deg = current_tilt_deg

        # 状态 B：开心点头 (Nodding)
        elif emotion_state == "NODDING":
            elapsed = time.time() - emotion_start_time
            if elapsed < EMOTION_DURATION:
                # 叠加正弦波运动：sin(时间 * 频率) * 振幅
                # 频率=20(约3次点头), 振幅=25度
                offset = math.sin(elapsed * 25) * 25.0
                final_tilt_deg = current_tilt_deg + offset
                ui_text = "STATE: HAPPY (NODDING)"
                ui_color = (0, 255, 255)
            else:
                emotion_state = "TRACKING"  # 动作结束，恢复追踪

        # 状态 C：拒绝摇头 (Shaking)
        elif emotion_state == "SHAKING":
            elapsed = time.time() - emotion_start_time
            if elapsed < EMOTION_DURATION:
                # 叠加正弦波运动：频率=25(剧烈晃动), 振幅=25度
                offset = math.sin(elapsed * 25) * 25.0
                final_pan_deg = current_pan_deg + offset
                ui_text = "STATE: REFUSE (SHAKING)"
                ui_color = (0, 0, 255)
            else:
                emotion_state = "TRACKING"

        # 安全限位把关
        final_pan_deg = clamp(final_pan_deg, PAN_MIN_DEG, PAN_MAX_DEG)
        final_tilt_deg = clamp(final_tilt_deg, TILT_MIN_DEG, TILT_MAX_DEG)

        # 统一发送指令到底层驱动
        scs.groupSyncWrite.clearParam()
        scs.SyncWritePosEx(ID_PAN, angle_to_pos(final_pan_deg), 0, 0)
        scs.SyncWritePosEx(ID_TILT, angle_to_pos(final_tilt_deg), 0, 0)
        scs.groupSyncWrite.txPacket()

        # UI 渲染
        cv2.putText(img, ui_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, ui_color, 2)
        cv2.imshow("Emotional Pet Gimbal", img)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()


# ================= 5. 执行流程 =================
try:
    home_servos()
    pet_loop()
finally:
    print("\n宠物进入休眠...")
    scs.write1ByteTxRx(ID_PAN, STS_TORQUE_ENABLE, 0)
    scs.write1ByteTxRx(ID_TILT, STS_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("✨ 设备已安全释放。")
