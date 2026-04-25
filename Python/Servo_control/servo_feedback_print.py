import time

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


# ================= 1. 基础参数设置 =================
BAUDRATE = 115200  # 必须与刚才 ESP32 的 Serial.begin 匹配！
# DEVICENAME = 'COM6'  # 请确认这是你当前的正确串口号
DEVICENAME = '/dev/ttyUSB0'  # 请确认这是你当前的正确串口号
ID_1 = 1
ID_2 = 2

# ST3215 角度转换参数
SERVO_DIGITAL_RANGE = 4095.0
SERVO_ANGLE_RANGE = 360.0

# ================= 2. 初始化硬件 =================
portHandler = PortHandler(DEVICENAME)
scs = sts(portHandler)

if portHandler.openPort():
    print(f"成功打开串口: {DEVICENAME}")
else:
    print(f"无法打开串口 {DEVICENAME}，请检查端口号！")
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print(f"成功设置波特率: {BAUDRATE}")
else:
    print("无法设置波特率!")
    portHandler.closePort()
    quit()

# ================= 新增的关键修复代码 =================
print("正在等待 ESP32 开机重启完毕，请稍候...")
time.sleep(2.5)  # 强行等待 2.5 秒，让 ESP32 彻底进入透传状态
print("ESP32 就绪，开始发送指令！")
# ===================================================

# 上电安全机制：释放两个舵机的力矩 (相当于 Arduino 里的 releaseServoTorque)
scs.unLockEprom(ID_1)  # 很多指令需要先解锁
scs.write1ByteTxRx(ID_1, STS_TORQUE_ENABLE, 0)
scs.write1ByteTxRx(ID_2, STS_TORQUE_ENABLE, 0)

print("\n" + "=" * 50)
print("开始读取舵机状态 (刷屏模式，确保PyCharm能显示)")
print("你可以用手转动舵机观察数据变化。按 Ctrl+C 退出...")
print("=" * 50 + "\n")

# ================= 3. 主循环读取 =================
try:
    start_time = time.time()

    while True:
        current_time_ms = int((time.time() - start_time) * 1000)

        # --- 抓取 1 号舵机数据 ---
        pos_1, speed_1, res_1, err_1 = scs.ReadPosSpeed(ID_1)
        # 手动读取 1字节 的电压和温度寄存器 (参考 stservo_def.py 里的地址)
        volt_1, _, _ = scs.read1ByteTxRx(ID_1, STS_PRESENT_VOLTAGE)
        temp_1, _, _ = scs.read1ByteTxRx(ID_1, STS_PRESENT_TEMPERATURE)

        # --- 抓取 2 号舵机数据 ---
        pos_2, speed_2, res_2, err_2 = scs.ReadPosSpeed(ID_2)
        volt_2, _, _ = scs.read1ByteTxRx(ID_2, STS_PRESENT_VOLTAGE)
        temp_2, _, _ = scs.read1ByteTxRx(ID_2, STS_PRESENT_TEMPERATURE)

        # --- 格式化打印 1 号舵机 ---
        if res_1 == COMM_SUCCESS:
            angle_1 = (pos_1 / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE
            real_volt_1 = volt_1 / 10.0
            print(
                f"Time={current_time_ms:05d}ms | ID={ID_1} | Pos={pos_1:4d} | Angle={angle_1:6.2f}° | Speed={speed_1:4d} | Volt={real_volt_1:4.1f}V | Temp={temp_1:2d}C")
        else:
            # print(f"Time={current_time_ms:05d}ms | ID={ID_1} | 通信失败/掉线 (错误码: {err_1})")
            # 使用 scs.getTxRxResult 把底层的 -6, -7 等通信代号翻译成人话
            print(f"Time={current_time_ms:05d}ms | ID={ID_1} | {scs.getTxRxResult(res_1)}")

        # --- 格式化打印 2 号舵机 ---
        if res_2 == COMM_SUCCESS:
            angle_2 = (pos_2 / SERVO_DIGITAL_RANGE) * SERVO_ANGLE_RANGE
            real_volt_2 = volt_2 / 10.0
            print(
                f"Time={current_time_ms:05d}ms | ID={ID_2} | Pos={pos_2:4d} | Angle={angle_2:6.2f}° | Speed={speed_2:4d} | Volt={real_volt_2:4.1f}V | Temp={temp_2:2d}C")
        else:
            # print(f"Time={current_time_ms:05d}ms | ID={ID_2} | 通信失败/掉线 (错误码: {err_2})")
            # 使用 scs.getTxRxResult 把底层的 -6, -7 等通信代号翻译成人话
            print(f"Time={current_time_ms:05d}ms | ID={ID_2} | {scs.getTxRxResult(res_2)}")

        print("-" * 60)  # 打印分割线，像 Arduino 那样

        # 延时 100ms (0.1秒)，与你的 Arduino 脚本频率保持一致
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\n检测到 Ctrl+C，正在安全退出...")

# ================= 4. 收尾清理 =================
portHandler.closePort()
print("串口已关闭。")
