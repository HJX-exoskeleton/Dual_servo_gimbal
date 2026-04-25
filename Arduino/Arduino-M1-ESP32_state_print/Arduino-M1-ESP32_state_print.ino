#include <SCServo.h>

// ====================== 串口引脚设置 ======================
// 根据你自己的 ESP32 实际接线修改
#define S_RXD 18
#define S_TXD 19

// ====================== 舵机 ID 设置 ======================
#define SERVO1_ID 1
#define SERVO2_ID 2

// 是否读取 2 号舵机
// 只测试 1 号舵机时改成 false
#define USE_SERVO2 true

// ====================== 串口波特率 ======================
#define SERVO_BAUDRATE 1000000
#define USB_BAUDRATE   115200

// ====================== 反馈打印间隔 ======================
#define FEEDBACK_INTERVAL 100   // 单位 ms，100ms 打印一次

// ====================== ST 舵机参数 ======================
// ST/SMS_STS 通常 0~4095 对应 0~360°
float ServoDigitalRange_ST = 4095.0;
float ServoAngleRange_ST   = 360.0;

// ====================== 创建舵机对象 ======================
SMS_STS st;

// ====================== 反馈数据缓存 ======================
s16  loadRead[253];
s16  speedRead[253];
byte voltageRead[253];
int  currentRead[253];
s16  posRead[253];
s16  modeRead[253];
s16  temperRead[253];

// ====================== 时间变量 ======================
unsigned long lastFeedbackTime = 0;


// ====================== 初始化舵机串口 ======================
void servoInit() {
  Serial1.begin(SERVO_BAUDRATE, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(500);

  Serial.println("Servo Serial1 initialized.");
}


// ====================== 读取单个舵机反馈 ======================
bool getFeedBack(byte servoID) {
  if (st.FeedBack(servoID) != -1) {
    posRead[servoID]     = st.ReadPos(-1);
    speedRead[servoID]   = st.ReadSpeed(-1);
    loadRead[servoID]    = st.ReadLoad(-1);
    voltageRead[servoID] = st.ReadVoltage(-1);
    currentRead[servoID] = st.ReadCurrent(-1);
    temperRead[servoID]  = st.ReadTemper(-1);
    modeRead[servoID]    = st.ReadMode(servoID);
    return true;
  } 
  else {
    Serial.print("ID=");
    Serial.print(servoID);
    Serial.println(" feedback failed!");
    return false;
  }
}


// ====================== 打印单个舵机状态 ======================
void printServoStatus(byte servoID) {
  if (getFeedBack(servoID)) {
    float angle = posRead[servoID] / ServoDigitalRange_ST * ServoAngleRange_ST;
    float voltage = voltageRead[servoID] / 10.0;

    Serial.print("Time=");
    Serial.print(millis());
    Serial.print(" ms");

    Serial.print(" | ID=");
    Serial.print(servoID);

    Serial.print(" | Pos=");
    Serial.print(posRead[servoID]);

    Serial.print(" | Angle=");
    Serial.print(angle, 2);
    Serial.print(" deg");

    Serial.print(" | Speed=");
    Serial.print(speedRead[servoID]);

    Serial.print(" | Load=");
    Serial.print(loadRead[servoID]);

    Serial.print(" | Voltage=");
    Serial.print(voltage, 1);
    Serial.print(" V");

    Serial.print(" | Current=");
    Serial.print(currentRead[servoID]);

    Serial.print(" | Temp=");
    Serial.print(temperRead[servoID]);
    Serial.print(" C");

    Serial.print(" | Mode=");
    Serial.println(modeRead[servoID]);
  }
}


// ====================== 打印所有舵机状态 ======================
void printAllServoStatus() {
  printServoStatus(SERVO1_ID);

#if USE_SERVO2
  printServoStatus(SERVO2_ID);
#endif

  Serial.println("----------------------------------------");
}


// ====================== 释放舵机力矩 ======================
void releaseServoTorque() {
  Serial.println("Releasing servo torque...");

  st.EnableTorque(SERVO1_ID, 0);
  delay(100);

#if USE_SERVO2
  st.EnableTorque(SERVO2_ID, 0);
  delay(100);
#endif

  Serial.println("Torque disabled. You can rotate the servo by hand now.");
}


// ====================== setup ======================
void setup() {
  Serial.begin(USB_BAUDRATE);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32 ST/SMS_STS Servo Feedback Monitor");
  Serial.println("No motion command will be sent.");
  Serial.println("Rotate the servo by hand to observe data.");
  Serial.println("========================================");

  servoInit();

  // 上电后关闭力矩，保证舵机不主动运动、不锁死
  releaseServoTorque();

  // 上电先读一次状态
  Serial.println("Initial feedback:");
  printAllServoStatus();
}


// ====================== loop ======================
void loop() {
  unsigned long now = millis();

  if (now - lastFeedbackTime >= FEEDBACK_INTERVAL) {
    lastFeedbackTime = now;

    // 不发送任何运动指令，只读取反馈
    printAllServoStatus();
  }
}
