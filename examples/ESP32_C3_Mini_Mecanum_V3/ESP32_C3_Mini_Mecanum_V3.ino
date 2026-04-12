#include <Arduino.h>
#include <V7RCRobot-esp32.h>
#include <esp_mac.h>  // ESP32 SDK for MAC address

V7RCServoDriver v7rc;

// ───── Servo 設定（目前沒有伺服，可之後擴充） ─────
V7RC_ServoConfig myServos[] = {
  // 例：之後想加云台伺服再補
  // { pin, minUs, maxUs, minDeg, maxDeg, initDeg }
  { 7, 1000, 2000, 0.0f, 90.0f, 45.0f },
  { 8, 1000, 2000, 0.0f, 90.0f, 45.0f },
  { 5, 1000, 2000, 0.0f, 90.0f, 45.0f },
  { 6, 1000, 2000, 0.0f, 90.0f, 45.0f }
};
// const uint8_t NUM_SERVOS = sizeof(myServos) / sizeof(myServos[0]);
const uint8_t NUM_SERVOS = 1;

// ───── DC 馬達設定（麥克納姆 4 輪） ─────
// 依照你實際接線修改 pinDir / pinPwm
V7RC_DCMotorConfig myMotors[] = {
  { 20, 21, false }, // 0: Front-Left
  { 10, 0, false }, // 1: Front-Right
  { 1, 2, false }, // 2: Rear-Left
  { 3, 4, false }  // 3: Rear-Right
};
const uint8_t NUM_MOTORS = sizeof(myMotors) / sizeof(myMotors[0]);

// ───── Smooth 參數（如果將來有 Servo） ─────
V7RC_SmoothConfig mySmooth = {
  .maxStepDeg  = 45.0f,
  .deadbandDeg = 2.0f,
  .updateMs    = 10
};

// ───── Drive 設定：麥克納姆 ─────
V7RC_DriveConfig myDriveCfg = {
  .type          = DRIVE_MECANUM,
  .diffLeftMotor = -1,
  .diffRightMotor= -1,
  .mecFrontLeft  = 0,  // dcMotors[0]
  .mecFrontRight = 1,  // dcMotors[1]
  .mecRearLeft   = 2,  // dcMotors[2]
  .mecRearRight  = 3   // dcMotors[3]
};

// ───── Channel Map：適用麥克納姆車 ─────
// Channel 說明：
// ch0 → CH_DRIVE_MEC_VX    （前後速度）
// ch1 → CH_DRIVE_MEC_VY    （左右平移）
// ch2 → CH_DRIVE_MEC_OMEGA （旋轉）
// 其他 Channel 目前不使用，可之後擴充成 Servo / DC_MOTOR 等
// ───── Channel Map：適用麥克納姆車 ─────
// 建議先依照「你在 V7RC APP 的搖桿定義」對應：
//   CH_DRIVE_MEC_VX    ：前後 (Forward/Backward)   範圍 -1..+1
//   CH_DRIVE_MEC_VY    ：左右 (Strafe Left/Right)  範圍 -1..+1
//   CH_DRIVE_MEC_OMEGA ：旋轉 (Yaw)                範圍 -1..+1
//
// 若你遇到「Y 變 X / X 變 Y」：請交換 ch0/ch1 的角色即可。
//
// 此範例預設：
//   ch0 → VX, ch1 → VY, ch2 → OMEGA
// 其餘 Channel 先不使用（CH_NONE），之後可擴充成 Servo / 直接 DC motor 控制。
V7RC_ChannelConfig myChannelMap[] = {
  { CH_DRIVE_MEC_VX,    -1 }, // ch0
  { CH_DRIVE_MEC_VY,    -1 }, // ch1
  { CH_SERVO,           0  }, // ch2
  { CH_DRIVE_MEC_OMEGA, -1 }, // ch3
  { CH_NONE,            -1 }, // ch4
  { CH_NONE,            -1 }, // ch5
  { CH_NONE,            -1 }, // ch6
  { CH_NONE,            -1 }, // ch7
  { CH_NONE,            -1 }, // ch8
  { CH_NONE,            -1 }, // ch9
  { CH_NONE,            -1 }, // ch10
  { CH_NONE,            -1 }, // ch11
  { CH_NONE,            -1 }, // ch12
  { CH_NONE,            -1 }, // ch13
  { CH_NONE,            -1 }, // ch14
  { CH_NONE,            -1 }  // ch15
};


const uint8_t NUM_CHANNEL_MAP = sizeof(myChannelMap) / sizeof(myChannelMap[0]);

// ───── Driver 總設定 ─────
V7RC_DriverConfig driverCfg = {
  .bleBaseName        = "V7RCDOM",
  .servos             = myServos,
  .numServos          = NUM_SERVOS,
  .smooth             = mySmooth,
  .waveDemoServoIndex = -1,
  .dcMotors           = myMotors,
  .numDCMotors        = NUM_MOTORS,
  .drive              = myDriveCfg,
  .channelMap         = myChannelMap,
  .numChannelMap      = NUM_CHANNEL_MAP,

  .ws2812Brightness   = 50, // brightness 0-255 (0→default)
  .ws2812Enable       = true,
  .ws2812Pin          = 8,
  .ws2812Count        = 8
};

// ───── 自動生成 ROBOT_ID 從 BLE MAC 位址最後 4 個 Byte ─────
uint32_t getRobotIdFromMac() {
  uint8_t mac[6];
  
  // 讀取 BLE MAC 位址
  esp_read_mac(mac, ESP_MAC_BT);
  
  // 將最後 4 個 Byte 組合成 uint32_t (mac[2]~mac[5])
  uint32_t robotId = ((uint32_t)mac[4] << 8)  | 
                     (uint32_t)mac[5];
  
  // 輸出到 Serial 供參考
  Serial.printf("BLE MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("Generated ROBOT_ID: %u (0x%08X) from last 4 bytes of MAC\n", robotId, robotId);
  Serial.printf("MAC Last 4 Bytes: %02X:%02X:%02X:%02X\n", mac[2], mac[3], mac[4], mac[5]);
  
  return robotId;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 自動生成 ROBOT_ID
  uint32_t robotId = getRobotIdFromMac();
  
  v7rc.begin(robotId, driverCfg);

  // strip will blink red initially; call v7rc.setAllLeds() to change
  // individual control via LE* protocol is also supported by the lib
}

void loop() {
  v7rc.loop();
}
