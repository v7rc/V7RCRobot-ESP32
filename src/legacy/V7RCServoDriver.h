#pragma once
#include <Arduino.h>

// WS2812 support
#include <Adafruit_NeoPixel.h>

// ===== Servo 設定 =====
struct V7RC_ServoConfig {
  uint8_t pin;      // Servo 訊號腳位
  uint16_t minUs;   // 對應 minDeg 的 PWM us
  uint16_t maxUs;   // 對應 maxDeg 的 PWM us
  float minDeg;     // 最小角度
  float maxDeg;     // 最大角度
  float initDeg;    // 啟動 / 中立角度
};

// ===== DC 馬達設定 =====
struct V7RC_DCMotorConfig {
  uint8_t pinDir;   // 方向腳位
  uint8_t pinPwm;   // PWM 腳位
  bool dirInvert;   // true 代表這顆馬達方向相反
};

// ===== Smooth 參數 =====
struct V7RC_SmoothConfig {
  float maxStepDeg;   // 每次更新最大變化角度
  float deadbandDeg;  // 小於這個變化就忽略，避免抖動
  uint16_t updateMs;  // 更新週期 (ms)
};

// ===== Drive 類型 =====
enum V7RC_DriveType : uint8_t {
  DRIVE_NONE = 0,
  DRIVE_DIFF,
  DRIVE_MECANUM
};

// Drive 設定：指定哪些 DC Motor 當差速 / 麥克納姆
struct V7RC_DriveConfig {
  V7RC_DriveType type;

  // 差速用
  int diffLeftMotor;    // index in dcMotors[]
  int diffRightMotor;

  // 麥克納姆用
  int mecFrontLeft;
  int mecFrontRight;
  int mecRearLeft;
  int mecRearRight;
};

// ===== Channel 角色 =====
enum V7RC_ChannelRole : uint8_t {
  CH_NONE = 0,          // 不使用
  CH_SERVO,             // 控某顆 Servo
  CH_DC_MOTOR,          // 直接 PWM 控制某顆 DC Motor
  CH_DRIVE_THROTTLE,    // 差速 Throttle  (-1..1)
  CH_DRIVE_STEER,       // 差速 Steer     (-1..1)
  CH_DRIVE_MEC_VX,      // Mecanum 前後   (-1..1)
  CH_DRIVE_MEC_VY,      // Mecanum 左右   (-1..1)
  CH_DRIVE_MEC_OMEGA    // Mecanum 旋轉   (-1..1)
};

// 每個 Channel 的設定
struct V7RC_ChannelConfig {
  V7RC_ChannelRole role;
  int8_t targetIndex;   // CH_SERVO → servo index
                        // CH_DC_MOTOR → dcMotor index
                        // 其他角色填 -1 即可
};

// ===== Driver 總設定 =====
struct V7RC_DriverConfig {
  const char* bleBaseName;      // e.g. "V7RC-ROBOT"

  V7RC_ServoConfig* servos;
  uint8_t numServos;

  V7RC_SmoothConfig smooth;

  int waveDemoServoIndex;       // 可留 -1 不用

  V7RC_DCMotorConfig* dcMotors;
  uint8_t numDCMotors;

  V7RC_DriveConfig drive;

  // Channel Map（建議長度 16）
  V7RC_ChannelConfig* channelMap;
  uint8_t numChannelMap;

  // optional WS2812 RGB LED strip configuration
  // enable: when true the strip will be initialized; default false
  // pin: data pin to use (defaults to 8 if set to zero)
  // count: number of LEDs in the strip (0 defaults to 8)
  // brightness: 0-255 brightness level for all LEDs; if 0 a sane
  //   default (50) is applied.  Only used when ws2812Enable is true.
  uint8_t ws2812Brightness;
  bool    ws2812Enable;
  uint8_t ws2812Pin;
  uint8_t ws2812Count;
};

class V7RCServoDriver {
public:
  // robotId: 1 → <bleBaseName>-01, 2 → <bleBaseName>-02 ...
  void begin(uint32_t robotId, const V7RC_DriverConfig& cfg);
  void loop();

  // ------------ WS2812 RGB LED strip ------------
  // these functions are no-ops if the strip was not initialized
  static void setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
  static void setAllLeds(uint8_t r, uint8_t g, uint8_t b);
};
