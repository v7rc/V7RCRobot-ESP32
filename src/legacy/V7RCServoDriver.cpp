#include "V7RCServoDriver.h"
#include "../vehicle/car/V7RCCarControl.h"
#include "../core/V7RCRuntimeState.h"
#include "../protocol/V7RCProtocol.h"
#include "../transport/V7RCBleTransportEsp32.h"

#include <ESP32Servo.h>
#include <math.h>

// WS2812 (NeoPixel) support
#include <Adafruit_NeoPixel.h>

static const int V7RC_NUM_CHANNELS = V7RCProtocolDecoder::kNumChannels;
static const int V7RC_FRAME_SIZE = V7RCProtocolDecoder::kBinaryFrameSize;

// ===== 全域 Driver 狀態 =====
static V7RC_ServoConfig*   g_servos      = nullptr;
static uint8_t             g_numServos   = 0;
static V7RC_DCMotorConfig* g_dcMotors    = nullptr;
static uint8_t             g_numDCMotors = 0;

static V7RC_SmoothConfig   g_smooth      = {3.0f, 0.8f, 10};
static int                 g_waveDemoServoIndex = -1;

static V7RC_ChannelConfig* g_channelMap   = nullptr;
static uint8_t             g_numChannelMap= 0;

static V7RC_DriveConfig    g_driveCfg;
static bool                g_driveEnabled = false;
static V7RCCarControlState g_carControlState = V7RCCarControl::neutralState();
static V7RCRuntimeState    g_runtimeState;

// WS2812 globals
static Adafruit_NeoPixel*  g_ws2812       = nullptr;
static uint8_t             g_ws2812Count  = 0;

// per-LED custom state (set by new LE* protocol)
struct LedState {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t blinkLevel; // 0..10 (100ms units)
};
static LedState*          g_ledStates       = nullptr;
static bool               g_ledCustomActive = false;

// LED status for connection animation (used when no custom active)
static bool               g_ledBlinkState    = false;
static unsigned long      g_ledLastToggleMs  = 0;

// Servo 物件與狀態
static Servo* g_servoObjs  = nullptr;
static float* g_currentDeg = nullptr;
static float* g_targetDeg  = nullptr;

static char                  g_bleDeviceName[24] = "V7RC-ROBOT-01";
static bool                  deviceConnected     = false;
static V7RCBleTransportEsp32 g_bleTransport;

// Binary Frame 接收 buffer
static char rxBuf[V7RC_FRAME_SIZE];
static int  rxIndex = 0;

// 心跳 / 逾時
static const unsigned long COMMAND_TIMEOUT_MS = 1000;

// ===== 工具函式 =====
static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (fabsf(in_max - in_min) < 1e-6f) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return 0;
}

// 角度 ↔ us
static uint16_t degToUs(const V7RC_ServoConfig &cfg, float deg) {
  float d  = clampFloat(deg, cfg.minDeg, cfg.maxDeg);
  float us = mapFloat(d, cfg.minDeg, cfg.maxDeg, cfg.minUs, cfg.maxUs);
  if (us < cfg.minUs) us = cfg.minUs;
  if (us > cfg.maxUs) us = cfg.maxUs;
  return (uint16_t)us;
}

static float usToDeg(const V7RC_ServoConfig &cfg, float us) {
  float u = clampFloat(us, cfg.minUs, cfg.maxUs);
  float d = mapFloat(u, cfg.minUs, cfg.maxUs, cfg.minDeg, cfg.maxDeg);
  return clampFloat(d, cfg.minDeg, cfg.maxDeg);
}

// DC 馬達：PWM(us) → DIR/PWM 輸出
static void processDCMotorPwm(uint16_t pwmUs, const V7RC_DCMotorConfig& m) {
  if (pwmUs < 1000) pwmUs = 1000;
  if (pwmUs > 2000) pwmUs = 2000;

  if (pwmUs == 1500) {
    digitalWrite(m.pinDir, LOW);
    analogWrite(m.pinPwm, 0);
  } else if (pwmUs > 1500) {
    int power = map(pwmUs, 1500, 2000, 0 , 255);
    digitalWrite(m.pinDir, LOW);
    analogWrite(m.pinPwm, power);
  } else {
    int power = map(pwmUs, 1500, 1000, 255 , 0);
    digitalWrite(m.pinDir, HIGH);
    analogWrite(m.pinPwm, power);
  }
}

// ===== BLE TX =====
static void sendTxLine(const String &line) {
  if (!deviceConnected) return;
  g_bleTransport.send((const uint8_t*)line.c_str(), (size_t)line.length());
}

static void applyLegacyFrameDefaults(V7RC_Frame* frame) {
  if (!frame) return;

  for (int ch = 0; ch < V7RC_NUM_CHANNELS; ch++) {
    if (frame->channelPresent[ch]) continue;

    uint16_t us = 1500;

    if (g_channelMap && ch < g_numChannelMap) {
      V7RC_ChannelConfig &cc = g_channelMap[ch];
      if (cc.role == CH_SERVO &&
          cc.targetIndex >= 0 &&
          cc.targetIndex < (int)g_numServos &&
          g_servos) {
        const V7RC_ServoConfig &cfg = g_servos[cc.targetIndex];
        us = degToUs(cfg, cfg.initDeg);
      }
    }

    frame->values[ch] = (int16_t)us;
  }
}

static void driveDCMotorNorm(float v, const V7RC_DCMotorConfig& m) {
  v = constrain(v, -1.0f, 1.0f);
  // 可依你的遙控抖動調大到 0.03~0.06
  const float dead = 0.03f;

  if (fabs(v) < dead) {
    analogWrite(m.pinPwm, 0);
    // 建議：停車時 DIR 固定到一個狀態，避免某些驅動板在另一狀態是 brake
    digitalWrite(m.pinDir, m.dirInvert ? HIGH : LOW);
    return;
  }

  int power = (int)(fabs(v) * 255.0f);

  if(v < 0) {
    
     power = (int)(fabs(1.0f + v) * 255.0f); 

  }

  power = constrain(power, 0, 255);

    bool dirLevel = (v > 0) ? LOW : HIGH;
    if (m.dirInvert) dirLevel = !dirLevel;

    digitalWrite(m.pinDir, dirLevel);
    analogWrite(m.pinPwm, power);
}

static void driveDCMotorNorm_IN1IN2(float v, const V7RC_DCMotorConfig& m) {
  v = constrain(v, -1.0f, 1.0f);
  const float dead = 0.03f;

  int power = (int)(fabs(v) * 255.0f);
  power = constrain(power, 0, 255);

  if (fabs(v) < dead) {
    // Coast：兩腳都 LOW（不煞車）
    digitalWrite(m.pinDir, LOW);
    digitalWrite(m.pinPwm, LOW);
    return;
  }

  // 可選：最低起轉力，避免低速卡住
  const int minPwm = 35;
  if (power > 0 && power < minPwm) power = minPwm;

  bool forward = (v > 0);
  if (m.dirInvert) forward = !forward;

  if (forward) {
    // IN1 = PWM, IN2 = LOW
    analogWrite(m.pinDir, power);
    digitalWrite(m.pinPwm, LOW);
  } else {
    // IN2 = PWM, IN1 = LOW
    analogWrite(m.pinPwm, power);
    digitalWrite(m.pinDir, LOW);
  }
}

// ===== 把 Frame 套用到 Servo / Motor / Drive =====
static void applyFrameToTargets(const V7RC_Frame &frame) {
  if (!frame.valid) return;

  bool anyDriveChannelUsed = false;
  unsigned long now = millis();

  int numCh = g_numChannelMap;
  if (numCh > V7RC_NUM_CHANNELS) numCh = V7RC_NUM_CHANNELS;

  for (int ch = 0; ch < numCh; ch++) {
    V7RC_ChannelConfig &cc = g_channelMap[ch];
    int16_t v = frame.values[ch];
    g_runtimeState.setChannel((uint8_t)ch, v);

    switch (cc.role) {
      case CH_NONE:
        break;

      case CH_SERVO: {
        int idx = cc.targetIndex;
        if (idx < 0 || idx >= (int)g_numServos || !g_servos) break;
        const V7RC_ServoConfig &cfg = g_servos[idx];

        if (frame.type == V7RC_HEX ||
            frame.type == V7RC_SRV ||
            frame.type == V7RC_SS8) {
          float deg = usToDeg(cfg, (float)v);
          g_targetDeg[idx] = deg;
        } else if (frame.type == V7RC_DEG) {
          float deg = (float)v;
          deg = clampFloat(deg, cfg.minDeg, cfg.maxDeg);
          g_targetDeg[idx] = deg;
        }
        break;
      }

      case CH_DC_MOTOR: {
        int midx = cc.targetIndex;
        if (midx < 0 || midx >= (int)g_numDCMotors || !g_dcMotors) break;

        if (frame.type == V7RC_HEX ||
            frame.type == V7RC_SRV ||
            frame.type == V7RC_SS8) {
          uint16_t pwmUs = (uint16_t)v;
          processDCMotorPwm(pwmUs, g_dcMotors[midx]);
        }
        break;
      }

      case CH_DRIVE_THROTTLE:
      case CH_DRIVE_STEER:
      case CH_DRIVE_MEC_VX:
      case CH_DRIVE_MEC_VY:
      case CH_DRIVE_MEC_OMEGA: {
        float s = V7RCCarControl::normalizedInputFromFrameValue(frame.type, v);

        if (cc.role == CH_DRIVE_THROTTLE) {
          V7RCCarControl::applyInput(V7RC_CAR_INPUT_THROTTLE, s, &g_carControlState);
        } else if (cc.role == CH_DRIVE_STEER) {
          V7RCCarControl::applyInput(V7RC_CAR_INPUT_STEER, s, &g_carControlState);
        } else if (cc.role == CH_DRIVE_MEC_VX) {
          V7RCCarControl::applyInput(V7RC_CAR_INPUT_VX, s, &g_carControlState);
        } else if (cc.role == CH_DRIVE_MEC_VY) {
          V7RCCarControl::applyInput(V7RC_CAR_INPUT_VY, s, &g_carControlState);
        } else if (cc.role == CH_DRIVE_MEC_OMEGA) {
          V7RCCarControl::applyInput(V7RC_CAR_INPUT_OMEGA, s, &g_carControlState);
        }

        anyDriveChannelUsed = true;
        break;
      }
    }
  }

  if (anyDriveChannelUsed) {
    g_driveEnabled = true;
    if (g_driveCfg.type == DRIVE_NONE) {
      g_driveCfg.type = DRIVE_MECANUM;
    }
  }

  g_runtimeState.markFrameReceived(now);
}

// ===== Drive 更新（差速 / 麥克納姆） =====
static void updateDrive() {
  if (!g_driveEnabled) return;
  if (!g_dcMotors || g_numDCMotors == 0) return;

  if (g_driveCfg.type == DRIVE_DIFF) {
    int li = g_driveCfg.diffLeftMotor;
    int ri = g_driveCfg.diffRightMotor;
    if (li < 0 || ri < 0 || li >= (int)g_numDCMotors || ri >= (int)g_numDCMotors) return;

    V7RCCarMotorMix mix = V7RCCarControl::mixDifferential(g_carControlState.throttle, g_carControlState.steer);
    driveDCMotorNorm(mix.frontLeft, g_dcMotors[li]);
    driveDCMotorNorm(mix.frontRight, g_dcMotors[ri]);
  }
  else if (g_driveCfg.type == DRIVE_MECANUM) {
    int fl = g_driveCfg.mecFrontLeft;
    int fr = g_driveCfg.mecFrontRight;
    int rl = g_driveCfg.mecRearLeft;
    int rr = g_driveCfg.mecRearRight;


    if (fl < 0 || fr < 0 || rl < 0 || rr < 0) return;
    if (fl >= (int)g_numDCMotors || fr >= (int)g_numDCMotors ||
        rl >= (int)g_numDCMotors || rr >= (int)g_numDCMotors) return;

    V7RCCarMotorMix mix = V7RCCarControl::mixMecanum(g_carControlState.vx, g_carControlState.vy, g_carControlState.omega);
    driveDCMotorNorm(mix.frontLeft, g_dcMotors[fl]);
    driveDCMotorNorm(mix.frontRight, g_dcMotors[fr]);
    driveDCMotorNorm(mix.rearLeft, g_dcMotors[rl]);
    driveDCMotorNorm(mix.rearRight, g_dcMotors[rr]);
  }
}

// ===== Servo Smooth 更新 =====
static void updateServosSmooth() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  lastUpdate = now;

  if (!g_servos || !g_servoObjs) return;

  for (int i = 0; i < (int)g_numServos; i++) {
    const V7RC_ServoConfig &cfg = g_servos[i];

    float delta = g_targetDeg[i] - g_currentDeg[i];
    if (fabsf(delta) < g_smooth.deadbandDeg) continue;

    float step = g_smooth.maxStepDeg;
    if (fabsf(delta) <= step) {
      g_currentDeg[i] = g_targetDeg[i];
    } else {
      g_currentDeg[i] += (delta > 0 ? step : -step);
    }

    uint16_t us = degToUs(cfg, g_currentDeg[i]);
    g_servoObjs[i].writeMicroseconds(us);
  }
}

// ===== 安全保護（逾時停車 + Servo 回中立） =====
// ---------- WS2812 helpers ----------

static void setWs2812Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  if (!g_ws2812 || index >= g_ws2812Count) return;
  g_ws2812->setPixelColor(index, g_ws2812->Color(r, g, b));
  g_ws2812->show();
}

static void setWs2812All(uint8_t r, uint8_t g, uint8_t b) {
  if (!g_ws2812) return;
  for (uint8_t i = 0; i < g_ws2812Count; i++) {
    g_ws2812->setPixelColor(i, g_ws2812->Color(r, g, b));
  }
  g_ws2812->show();
}

// public wrappers exposed via V7RCServoDriver class
void V7RCServoDriver::setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  setWs2812Color(index, r, g, b);
}

void V7RCServoDriver::setAllLeds(uint8_t r, uint8_t g, uint8_t b) {
  setWs2812All(r, g, b);
}

static void safetyStop() {
  // Servo 回 initDeg
  if (g_servos && g_servoObjs && g_currentDeg && g_targetDeg) {
    for (int i = 0; i < (int)g_numServos; i++) {
      g_targetDeg[i]  = g_servos[i].initDeg;
      g_currentDeg[i] = g_servos[i].initDeg;
      uint16_t us = degToUs(g_servos[i], g_servos[i].initDeg);
      g_servoObjs[i].writeMicroseconds(us);
    }
  }

  // DC Motor 停車
  if (g_dcMotors) {
    for (int i = 0; i < (int)g_numDCMotors; i++) {
      processDCMotorPwm(1500, g_dcMotors[i]);
    }
  }

  // turn off WS2812 leds if any
  if (g_ws2812) {
    setWs2812All(0, 0, 0);
  }
  // clear custom state so connection animation resumes next time
  g_ledCustomActive = false;

  g_driveEnabled = false;
  g_carControlState = V7RCCarControl::neutralState();
}

static void handleTimeoutSafety() {
  unsigned long now = millis();
  if (!g_runtimeState.hasFrame()) return;
  if (!g_runtimeState.signalValid(now, COMMAND_TIMEOUT_MS)) {
    safetyStop();
    g_runtimeState.reset();
    Serial.println("Timeout → safety stop");
  }
}

// update LED animation (connection state or custom per-led settings)
static void updateLedAnimation() {
  if (!g_ws2812) return;

  if (g_ledCustomActive) {
    uint32_t now = millis();
    for (uint8_t i = 0; i < g_ws2812Count; i++) {
      LedState &s = g_ledStates[i];
      uint8_t r = 0, g = 0, b = 0;
      if (s.blinkLevel > 0) {
        uint32_t period = 1000;
        uint32_t onTime = s.blinkLevel * 100;
        if (onTime >= period) {
          r = s.r; g = s.g; b = s.b;
        } else {
          uint32_t phase = now % period;
          if (phase < onTime) {
            r = s.r; g = s.g; b = s.b;
          }
        }
      }
      g_ws2812->setPixelColor(i, g_ws2812->Color(r, g, b));
    }
    g_ws2812->show();
  } else {
    if (!deviceConnected) {
      unsigned long now = millis();
      if (now - g_ledLastToggleMs >= 1000) {
        g_ledBlinkState = !g_ledBlinkState;
        g_ledLastToggleMs = now;
        if (g_ledBlinkState) {
          setWs2812All(255, 0, 0);
        } else {
          setWs2812All(0, 0, 0);
        }
      }
    } else {
      setWs2812All(0, 255, 0);
    }
  }
}

// ===== Frame Byte 流處理（共用 Serial / BLE） =====
static void handleLedCommand(const char *cmd) {
  // cmd does not include trailing '#'
  int len = strlen(cmd);
  if (len < 3) return;
  if (cmd[0] != 'L' || cmd[1] != 'E') return;
  char grpChar = cmd[2];
  int group;
  // LED or LE1 = group0 (first four leds)
  if (grpChar == 'D' || grpChar == '1') {
    group = 0;
  } else if (grpChar >= '2' && grpChar <= '9') {
    // LE2->group1, LE3->group2, ...
    group = grpChar - '1';
  } else {
    return;
  }
  if (!g_ws2812 || g_ws2812Count == 0) return;
  int start = group * 4;
  if (start >= g_ws2812Count) return;

  const char *p = cmd + 3;
  for (int led = 0; led < 4; led++) {
    int idx = start + led;
    if (idx >= g_ws2812Count) break;
    if ((int)(p - cmd) + 4 > len) break;
    uint8_t r = hexNibble(p[0]) * 17;
    uint8_t g = hexNibble(p[1]) * 17;
    uint8_t b = hexNibble(p[2]) * 17;
    uint8_t blink = hexNibble(p[3]);
    if (blink > 10) blink = 10;
    g_ledStates[idx].r = r;
    g_ledStates[idx].g = g;
    g_ledStates[idx].b = b;
    g_ledStates[idx].blinkLevel = blink;
    p += 4;
  }

  g_ledCustomActive = true;
}

static char ledBuf[64];
static int  ledIndex = 0;

static void processIncomingByte(char c) {
  if (c == '#') {
    // finalize text buffer
    if (ledIndex < (int)sizeof(ledBuf) - 1) {
      ledBuf[ledIndex] = '#';
      ledBuf[ledIndex + 1] = '\0';
    } else {
      ledBuf[0] = '\0';
    }

    // try to decode as frame (including LED/LE1..LE4)
    V7RC_Frame frame = V7RCProtocolDecoder::decode(ledBuf, (size_t)(ledIndex + 1));
    if (frame.valid) {
      if (frame.type == V7RC_LED) {
        handleLedCommand(ledBuf);
      } else {
        applyLegacyFrameDefaults(&frame);
        applyFrameToTargets(frame);
      }
    }

    // reset all indices
    ledIndex = 0;
    rxIndex  = 0;
  } else {
    // accumulate both raw bytes and text buffer
    if (rxIndex < V7RC_FRAME_SIZE - 1) {
      rxBuf[rxIndex++] = c;
    } else {
      rxIndex = 0;  // corrupted frame, restart
    }
    if (ledIndex < (int)(sizeof(ledBuf) - 1)) {
      ledBuf[ledIndex++] = c;
    } else {
      ledIndex = 0;
    }
  }
}

static void processSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    processIncomingByte(c);
  }
}

static void handleTransportByte(uint8_t byte, void* context) {
  (void)context;
  processIncomingByte((char)byte);
}

static void handleTransportConnectionChanged(bool connected, void* context) {
  (void)context;
  deviceConnected = connected;

  if (connected) {
    g_runtimeState.reset();
    Serial.println("BLE connected");
    if (g_ws2812) {
      setWs2812All(0, 255, 0);
    }
    return;
  }

  Serial.println("BLE disconnected");
  safetyStop();
  if (g_ws2812) {
    g_ledBlinkState = false;
    g_ledLastToggleMs = millis();
    setWs2812All(0, 0, 0);
  }
}

static void setupBLE() {
  g_bleTransport.setByteHandler(handleTransportByte, nullptr);
  g_bleTransport.setConnectionHandler(handleTransportConnectionChanged, nullptr);
  g_bleTransport.begin(g_bleDeviceName);

  Serial.print("BLE advertising as: ");
  Serial.println(g_bleDeviceName);
}

// ===== Servo / DC 初始化 =====
static void initServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  if (!g_servos || g_numServos == 0) return;

  g_servoObjs  = new Servo[g_numServos];
  g_currentDeg = new float[g_numServos];
  g_targetDeg  = new float[g_numServos];

  for (int i = 0; i < (int)g_numServos; i++) {
    const V7RC_ServoConfig &cfg = g_servos[i];

    g_servoObjs[i].setPeriodHertz(50);
    g_servoObjs[i].attach(cfg.pin, cfg.minUs, cfg.maxUs);

    g_currentDeg[i] = cfg.initDeg;
    g_targetDeg[i]  = cfg.initDeg;

    uint16_t us = degToUs(cfg, cfg.initDeg);
    g_servoObjs[i].writeMicroseconds(us);

    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(" on pin ");
    Serial.print(cfg.pin);
    Serial.print(", initDeg=");
    Serial.print(cfg.initDeg);
    Serial.print(" (");
    Serial.print(us);
    Serial.println(" us)");
  }
}

static void initDCMotors() {
  if (!g_dcMotors || g_numDCMotors == 0) return;

  for (int i = 0; i < (int)g_numDCMotors; i++) {
    pinMode(g_dcMotors[i].pinDir, OUTPUT);
    pinMode(g_dcMotors[i].pinPwm, OUTPUT);
    digitalWrite(g_dcMotors[i].pinDir, LOW);
    analogWrite(g_dcMotors[i].pinPwm, 0);
  }
}

// ===== V7RCServoDriver 實作 =====
void V7RCServoDriver::begin(uint32_t robotId, const V7RC_DriverConfig& cfg) {
  if (robotId < 1)  robotId = 1;
  // if (robotId > 99) robotId = 99;

  g_servos      = cfg.servos;
  g_numServos   = cfg.numServos;
  g_smooth      = cfg.smooth;
  g_waveDemoServoIndex = cfg.waveDemoServoIndex;

  g_dcMotors    = cfg.dcMotors;
  g_numDCMotors = cfg.numDCMotors;

  g_channelMap   = cfg.channelMap;
  g_numChannelMap= cfg.numChannelMap;

  g_driveCfg     = cfg.drive;
  g_driveEnabled = false;
  g_carControlState = V7RCCarControl::neutralState();
  g_runtimeState.reset();

  snprintf(g_bleDeviceName, sizeof(g_bleDeviceName),
           "%s-%02u", cfg.bleBaseName ? cfg.bleBaseName : "V7RC", robotId);

  Serial.println();
  Serial.print("V7RC Servo Driver Begin, BLE Name = ");
  Serial.println(g_bleDeviceName);

  initServos();
  initDCMotors();
  setupBLE();

  // optional WS2812 initialization
  if (cfg.ws2812Enable) {
    uint8_t count = cfg.ws2812Count;
    if (count == 0) {
      count = 8; // default count
    }
    uint8_t pin = cfg.ws2812Pin ? cfg.ws2812Pin : 8;
    g_ws2812Count = count;
    g_ws2812 = new Adafruit_NeoPixel(g_ws2812Count, pin, NEO_GRB + NEO_KHZ800);
    g_ws2812->begin();

    // apply brightness (user-specified, or fallback default)
    uint8_t br = cfg.ws2812Brightness;
    if (br == 0) {
      br = 50; // sensible default to avoid full power
    }
    g_ws2812->setBrightness(br);

    // allocate state array
    g_ledStates = new LedState[g_ws2812Count];
    for (uint8_t i = 0; i < g_ws2812Count; i++) {
      g_ledStates[i].r = 255;
      g_ledStates[i].g = 0;
      g_ledStates[i].b = 0;
      g_ledStates[i].blinkLevel = 5; // default blink 500ms on
      g_ws2812->setPixelColor(i, g_ws2812->Color(255, 0, 0));
    }
    g_ws2812->show();
    // initialize blink timing
    g_ledBlinkState   = true;
    g_ledLastToggleMs = millis();
  }
}

void V7RCServoDriver::loop() {
  unsigned long startTime = millis();

  g_bleTransport.poll();
  processSerial();
  updateDrive();
  updateServosSmooth();
  handleTimeoutSafety();
  updateLedAnimation();

  unsigned long endTime = millis();
  int waitTime = (int)g_smooth.updateMs - (int)(endTime - startTime);
  if (waitTime > 2) {
    delay(waitTime - 1);
  }
}
