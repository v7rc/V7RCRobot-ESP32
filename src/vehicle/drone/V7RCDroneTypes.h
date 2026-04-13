#pragma once

#include <Arduino.h>
#include "../../legacy/V7RCServoDriver.h"

enum V7RCDroneOutputMode : uint8_t {
  V7RC_DRONE_OUTPUT_SERVO_PWM = 0,
  V7RC_DRONE_OUTPUT_DC_MOTOR
};

struct V7RCDroneControlState {
  float throttle;  // 0..1
  float roll;      // -1..1
  float pitch;     // -1..1
  float yaw;       // -1..1
};

struct V7RCDroneAttitude {
  float rollDeg;
  float pitchDeg;
  float yawRateDegPerSec;
  bool valid;
};

struct V7RCDroneMotorMix {
  float frontLeft;
  float frontRight;
  float rearLeft;
  float rearRight;
};

struct V7RCDroneRuntimeOptions {
  V7RCDroneOutputMode outputMode;
  uint8_t motorPins[4];
  V7RC_DCMotorConfig* dcMotors;
  uint8_t numDCMotors;
  bool stabilizationEnabled;
  float maxTiltDeg;
  float rollKp;
  float pitchKp;
  float yawGain;
  float yawRateKp;
  uint16_t escMinUs;
  uint16_t escMaxUs;
  uint16_t escIdleUs;
};
