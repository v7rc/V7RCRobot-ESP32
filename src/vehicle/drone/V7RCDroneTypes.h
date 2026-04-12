#pragma once

#include <Arduino.h>

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
  uint8_t motorPins[4];
  bool stabilizationEnabled;
  float maxTiltDeg;
  float rollKp;
  float pitchKp;
  float yawGain;
  uint16_t escMinUs;
  uint16_t escMaxUs;
  uint16_t escIdleUs;
};
