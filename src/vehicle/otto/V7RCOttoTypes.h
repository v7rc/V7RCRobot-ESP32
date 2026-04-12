#pragma once

#include <Arduino.h>

struct V7RCOttoServoConfig {
  uint8_t pin;
  bool invert;
};

struct V7RCOttoControlState {
  float walk;
  float turn;
  float bounce;
};

struct V7RCOttoRuntimeOptions {
  V7RCOttoServoConfig leftHip;
  V7RCOttoServoConfig rightHip;
  V7RCOttoServoConfig leftFoot;
  V7RCOttoServoConfig rightFoot;
  uint16_t servoMinUs;
  uint16_t servoMaxUs;
  float hipAmplitudeDeg;
  float footAmplitudeDeg;
  float gaitCycleMs;
};
