#pragma once

#include <Arduino.h>

struct V7RCQuadrupedControlState {
  float vx;
  float vy;
  float yaw;
  float bodyHeight;
};

struct V7RCQuadrupedFootPoint {
  float x;
  float z;
};

struct V7RCQuadrupedLegIk {
  float hipDeg;
  float kneeDeg;
};

struct V7RCQuadrupedServoPair {
  uint8_t hipPin;
  uint8_t kneePin;
  bool hipInvert;
  bool kneeInvert;
};

struct V7RCQuadrupedRuntimeOptions {
  V7RCQuadrupedServoPair legs[4];
  float upperLegLength;
  float lowerLegLength;
  float nominalX;
  float nominalBodyHeight;
  float stepHeight;
  float stepLengthScale;
  float gaitCycleMs;
  uint16_t servoMinUs;
  uint16_t servoMaxUs;
};
