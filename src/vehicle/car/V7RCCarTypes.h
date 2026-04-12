#pragma once

#include <Arduino.h>
#include "../../legacy/V7RCServoDriver.h"

enum V7RCCarDriveMode : uint8_t {
  V7RC_CAR_DRIVE_DISABLED = 0,
  V7RC_CAR_DRIVE_DIFFERENTIAL,
  V7RC_CAR_DRIVE_MECANUM
};

enum V7RCCarInputAxis : uint8_t {
  V7RC_CAR_INPUT_THROTTLE = 0,
  V7RC_CAR_INPUT_STEER,
  V7RC_CAR_INPUT_VX,
  V7RC_CAR_INPUT_VY,
  V7RC_CAR_INPUT_OMEGA
};

struct V7RCCarControlState {
  float throttle;
  float steer;
  float vx;
  float vy;
  float omega;
};

struct V7RCCarMotorMix {
  float frontLeft;
  float frontRight;
  float rearLeft;
  float rearRight;
};

struct V7RCCarRobotOptions {
  const char* bleBaseName;
  V7RC_DCMotorConfig* motors;
  uint8_t numMotors;
  uint8_t ws2812Brightness;
  bool ws2812Enable;
  uint8_t ws2812Pin;
  uint8_t ws2812Count;
};
