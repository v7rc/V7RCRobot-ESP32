#pragma once

#include <Arduino.h>

enum V7RCCarDriveMode : uint8_t {
  V7RC_CAR_DRIVE_DISABLED = 0,
  V7RC_CAR_DRIVE_DIFFERENTIAL,
  V7RC_CAR_DRIVE_MECANUM
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
