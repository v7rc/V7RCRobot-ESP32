#pragma once

#include <Arduino.h>

struct V7RCRobotArmServoConfig {
  uint8_t pin;
  bool invert;
};

struct V7RCRobotArmPose {
  float baseDeg;
  float shoulderDeg;
  float elbowDeg;
  float wristDeg;
  float gripperDeg;
};

struct V7RCRobotArmRuntimeOptions {
  V7RCRobotArmServoConfig base;
  V7RCRobotArmServoConfig shoulder;
  V7RCRobotArmServoConfig elbow;
  V7RCRobotArmServoConfig wrist;
  V7RCRobotArmServoConfig gripper;
  uint16_t servoMinUs;
  uint16_t servoMaxUs;
};
