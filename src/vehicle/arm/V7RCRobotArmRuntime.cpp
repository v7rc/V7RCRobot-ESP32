#include "V7RCRobotArmRuntime.h"

namespace {

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

}  // namespace

V7RCRobotArmRuntime::V7RCRobotArmRuntime()
  : started_(false) {}

void V7RCRobotArmRuntime::begin(const V7RCRobotArmRuntimeOptions& options) {
  options_ = options;
  started_ = true;

  base_.attach(options_.base.pin, options_.servoMinUs, options_.servoMaxUs);
  shoulder_.attach(options_.shoulder.pin, options_.servoMinUs, options_.servoMaxUs);
  elbow_.attach(options_.elbow.pin, options_.servoMinUs, options_.servoMaxUs);
  wrist_.attach(options_.wrist.pin, options_.servoMinUs, options_.servoMaxUs);
  gripper_.attach(options_.gripper.pin, options_.servoMinUs, options_.servoMaxUs);

  home();
}

void V7RCRobotArmRuntime::moveTo(const V7RCRobotArmPose& pose) {
  if (!started_) return;

  base_.writeMicroseconds(angleToPulse(pose.baseDeg, options_.base.invert));
  shoulder_.writeMicroseconds(angleToPulse(pose.shoulderDeg, options_.shoulder.invert));
  elbow_.writeMicroseconds(angleToPulse(pose.elbowDeg, options_.elbow.invert));
  wrist_.writeMicroseconds(angleToPulse(pose.wristDeg, options_.wrist.invert));
  gripper_.writeMicroseconds(angleToPulse(pose.gripperDeg, options_.gripper.invert));
}

void V7RCRobotArmRuntime::home() {
  moveTo({0.0f, -20.0f, 35.0f, 0.0f, -30.0f});
}

uint16_t V7RCRobotArmRuntime::angleToPulse(float angleDeg, bool inverted) const {
  float normalized = (clampFloat(angleDeg, -90.0f, 90.0f) + 90.0f) / 180.0f;
  if (inverted) {
    normalized = 1.0f - normalized;
  }
  return (uint16_t)(options_.servoMinUs + (normalized * (float)(options_.servoMaxUs - options_.servoMinUs)));
}
