#include "V7RCCarRobot.h"

V7RCCarRobot::V7RCCarRobot()
  : runtimeMode_(false),
    currentControlState_(V7RCCarControl::neutralState()) {}

void V7RCCarRobot::beginDifferential(uint32_t robotId, const V7RCCarRobotOptions& options, int leftMotorIndex, int rightMotorIndex) {
  runtimeMode_ = false;
  driver_.begin(robotId, runtimeConfig_.buildDifferential(options, leftMotorIndex, rightMotorIndex));
}

void V7RCCarRobot::beginMecanum(
  uint32_t robotId,
  const V7RCCarRobotOptions& options,
  int frontLeftMotorIndex,
  int frontRightMotorIndex,
  int rearLeftMotorIndex,
  int rearRightMotorIndex
) {
  runtimeMode_ = false;
  driver_.begin(
    robotId,
    runtimeConfig_.buildMecanum(
      options,
      frontLeftMotorIndex,
      frontRightMotorIndex,
      rearLeftMotorIndex,
      rearRightMotorIndex
    )
  );
}

void V7RCCarRobot::beginDifferentialRuntime(const V7RCCarRobotOptions& options, int leftMotorIndex, int rightMotorIndex) {
  runtimeMode_ = true;
  currentControlState_ = V7RCCarControl::neutralState();
  runtime_.beginDifferential(options.motors, options.numMotors, leftMotorIndex, rightMotorIndex);
}

void V7RCCarRobot::beginMecanumRuntime(
  const V7RCCarRobotOptions& options,
  int frontLeftMotorIndex,
  int frontRightMotorIndex,
  int rearLeftMotorIndex,
  int rearRightMotorIndex
) {
  runtimeMode_ = true;
  currentControlState_ = V7RCCarControl::neutralState();
  runtime_.beginMecanum(
    options.motors,
    options.numMotors,
    frontLeftMotorIndex,
    frontRightMotorIndex,
    rearLeftMotorIndex,
    rearRightMotorIndex
  );
}

void V7RCCarRobot::loop() {
  if (runtimeMode_) {
    runtime_.apply(currentControlState_);
    return;
  }
  driver_.loop();
}

void V7RCCarRobot::applyControl(const V7RCCarControlState& state) {
  currentControlState_ = state;
  if (runtimeMode_) {
    runtime_.apply(currentControlState_);
  }
}

void V7RCCarRobot::stop() {
  currentControlState_ = V7RCCarControl::neutralState();
  if (runtimeMode_) {
    runtime_.stop();
  }
}
