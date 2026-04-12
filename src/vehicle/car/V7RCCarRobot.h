#pragma once

#include "../../legacy/V7RCServoDriver.h"
#include "V7RCCarControl.h"
#include "V7RCCarRuntime.h"
#include "V7RCCarRuntimeConfig.h"

class V7RCCarRobot {
public:
  V7RCCarRobot();

  void beginDifferential(uint32_t robotId, const V7RCCarRobotOptions& options, int leftMotorIndex, int rightMotorIndex);
  void beginMecanum(
    uint32_t robotId,
    const V7RCCarRobotOptions& options,
    int frontLeftMotorIndex,
    int frontRightMotorIndex,
    int rearLeftMotorIndex,
    int rearRightMotorIndex
  );

  void beginDifferentialRuntime(const V7RCCarRobotOptions& options, int leftMotorIndex, int rightMotorIndex);
  void beginMecanumRuntime(
    const V7RCCarRobotOptions& options,
    int frontLeftMotorIndex,
    int frontRightMotorIndex,
    int rearLeftMotorIndex,
    int rearRightMotorIndex
  );

  void loop();
  void applyControl(const V7RCCarControlState& state);
  void stop();

private:
  bool runtimeMode_;
  V7RCCarControlState currentControlState_;
  V7RCServoDriver driver_;
  V7RCCarRuntime runtime_;
  V7RCCarRuntimeConfig runtimeConfig_;
};
