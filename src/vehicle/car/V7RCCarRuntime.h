#pragma once

#include "../../io/V7RCEsp32Outputs.h"
#include "V7RCCarControl.h"

class V7RCCarRuntime {
public:
  V7RCCarRuntime();

  void beginDifferential(V7RC_DCMotorConfig* motors, uint8_t numMotors, int leftMotorIndex, int rightMotorIndex);
  void beginMecanum(
    V7RC_DCMotorConfig* motors,
    uint8_t numMotors,
    int frontLeftMotorIndex,
    int frontRightMotorIndex,
    int rearLeftMotorIndex,
    int rearRightMotorIndex
  );

  void apply(const V7RCCarControlState& state);
  void stop();

  V7RCCarDriveMode driveMode() const;

private:
  void attachMotorOutputs(V7RC_DCMotorConfig* motors, uint8_t numMotors);
  bool hasMotorIndex(int index) const;

  static const uint8_t kMaxMotors = 8;

  V7RCEsp32DCMotorOutput motorOutputs_[kMaxMotors];
  uint8_t motorCount_;
  V7RCCarDriveMode driveMode_;
  int diffLeftMotorIndex_;
  int diffRightMotorIndex_;
  int mecFrontLeftIndex_;
  int mecFrontRightIndex_;
  int mecRearLeftIndex_;
  int mecRearRightIndex_;
};
