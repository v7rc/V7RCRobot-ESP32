#include "V7RCCarRuntime.h"

V7RCCarRuntime::V7RCCarRuntime()
  : motorCount_(0),
    driveMode_(V7RC_CAR_DRIVE_DISABLED),
    diffLeftMotorIndex_(-1),
    diffRightMotorIndex_(-1),
    mecFrontLeftIndex_(-1),
    mecFrontRightIndex_(-1),
    mecRearLeftIndex_(-1),
    mecRearRightIndex_(-1) {}

void V7RCCarRuntime::attachMotorOutputs(V7RC_DCMotorConfig* motors, uint8_t numMotors) {
  motorCount_ = numMotors > kMaxMotors ? kMaxMotors : numMotors;
  for (uint8_t i = 0; i < motorCount_; ++i) {
    motorOutputs_[i].attach(motors[i].pinDir, motors[i].pinPwm);
  }
}

bool V7RCCarRuntime::hasMotorIndex(int index) const {
  return index >= 0 && index < motorCount_;
}

void V7RCCarRuntime::beginDifferential(V7RC_DCMotorConfig* motors, uint8_t numMotors, int leftMotorIndex, int rightMotorIndex) {
  attachMotorOutputs(motors, numMotors);
  driveMode_ = V7RC_CAR_DRIVE_DIFFERENTIAL;
  diffLeftMotorIndex_ = leftMotorIndex;
  diffRightMotorIndex_ = rightMotorIndex;
  mecFrontLeftIndex_ = -1;
  mecFrontRightIndex_ = -1;
  mecRearLeftIndex_ = -1;
  mecRearRightIndex_ = -1;
}

void V7RCCarRuntime::beginMecanum(
  V7RC_DCMotorConfig* motors,
  uint8_t numMotors,
  int frontLeftMotorIndex,
  int frontRightMotorIndex,
  int rearLeftMotorIndex,
  int rearRightMotorIndex
) {
  attachMotorOutputs(motors, numMotors);
  driveMode_ = V7RC_CAR_DRIVE_MECANUM;
  diffLeftMotorIndex_ = -1;
  diffRightMotorIndex_ = -1;
  mecFrontLeftIndex_ = frontLeftMotorIndex;
  mecFrontRightIndex_ = frontRightMotorIndex;
  mecRearLeftIndex_ = rearLeftMotorIndex;
  mecRearRightIndex_ = rearRightMotorIndex;
}

void V7RCCarRuntime::apply(const V7RCCarControlState& state) {
  if (driveMode_ == V7RC_CAR_DRIVE_DIFFERENTIAL) {
    if (!hasMotorIndex(diffLeftMotorIndex_) || !hasMotorIndex(diffRightMotorIndex_)) return;
    V7RCCarMotorMix mix = V7RCCarControl::mixDifferential(state.throttle, state.steer);
    motorOutputs_[diffLeftMotorIndex_].writeNormalized(mix.frontLeft);
    motorOutputs_[diffRightMotorIndex_].writeNormalized(mix.frontRight);
    return;
  }

  if (driveMode_ == V7RC_CAR_DRIVE_MECANUM) {
    if (!hasMotorIndex(mecFrontLeftIndex_) || !hasMotorIndex(mecFrontRightIndex_) ||
        !hasMotorIndex(mecRearLeftIndex_) || !hasMotorIndex(mecRearRightIndex_)) {
      return;
    }
    V7RCCarMotorMix mix = V7RCCarControl::mixMecanum(state.vx, state.vy, state.omega);
    motorOutputs_[mecFrontLeftIndex_].writeNormalized(mix.frontLeft);
    motorOutputs_[mecFrontRightIndex_].writeNormalized(mix.frontRight);
    motorOutputs_[mecRearLeftIndex_].writeNormalized(mix.rearLeft);
    motorOutputs_[mecRearRightIndex_].writeNormalized(mix.rearRight);
  }
}

void V7RCCarRuntime::stop() {
  for (uint8_t i = 0; i < motorCount_; ++i) {
    motorOutputs_[i].stop();
  }
}

V7RCCarDriveMode V7RCCarRuntime::driveMode() const {
  return driveMode_;
}
