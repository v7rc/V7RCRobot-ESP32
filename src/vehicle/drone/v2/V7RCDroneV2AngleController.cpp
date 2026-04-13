#include "V7RCDroneV2AngleController.h"

V7RCDroneV2AngleController::V7RCDroneV2AngleController()
    : config_{} {}

void V7RCDroneV2AngleController::configure(const V7RCDroneV2AngleConfig& config) {
  config_ = config;
}

V7RCDroneV2AngleLoopOutput V7RCDroneV2AngleController::update(
    const V7RCDroneV2ControlState& control,
    const V7RCDroneV2CorrectedAttitude& attitude) const {
  V7RCDroneV2AngleLoopOutput out{};

  const float desiredRollDeg = control.roll * config_.maxTiltDeg;
  const float desiredPitchDeg = control.pitch * config_.maxTiltDeg;

  const float rollErrorDeg = desiredRollDeg - attitude.rollDeg;
  const float pitchErrorDeg = desiredPitchDeg - attitude.pitchDeg;

  out.desiredRollRateDegPerSec =
      V7RCDroneV2Clamp(rollErrorDeg * config_.rollKp,
                       -config_.maxRollRateDegPerSec,
                       config_.maxRollRateDegPerSec);

  out.desiredPitchRateDegPerSec =
      V7RCDroneV2Clamp(pitchErrorDeg * config_.pitchKp,
                       -config_.maxPitchRateDegPerSec,
                       config_.maxPitchRateDegPerSec);

  return out;
}

float V7RCDroneV2AngleController::maxTiltDeg() const {
  return config_.maxTiltDeg;
}
