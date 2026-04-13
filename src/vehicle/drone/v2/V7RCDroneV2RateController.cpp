#include "V7RCDroneV2RateController.h"

V7RCDroneV2RateController::V7RCDroneV2RateController()
    : config_{} {}

void V7RCDroneV2RateController::configure(const V7RCDroneV2RateConfig& config) {
  config_ = config;
  rollPid_.configure(config.roll.kp, config.roll.ki, config.roll.kd, config.roll.integralLimit, config.roll.outputLimit);
  pitchPid_.configure(config.pitch.kp, config.pitch.ki, config.pitch.kd, config.pitch.integralLimit, config.pitch.outputLimit);
  yawPid_.configure(config.yaw.kp, config.yaw.ki, config.yaw.kd, config.yaw.integralLimit, config.yaw.outputLimit);
}

void V7RCDroneV2RateController::reset() {
  rollPid_.reset();
  pitchPid_.reset();
  yawPid_.reset();
}

V7RCDroneV2RateLoopOutput V7RCDroneV2RateController::update(
    const V7RCDroneV2ControlState& control,
    const V7RCDroneV2CorrectedAttitude& attitude,
    const V7RCDroneV2AngleLoopOutput& angleOut,
    float dt,
    bool airborne,
    bool motorsSaturated,
    V7RCDroneV2DebugData* debug) {
  V7RCDroneV2RateLoopOutput out{};

  const float desiredRollRate = angleOut.desiredRollRateDegPerSec;
  const float desiredPitchRate = angleOut.desiredPitchRateDegPerSec;
  const float desiredYawRate = control.yaw * config_.maxYawRateDegPerSec;
  const bool allowIntegral = airborne && !motorsSaturated;

  out.rollCmd = rollPid_.update(desiredRollRate, attitude.rollRateDegPerSec, dt, allowIntegral);
  out.pitchCmd = pitchPid_.update(desiredPitchRate, attitude.pitchRateDegPerSec, dt, allowIntegral);
  out.yawCmd = yawPid_.update(desiredYawRate, attitude.yawRateDegPerSec, dt, allowIntegral);

  if (debug) {
    debug->desiredRollRateDegPerSec = desiredRollRate;
    debug->desiredPitchRateDegPerSec = desiredPitchRate;
    debug->desiredYawRateDegPerSec = desiredYawRate;
    debug->measuredRollRateDegPerSec = attitude.rollRateDegPerSec;
    debug->measuredPitchRateDegPerSec = attitude.pitchRateDegPerSec;
    debug->measuredYawRateDegPerSec = attitude.yawRateDegPerSec;
    debug->rollCmd = out.rollCmd;
    debug->pitchCmd = out.pitchCmd;
    debug->yawCmd = out.yawCmd;
    debug->airborne = airborne;
  }

  return out;
}
