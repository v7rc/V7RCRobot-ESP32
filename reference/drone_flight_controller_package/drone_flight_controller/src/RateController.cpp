#include "RateController.h"

RateController::RateController()
    : maxYawRateDegPerSec_(150.0f) {}

void RateController::configure(float rollKp, float rollKi, float rollKd,
                               float pitchKp, float pitchKi, float pitchKd,
                               float yawKp, float yawKi, float yawKd,
                               float integralLimit,
                               float rollPitchOutputLimit,
                               float yawOutputLimit,
                               float maxYawRateDegPerSec) {
  rollPid_.configure(rollKp, rollKi, rollKd, integralLimit, rollPitchOutputLimit);
  pitchPid_.configure(pitchKp, pitchKi, pitchKd, integralLimit, rollPitchOutputLimit);
  yawPid_.configure(yawKp, yawKi, yawKd, integralLimit, yawOutputLimit);
  maxYawRateDegPerSec_ = maxYawRateDegPerSec;
}

void RateController::reset() {
  rollPid_.reset();
  pitchPid_.reset();
  yawPid_.reset();
}

RateLoopOutput RateController::update(const ControlState& control,
                                      const CorrectedAttitude& attitude,
                                      const AngleLoopOutput& angleOut,
                                      float dt,
                                      bool airborne,
                                      bool motorsSaturated,
                                      FlightDebugData* debug) {
  RateLoopOutput out{};

  const float desiredYawRateDegPerSec =
      control.yaw * maxYawRateDegPerSec_;

  const bool allowIntegral = airborne && !motorsSaturated;

  out.rollCmd = rollPid_.update(
      angleOut.desiredRollRateDegPerSec,
      attitude.rollRateDegPerSec,
      dt,
      allowIntegral);

  out.pitchCmd = pitchPid_.update(
      angleOut.desiredPitchRateDegPerSec,
      attitude.pitchRateDegPerSec,
      dt,
      allowIntegral);

  out.yawCmd = yawPid_.update(
      desiredYawRateDegPerSec,
      attitude.yawRateDegPerSec,
      dt,
      allowIntegral);

  if (debug) {
    debug->desiredRollRateDegPerSec = angleOut.desiredRollRateDegPerSec;
    debug->desiredPitchRateDegPerSec = angleOut.desiredPitchRateDegPerSec;
    debug->desiredYawRateDegPerSec = desiredYawRateDegPerSec;
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
