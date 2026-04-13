#include "AngleController.h"

AngleController::AngleController()
    : maxTiltDeg_(18.0f),
      angleKpRoll_(4.0f),
      angleKpPitch_(4.0f),
      maxRollRateDegPerSec_(180.0f),
      maxPitchRateDegPerSec_(180.0f) {}

void AngleController::configure(float maxTiltDeg,
                                float angleKpRoll,
                                float angleKpPitch,
                                float maxRollRateDegPerSec,
                                float maxPitchRateDegPerSec) {
  maxTiltDeg_ = maxTiltDeg;
  angleKpRoll_ = angleKpRoll;
  angleKpPitch_ = angleKpPitch;
  maxRollRateDegPerSec_ = maxRollRateDegPerSec;
  maxPitchRateDegPerSec_ = maxPitchRateDegPerSec;
}

AngleLoopOutput AngleController::update(const ControlState& control,
                                        const CorrectedAttitude& attitude) const {
  AngleLoopOutput out{};

  const float desiredRollDeg = control.roll * maxTiltDeg_;
  const float desiredPitchDeg = control.pitch * maxTiltDeg_;

  const float rollErrorDeg = desiredRollDeg - attitude.rollDeg;
  const float pitchErrorDeg = desiredPitchDeg - attitude.pitchDeg;

  out.desiredRollRateDegPerSec =
      clampf(angleKpRoll_ * rollErrorDeg,
             -maxRollRateDegPerSec_,
              maxRollRateDegPerSec_);

  out.desiredPitchRateDegPerSec =
      clampf(angleKpPitch_ * pitchErrorDeg,
             -maxPitchRateDegPerSec_,
              maxPitchRateDegPerSec_);

  return out;
}
