#include "FlightController.h"

FlightController::FlightController()
    : levelRollTrimDeg_(0.0f),
      levelPitchTrimDeg_(0.0f),
      airborne_(false),
      motorsSaturatedLastCycle_(false),
      debugEnabled_(false),
      debugData_{} {}

void FlightController::begin() {
  angleController_.configure(
      18.0f,
      4.0f,
      4.0f,
      180.0f,
      180.0f);

  rateController_.configure(
      0.015f, 0.010f, 0.0008f,
      0.015f, 0.010f, 0.0008f,
      0.010f, 0.005f, 0.0000f,
      80.0f,
      0.5f,
      0.3f,
      150.0f);
}

void FlightController::reset() {
  rateController_.reset();
  motorsSaturatedLastCycle_ = false;
}

void FlightController::setLevelTrim(float rollTrimDeg, float pitchTrimDeg) {
  levelRollTrimDeg_ = rollTrimDeg;
  levelPitchTrimDeg_ = pitchTrimDeg;
}

void FlightController::setAirborne(bool airborne) {
  airborne_ = airborne;
}

bool FlightController::isAirborne() const {
  return airborne_;
}

void FlightController::setDebugEnabled(bool enabled) {
  debugEnabled_ = enabled;
}

const FlightDebugData& FlightController::debugData() const {
  return debugData_;
}

CorrectedAttitude FlightController::makeCorrectedAttitude(const AttitudeState& raw) const {
  CorrectedAttitude out{};
  out.rollDeg = raw.rollDeg - levelRollTrimDeg_;
  out.pitchDeg = raw.pitchDeg - levelPitchTrimDeg_;
  out.rollRateDegPerSec = raw.rollRateDegPerSec;
  out.pitchRateDegPerSec = raw.pitchRateDegPerSec;
  out.yawRateDegPerSec = raw.yawRateDegPerSec;
  return out;
}

float FlightController::computeAttitudeAuthority(float throttle) const {
  const float minAuthority = 0.35f;
  return minAuthority + (1.0f - minAuthority) * throttle;
}

MotorMixOutput FlightController::update(const ControlState& control,
                                        const AttitudeState& rawAttitude,
                                        float dt,
                                        bool armed) {
  MotorMixOutput zero{};
  zero.frontLeft = 0.0f;
  zero.frontRight = 0.0f;
  zero.rearLeft = 0.0f;
  zero.rearRight = 0.0f;

  if (!armed || !rawAttitude.valid) {
    rateController_.reset();
    motorsSaturatedLastCycle_ = false;
    return zero;
  }

  const CorrectedAttitude corrected = makeCorrectedAttitude(rawAttitude);

  debugData_ = {};
  debugData_.desiredRollDeg = control.roll * 18.0f;
  debugData_.desiredPitchDeg = control.pitch * 18.0f;
  debugData_.correctedRollDeg = corrected.rollDeg;
  debugData_.correctedPitchDeg = corrected.pitchDeg;
  debugData_.stabilize = control.stabilize;
  debugData_.throttle = control.throttle;

  AngleLoopOutput angleOut{};
  if (control.stabilize) {
    angleOut = angleController_.update(control, corrected);
  } else {
    angleOut.desiredRollRateDegPerSec = control.roll * 180.0f;
    angleOut.desiredPitchRateDegPerSec = control.pitch * 180.0f;
  }

  RateLoopOutput rateOut = rateController_.update(
      control,
      corrected,
      angleOut,
      dt,
      airborne_,
      motorsSaturatedLastCycle_,
      debugEnabled_ ? &debugData_ : nullptr);

  const float authority = computeAttitudeAuthority(control.throttle);
  rateOut.rollCmd *= authority;
  rateOut.pitchCmd *= authority;
  rateOut.yawCmd *= authority;

  if (debugEnabled_) {
    debugData_.rollCmd = rateOut.rollCmd;
    debugData_.pitchCmd = rateOut.pitchCmd;
    debugData_.yawCmd = rateOut.yawCmd;
    debugData_.attitudeAuthority = authority;
  }

  MotorMixOutput mixed = motorMixer_.mix(control.throttle, rateOut);
  motorsSaturatedLastCycle_ = motorMixer_.applySaturationAndDesaturation(mixed, 0.0f, 1.0f);

  if (debugEnabled_) {
    debugData_.motorsSaturated = motorsSaturatedLastCycle_;
  }

  return mixed;
}
