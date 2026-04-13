#include <Arduino.h>
#include "V7RCDroneV2Runtime.h"

namespace {

float clampUnit(float value) {
  return V7RCDroneV2Clamp(value, 0.0f, 1.0f);
}

}  // namespace

V7RCDroneV2Runtime::V7RCDroneV2Runtime()
    : options_(V7RCDroneV2DefaultRuntimeOptions()),
      estimator_(nullptr),
      lastAttitude_{},
      debugData_{},
      begun_(false),
      armed_(false),
      unlockPending_(false),
      calibrationPending_(false),
      readyCuePending_(false),
      airborne_(false),
      debugEnabled_(false),
      motorsSaturatedLastCycle_(false),
      lastUpdateMs_(0),
      unlockStartMs_(0),
      calibrationStartMs_(0),
      readyCueStartMs_(0),
      levelRollTrimDeg_(0.0f),
      levelPitchTrimDeg_(0.0f),
      calibrationRollSumDeg_(0.0f),
      calibrationPitchSumDeg_(0.0f),
      calibrationSamples_(0) {}

bool V7RCDroneV2Runtime::begin(const V7RCDroneV2RuntimeOptions& options, V7RCDroneV2Estimator* estimator) {
  options_ = options;
  estimator_ = estimator;
  debugEnabled_ = options_.debugEnabled;
  angleController_.configure(options_.angle);
  rateController_.configure(options_.rate);

  if (options_.outputMode == V7RC_DRONE_V2_OUTPUT_DC_MOTOR) {
    if (!options_.dcMotors || options_.numDCMotors < 4) {
      begun_ = false;
      return false;
    }

    for (uint8_t i = 0; i < 4; ++i) {
      dcMotorOutputs_[i].attach(options_.dcMotors[i].pinDir, options_.dcMotors[i].pinPwm, options_.dcMotors[i].dirInvert);
      dcMotorOutputs_[i].stop();
    }
  } else {
    for (uint8_t i = 0; i < 4; ++i) {
      escOutputs_[i].attach(options_.motorPins[i], options_.escMinUs, options_.escMaxUs);
      escOutputs_[i].writeMicroseconds(options_.escMinUs);
    }
  }

  begun_ = true;
  armed_ = false;
  unlockPending_ = false;
  calibrationPending_ = false;
  readyCuePending_ = false;
  airborne_ = false;
  motorsSaturatedLastCycle_ = false;
  lastUpdateMs_ = 0;
  unlockStartMs_ = 0;
  calibrationStartMs_ = 0;
  readyCueStartMs_ = 0;
  levelRollTrimDeg_ = 0.0f;
  levelPitchTrimDeg_ = 0.0f;
  calibrationRollSumDeg_ = 0.0f;
  calibrationPitchSumDeg_ = 0.0f;
  calibrationSamples_ = 0;
  lastAttitude_ = {};
  debugData_ = {};
  rateController_.reset();

  if (!estimator_) {
    return true;
  }

  return estimator_->begin();
}

void V7RCDroneV2Runtime::beginUnlock() {
  if (!begun_ || armed_) return;
  unlockPending_ = true;
  calibrationPending_ = false;
  readyCuePending_ = false;
  unlockStartMs_ = millis();
}

void V7RCDroneV2Runtime::disarm() {
  armed_ = false;
  unlockPending_ = false;
  calibrationPending_ = false;
  readyCuePending_ = false;
  airborne_ = false;
  motorsSaturatedLastCycle_ = false;
  unlockStartMs_ = 0;
  calibrationStartMs_ = 0;
  readyCueStartMs_ = 0;
  lastUpdateMs_ = 0;
  levelRollTrimDeg_ = 0.0f;
  levelPitchTrimDeg_ = 0.0f;
  calibrationRollSumDeg_ = 0.0f;
  calibrationPitchSumDeg_ = 0.0f;
  calibrationSamples_ = 0;
  debugData_ = {};
  rateController_.reset();
  stopAllOutputs();
}

void V7RCDroneV2Runtime::update(const V7RCDroneV2ControlState& controlState, unsigned long nowMs) {
  if (!begun_) return;

  if (estimator_) {
    estimator_->update(nowMs);
    lastAttitude_ = estimator_->attitude();
  } else {
    lastAttitude_ = {};
  }

  if (unlockPending_) {
    stopAllOutputs();

    if (!canFinishUnlock(controlState)) {
      unlockPending_ = false;
      unlockStartMs_ = 0;
      return;
    }

    if (unlockStartMs_ != 0 && (nowMs - unlockStartMs_) >= options_.unlockHoldMs) {
      unlockPending_ = false;
      calibrationPending_ = true;
      unlockStartMs_ = 0;
      calibrationStartMs_ = nowMs;
      calibrationRollSumDeg_ = 0.0f;
      calibrationPitchSumDeg_ = 0.0f;
      calibrationSamples_ = 0;
      rateController_.reset();
    }
    return;
  }

  if (calibrationPending_) {
    stopAllOutputs();

    if (lastAttitude_.valid) {
      if (calibrationSamples_ == 0 && estimator_) {
        estimator_->calibrateGyroBias(200, 2);
        lastAttitude_ = estimator_->attitude();
      }
      calibrationRollSumDeg_ += lastAttitude_.rollDeg;
      calibrationPitchSumDeg_ += lastAttitude_.pitchDeg;
      ++calibrationSamples_;
    }

    if (calibrationStartMs_ != 0 && (nowMs - calibrationStartMs_) >= options_.calibrationHoldMs) {
      if (calibrationSamples_ > 0) {
        levelRollTrimDeg_ = calibrationRollSumDeg_ / calibrationSamples_;
        levelPitchTrimDeg_ = calibrationPitchSumDeg_ / calibrationSamples_;
      }
      calibrationPending_ = false;
      calibrationStartMs_ = 0;
      readyCuePending_ = true;
      readyCueStartMs_ = nowMs;
    }
    return;
  }

  if (readyCuePending_) {
    writeReadyCueOutputs();
    if (readyCueStartMs_ != 0 && (nowMs - readyCueStartMs_) >= options_.readyCueMs) {
      readyCuePending_ = false;
      readyCueStartMs_ = 0;
      armed_ = true;
      lastUpdateMs_ = nowMs;
    }
    return;
  }

  if (!armed_) {
    stopAllOutputs();
    return;
  }

  if (!lastAttitude_.valid) {
    rateController_.reset();
    motorsSaturatedLastCycle_ = false;
    stopAllOutputs();
    return;
  }

  if (options_.autoDetectAirborne) {
    airborne_ = controlState.throttle >= options_.airborneThrottleThreshold;
  }

  debugData_ = {};
  debugData_.desiredRollDeg = controlState.roll * options_.angle.maxTiltDeg;
  debugData_.desiredPitchDeg = controlState.pitch * options_.angle.maxTiltDeg;
  debugData_.correctedRollDeg = lastAttitude_.rollDeg - levelRollTrimDeg_;
  debugData_.correctedPitchDeg = lastAttitude_.pitchDeg - levelPitchTrimDeg_;
  debugData_.throttle = controlState.throttle;
  debugData_.stabilize = controlState.stabilize;

  float dt = 0.0f;
  if (lastUpdateMs_ != 0 && nowMs > lastUpdateMs_) {
    dt = (float)(nowMs - lastUpdateMs_) / 1000.0f;
  }
  lastUpdateMs_ = nowMs;

  const V7RCDroneV2CorrectedAttitude corrected = makeCorrectedAttitude(lastAttitude_);
  V7RCDroneV2AngleLoopOutput angleOut{};
  if (controlState.stabilize) {
    angleOut = angleController_.update(controlState, corrected);
  } else {
    angleOut.desiredRollRateDegPerSec = controlState.roll * options_.angle.maxRollRateDegPerSec;
    angleOut.desiredPitchRateDegPerSec = controlState.pitch * options_.angle.maxPitchRateDegPerSec;
  }

  V7RCDroneV2RateLoopOutput rateOut = rateController_.update(
      controlState,
      corrected,
      angleOut,
      dt,
      airborne_,
      motorsSaturatedLastCycle_,
      debugEnabled_ ? &debugData_ : nullptr);

  const float authority = computeAttitudeAuthority(controlState.throttle);
  rateOut.rollCmd *= authority;
  rateOut.pitchCmd *= authority;
  rateOut.yawCmd *= authority;

  if (debugEnabled_) {
    debugData_.rollCmd = rateOut.rollCmd;
    debugData_.pitchCmd = rateOut.pitchCmd;
    debugData_.yawCmd = rateOut.yawCmd;
    debugData_.attitudeAuthority = authority;
  }

  V7RCDroneV2MotorMix mix = motorMixer_.mix(clampUnit(controlState.throttle), rateOut);
  motorsSaturatedLastCycle_ = motorMixer_.applySaturationAndDesaturation(mix, 0.0f, 1.0f);

  if (debugEnabled_) {
    debugData_.motorsSaturated = motorsSaturatedLastCycle_;
  }

  if (options_.outputMode == V7RC_DRONE_V2_OUTPUT_DC_MOTOR) {
    dcMotorOutputs_[0].writeNormalized(normalizeMotorOutput(mix.frontLeft));
    dcMotorOutputs_[1].writeNormalized(normalizeMotorOutput(mix.frontRight));
    dcMotorOutputs_[2].writeNormalized(normalizeMotorOutput(mix.rearLeft));
    dcMotorOutputs_[3].writeNormalized(normalizeMotorOutput(mix.rearRight));
  } else {
    escOutputs_[0].writeMicroseconds(outputToEscUs(mix.frontLeft));
    escOutputs_[1].writeMicroseconds(outputToEscUs(mix.frontRight));
    escOutputs_[2].writeMicroseconds(outputToEscUs(mix.rearLeft));
    escOutputs_[3].writeMicroseconds(outputToEscUs(mix.rearRight));
  }
}

V7RCDroneV2CorrectedAttitude V7RCDroneV2Runtime::makeCorrectedAttitude(const V7RCDroneV2AttitudeState& raw) const {
  V7RCDroneV2CorrectedAttitude corrected{};
  corrected.rollDeg = raw.rollDeg - levelRollTrimDeg_;
  corrected.pitchDeg = raw.pitchDeg - levelPitchTrimDeg_;
  corrected.rollRateDegPerSec = raw.rollRateDegPerSec;
  corrected.pitchRateDegPerSec = raw.pitchRateDegPerSec;
  corrected.yawRateDegPerSec = raw.yawRateDegPerSec;
  return corrected;
}

float V7RCDroneV2Runtime::computeAttitudeAuthority(float throttle) const {
  return options_.attitudeAuthorityMin + (1.0f - options_.attitudeAuthorityMin) * clampUnit(throttle);
}

bool V7RCDroneV2Runtime::canFinishUnlock(const V7RCDroneV2ControlState& controlState) const {
  return controlState.throttle <= 0.05f;
}

void V7RCDroneV2Runtime::stopAllOutputs() {
  for (uint8_t i = 0; i < 4; ++i) {
    if (options_.outputMode == V7RC_DRONE_V2_OUTPUT_DC_MOTOR) {
      dcMotorOutputs_[i].stop();
    } else {
      escOutputs_[i].writeMicroseconds(options_.escMinUs);
    }
  }
}

void V7RCDroneV2Runtime::writeReadyCueOutputs() {
  const float readyOutput = normalizeMotorOutput(options_.readyCueThrottle);
  if (options_.outputMode == V7RC_DRONE_V2_OUTPUT_DC_MOTOR) {
    for (uint8_t i = 0; i < 4; ++i) {
      dcMotorOutputs_[i].writeNormalized(readyOutput);
    }
  } else {
    const uint16_t readyUs = outputToEscUs(readyOutput);
    for (uint8_t i = 0; i < 4; ++i) {
      escOutputs_[i].writeMicroseconds(readyUs);
    }
  }
}

float V7RCDroneV2Runtime::normalizeMotorOutput(float mixValue) const {
  return clampUnit(mixValue);
}

uint16_t V7RCDroneV2Runtime::outputToEscUs(float output) const {
  const float clamped = normalizeMotorOutput(output);
  return (uint16_t)(options_.escMinUs + clamped * (float)(options_.escMaxUs - options_.escMinUs));
}

bool V7RCDroneV2Runtime::isArmed() const {
  return armed_;
}

bool V7RCDroneV2Runtime::unlockInProgress() const {
  return unlockPending_;
}

bool V7RCDroneV2Runtime::calibrationInProgress() const {
  return calibrationPending_;
}

bool V7RCDroneV2Runtime::readyCueInProgress() const {
  return readyCuePending_;
}

bool V7RCDroneV2Runtime::airborne() const {
  return airborne_;
}

void V7RCDroneV2Runtime::setDebugEnabled(bool enabled) {
  debugEnabled_ = enabled;
}

const V7RCDroneV2DebugData& V7RCDroneV2Runtime::debugData() const {
  return debugData_;
}

V7RCDroneV2AttitudeState V7RCDroneV2Runtime::attitude() const {
  return lastAttitude_;
}
