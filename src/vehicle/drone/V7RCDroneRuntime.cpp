#include <Arduino.h>
#include "V7RCDroneRuntime.h"

namespace {

float clampUnit(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

}  // namespace

V7RCDroneRuntime::V7RCDroneRuntime()
  : imu_(nullptr),
    begun_(false),
    armed_(false),
    unlockPending_(false),
    calibrationPending_(false),
    readyCuePending_(false),
    stabilizationEnabled_(false),
    unlockStartMs_(0),
    calibrationStartMs_(0),
    readyCueStartMs_(0),
    filteredYawRateDegPerSec_(0.0f),
    levelRollTrimDeg_(0.0f),
    levelPitchTrimDeg_(0.0f),
    levelTrimValid_(false),
    calibrationRollSumDeg_(0.0f),
    calibrationPitchSumDeg_(0.0f),
    calibrationSamples_(0) {
  lastAttitude_.rollDeg = 0.0f;
  lastAttitude_.pitchDeg = 0.0f;
  lastAttitude_.yawRateDegPerSec = 0.0f;
  lastAttitude_.valid = false;
}

bool V7RCDroneRuntime::begin(const V7RCDroneRuntimeOptions& options, V7RCDroneImu* imu) {
  options_ = options;
  imu_ = imu;
  stabilizationEnabled_ = options.stabilizationEnabled;

  if (options_.outputMode == V7RC_DRONE_OUTPUT_DC_MOTOR) {
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
  unlockStartMs_ = 0;
  calibrationStartMs_ = 0;
  readyCueStartMs_ = 0;
  lastAttitude_.valid = false;
  filteredYawRateDegPerSec_ = 0.0f;
  levelRollTrimDeg_ = 0.0f;
  levelPitchTrimDeg_ = 0.0f;
  levelTrimValid_ = false;
  calibrationRollSumDeg_ = 0.0f;
  calibrationPitchSumDeg_ = 0.0f;
  calibrationSamples_ = 0;

  if (!imu_) {
    return true;
  }

  return imu_->begin();
}

void V7RCDroneRuntime::beginUnlock() {
  if (!begun_ || armed_) return;
  unlockPending_ = true;
  calibrationPending_ = false;
  readyCuePending_ = false;
  unlockStartMs_ = millis();
}

bool V7RCDroneRuntime::canFinishUnlock(const V7RCDroneControlState& controlState) const {
  return controlState.throttle <= 0.05f;
}

V7RCDroneMotorMix V7RCDroneRuntime::mixOutputs(const V7RCDroneControlState& controlState, const V7RCDroneAttitude& attitude) const {
  const float base = clampUnit(controlState.throttle);
  const float compensationScale = base;
  const float desiredRollDeg = clampUnit(controlState.roll) * options_.maxTiltDeg;
  const float desiredPitchDeg = clampUnit(controlState.pitch) * options_.maxTiltDeg;
  const float correctedRollDeg = attitude.rollDeg - levelRollTrimDeg_;
  const float correctedPitchDeg = attitude.pitchDeg - levelPitchTrimDeg_;

  float rollCorrection = 0.0f;
  float pitchCorrection = 0.0f;
  if (stabilizationEnabled_ && attitude.valid) {
    rollCorrection = (desiredRollDeg - correctedRollDeg) * options_.rollKp / options_.maxTiltDeg;
    pitchCorrection = (desiredPitchDeg - correctedPitchDeg) * options_.pitchKp / options_.maxTiltDeg;
  } else {
    rollCorrection = desiredRollDeg / options_.maxTiltDeg;
    pitchCorrection = desiredPitchDeg / options_.maxTiltDeg;
  }

  rollCorrection *= compensationScale;
  pitchCorrection *= compensationScale;

  float yawComponent = clampUnit(controlState.yaw) * options_.yawGain;
  if (attitude.valid) {
    float dampedYawRate = filteredYawRateDegPerSec_;
    if (fabsf(dampedYawRate) < options_.yawRateDeadbandDegPerSec) {
      dampedYawRate = 0.0f;
    }

    yawComponent += dampedYawRate * options_.yawRateDampingKp;
    yawComponent = clampUnit(yawComponent);
  }
  yawComponent *= compensationScale;

  V7RCDroneMotorMix mix;
  mix.frontLeft = base + pitchCorrection + rollCorrection - yawComponent;
  mix.frontRight = base + pitchCorrection - rollCorrection + yawComponent;
  mix.rearLeft = base - pitchCorrection + rollCorrection + yawComponent;
  mix.rearRight = base - pitchCorrection - rollCorrection - yawComponent;
  return mix;
}

uint16_t V7RCDroneRuntime::throttleToEscUs(float throttle) const {
  throttle = clampUnit(throttle);
  return (uint16_t)(options_.escMinUs + throttle * (float)(options_.escMaxUs - options_.escMinUs));
}

uint16_t V7RCDroneRuntime::mixToEscUs(float value) const {
  value = clampUnit(value);
  if (value < 0.0f) {
    value = 0.0f;
  }
  return throttleToEscUs(value);
}

float V7RCDroneRuntime::mixToDcNorm(float value) const {
  value = clampUnit(value);
  if (value < 0.0f) {
    value = 0.0f;
  }
  return value;
}

void V7RCDroneRuntime::stopAllOutputs() {
  for (uint8_t i = 0; i < 4; ++i) {
    if (options_.outputMode == V7RC_DRONE_OUTPUT_DC_MOTOR) {
      dcMotorOutputs_[i].stop();
    } else {
      escOutputs_[i].writeMicroseconds(options_.escMinUs);
    }
  }
}

void V7RCDroneRuntime::writeReadyCueOutputs() {
  if (options_.outputMode == V7RC_DRONE_OUTPUT_DC_MOTOR) {
    for (uint8_t i = 0; i < 4; ++i) {
      dcMotorOutputs_[i].writeNormalized(0.08f);
    }
  } else {
    const uint16_t cueUs = options_.escMinUs + (uint16_t)((options_.escMaxUs - options_.escMinUs) * 0.08f);
    for (uint8_t i = 0; i < 4; ++i) {
      escOutputs_[i].writeMicroseconds(cueUs);
    }
  }
}

void V7RCDroneRuntime::update(const V7RCDroneControlState& controlState, unsigned long nowMs) {
  if (!begun_) return;

  if (imu_) {
    imu_->update(nowMs);
    lastAttitude_ = imu_->attitude();
    if (lastAttitude_.valid) {
      const float alpha = constrain(options_.yawRateFilterAlpha, 0.0f, 1.0f);
      filteredYawRateDegPerSec_ =
        filteredYawRateDegPerSec_ + alpha * (lastAttitude_.yawRateDegPerSec - filteredYawRateDegPerSec_);
    } else {
      filteredYawRateDegPerSec_ = 0.0f;
      levelTrimValid_ = false;
    }
  } else {
    filteredYawRateDegPerSec_ = 0.0f;
    levelTrimValid_ = false;
  }

  if (unlockPending_) {
    stopAllOutputs();

    if (!canFinishUnlock(controlState)) {
      unlockPending_ = false;
      unlockStartMs_ = 0;
      return;
    }

    if (unlockStartMs_ != 0 && (nowMs - unlockStartMs_) >= 1500UL) {
      unlockPending_ = false;
      unlockStartMs_ = 0;
      calibrationPending_ = true;
      calibrationStartMs_ = nowMs;
      calibrationRollSumDeg_ = 0.0f;
      calibrationPitchSumDeg_ = 0.0f;
      calibrationSamples_ = 0;
      filteredYawRateDegPerSec_ = 0.0f;
      levelTrimValid_ = false;
    }
    return;
  }

  if (calibrationPending_) {
    stopAllOutputs();

    if (lastAttitude_.valid) {
      if (calibrationSamples_ == 0 && imu_) {
        imu_->calibrateGyroBias(200, 2);
        filteredYawRateDegPerSec_ = 0.0f;
        lastAttitude_ = imu_->attitude();
      }
      calibrationRollSumDeg_ += lastAttitude_.rollDeg;
      calibrationPitchSumDeg_ += lastAttitude_.pitchDeg;
      ++calibrationSamples_;
    }

    if (calibrationStartMs_ != 0 && (nowMs - calibrationStartMs_) >= 1500UL) {
      if (calibrationSamples_ > 0) {
        levelRollTrimDeg_ = calibrationRollSumDeg_ / calibrationSamples_;
        levelPitchTrimDeg_ = calibrationPitchSumDeg_ / calibrationSamples_;
        levelTrimValid_ = true;
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
    if (readyCueStartMs_ != 0 && (nowMs - readyCueStartMs_) >= 2000UL) {
      readyCuePending_ = false;
      readyCueStartMs_ = 0;
      armed_ = true;
    }
    return;
  }

  if (!armed_) {
    stopAllOutputs();
    return;
  }

  V7RCDroneMotorMix mix = mixOutputs(controlState, lastAttitude_);
  if (options_.outputMode == V7RC_DRONE_OUTPUT_DC_MOTOR) {
    dcMotorOutputs_[0].writeNormalized(mixToDcNorm(mix.frontLeft));
    dcMotorOutputs_[1].writeNormalized(mixToDcNorm(mix.frontRight));
    dcMotorOutputs_[2].writeNormalized(mixToDcNorm(mix.rearLeft));
    dcMotorOutputs_[3].writeNormalized(mixToDcNorm(mix.rearRight));
  } else {
    escOutputs_[0].writeMicroseconds(mixToEscUs(mix.frontLeft));
    escOutputs_[1].writeMicroseconds(mixToEscUs(mix.frontRight));
    escOutputs_[2].writeMicroseconds(mixToEscUs(mix.rearLeft));
    escOutputs_[3].writeMicroseconds(mixToEscUs(mix.rearRight));
  }
}

void V7RCDroneRuntime::disarm() {
  armed_ = false;
  unlockPending_ = false;
  calibrationPending_ = false;
  readyCuePending_ = false;
  unlockStartMs_ = 0;
  calibrationStartMs_ = 0;
  readyCueStartMs_ = 0;
  filteredYawRateDegPerSec_ = 0.0f;
  levelTrimValid_ = false;
  levelRollTrimDeg_ = 0.0f;
  levelPitchTrimDeg_ = 0.0f;
  calibrationRollSumDeg_ = 0.0f;
  calibrationPitchSumDeg_ = 0.0f;
  calibrationSamples_ = 0;
  stopAllOutputs();
}

bool V7RCDroneRuntime::isArmed() const {
  return armed_;
}

bool V7RCDroneRuntime::unlockInProgress() const {
  return unlockPending_;
}

bool V7RCDroneRuntime::calibrationInProgress() const {
  return calibrationPending_;
}

bool V7RCDroneRuntime::readyCueInProgress() const {
  return readyCuePending_;
}

void V7RCDroneRuntime::setStabilizationEnabled(bool enabled) {
  stabilizationEnabled_ = enabled;
}

bool V7RCDroneRuntime::stabilizationEnabled() const {
  return stabilizationEnabled_;
}

V7RCDroneAttitude V7RCDroneRuntime::attitude() const {
  return lastAttitude_;
}
