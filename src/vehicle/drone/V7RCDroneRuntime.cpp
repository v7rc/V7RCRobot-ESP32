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
    stabilizationEnabled_(false),
    unlockStartMs_(0),
    filteredYawRateDegPerSec_(0.0f) {
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
  unlockStartMs_ = 0;
  lastAttitude_.valid = false;
  filteredYawRateDegPerSec_ = 0.0f;

  if (!imu_) {
    return true;
  }

  return imu_->begin();
}

void V7RCDroneRuntime::beginUnlock() {
  if (!begun_ || armed_) return;
  unlockPending_ = true;
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

  float rollCorrection = 0.0f;
  float pitchCorrection = 0.0f;
  if (stabilizationEnabled_ && attitude.valid) {
    rollCorrection = (desiredRollDeg - attitude.rollDeg) * options_.rollKp / options_.maxTiltDeg;
    pitchCorrection = (desiredPitchDeg - attitude.pitchDeg) * options_.pitchKp / options_.maxTiltDeg;
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
    }
  } else {
    filteredYawRateDegPerSec_ = 0.0f;
  }

  if (unlockPending_) {
    for (uint8_t i = 0; i < 4; ++i) {
      if (options_.outputMode == V7RC_DRONE_OUTPUT_DC_MOTOR) {
        dcMotorOutputs_[i].stop();
      } else {
        escOutputs_[i].writeMicroseconds(options_.escMinUs);
      }
    }

    if (!canFinishUnlock(controlState)) {
      unlockPending_ = false;
      unlockStartMs_ = 0;
      return;
    }

    if (unlockStartMs_ != 0 && (nowMs - unlockStartMs_) >= 1500UL) {
      armed_ = true;
      unlockPending_ = false;
      unlockStartMs_ = 0;
    }
    return;
  }

  if (!armed_) {
    for (uint8_t i = 0; i < 4; ++i) {
      if (options_.outputMode == V7RC_DRONE_OUTPUT_DC_MOTOR) {
        dcMotorOutputs_[i].stop();
      } else {
        escOutputs_[i].writeMicroseconds(options_.escMinUs);
      }
    }
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
  unlockStartMs_ = 0;
  filteredYawRateDegPerSec_ = 0.0f;
  for (uint8_t i = 0; i < 4; ++i) {
    if (options_.outputMode == V7RC_DRONE_OUTPUT_DC_MOTOR) {
      dcMotorOutputs_[i].stop();
    } else {
      escOutputs_[i].writeMicroseconds(options_.escMinUs);
    }
  }
}

bool V7RCDroneRuntime::isArmed() const {
  return armed_;
}

bool V7RCDroneRuntime::unlockInProgress() const {
  return unlockPending_;
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
