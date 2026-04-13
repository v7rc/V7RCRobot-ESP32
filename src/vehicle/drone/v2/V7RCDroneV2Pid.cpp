#include "V7RCDroneV2Pid.h"
#include "V7RCDroneV2Types.h"

V7RCDroneV2Pid::V7RCDroneV2Pid()
    : kp_(0.0f),
      ki_(0.0f),
      kd_(0.0f),
      integral_(0.0f),
      integralLimit_(1.0f),
      outputLimit_(1.0f),
      previousMeasurement_(0.0f),
      hasPreviousMeasurement_(false) {}

void V7RCDroneV2Pid::configure(float kp,
                               float ki,
                               float kd,
                               float integralLimit,
                               float outputLimit) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  integralLimit_ = integralLimit;
  outputLimit_ = outputLimit;
}

void V7RCDroneV2Pid::reset() {
  integral_ = 0.0f;
  previousMeasurement_ = 0.0f;
  hasPreviousMeasurement_ = false;
}

float V7RCDroneV2Pid::update(float setpoint,
                             float measurement,
                             float dt,
                             bool allowIntegral) {
  if (dt <= 0.000001f) {
    return 0.0f;
  }

  const float error = setpoint - measurement;
  const float pTerm = kp_ * error;

  if (allowIntegral) {
    integral_ += error * dt;
    integral_ = V7RCDroneV2Clamp(integral_, -integralLimit_, integralLimit_);
  }

  const float iTerm = ki_ * integral_;

  float dTerm = 0.0f;
  if (hasPreviousMeasurement_) {
    const float derivative = (measurement - previousMeasurement_) / dt;
    dTerm = -kd_ * derivative;
  }

  previousMeasurement_ = measurement;
  hasPreviousMeasurement_ = true;

  return V7RCDroneV2Clamp(pTerm + iTerm + dTerm, -outputLimit_, outputLimit_);
}
