#include "PidController.h"

PidController::PidController()
    : kp_(0.0f),
      ki_(0.0f),
      kd_(0.0f),
      integral_(0.0f),
      integralLimit_(1.0f),
      outputLimit_(1.0f),
      prevMeasurement_(0.0f),
      hasPrevMeasurement_(false) {}

void PidController::configure(float kp,
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

void PidController::reset() {
  integral_ = 0.0f;
  prevMeasurement_ = 0.0f;
  hasPrevMeasurement_ = false;
}

float PidController::update(float setpoint,
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
    if (integral_ > integralLimit_) integral_ = integralLimit_;
    if (integral_ < -integralLimit_) integral_ = -integralLimit_;
  }

  const float iTerm = ki_ * integral_;

  float dTerm = 0.0f;
  if (hasPrevMeasurement_) {
    const float measurementDerivative = (measurement - prevMeasurement_) / dt;
    dTerm = -kd_ * measurementDerivative;
  }

  prevMeasurement_ = measurement;
  hasPrevMeasurement_ = true;

  float output = pTerm + iTerm + dTerm;
  if (output > outputLimit_) output = outputLimit_;
  if (output < -outputLimit_) output = -outputLimit_;
  return output;
}
