#include "V7RCCarControl.h"

#include <math.h>

float V7RCCarControl::clampUnit(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

void V7RCCarControl::normalize(V7RCCarMotorMix* mix) {
  if (!mix) return;

  float maxMagnitude = fabsf(mix->frontLeft);
  if (fabsf(mix->frontRight) > maxMagnitude) maxMagnitude = fabsf(mix->frontRight);
  if (fabsf(mix->rearLeft) > maxMagnitude) maxMagnitude = fabsf(mix->rearLeft);
  if (fabsf(mix->rearRight) > maxMagnitude) maxMagnitude = fabsf(mix->rearRight);

  if (maxMagnitude <= 1.0f) return;

  mix->frontLeft /= maxMagnitude;
  mix->frontRight /= maxMagnitude;
  mix->rearLeft /= maxMagnitude;
  mix->rearRight /= maxMagnitude;
}

V7RCCarMotorMix V7RCCarControl::mixDifferential(float throttle, float steer) {
  V7RCCarMotorMix mix;
  mix.frontLeft = clampUnit(throttle + steer);
  mix.frontRight = clampUnit(throttle - steer);
  mix.rearLeft = mix.frontLeft;
  mix.rearRight = mix.frontRight;
  normalize(&mix);
  return mix;
}

V7RCCarMotorMix V7RCCarControl::mixMecanum(float vx, float vy, float omega) {
  V7RCCarMotorMix mix;
  mix.frontLeft = vy - vx + omega;
  mix.frontRight = vy + vx - omega;
  mix.rearLeft = vy + vx + omega;
  mix.rearRight = vy - vx - omega;
  normalize(&mix);
  return mix;
}
