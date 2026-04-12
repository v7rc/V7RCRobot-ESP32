#include "V7RCCarControl.h"

#include <math.h>

V7RCCarControlState V7RCCarControl::neutralState() {
  V7RCCarControlState state;
  state.throttle = 0.0f;
  state.steer = 0.0f;
  state.vx = 0.0f;
  state.vy = 0.0f;
  state.omega = 0.0f;
  return state;
}

float V7RCCarControl::normalizedInputFromFrameValue(V7RC_ProtocolType protocolType, int16_t rawValue) {
  float normalized = 0.0f;

  if (protocolType == V7RC_HEX || protocolType == V7RC_SRV || protocolType == V7RC_SS8) {
    normalized = ((float)rawValue - 1500.0f) / 500.0f;
  } else if (protocolType == V7RC_DEG) {
    normalized = (float)rawValue / 127.0f;
  }

  return clampUnit(normalized);
}

void V7RCCarControl::applyInput(V7RCCarInputAxis axis, float normalizedValue, V7RCCarControlState* state) {
  if (!state) return;

  switch (axis) {
    case V7RC_CAR_INPUT_THROTTLE:
      state->throttle = normalizedValue;
      break;
    case V7RC_CAR_INPUT_STEER:
      state->steer = normalizedValue;
      break;
    case V7RC_CAR_INPUT_VX:
      state->vx = normalizedValue;
      break;
    case V7RC_CAR_INPUT_VY:
      state->vy = normalizedValue;
      break;
    case V7RC_CAR_INPUT_OMEGA:
      state->omega = normalizedValue;
      break;
  }
}

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
  // Align Vx with the physical mecanum direction used by the shipped examples:
  // positive Vx should match the user's intuitive X-axis strafe direction.
  vx = -vx;

  V7RCCarMotorMix mix;
  mix.frontLeft = vy - vx + omega;
  mix.frontRight = vy + vx - omega;
  mix.rearLeft = vy + vx + omega;
  mix.rearRight = vy - vx - omega;
  normalize(&mix);
  return mix;
}
