#include "V7RCDroneV2MotorMixer.h"

V7RCDroneV2MotorMix V7RCDroneV2MotorMixer::mix(float throttle, const V7RCDroneV2RateLoopOutput& command) const {
  V7RCDroneV2MotorMix out{};
  out.frontLeft = throttle + command.pitchCmd + command.rollCmd - command.yawCmd;
  out.frontRight = throttle + command.pitchCmd - command.rollCmd + command.yawCmd;
  out.rearLeft = throttle - command.pitchCmd + command.rollCmd + command.yawCmd;
  out.rearRight = throttle - command.pitchCmd - command.rollCmd - command.yawCmd;
  return out;
}

bool V7RCDroneV2MotorMixer::applySaturationAndDesaturation(V7RCDroneV2MotorMix& mix,
                                                           float minOutput,
                                                           float maxOutput) const {
  float values[4] = {mix.frontLeft, mix.frontRight, mix.rearLeft, mix.rearRight};
  float currentMin = values[0];
  float currentMax = values[0];
  for (uint8_t i = 1; i < 4; ++i) {
    if (values[i] < currentMin) currentMin = values[i];
    if (values[i] > currentMax) currentMax = values[i];
  }

  bool saturated = false;

  if (currentMin < minOutput) {
    const float shiftUp = minOutput - currentMin;
    mix.frontLeft += shiftUp;
    mix.frontRight += shiftUp;
    mix.rearLeft += shiftUp;
    mix.rearRight += shiftUp;
    saturated = true;
  }

  values[0] = mix.frontLeft;
  values[1] = mix.frontRight;
  values[2] = mix.rearLeft;
  values[3] = mix.rearRight;
  currentMax = values[0];
  for (uint8_t i = 1; i < 4; ++i) {
    if (values[i] > currentMax) currentMax = values[i];
  }

  if (currentMax > maxOutput) {
    const float shiftDown = currentMax - maxOutput;
    mix.frontLeft -= shiftDown;
    mix.frontRight -= shiftDown;
    mix.rearLeft -= shiftDown;
    mix.rearRight -= shiftDown;
    saturated = true;
  }

  mix.frontLeft = V7RCDroneV2Clamp(mix.frontLeft, minOutput, maxOutput);
  mix.frontRight = V7RCDroneV2Clamp(mix.frontRight, minOutput, maxOutput);
  mix.rearLeft = V7RCDroneV2Clamp(mix.rearLeft, minOutput, maxOutput);
  mix.rearRight = V7RCDroneV2Clamp(mix.rearRight, minOutput, maxOutput);
  return saturated;
}
