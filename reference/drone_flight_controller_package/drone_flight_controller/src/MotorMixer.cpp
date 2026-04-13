#include "MotorMixer.h"

MotorMixOutput MotorMixer::mix(float throttle, const RateLoopOutput& cmd) const {
  MotorMixOutput out{};

  out.frontLeft  = throttle + cmd.pitchCmd + cmd.rollCmd - cmd.yawCmd;
  out.frontRight = throttle + cmd.pitchCmd - cmd.rollCmd + cmd.yawCmd;
  out.rearLeft   = throttle - cmd.pitchCmd + cmd.rollCmd + cmd.yawCmd;
  out.rearRight  = throttle - cmd.pitchCmd - cmd.rollCmd - cmd.yawCmd;

  return out;
}

bool MotorMixer::applySaturationAndDesaturation(MotorMixOutput& m,
                                                float minOut,
                                                float maxOut) const {
  float vals[4] = {m.frontLeft, m.frontRight, m.rearLeft, m.rearRight};

  float currentMin = vals[0];
  float currentMax = vals[0];
  for (int i = 1; i < 4; ++i) {
    if (vals[i] < currentMin) currentMin = vals[i];
    if (vals[i] > currentMax) currentMax = vals[i];
  }

  bool saturated = false;

  if (currentMin < minOut) {
    const float shiftUp = minOut - currentMin;
    m.frontLeft  += shiftUp;
    m.frontRight += shiftUp;
    m.rearLeft   += shiftUp;
    m.rearRight  += shiftUp;
    saturated = true;
  }

  vals[0] = m.frontLeft;
  vals[1] = m.frontRight;
  vals[2] = m.rearLeft;
  vals[3] = m.rearRight;

  currentMax = vals[0];
  for (int i = 1; i < 4; ++i) {
    if (vals[i] > currentMax) currentMax = vals[i];
  }

  if (currentMax > maxOut) {
    const float shiftDown = currentMax - maxOut;
    m.frontLeft  -= shiftDown;
    m.frontRight -= shiftDown;
    m.rearLeft   -= shiftDown;
    m.rearRight  -= shiftDown;
    saturated = true;
  }

  m.frontLeft  = clampf(m.frontLeft,  minOut, maxOut);
  m.frontRight = clampf(m.frontRight, minOut, maxOut);
  m.rearLeft   = clampf(m.rearLeft,   minOut, maxOut);
  m.rearRight  = clampf(m.rearRight,  minOut, maxOut);

  return saturated;
}
