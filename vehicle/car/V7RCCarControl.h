#pragma once

#include "V7RCCarTypes.h"

class V7RCCarControl {
public:
  static V7RCCarMotorMix mixDifferential(float throttle, float steer);
  static V7RCCarMotorMix mixMecanum(float vx, float vy, float omega);

private:
  static float clampUnit(float value);
  static void normalize(V7RCCarMotorMix* mix);
};
