#pragma once

#include "../../protocol/V7RCProtocol.h"
#include "V7RCCarTypes.h"

class V7RCCarControl {
public:
  static V7RCCarControlState neutralState();
  static float normalizedInputFromFrameValue(V7RC_ProtocolType protocolType, int16_t rawValue);
  static void applyInput(V7RCCarInputAxis axis, float normalizedValue, V7RCCarControlState* state);
  static V7RCCarMotorMix mixDifferential(float throttle, float steer);
  static V7RCCarMotorMix mixMecanum(float vx, float vy, float omega);

private:
  static float clampUnit(float value);
  static void normalize(V7RCCarMotorMix* mix);
};
