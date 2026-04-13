#pragma once

#include "V7RCDroneV2Types.h"

class V7RCDroneV2MotorMixer {
public:
  V7RCDroneV2MotorMix mix(float throttle, const V7RCDroneV2RateLoopOutput& command) const;
  bool applySaturationAndDesaturation(V7RCDroneV2MotorMix& mix,
                                      float minOutput = 0.0f,
                                      float maxOutput = 1.0f) const;
};
