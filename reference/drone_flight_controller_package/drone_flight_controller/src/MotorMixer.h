#pragma once

#include "FlightTypes.h"

class MotorMixer {
public:
  MotorMixOutput mix(float throttle, const RateLoopOutput& cmd) const;

  bool applySaturationAndDesaturation(MotorMixOutput& mix,
                                      float minOut = 0.0f,
                                      float maxOut = 1.0f) const;
};
