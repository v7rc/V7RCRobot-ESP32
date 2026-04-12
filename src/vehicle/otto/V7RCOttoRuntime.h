#pragma once

#include "../../io/V7RCEsp32Outputs.h"
#include "V7RCOttoTypes.h"

class V7RCOttoRuntime {
public:
  V7RCOttoRuntime();

  void begin(const V7RCOttoRuntimeOptions& options);
  void update(const V7RCOttoControlState& state, unsigned long nowMs);
  void stand();

private:
  uint16_t angleToPulse(float angleDeg, bool inverted) const;

  bool started_;
  unsigned long startMs_;
  V7RCOttoRuntimeOptions options_;
  V7RCEsp32ServoOutput leftHip_;
  V7RCEsp32ServoOutput rightHip_;
  V7RCEsp32ServoOutput leftFoot_;
  V7RCEsp32ServoOutput rightFoot_;
};
