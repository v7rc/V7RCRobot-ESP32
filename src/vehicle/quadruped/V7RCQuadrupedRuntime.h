#pragma once

#include "../../io/V7RCEsp32Outputs.h"
#include "V7RCQuadrupedTypes.h"

class V7RCQuadrupedRuntime {
public:
  V7RCQuadrupedRuntime();

  void begin(const V7RCQuadrupedRuntimeOptions& options);
  void update(const V7RCQuadrupedControlState& state, unsigned long nowMs);
  void stand(float bodyHeight = -1.0f);

private:
  void writeLeg(uint8_t legIndex, const V7RCQuadrupedFootPoint& footPoint);
  V7RCQuadrupedLegIk solveIk(const V7RCQuadrupedFootPoint& footPoint) const;
  uint16_t angleToPulse(float angleDeg, bool inverted) const;

  bool started_;
  unsigned long startMs_;
  V7RCQuadrupedRuntimeOptions options_;
  V7RCEsp32ServoOutput hipOutputs_[4];
  V7RCEsp32ServoOutput kneeOutputs_[4];
};
