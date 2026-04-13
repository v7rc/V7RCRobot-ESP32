#pragma once

#include "V7RCDroneV2Types.h"

class V7RCDroneV2AngleController {
public:
  V7RCDroneV2AngleController();

  void configure(const V7RCDroneV2AngleConfig& config);
  V7RCDroneV2AngleLoopOutput update(const V7RCDroneV2ControlState& control,
                                    const V7RCDroneV2CorrectedAttitude& attitude) const;

  float maxTiltDeg() const;

private:
  V7RCDroneV2AngleConfig config_;
};
