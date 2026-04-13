#pragma once

#include "V7RCDroneV2Pid.h"
#include "V7RCDroneV2Types.h"

class V7RCDroneV2RateController {
public:
  V7RCDroneV2RateController();

  void configure(const V7RCDroneV2RateConfig& config);
  void reset();

  V7RCDroneV2RateLoopOutput update(const V7RCDroneV2ControlState& control,
                                   const V7RCDroneV2CorrectedAttitude& attitude,
                                   const V7RCDroneV2AngleLoopOutput& angleOut,
                                   float dt,
                                   bool airborne,
                                   bool motorsSaturated,
                                   V7RCDroneV2DebugData* debug = nullptr);

private:
  V7RCDroneV2Pid rollPid_;
  V7RCDroneV2Pid pitchPid_;
  V7RCDroneV2Pid yawPid_;
  V7RCDroneV2RateConfig config_;
};
