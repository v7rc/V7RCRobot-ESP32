#pragma once

#include "FlightTypes.h"
#include "PidController.h"

class RateController {
public:
  RateController();

  void configure(float rollKp, float rollKi, float rollKd,
                 float pitchKp, float pitchKi, float pitchKd,
                 float yawKp, float yawKi, float yawKd,
                 float integralLimit,
                 float rollPitchOutputLimit,
                 float yawOutputLimit,
                 float maxYawRateDegPerSec);

  void reset();

  RateLoopOutput update(const ControlState& control,
                        const CorrectedAttitude& attitude,
                        const AngleLoopOutput& angleOut,
                        float dt,
                        bool airborne,
                        bool motorsSaturated,
                        FlightDebugData* debug = nullptr);

private:
  PidController rollPid_;
  PidController pitchPid_;
  PidController yawPid_;

  float maxYawRateDegPerSec_;
};
