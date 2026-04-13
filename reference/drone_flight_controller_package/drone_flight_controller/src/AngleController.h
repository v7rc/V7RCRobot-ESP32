#pragma once

#include "FlightTypes.h"

class AngleController {
public:
  AngleController();

  void configure(float maxTiltDeg,
                 float angleKpRoll,
                 float angleKpPitch,
                 float maxRollRateDegPerSec,
                 float maxPitchRateDegPerSec);

  AngleLoopOutput update(const ControlState& control,
                         const CorrectedAttitude& attitude) const;

  float maxTiltDeg() const { return maxTiltDeg_; }

private:
  float maxTiltDeg_;
  float angleKpRoll_;
  float angleKpPitch_;
  float maxRollRateDegPerSec_;
  float maxPitchRateDegPerSec_;
};
