#pragma once

#include "AngleController.h"
#include "FlightTypes.h"
#include "MotorMixer.h"
#include "RateController.h"

class FlightController {
public:
  FlightController();

  void begin();
  void reset();

  void setLevelTrim(float rollTrimDeg, float pitchTrimDeg);
  void setAirborne(bool airborne);
  bool isAirborne() const;

  void setDebugEnabled(bool enabled);
  const FlightDebugData& debugData() const;

  MotorMixOutput update(const ControlState& control,
                        const AttitudeState& rawAttitude,
                        float dt,
                        bool armed);

private:
  CorrectedAttitude makeCorrectedAttitude(const AttitudeState& raw) const;
  float computeAttitudeAuthority(float throttle) const;

private:
  AngleController angleController_;
  RateController rateController_;
  MotorMixer motorMixer_;

  float levelRollTrimDeg_;
  float levelPitchTrimDeg_;
  bool airborne_;
  bool motorsSaturatedLastCycle_;
  bool debugEnabled_;
  FlightDebugData debugData_;
};
