#pragma once

#include "../../../io/V7RCEsp32Outputs.h"
#include "V7RCDroneV2AngleController.h"
#include "V7RCDroneV2Estimator.h"
#include "V7RCDroneV2MotorMixer.h"
#include "V7RCDroneV2RateController.h"

class V7RCDroneV2Runtime {
public:
  V7RCDroneV2Runtime();

  bool begin(const V7RCDroneV2RuntimeOptions& options, V7RCDroneV2Estimator* estimator);
  void update(const V7RCDroneV2ControlState& controlState, unsigned long nowMs);
  void beginUnlock();
  void disarm();

  bool isArmed() const;
  bool unlockInProgress() const;
  bool calibrationInProgress() const;
  bool readyCueInProgress() const;
  bool airborne() const;

  void setDebugEnabled(bool enabled);
  const V7RCDroneV2DebugData& debugData() const;
  V7RCDroneV2AttitudeState attitude() const;

private:
  V7RCDroneV2CorrectedAttitude makeCorrectedAttitude(const V7RCDroneV2AttitudeState& raw) const;
  float computeAttitudeAuthority(float throttle) const;
  bool canFinishUnlock(const V7RCDroneV2ControlState& controlState) const;
  void stopAllOutputs();
  void writeReadyCueOutputs();
  float normalizeMotorOutput(float mixValue) const;
  uint16_t outputToEscUs(float output) const;

  V7RCEsp32ServoOutput escOutputs_[4];
  V7RCEsp32DCMotorOutput dcMotorOutputs_[4];
  V7RCDroneV2RuntimeOptions options_;
  V7RCDroneV2Estimator* estimator_;
  V7RCDroneV2AngleController angleController_;
  V7RCDroneV2RateController rateController_;
  V7RCDroneV2MotorMixer motorMixer_;
  V7RCDroneV2AttitudeState lastAttitude_;
  V7RCDroneV2DebugData debugData_;

  bool begun_;
  bool armed_;
  bool unlockPending_;
  bool calibrationPending_;
  bool readyCuePending_;
  bool airborne_;
  bool debugEnabled_;
  bool motorsSaturatedLastCycle_;

  unsigned long lastUpdateMs_;
  unsigned long unlockStartMs_;
  unsigned long calibrationStartMs_;
  unsigned long readyCueStartMs_;

  float levelRollTrimDeg_;
  float levelPitchTrimDeg_;
  float calibrationRollSumDeg_;
  float calibrationPitchSumDeg_;
  uint16_t calibrationSamples_;
};
