#pragma once

#include "../../io/V7RCEsp32Outputs.h"
#include "V7RCDroneImu.h"

class V7RCDroneRuntime {
public:
  V7RCDroneRuntime();

  bool begin(const V7RCDroneRuntimeOptions& options, V7RCDroneImu* imu);
  void update(const V7RCDroneControlState& controlState, unsigned long nowMs);
  void disarm();
  void beginUnlock();
  bool isArmed() const;
  bool unlockInProgress() const;
  bool calibrationInProgress() const;
  bool readyCueInProgress() const;
  void setStabilizationEnabled(bool enabled);
  bool stabilizationEnabled() const;
  V7RCDroneAttitude attitude() const;

private:
  V7RCDroneMotorMix mixOutputs(const V7RCDroneControlState& controlState, const V7RCDroneAttitude& attitude) const;
  uint16_t throttleToEscUs(float throttle) const;
  uint16_t mixToEscUs(float value) const;
  float mixToDcNorm(float value) const;
  bool canFinishUnlock(const V7RCDroneControlState& controlState) const;
  void stopAllOutputs();
  void writeReadyCueOutputs();

  V7RCEsp32ServoOutput escOutputs_[4];
  V7RCEsp32DCMotorOutput dcMotorOutputs_[4];
  V7RCDroneRuntimeOptions options_;
  V7RCDroneImu* imu_;
  bool begun_;
  bool armed_;
  bool unlockPending_;
  bool calibrationPending_;
  bool readyCuePending_;
  bool stabilizationEnabled_;
  unsigned long unlockStartMs_;
  unsigned long calibrationStartMs_;
  unsigned long readyCueStartMs_;
  V7RCDroneAttitude lastAttitude_;
  float filteredYawRateDegPerSec_;
  float levelRollTrimDeg_;
  float levelPitchTrimDeg_;
  bool levelTrimValid_;
  float calibrationRollSumDeg_;
  float calibrationPitchSumDeg_;
  uint16_t calibrationSamples_;
};
