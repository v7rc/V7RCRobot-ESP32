#pragma once

#include "../V7RCDroneImu.h"
#include "V7RCDroneV2Estimator.h"

class V7RCDroneV2LegacyImuAdapter : public V7RCDroneV2Estimator {
public:
  explicit V7RCDroneV2LegacyImuAdapter(V7RCDroneImu* imu, float rateFilterAlpha = 0.35f);

  bool begin() override;
  bool update(unsigned long nowMs) override;
  V7RCDroneV2AttitudeState attitude() const override;
  const char* sensorName() const override;
  bool calibrateGyroBias(uint16_t samples = 200, uint16_t sampleDelayMs = 2) override;

  void setRateFilterAlpha(float alpha);

private:
  V7RCDroneImu* imu_;
  float rateFilterAlpha_;
  unsigned long lastUpdateMs_;
  bool hasPreviousAngles_;
  float previousRollDeg_;
  float previousPitchDeg_;
  V7RCDroneV2AttitudeState attitude_;
};
