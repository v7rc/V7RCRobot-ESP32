#pragma once

#include "../V7RCIcm20948Imu.h"
#include "V7RCDroneV2Estimator.h"

class V7RCDroneV2Icm20948Estimator : public V7RCDroneV2Estimator {
public:
  explicit V7RCDroneV2Icm20948Estimator(V7RCIcm20948Imu* imu);

  bool begin() override;
  bool update(unsigned long nowMs) override;
  V7RCDroneV2AttitudeState attitude() const override;
  const char* sensorName() const override;
  bool calibrateGyroBias(uint16_t samples = 200, uint16_t sampleDelayMs = 2) override;

private:
  V7RCIcm20948Imu* imu_;
  V7RCDroneV2AttitudeState attitude_;
};
