#pragma once

#include "V7RCDroneV2Types.h"

class V7RCDroneV2Estimator {
public:
  virtual ~V7RCDroneV2Estimator() {}
  virtual bool begin() = 0;
  virtual bool update(unsigned long nowMs) = 0;
  virtual V7RCDroneV2AttitudeState attitude() const = 0;
  virtual const char* sensorName() const = 0;
  virtual bool calibrateGyroBias(uint16_t samples = 200, uint16_t sampleDelayMs = 2) {
    (void)samples;
    (void)sampleDelayMs;
    return true;
  }
};
