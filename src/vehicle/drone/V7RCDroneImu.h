#pragma once

#include "V7RCDroneTypes.h"

class V7RCDroneImu {
public:
  virtual ~V7RCDroneImu() {}
  virtual bool begin() = 0;
  virtual bool update(unsigned long nowMs) = 0;
  virtual V7RCDroneAttitude attitude() const = 0;
  virtual const char* sensorName() const = 0;
  virtual bool calibrateGyroBias(uint16_t samples = 200, uint16_t sampleDelayMs = 2) {
    (void)samples;
    (void)sampleDelayMs;
    return true;
  }
};
