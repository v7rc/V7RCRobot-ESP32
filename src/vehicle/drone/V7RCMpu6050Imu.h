#pragma once

#include "V7RCDroneImu.h"

class V7RCMpu6050Imu : public V7RCDroneImu {
public:
  explicit V7RCMpu6050Imu(uint8_t i2cAddress = 0x68);

  bool begin() override;
  bool update(unsigned long nowMs) override;
  V7RCDroneAttitude attitude() const override;
  const char* sensorName() const override;

private:
  bool readRaw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

  uint8_t address_;
  bool begun_;
  unsigned long lastUpdateMs_;
  V7RCDroneAttitude attitude_;
};
