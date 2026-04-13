#pragma once

#include "V7RCDroneImu.h"

class V7RCIcm20948Imu : public V7RCDroneImu {
public:
  explicit V7RCIcm20948Imu(uint8_t sdaPin = 5, uint8_t sclPin = 6, uint8_t i2cAddress = 0x68);

  bool begin() override;
  bool update(unsigned long nowMs) override;
  V7RCDroneAttitude attitude() const override;
  const char* sensorName() const override;

  float accelXg() const;
  float accelYg() const;
  float accelZg() const;
  float gyroXDegPerSec() const;
  float gyroYDegPerSec() const;
  float gyroZDegPerSec() const;

private:
  bool selectBank(uint8_t bank);
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t reg, uint8_t* data, size_t len);
  bool readRaw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

  uint8_t sdaPin_;
  uint8_t sclPin_;
  uint8_t address_;
  uint8_t currentBank_;
  bool begun_;
  unsigned long lastUpdateMs_;
  float accelXg_;
  float accelYg_;
  float accelZg_;
  float gyroXDegPerSec_;
  float gyroYDegPerSec_;
  float gyroZDegPerSec_;
  V7RCDroneAttitude attitude_;
};
