#pragma once

#include "V7RCDroneImu.h"

enum V7RCAdxl345Axis : uint8_t {
  V7RC_ADXL345_AXIS_X = 0,
  V7RC_ADXL345_AXIS_Y,
  V7RC_ADXL345_AXIS_Z
};

class V7RCAdxl345Imu : public V7RCDroneImu {
public:
  explicit V7RCAdxl345Imu(uint8_t sdaPin = 5, uint8_t sclPin = 6, uint8_t i2cAddress = 0x53);

  bool begin() override;
  bool update(unsigned long nowMs) override;
  V7RCDroneAttitude attitude() const override;
  const char* sensorName() const override;
  float accelXg() const;
  float accelYg() const;
  float accelZg() const;
  void setAxisTransform(
    V7RCAdxl345Axis logicalXSource,
    int8_t logicalXSign,
    V7RCAdxl345Axis logicalYSource,
    int8_t logicalYSign,
    V7RCAdxl345Axis logicalZSource,
    int8_t logicalZSign
  );

private:
  bool readRaw(int16_t* ax, int16_t* ay, int16_t* az);
  float axisValue(float rawXg, float rawYg, float rawZg, V7RCAdxl345Axis axis, int8_t sign) const;

  uint8_t sdaPin_;
  uint8_t sclPin_;
  uint8_t address_;
  bool begun_;
  V7RCAdxl345Axis logicalXSource_;
  V7RCAdxl345Axis logicalYSource_;
  V7RCAdxl345Axis logicalZSource_;
  int8_t logicalXSign_;
  int8_t logicalYSign_;
  int8_t logicalZSign_;
  float accelXg_;
  float accelYg_;
  float accelZg_;
  V7RCDroneAttitude attitude_;
};
