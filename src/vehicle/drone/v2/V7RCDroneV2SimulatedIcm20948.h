#pragma once

#include "../V7RCIcm20948Imu.h"
#include "V7RCDroneV2Types.h"

struct V7RCDroneV2Icm20948InputSample {
  float accelXg;
  float accelYg;
  float accelZg;
  float gyroXDegPerSec;
  float gyroYDegPerSec;
  float gyroZDegPerSec;
  bool valid;
};

class V7RCDroneV2SimulatedIcm20948 {
public:
  V7RCDroneV2SimulatedIcm20948();

  void reset();
  void setInputSample(const V7RCDroneV2Icm20948InputSample& sample);
  bool update(unsigned long nowMs);

  V7RCDroneV2AttitudeState attitude() const;
  V7RCDroneV2Icm20948InputSample inputSample() const;

  float filteredAccelXg() const;
  float filteredAccelYg() const;
  float filteredAccelZg() const;
  float filteredGyroXDegPerSec() const;
  float filteredGyroYDegPerSec() const;
  float filteredGyroZDegPerSec() const;

  void setAxisTransform(
    V7RCIcm20948Axis logicalXSource,
    int8_t logicalXSign,
    V7RCIcm20948Axis logicalYSource,
    int8_t logicalYSign,
    V7RCIcm20948Axis logicalZSource,
    int8_t logicalZSign
  );

private:
  float axisValue(float rawX, float rawY, float rawZ, V7RCIcm20948Axis axis, int8_t sign) const;

  V7RCDroneV2Icm20948InputSample sample_;
  V7RCDroneV2AttitudeState attitude_;
  unsigned long lastUpdateMs_;
  bool filterPrimed_;

  V7RCIcm20948Axis logicalXSource_;
  V7RCIcm20948Axis logicalYSource_;
  V7RCIcm20948Axis logicalZSource_;
  int8_t logicalXSign_;
  int8_t logicalYSign_;
  int8_t logicalZSign_;

  float accelXg_;
  float accelYg_;
  float accelZg_;
  float gyroXDegPerSec_;
  float gyroYDegPerSec_;
  float gyroZDegPerSec_;
};
