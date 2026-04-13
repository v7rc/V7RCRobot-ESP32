#include "V7RCDroneV2SimulatedIcm20948.h"

#include <math.h>

V7RCDroneV2SimulatedIcm20948::V7RCDroneV2SimulatedIcm20948()
    : sample_{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, false},
      attitude_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false},
      lastUpdateMs_(0),
      filterPrimed_(false),
      logicalXSource_(V7RC_ICM20948_AXIS_X),
      logicalYSource_(V7RC_ICM20948_AXIS_Y),
      logicalZSource_(V7RC_ICM20948_AXIS_Z),
      logicalXSign_(1),
      logicalYSign_(1),
      logicalZSign_(1),
      accelXg_(0.0f),
      accelYg_(0.0f),
      accelZg_(0.0f),
      gyroXDegPerSec_(0.0f),
      gyroYDegPerSec_(0.0f),
      gyroZDegPerSec_(0.0f) {}

void V7RCDroneV2SimulatedIcm20948::reset() {
  lastUpdateMs_ = 0;
  filterPrimed_ = false;
  attitude_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};
  accelXg_ = 0.0f;
  accelYg_ = 0.0f;
  accelZg_ = 0.0f;
  gyroXDegPerSec_ = 0.0f;
  gyroYDegPerSec_ = 0.0f;
  gyroZDegPerSec_ = 0.0f;
}

void V7RCDroneV2SimulatedIcm20948::setInputSample(const V7RCDroneV2Icm20948InputSample& sample) {
  sample_ = sample;
}

float V7RCDroneV2SimulatedIcm20948::axisValue(float rawX, float rawY, float rawZ, V7RCIcm20948Axis axis, int8_t sign) const {
  float value = rawX;
  if (axis == V7RC_ICM20948_AXIS_Y) {
    value = rawY;
  } else if (axis == V7RC_ICM20948_AXIS_Z) {
    value = rawZ;
  }
  return sign >= 0 ? value : -value;
}

bool V7RCDroneV2SimulatedIcm20948::update(unsigned long nowMs) {
  if (!sample_.valid) {
    attitude_.valid = false;
    return false;
  }

  float dt = 0.0f;
  if (lastUpdateMs_ != 0 && nowMs > lastUpdateMs_) {
    dt = (float)(nowMs - lastUpdateMs_) / 1000.0f;
  }
  lastUpdateMs_ = nowMs;

  const float mappedAccelXg = axisValue(sample_.accelXg, sample_.accelYg, sample_.accelZg, logicalXSource_, logicalXSign_);
  const float mappedAccelYg = axisValue(sample_.accelXg, sample_.accelYg, sample_.accelZg, logicalYSource_, logicalYSign_);
  const float mappedAccelZg = axisValue(sample_.accelXg, sample_.accelYg, sample_.accelZg, logicalZSource_, logicalZSign_);
  const float mappedGyroXDegPerSec =
      axisValue(sample_.gyroXDegPerSec, sample_.gyroYDegPerSec, sample_.gyroZDegPerSec, logicalXSource_, logicalXSign_);
  const float mappedGyroYDegPerSec =
      axisValue(sample_.gyroXDegPerSec, sample_.gyroYDegPerSec, sample_.gyroZDegPerSec, logicalYSource_, logicalYSign_);
  const float mappedGyroZDegPerSec =
      axisValue(sample_.gyroXDegPerSec, sample_.gyroYDegPerSec, sample_.gyroZDegPerSec, logicalZSource_, logicalZSign_);

  if (!filterPrimed_) {
    accelXg_ = mappedAccelXg;
    accelYg_ = mappedAccelYg;
    accelZg_ = mappedAccelZg;
    gyroXDegPerSec_ = mappedGyroXDegPerSec;
    gyroYDegPerSec_ = mappedGyroYDegPerSec;
    gyroZDegPerSec_ = mappedGyroZDegPerSec;
    filterPrimed_ = true;
  } else {
    const float accelAlpha = 0.20f;
    const float gyroAlpha = 0.15f;
    accelXg_ = accelXg_ + accelAlpha * (mappedAccelXg - accelXg_);
    accelYg_ = accelYg_ + accelAlpha * (mappedAccelYg - accelYg_);
    accelZg_ = accelZg_ + accelAlpha * (mappedAccelZg - accelZg_);
    gyroXDegPerSec_ = gyroXDegPerSec_ + gyroAlpha * (mappedGyroXDegPerSec - gyroXDegPerSec_);
    gyroYDegPerSec_ = gyroYDegPerSec_ + gyroAlpha * (mappedGyroYDegPerSec - gyroYDegPerSec_);
    gyroZDegPerSec_ = gyroZDegPerSec_ + gyroAlpha * (mappedGyroZDegPerSec - gyroZDegPerSec_);
  }

  const float accelRoll = atan2f(accelYg_, sqrtf(accelXg_ * accelXg_ + accelZg_ * accelZg_)) * 180.0f / PI;
  const float accelPitch = atan2f(-accelXg_, sqrtf(accelYg_ * accelYg_ + accelZg_ * accelZg_)) * 180.0f / PI;

  if (!attitude_.valid || dt <= 0.0f) {
    attitude_.rollDeg = accelRoll;
    attitude_.pitchDeg = accelPitch;
  } else {
    const float alpha = 0.98f;
    attitude_.rollDeg = alpha * (attitude_.rollDeg + gyroXDegPerSec_ * dt) + (1.0f - alpha) * accelRoll;
    attitude_.pitchDeg = alpha * (attitude_.pitchDeg + gyroYDegPerSec_ * dt) + (1.0f - alpha) * accelPitch;
  }

  attitude_.rollRateDegPerSec = gyroXDegPerSec_;
  attitude_.pitchRateDegPerSec = gyroYDegPerSec_;
  attitude_.yawRateDegPerSec = gyroZDegPerSec_;
  attitude_.valid = true;
  return true;
}

V7RCDroneV2AttitudeState V7RCDroneV2SimulatedIcm20948::attitude() const {
  return attitude_;
}

V7RCDroneV2Icm20948InputSample V7RCDroneV2SimulatedIcm20948::inputSample() const {
  return sample_;
}

float V7RCDroneV2SimulatedIcm20948::filteredAccelXg() const {
  return accelXg_;
}

float V7RCDroneV2SimulatedIcm20948::filteredAccelYg() const {
  return accelYg_;
}

float V7RCDroneV2SimulatedIcm20948::filteredAccelZg() const {
  return accelZg_;
}

float V7RCDroneV2SimulatedIcm20948::filteredGyroXDegPerSec() const {
  return gyroXDegPerSec_;
}

float V7RCDroneV2SimulatedIcm20948::filteredGyroYDegPerSec() const {
  return gyroYDegPerSec_;
}

float V7RCDroneV2SimulatedIcm20948::filteredGyroZDegPerSec() const {
  return gyroZDegPerSec_;
}

void V7RCDroneV2SimulatedIcm20948::setAxisTransform(
    V7RCIcm20948Axis logicalXSource,
    int8_t logicalXSign,
    V7RCIcm20948Axis logicalYSource,
    int8_t logicalYSign,
    V7RCIcm20948Axis logicalZSource,
    int8_t logicalZSign) {
  logicalXSource_ = logicalXSource;
  logicalYSource_ = logicalYSource;
  logicalZSource_ = logicalZSource;
  logicalXSign_ = logicalXSign >= 0 ? 1 : -1;
  logicalYSign_ = logicalYSign >= 0 ? 1 : -1;
  logicalZSign_ = logicalZSign >= 0 ? 1 : -1;
}
