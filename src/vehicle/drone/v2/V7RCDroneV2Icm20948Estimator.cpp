#include "V7RCDroneV2Icm20948Estimator.h"

V7RCDroneV2Icm20948Estimator::V7RCDroneV2Icm20948Estimator(V7RCIcm20948Imu* imu)
    : imu_(imu),
      attitude_{} {}

bool V7RCDroneV2Icm20948Estimator::begin() {
  attitude_ = {};
  if (!imu_) {
    return false;
  }
  return imu_->begin();
}

bool V7RCDroneV2Icm20948Estimator::update(unsigned long nowMs) {
  if (!imu_) {
    attitude_ = {};
    return false;
  }

  const bool updated = imu_->update(nowMs);
  const V7RCDroneAttitude baseAttitude = imu_->attitude();

  attitude_.rollDeg = baseAttitude.rollDeg;
  attitude_.pitchDeg = baseAttitude.pitchDeg;
  attitude_.rollRateDegPerSec = imu_->gyroXDegPerSec();
  attitude_.pitchRateDegPerSec = imu_->gyroYDegPerSec();
  attitude_.yawRateDegPerSec = imu_->gyroZDegPerSec();
  attitude_.valid = baseAttitude.valid;
  return updated;
}

V7RCDroneV2AttitudeState V7RCDroneV2Icm20948Estimator::attitude() const {
  return attitude_;
}

const char* V7RCDroneV2Icm20948Estimator::sensorName() const {
  if (!imu_) {
    return "NO_IMU";
  }
  return imu_->sensorName();
}

bool V7RCDroneV2Icm20948Estimator::calibrateGyroBias(uint16_t samples, uint16_t sampleDelayMs) {
  if (!imu_) {
    return false;
  }
  return imu_->calibrateGyroBias(samples, sampleDelayMs);
}
