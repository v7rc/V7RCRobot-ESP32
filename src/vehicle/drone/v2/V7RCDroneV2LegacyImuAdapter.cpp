#include "V7RCDroneV2LegacyImuAdapter.h"

V7RCDroneV2LegacyImuAdapter::V7RCDroneV2LegacyImuAdapter(V7RCDroneImu* imu, float rateFilterAlpha)
    : imu_(imu),
      rateFilterAlpha_(V7RCDroneV2Clamp(rateFilterAlpha, 0.0f, 1.0f)),
      lastUpdateMs_(0),
      hasPreviousAngles_(false),
      previousRollDeg_(0.0f),
      previousPitchDeg_(0.0f),
      attitude_{} {}

bool V7RCDroneV2LegacyImuAdapter::begin() {
  lastUpdateMs_ = 0;
  hasPreviousAngles_ = false;
  attitude_ = {};
  if (!imu_) {
    return false;
  }
  return imu_->begin();
}

bool V7RCDroneV2LegacyImuAdapter::update(unsigned long nowMs) {
  if (!imu_) {
    attitude_ = {};
    return false;
  }

  const bool updated = imu_->update(nowMs);
  const V7RCDroneAttitude raw = imu_->attitude();

  attitude_.rollDeg = raw.rollDeg;
  attitude_.pitchDeg = raw.pitchDeg;
  attitude_.yawRateDegPerSec = raw.yawRateDegPerSec;
  attitude_.valid = raw.valid;

  if (!raw.valid) {
    attitude_.rollRateDegPerSec = 0.0f;
    attitude_.pitchRateDegPerSec = 0.0f;
    hasPreviousAngles_ = false;
    lastUpdateMs_ = nowMs;
    return updated;
  }

  float rollRate = 0.0f;
  float pitchRate = 0.0f;
  if (hasPreviousAngles_ && nowMs > lastUpdateMs_) {
    const float dt = (float)(nowMs - lastUpdateMs_) / 1000.0f;
    if (dt > 0.0001f) {
      rollRate = (raw.rollDeg - previousRollDeg_) / dt;
      pitchRate = (raw.pitchDeg - previousPitchDeg_) / dt;
    }
  }

  attitude_.rollRateDegPerSec += rateFilterAlpha_ * (rollRate - attitude_.rollRateDegPerSec);
  attitude_.pitchRateDegPerSec += rateFilterAlpha_ * (pitchRate - attitude_.pitchRateDegPerSec);

  previousRollDeg_ = raw.rollDeg;
  previousPitchDeg_ = raw.pitchDeg;
  lastUpdateMs_ = nowMs;
  hasPreviousAngles_ = true;
  return updated;
}

V7RCDroneV2AttitudeState V7RCDroneV2LegacyImuAdapter::attitude() const {
  return attitude_;
}

const char* V7RCDroneV2LegacyImuAdapter::sensorName() const {
  if (!imu_) {
    return "NO_IMU";
  }
  return imu_->sensorName();
}

bool V7RCDroneV2LegacyImuAdapter::calibrateGyroBias(uint16_t samples, uint16_t sampleDelayMs) {
  if (!imu_) {
    return false;
  }
  return imu_->calibrateGyroBias(samples, sampleDelayMs);
}

void V7RCDroneV2LegacyImuAdapter::setRateFilterAlpha(float alpha) {
  rateFilterAlpha_ = V7RCDroneV2Clamp(alpha, 0.0f, 1.0f);
}
