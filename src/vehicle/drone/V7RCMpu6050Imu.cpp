#include "V7RCMpu6050Imu.h"

#include <Wire.h>
#include <math.h>

namespace {

static const uint8_t kPwrMgmt1Reg = 0x6B;
static const uint8_t kAccelStartReg = 0x3B;
static const float kAccelScale = 16384.0f;
static const float kGyroScale = 131.0f;

int16_t readBigEndian16(uint8_t highByte, uint8_t lowByte) {
  return (int16_t)((highByte << 8) | lowByte);
}

}  // namespace

V7RCMpu6050Imu::V7RCMpu6050Imu(uint8_t i2cAddress)
  : address_(i2cAddress), begun_(false), lastUpdateMs_(0) {
  attitude_.rollDeg = 0.0f;
  attitude_.pitchDeg = 0.0f;
  attitude_.yawRateDegPerSec = 0.0f;
  attitude_.valid = false;
}

bool V7RCMpu6050Imu::begin() {
  Wire.begin();
  Wire.beginTransmission(address_);
  Wire.write(kPwrMgmt1Reg);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    begun_ = false;
    return false;
  }

  delay(50);
  begun_ = true;
  lastUpdateMs_ = 0;
  attitude_.valid = false;
  return true;
}

bool V7RCMpu6050Imu::readRaw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(address_);
  Wire.write(kAccelStartReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t bytesRequested = 14;
  if (Wire.requestFrom((int)address_, (int)bytesRequested) != bytesRequested) {
    return false;
  }

  uint8_t raw[14];
  for (uint8_t i = 0; i < 14; ++i) {
    raw[i] = (uint8_t)Wire.read();
  }

  *ax = readBigEndian16(raw[0], raw[1]);
  *ay = readBigEndian16(raw[2], raw[3]);
  *az = readBigEndian16(raw[4], raw[5]);
  *gx = readBigEndian16(raw[8], raw[9]);
  *gy = readBigEndian16(raw[10], raw[11]);
  *gz = readBigEndian16(raw[12], raw[13]);
  return true;
}

bool V7RCMpu6050Imu::update(unsigned long nowMs) {
  if (!begun_) return false;

  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (!readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
    attitude_.valid = false;
    return false;
  }

  float dt = 0.0f;
  if (lastUpdateMs_ != 0 && nowMs > lastUpdateMs_) {
    dt = (float)(nowMs - lastUpdateMs_) / 1000.0f;
  }
  lastUpdateMs_ = nowMs;

  const float accelX = (float)ax / kAccelScale;
  const float accelY = (float)ay / kAccelScale;
  const float accelZ = (float)az / kAccelScale;
  const float gyroX = (float)gx / kGyroScale;
  const float gyroY = (float)gy / kGyroScale;
  const float gyroZ = (float)gz / kGyroScale;

  const float accelRoll = atan2f(accelY, accelZ) * 180.0f / PI;
  const float accelPitch = atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ)) * 180.0f / PI;

  if (!attitude_.valid || dt <= 0.0f) {
    attitude_.rollDeg = accelRoll;
    attitude_.pitchDeg = accelPitch;
  } else {
    const float alpha = 0.98f;
    attitude_.rollDeg = alpha * (attitude_.rollDeg + gyroX * dt) + (1.0f - alpha) * accelRoll;
    attitude_.pitchDeg = alpha * (attitude_.pitchDeg + gyroY * dt) + (1.0f - alpha) * accelPitch;
  }

  attitude_.yawRateDegPerSec = gyroZ;
  attitude_.valid = true;
  return true;
}

V7RCDroneAttitude V7RCMpu6050Imu::attitude() const {
  return attitude_;
}

const char* V7RCMpu6050Imu::sensorName() const {
  return "MPU6050";
}
