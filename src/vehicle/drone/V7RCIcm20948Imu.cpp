#include "V7RCIcm20948Imu.h"

#include <Wire.h>
#include <math.h>

namespace {

static const uint8_t kRegBankSel = 0x7F;

static const uint8_t kBank0 = 0x00;
static const uint8_t kBank2 = 0x20;

static const uint8_t kWhoAmIReg = 0x00;
static const uint8_t kExpectedWhoAmI = 0xEA;
static const uint8_t kPwrMgmt1Reg = 0x06;
static const uint8_t kPwrMgmt2Reg = 0x07;
static const uint8_t kAccelXoutHReg = 0x2D;

static const uint8_t kGyroConfig1Reg = 0x01;
static const uint8_t kAccelConfigReg = 0x14;

static const float kAccelScale = 16384.0f;  // +/-2g
static const float kGyroScale = 131.0f;     // +/-250 dps

int16_t readBigEndian16(uint8_t highByte, uint8_t lowByte) {
  return (int16_t)((highByte << 8) | lowByte);
}

}  // namespace

V7RCIcm20948Imu::V7RCIcm20948Imu(uint8_t sdaPin, uint8_t sclPin, uint8_t i2cAddress)
  : sdaPin_(sdaPin),
    sclPin_(sclPin),
    address_(i2cAddress),
    currentBank_(0xFF),
    begun_(false),
    lastUpdateMs_(0),
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
    gyroZDegPerSec_(0.0f),
    gyroBiasXDegPerSec_(0.0f),
    gyroBiasYDegPerSec_(0.0f),
    gyroBiasZDegPerSec_(0.0f),
    filterPrimed_(false) {
  attitude_.rollDeg = 0.0f;
  attitude_.pitchDeg = 0.0f;
  attitude_.yawRateDegPerSec = 0.0f;
  attitude_.valid = false;
}

float V7RCIcm20948Imu::axisValue(float rawX, float rawY, float rawZ, V7RCIcm20948Axis axis, int8_t sign) const {
  float value = rawX;
  if (axis == V7RC_ICM20948_AXIS_Y) {
    value = rawY;
  } else if (axis == V7RC_ICM20948_AXIS_Z) {
    value = rawZ;
  }

  return sign >= 0 ? value : -value;
}

bool V7RCIcm20948Imu::selectBank(uint8_t bank) {
  if (currentBank_ == bank) {
    return true;
  }

  Wire.beginTransmission(address_);
  Wire.write(kRegBankSel);
  Wire.write(bank);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  currentBank_ = bank;
  return true;
}

bool V7RCIcm20948Imu::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address_);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool V7RCIcm20948Imu::readRegisters(uint8_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(address_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)address_, (int)len) != (int)len) {
    return false;
  }

  for (size_t i = 0; i < len; ++i) {
    data[i] = (uint8_t)Wire.read();
  }
  return true;
}

bool V7RCIcm20948Imu::begin() {
  Wire.begin(sdaPin_, sclPin_);
  currentBank_ = 0xFF;

  if (!selectBank(kBank0)) {
    begun_ = false;
    return false;
  }

  uint8_t whoAmI = 0;
  if (!readRegisters(kWhoAmIReg, &whoAmI, 1) || whoAmI != kExpectedWhoAmI) {
    begun_ = false;
    return false;
  }

  if (!writeRegister(kPwrMgmt1Reg, 0x01)) {  // Auto selects best available clock.
    begun_ = false;
    return false;
  }
  if (!writeRegister(kPwrMgmt2Reg, 0x00)) {  // Enable accel + gyro.
    begun_ = false;
    return false;
  }

  if (!selectBank(kBank2)) {
    begun_ = false;
    return false;
  }

  if (!writeRegister(kGyroConfig1Reg, 0x00)) {  // +/-250 dps, DLPF off.
    begun_ = false;
    return false;
  }
  if (!writeRegister(kAccelConfigReg, 0x00)) {  // +/-2g, DLPF off.
    begun_ = false;
    return false;
  }

  if (!selectBank(kBank0)) {
    begun_ = false;
    return false;
  }

  delay(50);
  begun_ = true;
  lastUpdateMs_ = 0;
  attitude_.valid = false;
  filterPrimed_ = false;
  gyroBiasXDegPerSec_ = 0.0f;
  gyroBiasYDegPerSec_ = 0.0f;
  gyroBiasZDegPerSec_ = 0.0f;
  return true;
}

bool V7RCIcm20948Imu::calibrateGyroBias(uint16_t samples, uint16_t sampleDelayMs) {
  if (!begun_ || samples == 0) return false;

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (!readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
      return false;
    }

    const float rawGyroXDegPerSec = (float)gx / kGyroScale;
    const float rawGyroYDegPerSec = (float)gy / kGyroScale;
    const float rawGyroZDegPerSec = (float)gz / kGyroScale;

    sumX += axisValue(rawGyroXDegPerSec, rawGyroYDegPerSec, rawGyroZDegPerSec, logicalXSource_, logicalXSign_);
    sumY += axisValue(rawGyroXDegPerSec, rawGyroYDegPerSec, rawGyroZDegPerSec, logicalYSource_, logicalYSign_);
    sumZ += axisValue(rawGyroXDegPerSec, rawGyroYDegPerSec, rawGyroZDegPerSec, logicalZSource_, logicalZSign_);

    delay(sampleDelayMs);
  }

  gyroBiasXDegPerSec_ = sumX / samples;
  gyroBiasYDegPerSec_ = sumY / samples;
  gyroBiasZDegPerSec_ = sumZ / samples;
  filterPrimed_ = false;
  lastUpdateMs_ = 0;
  return true;
}

bool V7RCIcm20948Imu::readRaw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  if (!selectBank(kBank0)) {
    return false;
  }

  uint8_t raw[12];
  if (!readRegisters(kAccelXoutHReg, raw, sizeof(raw))) {
    return false;
  }

  *ax = readBigEndian16(raw[0], raw[1]);
  *ay = readBigEndian16(raw[2], raw[3]);
  *az = readBigEndian16(raw[4], raw[5]);
  *gx = readBigEndian16(raw[6], raw[7]);
  *gy = readBigEndian16(raw[8], raw[9]);
  *gz = readBigEndian16(raw[10], raw[11]);
  return true;
}

bool V7RCIcm20948Imu::update(unsigned long nowMs) {
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

  const float rawAccelXg = (float)ax / kAccelScale;
  const float rawAccelYg = (float)ay / kAccelScale;
  const float rawAccelZg = (float)az / kAccelScale;
  const float rawGyroXDegPerSec = (float)gx / kGyroScale;
  const float rawGyroYDegPerSec = (float)gy / kGyroScale;
  const float rawGyroZDegPerSec = (float)gz / kGyroScale;

  const float mappedAccelXg = axisValue(rawAccelXg, rawAccelYg, rawAccelZg, logicalXSource_, logicalXSign_);
  const float mappedAccelYg = axisValue(rawAccelXg, rawAccelYg, rawAccelZg, logicalYSource_, logicalYSign_);
  const float mappedAccelZg = axisValue(rawAccelXg, rawAccelYg, rawAccelZg, logicalZSource_, logicalZSign_);
  const float mappedGyroXDegPerSec =
    axisValue(rawGyroXDegPerSec, rawGyroYDegPerSec, rawGyroZDegPerSec, logicalXSource_, logicalXSign_) -
    gyroBiasXDegPerSec_;
  const float mappedGyroYDegPerSec =
    axisValue(rawGyroXDegPerSec, rawGyroYDegPerSec, rawGyroZDegPerSec, logicalYSource_, logicalYSign_) -
    gyroBiasYDegPerSec_;
  const float mappedGyroZDegPerSec =
    axisValue(rawGyroXDegPerSec, rawGyroYDegPerSec, rawGyroZDegPerSec, logicalZSource_, logicalZSign_) -
    gyroBiasZDegPerSec_;

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

  // Use small-angle attitude formulas that stay in a stable [-90, +90] range.
  // This avoids roll jumping near 180 degrees when the mounting orientation flips Z.
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

  attitude_.yawRateDegPerSec = gyroZDegPerSec_;
  attitude_.valid = true;
  return true;
}

V7RCDroneAttitude V7RCIcm20948Imu::attitude() const {
  return attitude_;
}

const char* V7RCIcm20948Imu::sensorName() const {
  return "ICM20948";
}

float V7RCIcm20948Imu::accelXg() const {
  return accelXg_;
}

float V7RCIcm20948Imu::accelYg() const {
  return accelYg_;
}

float V7RCIcm20948Imu::accelZg() const {
  return accelZg_;
}

float V7RCIcm20948Imu::gyroXDegPerSec() const {
  return gyroXDegPerSec_;
}

float V7RCIcm20948Imu::gyroYDegPerSec() const {
  return gyroYDegPerSec_;
}

float V7RCIcm20948Imu::gyroZDegPerSec() const {
  return gyroZDegPerSec_;
}

void V7RCIcm20948Imu::setAxisTransform(
  V7RCIcm20948Axis logicalXSource,
  int8_t logicalXSign,
  V7RCIcm20948Axis logicalYSource,
  int8_t logicalYSign,
  V7RCIcm20948Axis logicalZSource,
  int8_t logicalZSign
) {
  logicalXSource_ = logicalXSource;
  logicalYSource_ = logicalYSource;
  logicalZSource_ = logicalZSource;
  logicalXSign_ = logicalXSign >= 0 ? 1 : -1;
  logicalYSign_ = logicalYSign >= 0 ? 1 : -1;
  logicalZSign_ = logicalZSign >= 0 ? 1 : -1;
}
