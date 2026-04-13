#include "V7RCAdxl345Imu.h"

#include <Wire.h>
#include <math.h>

namespace {

static const uint8_t kDeviceIdReg = 0x00;
static const uint8_t kExpectedDeviceId = 0xE5;
static const uint8_t kBwRateReg = 0x2C;
static const uint8_t kPowerCtlReg = 0x2D;
static const uint8_t kDataFormatReg = 0x31;
static const uint8_t kDataStartReg = 0x32;
static const float kAccelScaleLsbPerG = 256.0f;

int16_t readLittleEndian16(uint8_t lowByte, uint8_t highByte) {
  return (int16_t)((highByte << 8) | lowByte);
}

}  // namespace

V7RCAdxl345Imu::V7RCAdxl345Imu(uint8_t sdaPin, uint8_t sclPin, uint8_t i2cAddress)
  : sdaPin_(sdaPin), sclPin_(sclPin), address_(i2cAddress), begun_(false),
    logicalXSource_(V7RC_ADXL345_AXIS_X), logicalYSource_(V7RC_ADXL345_AXIS_Y), logicalZSource_(V7RC_ADXL345_AXIS_Z),
    logicalXSign_(1), logicalYSign_(1), logicalZSign_(1),
    accelXg_(0.0f), accelYg_(0.0f), accelZg_(0.0f) {
  attitude_.rollDeg = 0.0f;
  attitude_.pitchDeg = 0.0f;
  attitude_.yawRateDegPerSec = 0.0f;
  attitude_.valid = false;
}

bool V7RCAdxl345Imu::begin() {
  Wire.begin(sdaPin_, sclPin_);

  Wire.beginTransmission(address_);
  Wire.write(kDeviceIdReg);
  if (Wire.endTransmission(false) != 0) {
    begun_ = false;
    return false;
  }

  if (Wire.requestFrom((int)address_, 1) != 1) {
    begun_ = false;
    return false;
  }

  const uint8_t deviceId = (uint8_t)Wire.read();
  if (deviceId != kExpectedDeviceId) {
    begun_ = false;
    return false;
  }

  Wire.beginTransmission(address_);
  Wire.write(kBwRateReg);
  Wire.write(0x0A);  // 100 Hz output data rate.
  if (Wire.endTransmission() != 0) {
    begun_ = false;
    return false;
  }

  Wire.beginTransmission(address_);
  Wire.write(kDataFormatReg);
  Wire.write(0x08);  // Full-resolution, +/-2g range.
  if (Wire.endTransmission() != 0) {
    begun_ = false;
    return false;
  }

  Wire.beginTransmission(address_);
  Wire.write(kPowerCtlReg);
  Wire.write(0x08);  // Measurement mode.
  if (Wire.endTransmission() != 0) {
    begun_ = false;
    return false;
  }

  delay(20);
  begun_ = true;
  attitude_.valid = false;
  return true;
}

bool V7RCAdxl345Imu::readRaw(int16_t* ax, int16_t* ay, int16_t* az) {
  Wire.beginTransmission(address_);
  Wire.write(kDataStartReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t bytesRequested = 6;
  if (Wire.requestFrom((int)address_, (int)bytesRequested) != bytesRequested) {
    return false;
  }

  uint8_t raw[6];
  for (uint8_t i = 0; i < 6; ++i) {
    raw[i] = (uint8_t)Wire.read();
  }

  *ax = readLittleEndian16(raw[0], raw[1]);
  *ay = readLittleEndian16(raw[2], raw[3]);
  *az = readLittleEndian16(raw[4], raw[5]);
  return true;
}

float V7RCAdxl345Imu::axisValue(float rawXg, float rawYg, float rawZg, V7RCAdxl345Axis axis, int8_t sign) const {
  float value = rawXg;
  if (axis == V7RC_ADXL345_AXIS_Y) {
    value = rawYg;
  } else if (axis == V7RC_ADXL345_AXIS_Z) {
    value = rawZg;
  }

  return sign >= 0 ? value : -value;
}

bool V7RCAdxl345Imu::update(unsigned long nowMs) {
  (void)nowMs;

  if (!begun_) return false;

  int16_t ax = 0, ay = 0, az = 0;
  if (!readRaw(&ax, &ay, &az)) {
    attitude_.valid = false;
    return false;
  }

  const float rawXg = (float)ax / kAccelScaleLsbPerG;
  const float rawYg = (float)ay / kAccelScaleLsbPerG;
  const float rawZg = (float)az / kAccelScaleLsbPerG;

  accelXg_ = axisValue(rawXg, rawYg, rawZg, logicalXSource_, logicalXSign_);
  accelYg_ = axisValue(rawXg, rawYg, rawZg, logicalYSource_, logicalYSign_);
  accelZg_ = axisValue(rawXg, rawYg, rawZg, logicalZSource_, logicalZSign_);

  attitude_.rollDeg = atan2f(accelYg_, accelZg_) * 180.0f / PI;
  attitude_.pitchDeg = atan2f(-accelXg_, sqrtf(accelYg_ * accelYg_ + accelZg_ * accelZg_)) * 180.0f / PI;
  attitude_.yawRateDegPerSec = 0.0f;
  attitude_.valid = true;
  return true;
}

V7RCDroneAttitude V7RCAdxl345Imu::attitude() const {
  return attitude_;
}

const char* V7RCAdxl345Imu::sensorName() const {
  return "ADXL345";
}

float V7RCAdxl345Imu::accelXg() const {
  return accelXg_;
}

float V7RCAdxl345Imu::accelYg() const {
  return accelYg_;
}

float V7RCAdxl345Imu::accelZg() const {
  return accelZg_;
}

void V7RCAdxl345Imu::setAxisTransform(
  V7RCAdxl345Axis logicalXSource,
  int8_t logicalXSign,
  V7RCAdxl345Axis logicalYSource,
  int8_t logicalYSign,
  V7RCAdxl345Axis logicalZSource,
  int8_t logicalZSign
) {
  logicalXSource_ = logicalXSource;
  logicalYSource_ = logicalYSource;
  logicalZSource_ = logicalZSource;
  logicalXSign_ = logicalXSign >= 0 ? 1 : -1;
  logicalYSign_ = logicalYSign >= 0 ? 1 : -1;
  logicalZSign_ = logicalZSign >= 0 ? 1 : -1;
}
