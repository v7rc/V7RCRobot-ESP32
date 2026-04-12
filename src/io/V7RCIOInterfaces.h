#pragma once

#include <Arduino.h>

class V7RCServoOutput {
public:
  virtual ~V7RCServoOutput() {}
  virtual void attach(uint8_t pin, uint16_t minUs, uint16_t maxUs) = 0;
  virtual void writeMicroseconds(uint16_t valueUs) = 0;
};

class V7RCDCMotorOutput {
public:
  virtual ~V7RCDCMotorOutput() {}
  virtual void attach(uint8_t dirPin, uint8_t pwmPin) = 0;
  virtual void writeNormalized(float value) = 0;
  virtual void stop() = 0;
};

class V7RCStatusLedOutput {
public:
  virtual ~V7RCStatusLedOutput() {}
  virtual void begin() = 0;
  virtual void setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) = 0;
  virtual void show() = 0;
  virtual void clear() = 0;
};
