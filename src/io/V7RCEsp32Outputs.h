#pragma once

#include "V7RCIOInterfaces.h"

#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

class V7RCEsp32ServoOutput : public V7RCServoOutput {
public:
  V7RCEsp32ServoOutput();

  void attach(uint8_t pin, uint16_t minUs, uint16_t maxUs) override;
  void writeMicroseconds(uint16_t valueUs) override;

private:
  Servo servo_;
  bool attached_;
};

class V7RCEsp32DCMotorOutput : public V7RCDCMotorOutput {
public:
  V7RCEsp32DCMotorOutput();

  void attach(uint8_t dirPin, uint8_t pwmPin, bool dirInvert = false) override;
  void writeNormalized(float value) override;
  void stop() override;

private:
  uint8_t dirPin_;
  uint8_t pwmPin_;
  bool dirInvert_;
  bool attached_;
};

class V7RCWs2812StatusLedOutput : public V7RCStatusLedOutput {
public:
  V7RCWs2812StatusLedOutput(uint8_t pin, uint8_t count);
  ~V7RCWs2812StatusLedOutput() override;

  void begin() override;
  void setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) override;
  void show() override;
  void clear() override;

private:
  uint8_t pin_;
  uint8_t count_;
  Adafruit_NeoPixel* strip_;
};
