#include "V7RCEsp32Outputs.h"

namespace {

float clampUnit(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

}  // namespace

V7RCEsp32ServoOutput::V7RCEsp32ServoOutput() : attached_(false) {}

void V7RCEsp32ServoOutput::attach(uint8_t pin, uint16_t minUs, uint16_t maxUs) {
  servo_.setPeriodHertz(50);
  servo_.attach(pin, minUs, maxUs);
  attached_ = true;
}

void V7RCEsp32ServoOutput::writeMicroseconds(uint16_t valueUs) {
  if (!attached_) return;
  servo_.writeMicroseconds(valueUs);
}

V7RCEsp32DCMotorOutput::V7RCEsp32DCMotorOutput() : dirPin_(0), pwmPin_(0), dirInvert_(false), attached_(false) {}

void V7RCEsp32DCMotorOutput::attach(uint8_t dirPin, uint8_t pwmPin, bool dirInvert) {
  dirPin_ = dirPin;
  pwmPin_ = pwmPin;
  dirInvert_ = dirInvert;
  pinMode(dirPin_, OUTPUT);
  pinMode(pwmPin_, OUTPUT);
  digitalWrite(dirPin_, LOW);
  analogWrite(pwmPin_, 0);
  attached_ = true;
}

void V7RCEsp32DCMotorOutput::writeNormalized(float value) {
  if (!attached_) return;

  value = clampUnit(value);
  if (dirInvert_) {
    value = -value;
  }
  const float deadband = 0.03f;
  if (value > -deadband && value < deadband) {
    stop();
    return;
  }

  int power = (int)(fabsf(value) * 255.0f);
  if (value < 0) {
    power = (int)(fabsf(1.0f + value) * 255.0f);
  }
  power = constrain(power, 0, 255);

  bool dirLevel = (value > 0) ? LOW : HIGH;
  digitalWrite(dirPin_, dirLevel);
  analogWrite(pwmPin_, power);
}

void V7RCEsp32DCMotorOutput::stop() {
  if (!attached_) return;
  digitalWrite(dirPin_, LOW);
  analogWrite(pwmPin_, 0);
}

V7RCWs2812StatusLedOutput::V7RCWs2812StatusLedOutput(uint8_t pin, uint8_t count)
  : pin_(pin), count_(count), strip_(nullptr) {}

V7RCWs2812StatusLedOutput::~V7RCWs2812StatusLedOutput() {
  delete strip_;
}

void V7RCWs2812StatusLedOutput::begin() {
  if (!strip_) {
    strip_ = new Adafruit_NeoPixel(count_, pin_, NEO_GRB + NEO_KHZ800);
  }
  strip_->begin();
}

void V7RCWs2812StatusLedOutput::setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  if (!strip_ || index >= count_) return;
  strip_->setPixelColor(index, strip_->Color(r, g, b));
}

void V7RCWs2812StatusLedOutput::show() {
  if (!strip_) return;
  strip_->show();
}

void V7RCWs2812StatusLedOutput::clear() {
  if (!strip_) return;
  strip_->clear();
  strip_->show();
}
