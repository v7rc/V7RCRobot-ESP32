#include "V7RCOttoRuntime.h"

#include <math.h>

namespace {

const float kPi = 3.14159265f;

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

}  // namespace

V7RCOttoRuntime::V7RCOttoRuntime()
  : started_(false),
    startMs_(0) {}

void V7RCOttoRuntime::begin(const V7RCOttoRuntimeOptions& options) {
  options_ = options;
  startMs_ = millis();
  started_ = true;

  leftHip_.attach(options_.leftHip.pin, options_.servoMinUs, options_.servoMaxUs);
  rightHip_.attach(options_.rightHip.pin, options_.servoMinUs, options_.servoMaxUs);
  leftFoot_.attach(options_.leftFoot.pin, options_.servoMinUs, options_.servoMaxUs);
  rightFoot_.attach(options_.rightFoot.pin, options_.servoMinUs, options_.servoMaxUs);

  stand();
}

void V7RCOttoRuntime::update(const V7RCOttoControlState& state, unsigned long nowMs) {
  if (!started_) return;

  const float cycleMs = (options_.gaitCycleMs > 1.0f) ? options_.gaitCycleMs : 1100.0f;
  const float phase = (2.0f * kPi * (float)(nowMs - startMs_)) / cycleMs;
  const float walk = clampFloat(state.walk, -1.0f, 1.0f);
  const float turn = clampFloat(state.turn, -1.0f, 1.0f);
  const float bounce = clampFloat(state.bounce, -1.0f, 1.0f);

  const float hipWave = sinf(phase);
  const float footWave = cosf(phase);

  const float leftHipDeg = (hipWave * options_.hipAmplitudeDeg * walk) - (turn * options_.hipAmplitudeDeg * 0.6f);
  const float rightHipDeg = (-hipWave * options_.hipAmplitudeDeg * walk) - (turn * options_.hipAmplitudeDeg * 0.6f);
  const float leftFootDeg = (footWave * options_.footAmplitudeDeg * bounce) + (walk * options_.footAmplitudeDeg * 0.4f);
  const float rightFootDeg = (-footWave * options_.footAmplitudeDeg * bounce) + (walk * options_.footAmplitudeDeg * 0.4f);

  leftHip_.writeMicroseconds(angleToPulse(leftHipDeg, options_.leftHip.invert));
  rightHip_.writeMicroseconds(angleToPulse(rightHipDeg, options_.rightHip.invert));
  leftFoot_.writeMicroseconds(angleToPulse(leftFootDeg, options_.leftFoot.invert));
  rightFoot_.writeMicroseconds(angleToPulse(rightFootDeg, options_.rightFoot.invert));
}

void V7RCOttoRuntime::stand() {
  if (!started_) return;

  leftHip_.writeMicroseconds(angleToPulse(0.0f, options_.leftHip.invert));
  rightHip_.writeMicroseconds(angleToPulse(0.0f, options_.rightHip.invert));
  leftFoot_.writeMicroseconds(angleToPulse(0.0f, options_.leftFoot.invert));
  rightFoot_.writeMicroseconds(angleToPulse(0.0f, options_.rightFoot.invert));
}

uint16_t V7RCOttoRuntime::angleToPulse(float angleDeg, bool inverted) const {
  float normalized = (clampFloat(angleDeg, -90.0f, 90.0f) + 90.0f) / 180.0f;
  if (inverted) {
    normalized = 1.0f - normalized;
  }
  return (uint16_t)(options_.servoMinUs + (normalized * (float)(options_.servoMaxUs - options_.servoMinUs)));
}
