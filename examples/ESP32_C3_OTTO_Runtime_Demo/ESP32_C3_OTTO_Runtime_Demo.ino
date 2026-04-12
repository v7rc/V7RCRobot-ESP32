#include <Arduino.h>
#include <V7RCOtto-esp32.h>

namespace {

V7RCOttoRuntime runtime;

V7RCOttoRuntimeOptions options = {
  .leftHip = {20, false},
  .rightHip = {10, true},
  .leftFoot = {21, false},
  .rightFoot = {0, true},
  .servoMinUs = 500,
  .servoMaxUs = 2400,
  .hipAmplitudeDeg = 24.0f,
  .footAmplitudeDeg = 18.0f,
  .gaitCycleMs = 1000.0f,
};

V7RCOttoControlState state = {0.0f, 0.0f, 0.0f};

void updateCommand(unsigned long nowMs) {
  const unsigned long phase = (nowMs / 1800UL) % 5UL;
  state = {0.0f, 0.0f, 0.0f};

  if (phase == 0) {
    state.walk = 0.65f;
    state.bounce = 0.55f;
  } else if (phase == 1) {
    state.turn = 0.55f;
    state.bounce = 0.35f;
  } else if (phase == 2) {
    state.walk = -0.40f;
    state.bounce = 0.30f;
  } else if (phase == 3) {
    state.turn = -0.55f;
    state.bounce = 0.55f;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  runtime.begin(options);
  Serial.println("OTTO runtime ready.");
}

void loop() {
  updateCommand(millis());
  runtime.update(state, millis());
  delay(20);
}
