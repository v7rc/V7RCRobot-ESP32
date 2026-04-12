#include <Arduino.h>
#include <V7RCQuadruped-esp32.h>

namespace {

// This runtime follows the same high-level split used by practical quadruped
// controllers such as Stanford Pupper: command -> gait phase -> foot target ->
// inverse kinematics -> servo output.
V7RCQuadrupedRuntime runtime;

V7RCQuadrupedRuntimeOptions options = {
  .legs = {
    {20, 21, false, false},  // front-left
    {10, 0, true, true},     // front-right
    {1, 2, false, false},    // rear-left
    {3, 4, true, true},      // rear-right
  },
  .upperLegLength = 0.055f,
  .lowerLegLength = 0.080f,
  .nominalX = 0.015f,
  .nominalBodyHeight = 0.105f,
  .stepHeight = 0.020f,
  .stepLengthScale = 0.030f,
  .gaitCycleMs = 1200.0f,
  .servoMinUs = 500,
  .servoMaxUs = 2400,
};

V7RCQuadrupedControlState controlState = {
  0.0f,
  0.0f,
  0.0f,
  0.105f,
};

void updateCommand(unsigned long nowMs) {
  const unsigned long phase = (nowMs / 2200UL) % 6UL;

  controlState = {0.0f, 0.0f, 0.0f, 0.105f};

  if (phase == 0) {
    controlState.vx = 0.45f;   // forward trot
  } else if (phase == 1) {
    controlState.yaw = 0.35f;  // turn right
  } else if (phase == 2) {
    controlState.vx = 0.30f;
    controlState.yaw = -0.25f;
  } else if (phase == 3) {
    controlState.bodyHeight = 0.095f;  // crouch walk
    controlState.vx = 0.20f;
  } else if (phase == 4) {
    controlState.bodyHeight = 0.112f;  // taller stance
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  runtime.begin(options);
  Serial.println("Quadruped runtime ready.");
}

void loop() {
  const unsigned long nowMs = millis();
  updateCommand(nowMs);
  runtime.update(controlState, nowMs);
  delay(20);
}
