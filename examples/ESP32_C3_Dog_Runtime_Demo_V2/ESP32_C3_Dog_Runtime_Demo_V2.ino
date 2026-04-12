#include <Arduino.h>
#include <V7RCQuadruped-esp32.h>

namespace {

V7RCQuadrupedRuntime runtime;

V7RCQuadrupedRuntimeOptions options = {
  .legs = {
    {20, 21, false, false},
    {10, 0, true, true},
    {1, 2, false, false},
    {3, 4, true, true},
  },
  .upperLegLength = 0.055f,
  .lowerLegLength = 0.080f,
  .nominalX = 0.012f,
  .nominalBodyHeight = 0.098f,
  .stepHeight = 0.028f,
  .stepLengthScale = 0.040f,
  .gaitCycleMs = 900.0f,
  .servoMinUs = 500,
  .servoMaxUs = 2400,
};

V7RCQuadrupedControlState controlState = {
  0.0f,
  0.0f,
  0.0f,
  0.098f,
};

void updateCommand(unsigned long nowMs) {
  const unsigned long phase = (nowMs / 1800UL) % 7UL;

  controlState = {0.0f, 0.0f, 0.0f, 0.098f};

  if (phase == 0) {
    controlState.vx = 0.55f;
  } else if (phase == 1) {
    controlState.vx = 0.35f;
    controlState.yaw = 0.35f;
  } else if (phase == 2) {
    controlState.bodyHeight = 0.090f;
    controlState.vx = 0.25f;
  } else if (phase == 3) {
    controlState.yaw = -0.40f;
  } else if (phase == 4) {
    controlState.bodyHeight = 0.108f;
    controlState.vx = 0.20f;
  } else if (phase == 5) {
    controlState.vx = -0.20f;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  runtime.begin(options);
  Serial.println("Quadruped runtime V2 ready.");
}

void loop() {
  const unsigned long nowMs = millis();
  updateCommand(nowMs);
  runtime.update(controlState, nowMs);
  delay(20);
}
