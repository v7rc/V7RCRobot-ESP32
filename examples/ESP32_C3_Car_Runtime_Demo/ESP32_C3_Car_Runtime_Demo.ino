#include <Arduino.h>
#include <V7RCCar-esp32.h>

namespace {

V7RCCarRobot robot;

V7RC_DCMotorConfig motors[] = {
  {21, 20, false},  // left
  {10, 0, false},   // right
};

V7RCCarControlState currentState = V7RCCarControl::neutralState();

void applyStep(unsigned long nowMs) {
  const unsigned long phase = (nowMs / 1500UL) % 6UL;

  currentState = V7RCCarControl::neutralState();

  if (phase == 0) {
    currentState.throttle = 0.55f;   // forward
  } else if (phase == 1) {
    currentState.steer = 0.50f;      // pivot right
  } else if (phase == 2) {
    currentState.throttle = 0.35f;
    currentState.steer = 0.25f;      // forward-right arc
  } else if (phase == 3) {
    currentState.throttle = -0.45f;  // reverse
  } else if (phase == 4) {
    currentState.steer = -0.50f;     // pivot left
  } else {
    currentState.throttle = 0.0f;    // pause
    currentState.steer = 0.0f;
  }

  robot.applyControl(currentState);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  V7RCCarRobotOptions options = {
    .bleBaseName = "V7RC-CAR-RUNTIME",
    .motors = motors,
    .numMotors = sizeof(motors) / sizeof(motors[0]),
    .ws2812Brightness = 0,
    .ws2812Enable = false,
    .ws2812Pin = 0,
    .ws2812Count = 0,
  };

  robot.beginDifferentialRuntime(options, 0, 1);
}

void loop() {
  applyStep(millis());
  robot.loop();
  delay(20);
}
