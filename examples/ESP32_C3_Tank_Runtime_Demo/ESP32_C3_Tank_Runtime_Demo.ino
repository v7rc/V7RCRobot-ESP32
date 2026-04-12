#include <Arduino.h>
#include <V7RCCar-esp32.h>

namespace {

V7RCCarRobot robot;

V7RC_DCMotorConfig motors[] = {
  {21, 20, false},  // left track
  {10, 0, false},   // right track
};

V7RCCarControlState currentState = V7RCCarControl::neutralState();

void applyStep(unsigned long nowMs) {
  const unsigned long phase = (nowMs / 1800UL) % 5UL;

  currentState = V7RCCarControl::neutralState();

  if (phase == 0) {
    currentState.throttle = 0.55f;   // forward
  } else if (phase == 1) {
    currentState.steer = 0.65f;      // pivot right
  } else if (phase == 2) {
    currentState.throttle = 0.40f;   // forward again
  } else if (phase == 3) {
    currentState.steer = -0.65f;     // pivot left
  } else {
    currentState.throttle = -0.40f;  // reverse
  }

  robot.applyControl(currentState);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  V7RCCarRobotOptions options = {
    .bleBaseName = "V7RC-TANK-RUNTIME",
    .motors = motors,
    .numMotors = sizeof(motors) / sizeof(motors[0]),
    .differentialThrottleChannel = 0,
    .differentialSteerChannel = 1,
    .mecanumVxChannel = 0,
    .mecanumVyChannel = 1,
    .mecanumOmegaChannel = 2,
    .servos = nullptr,
    .numServos = 0,
    .auxiliaryServo0Channel = 15,
    .auxiliaryServo1Channel = 15,
    .auxiliaryServo2Channel = 15,
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
