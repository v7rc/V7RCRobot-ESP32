#include <Arduino.h>
#include <V7RCCar-esp32.h>

namespace {

V7RCCarRobot robot;

V7RC_DCMotorConfig motors[] = {
  {20, 21, false},  // front-left
  {10, 0, false},   // front-right
  {1, 2, false},    // rear-left
  {3, 4, false},    // rear-right
};

V7RCCarControlState currentState = V7RCCarControl::neutralState();

void applyStep(unsigned long nowMs) {
  const unsigned long phase = (nowMs / 2000UL) % 4UL;

  currentState = V7RCCarControl::neutralState();

  if (phase == 0) {
    currentState.vy = 0.45f;
  } else if (phase == 1) {
    currentState.vx = 0.45f;
  } else if (phase == 2) {
    currentState.omega = 0.35f;
  } else {
    currentState.vy = -0.40f;
  }

  robot.applyControl(currentState);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  V7RCCarRobotOptions options = {
    .bleBaseName = "V7RC-MEC-RUNTIME",
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

  robot.beginMecanumRuntime(options, 0, 1, 2, 3);
}

void loop() {
  applyStep(millis());
  robot.loop();
  delay(20);
}
