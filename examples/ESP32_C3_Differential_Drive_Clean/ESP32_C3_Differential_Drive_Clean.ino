#include <Arduino.h>
#include <V7RCCar-esp32.h>
#include <esp_mac.h>

namespace {

V7RCCarRobot robot;

V7RC_DCMotorConfig motors[] = {
  {21, 20, false},  // left
  {10, 0, false},   // right
};

V7RCCarRobotOptions robotOptions = {
  .bleBaseName = "V7RC-CAR",
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
  .ws2812Brightness = 50,
  .ws2812Enable = true,
  .ws2812Pin = 8,
  .ws2812Count = 8,
};

uint32_t robotIdFromBleMac() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  return ((uint32_t)mac[4] << 8) | mac[5];
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  robot.beginDifferential(robotIdFromBleMac(), robotOptions, 0, 1);
}

void loop() {
  robot.loop();
}
