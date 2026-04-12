#include <Arduino.h>
#include <V7RCCar-esp32.h>
#include <esp_mac.h>

namespace {

V7RCCarRobot robot;

V7RC_DCMotorConfig motors[] = {
  {20, 21, false},  // front-left
  {10, 0, false},   // front-right
  {1, 2, false},    // rear-left
  {3, 4, false},    // rear-right
};

V7RCCarRobotOptions robotOptions = {
  .bleBaseName = "V7RC-MEC",
  .motors = motors,
  .numMotors = sizeof(motors) / sizeof(motors[0]),
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

  robot.beginMecanum(robotIdFromBleMac(), robotOptions, 0, 1, 2, 3);
}

void loop() {
  robot.loop();
}
