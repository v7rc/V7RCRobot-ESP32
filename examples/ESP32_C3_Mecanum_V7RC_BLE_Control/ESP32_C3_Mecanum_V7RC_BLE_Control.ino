#include <Arduino.h>
#include <V7RCCar-esp32.h>
#include <esp_mac.h>

namespace {

V7RCCarRobot robot;

V7RC_ServoConfig servos[] = {
  {7, 1000, 2000, 0.0f, 90.0f, 45.0f},
  {6, 1000, 2000, 0.0f, 90.0f, 45.0f},
};

// Update these pins to match your actual motor driver wiring.
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
  .differentialThrottleChannel = 0,
  .differentialSteerChannel = 1,
  .mecanumVxChannel = 0,
  .mecanumVyChannel = 1,
  .mecanumOmegaChannel = 3,
  .servos = servos,
  .numServos = sizeof(servos) / sizeof(servos[0]),
  .auxiliaryServo0Channel = 2,
  .auxiliaryServo1Channel = 5,
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

  Serial.println("Starting V7RC BLE mecanum control example.");
  Serial.println("Use the V7RC App layout: ch0=Vx, ch1=Vy, ch3=Omega, ch2=Servo(GPIO7), ch5=Servo(GPIO6).");

  // This path uses the BLE transport + V7RC protocol decode + legacy-compatible
  // car facade, so the robot can be driven directly from the V7RC App.
  robot.beginMecanum(robotIdFromBleMac(), robotOptions, 0, 1, 2, 3);
}

void loop() {
  robot.loop();
}
