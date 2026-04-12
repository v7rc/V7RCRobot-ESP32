#include <Arduino.h>
#include <V7RCCar-esp32.h>
#include <esp_mac.h>

namespace {

V7RCCarRobot robot;

V7RC_ServoConfig servos[] = {
  {7, 1000, 2000, 0.0f, 90.0f, 45.0f},
  {6, 1000, 2000, 0.0f, 90.0f, 45.0f},
};

// Channel Map：使用與舊版範例相同的定義方式，直接在 .ino 配置每個 channel 的功能。
V7RC_ChannelConfig channelMap[] = {
  {CH_DRIVE_MEC_VX, -1},     // ch0
  {CH_DRIVE_MEC_VY, -1},     // ch1
  {CH_SERVO, 0},             // ch2 -> Servo 0 (GPIO7)
  {CH_DRIVE_MEC_OMEGA, -1},  // ch3
  {CH_NONE, -1},             // ch4
  {CH_SERVO, 1},             // ch5 -> Servo 1 (GPIO6)
  {CH_NONE, -1},             // ch6
  {CH_NONE, -1},             // ch7
  {CH_NONE, -1},             // ch8
  {CH_NONE, -1},             // ch9
  {CH_NONE, -1},             // ch10
  {CH_NONE, -1},             // ch11
  {CH_NONE, -1},             // ch12
  {CH_NONE, -1},             // ch13
  {CH_NONE, -1},             // ch14
  {CH_NONE, -1},             // ch15
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
  .customChannelMap = channelMap,
  .numCustomChannelMap = sizeof(channelMap) / sizeof(channelMap[0]),
  .differentialThrottleChannel = 0,
  .differentialSteerChannel = 1,
  .mecanumVxChannel = 0,
  .mecanumVyChannel = 1,
  .mecanumOmegaChannel = 3,
  .servos = servos,
  .numServos = sizeof(servos) / sizeof(servos[0]),
  .auxiliaryServo0Channel = 2,
  .auxiliaryServo1Channel = 5,
  .auxiliaryServo2Channel = 15,
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
  Serial.println("Use the V7RC App layout: ch0=Vx, ch1=Vy, ch2=Servo(GPIO7), ch3=Omega, ch5=Servo(GPIO6).");

  // This path uses the BLE transport + V7RC protocol decode + legacy-compatible
  // car facade, so the robot can be driven directly from the V7RC App.
  robot.beginMecanum(robotIdFromBleMac(), robotOptions, 0, 1, 2, 3);
}

void loop() {
  robot.loop();
}
