#include <Arduino.h>
#include <V7RCCar-esp32.h>
#include <esp_mac.h>

namespace {

V7RCCarRobot robot;

// Servo 0: turret rotation
// Servo 1: barrel elevation
// Servo 2: launch trigger
V7RC_ServoConfig servos[] = {
  {7, 1000, 2000, 0.0f, 180.0f, 90.0f},
  {6, 1000, 2000, 0.0f, 180.0f, 90.0f},
  {5, 1000, 2000, 0.0f, 180.0f, 0.0f},
};

V7RC_ChannelConfig channelMap[] = {
  {CH_DRIVE_STEER, -1},     // ch0
  {CH_DRIVE_THROTTLE, -1},  // ch1
  {CH_SERVO, 1},            // ch2 -> barrel elevation
  {CH_SERVO, 0},            // ch3 -> turret rotation
  {CH_SERVO, 2},            // ch4 -> launcher
  {CH_NONE, -1},            // ch5
  {CH_NONE, -1},            // ch6
  {CH_NONE, -1},            // ch7
  {CH_NONE, -1},            // ch8
  {CH_NONE, -1},            // ch9
  {CH_NONE, -1},            // ch10
  {CH_NONE, -1},            // ch11
  {CH_NONE, -1},            // ch12
  {CH_NONE, -1},            // ch13
  {CH_NONE, -1},            // ch14
  {CH_NONE, -1},            // ch15
};

// Motor 0: left track, Motor 1: right track
V7RC_DCMotorConfig motors[] = {
  {21, 20, false},
  {10, 0, false},
};

V7RCCarRobotOptions robotOptions = {
  .bleBaseName = "V7RC-TANK",
  .motors = motors,
  .numMotors = sizeof(motors) / sizeof(motors[0]),
  .customChannelMap = channelMap,
  .numCustomChannelMap = sizeof(channelMap) / sizeof(channelMap[0]),
  .differentialThrottleChannel = 1,
  .differentialSteerChannel = 0,
  .mecanumVxChannel = 0,
  .mecanumVyChannel = 1,
  .mecanumOmegaChannel = 2,
  .servos = servos,
  .numServos = sizeof(servos) / sizeof(servos[0]),
  .auxiliaryServo0Channel = 3,
  .auxiliaryServo1Channel = 2,
  .auxiliaryServo2Channel = 4,
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

  Serial.println("Starting V7RC BLE tank control example.");
  Serial.println("ch0=Steer, ch1=Throttle, ch2=Barrel, ch3=Turret, ch4=Launch.");

  robot.beginDifferential(robotIdFromBleMac(), robotOptions, 0, 1);
}

void loop() {
  robot.loop();
}
