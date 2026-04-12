#include <Arduino.h>
#include <V7RCRobotArm-esp32.h>

namespace {

V7RCRobotArmRuntime runtime;

V7RCRobotArmRuntimeOptions options = {
  .base = {20, false},
  .shoulder = {21, false},
  .elbow = {10, true},
  .wrist = {0, false},
  .gripper = {1, false},
  .servoMinUs = 500,
  .servoMaxUs = 2400,
};

const V7RCRobotArmPose poses[] = {
  {0.0f, -20.0f, 35.0f, 0.0f, -30.0f},
  {-35.0f, -10.0f, 55.0f, 15.0f, -10.0f},
  {35.0f, -30.0f, 25.0f, -15.0f, 20.0f},
  {0.0f, -45.0f, 60.0f, 25.0f, 30.0f},
};

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  runtime.begin(options);
  Serial.println("Robot arm runtime ready.");
}

void loop() {
  const size_t poseIndex = (millis() / 1800UL) % (sizeof(poses) / sizeof(poses[0]));
  runtime.moveTo(poses[poseIndex]);
  delay(20);
}
