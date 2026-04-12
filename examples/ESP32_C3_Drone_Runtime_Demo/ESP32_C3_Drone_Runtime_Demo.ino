#include <Arduino.h>
#include <V7RCDrone-esp32.h>

namespace {

V7RCDroneRuntime runtime;
V7RCMpu6050Imu imu;

V7RCDroneControlState currentState = {0.0f, 0.0f, 0.0f, 0.0f};
bool stabilizationEnabled = true;
bool imuReady = false;

V7RCDroneRuntimeOptions options = {
  .motorPins = {20, 10, 1, 3},
  .stabilizationEnabled = true,
  .maxTiltDeg = 18.0f,
  .rollKp = 0.85f,
  .pitchKp = 0.85f,
  .yawGain = 0.35f,
  .escMinUs = 1000,
  .escMaxUs = 2000,
  .escIdleUs = 1080,
};

void handleSerial() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "ARM") {
    runtime.beginUnlock();
    Serial.println("Unlock sequence started. Keep throttle low for 1.5s.");
  } else if (command == "DISARM") {
    runtime.disarm();
    Serial.println("Drone disarmed.");
  } else if (command == "STABILIZE ON") {
    stabilizationEnabled = true;
    runtime.setStabilizationEnabled(true);
    Serial.println("Stabilization enabled.");
  } else if (command == "STABILIZE OFF") {
    stabilizationEnabled = false;
    runtime.setStabilizationEnabled(false);
    Serial.println("Stabilization disabled.");
  } else if (command == "STATUS") {
    V7RCDroneAttitude attitude = runtime.attitude();
    Serial.printf(
      "armed=%d unlockPending=%d stabilize=%d imu=%d roll=%.2f pitch=%.2f yawRate=%.2f\n",
      runtime.isArmed() ? 1 : 0,
      runtime.unlockInProgress() ? 1 : 0,
      runtime.stabilizationEnabled() ? 1 : 0,
      imuReady ? 1 : 0,
      attitude.rollDeg,
      attitude.pitchDeg,
      attitude.yawRateDegPerSec
    );
  }
}

void applyStep(unsigned long nowMs) {
  const unsigned long phase = (nowMs / 1800UL) % 5UL;

  currentState = {0.0f, 0.0f, 0.0f, 0.0f};

  if (phase == 0) {
    currentState.throttle = 0.30f;
  } else if (phase == 1) {
    currentState.throttle = 0.35f;
    currentState.yaw = 0.25f;
  } else if (phase == 2) {
    currentState.throttle = 0.36f;
    currentState.roll = 0.25f;
  } else if (phase == 3) {
    currentState.throttle = 0.36f;
    currentState.pitch = -0.25f;
  } else {
    currentState.throttle = 0.22f;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  imuReady = runtime.begin(options, &imu);
  runtime.setStabilizationEnabled(stabilizationEnabled);

  Serial.println("Drone runtime ready.");
  Serial.println("Commands: ARM, DISARM, STABILIZE ON, STABILIZE OFF, STATUS");
  if (!imuReady) {
    Serial.println("MPU6050 init failed. Stabilization data will be invalid until sensor responds.");
  }
}

void loop() {
  handleSerial();
  applyStep(millis());
  runtime.update(currentState, millis());
  delay(20);
}
