#include <Arduino.h>
#include <V7RCDroneV2-esp32.h>

namespace {

V7RCIcm20948Imu imu(5, 6);
V7RCDroneV2LegacyImuAdapter estimator(&imu);
V7RCDroneV2Runtime runtime;

V7RC_DCMotorConfig motors[] = {
  {20, 21, false},
  {10, 0, true},
  {1, 2, true},
  {3, 4, false},
};

V7RCDroneV2ControlState currentState = {0.0f, 0.0f, 0.0f, 0.0f, true};
bool stabilize = true;

V7RCDroneV2RuntimeOptions options = V7RCDroneV2DefaultRuntimeOptions();

void configureOptions() {
  options.outputMode = V7RC_DRONE_V2_OUTPUT_DC_MOTOR;
  options.dcMotors = motors;
  options.numDCMotors = sizeof(motors) / sizeof(motors[0]);
  options.motorPins[0] = 20;
  options.motorPins[1] = 10;
  options.motorPins[2] = 1;
  options.motorPins[3] = 3;
  options.debugEnabled = true;
}

void handleSerial() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "ARM") {
    runtime.beginUnlock();
    Serial.println("DroneV2 unlock sequence started.");
  } else if (command == "DISARM") {
    runtime.disarm();
    Serial.println("DroneV2 disarmed.");
  } else if (command == "STABILIZE ON") {
    stabilize = true;
    Serial.println("DroneV2 stabilize enabled.");
  } else if (command == "STABILIZE OFF") {
    stabilize = false;
    Serial.println("DroneV2 stabilize disabled.");
  } else if (command == "STATUS") {
    const V7RCDroneV2AttitudeState attitude = runtime.attitude();
    const V7RCDroneV2DebugData& debug = runtime.debugData();
    Serial.printf(
      "armed=%d air=%d st=%d roll=%.2f pitch=%.2f rollRate=%.2f pitchRate=%.2f yawRate=%.2f sat=%d\n",
      runtime.isArmed() ? 1 : 0,
      runtime.airborne() ? 1 : 0,
      stabilize ? 1 : 0,
      attitude.rollDeg,
      attitude.pitchDeg,
      attitude.rollRateDegPerSec,
      attitude.pitchRateDegPerSec,
      attitude.yawRateDegPerSec,
      debug.motorsSaturated ? 1 : 0
    );
  }
}

void applyStep(unsigned long nowMs) {
  const unsigned long phase = (nowMs / 1800UL) % 5UL;
  currentState = {0.0f, 0.0f, 0.0f, 0.0f, stabilize};

  if (phase == 0) {
    currentState.throttle = 0.28f;
  } else if (phase == 1) {
    currentState.throttle = 0.34f;
    currentState.yaw = 0.22f;
  } else if (phase == 2) {
    currentState.throttle = 0.36f;
    currentState.roll = 0.25f;
  } else if (phase == 3) {
    currentState.throttle = 0.36f;
    currentState.pitch = -0.25f;
  } else {
    currentState.throttle = 0.20f;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  configureOptions();
  estimator.setRateFilterAlpha(options.legacyRateFilterAlpha);

  const bool ready = runtime.begin(options, &estimator);
  Serial.printf("DroneV2 runtime ready (IMU=%s).\n", ready ? estimator.sensorName() : "OFFLINE");
  Serial.println("Commands: ARM, DISARM, STABILIZE ON, STABILIZE OFF, STATUS");
}

void loop() {
  handleSerial();
  applyStep(millis());
  runtime.update(currentState, millis());
  delay(20);
}
