#include <Arduino.h>
#include <V7RCDrone-esp32.h>

namespace {

static const uint8_t kSdaPin = 5;
static const uint8_t kSclPin = 6;
static const unsigned long kPrintIntervalMs = 100;

V7RCAdxl345Imu imu(kSdaPin, kSclPin);
unsigned long lastPrintMs = 0;

void printFrame(unsigned long nowMs) {
  if ((nowMs - lastPrintMs) < kPrintIntervalMs) {
    return;
  }

  lastPrintMs = nowMs;

  const bool ok = imu.update(nowMs);
  const V7RCDroneAttitude attitude = imu.attitude();

  Serial.printf(
    "t=%lu ok=%d valid=%d ax=%.3f ay=%.3f az=%.3f roll=%.2f pitch=%.2f yawRate=%.2f\n",
    nowMs,
    ok ? 1 : 0,
    attitude.valid ? 1 : 0,
    imu.accelXg(),
    imu.accelYg(),
    imu.accelZg(),
    attitude.rollDeg,
    attitude.pitchDeg,
    attitude.yawRateDegPerSec
  );
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Default logical frame:
  // X = sensor X
  // Y = sensor Y
  // Z = sensor Z
  //
  // Example if the module is rotated 90 degrees:
  // imu.setAxisTransform(
  //   V7RC_ADXL345_AXIS_Y,  1,
  //   V7RC_ADXL345_AXIS_X, -1,
  //   V7RC_ADXL345_AXIS_Z,  1
  // );

  const bool imuReady = imu.begin();
  Serial.printf(
    "ADXL345 integration check start. SDA=%u SCL=%u ready=%d sensor=%s\n",
    kSdaPin,
    kSclPin,
    imuReady ? 1 : 0,
    imu.sensorName()
  );
  Serial.println("Keep the board still and verify az is near +1.0g, then tilt to confirm roll/pitch changes.");
  Serial.println("If roll/pitch moves in the wrong direction, adjust imu.setAxisTransform(...) in this sketch.");
}

void loop() {
  printFrame(millis());
  delay(10);
}
