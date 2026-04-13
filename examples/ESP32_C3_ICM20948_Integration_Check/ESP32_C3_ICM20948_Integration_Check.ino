#include <Arduino.h>
#include <V7RCDrone-esp32.h>

namespace {

static const uint8_t kSdaPin = 5;
static const uint8_t kSclPin = 6;
static const unsigned long kPrintIntervalMs = 100;

V7RCIcm20948Imu imu(kSdaPin, kSclPin);
unsigned long lastPrintMs = 0;

void printFrame(unsigned long nowMs) {
  if ((nowMs - lastPrintMs) < kPrintIntervalMs) {
    return;
  }

  lastPrintMs = nowMs;

  const bool ok = imu.update(nowMs);
  const V7RCDroneAttitude attitude = imu.attitude();

  Serial.printf(
    "t=%lu ok=%d valid=%d ax=%.3f ay=%.3f az=%.3f gx=%.2f gy=%.2f gz=%.2f roll=%.2f pitch=%.2f yawRate=%.2f\n",
    nowMs,
    ok ? 1 : 0,
    attitude.valid ? 1 : 0,
    imu.accelXg(),
    imu.accelYg(),
    imu.accelZg(),
    imu.gyroXDegPerSec(),
    imu.gyroYDegPerSec(),
    imu.gyroZDegPerSec(),
    attitude.rollDeg,
    attitude.pitchDeg,
    attitude.yawRateDegPerSec
  );
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  const bool imuReady = imu.begin();
  Serial.printf(
    "ICM20948 integration check start. SDA=%u SCL=%u ready=%d sensor=%s\n",
    kSdaPin,
    kSclPin,
    imuReady ? 1 : 0,
    imu.sensorName()
  );
  Serial.println("Check roll/pitch while tilting the board, and check yawRate while rotating it around Z.");
}

void loop() {
  printFrame(millis());
  delay(10);
}
