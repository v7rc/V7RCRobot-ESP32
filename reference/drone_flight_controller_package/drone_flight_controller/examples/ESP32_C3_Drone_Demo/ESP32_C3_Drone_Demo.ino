#include "FlightController.h"
#include "DebugPrinter.h"

FlightController gFlightController;
DebugPrinter gDebugPrinter;

static unsigned long gLastLoopUs = 0;
static unsigned long gLastDebugMs = 0;

// Replace these with your actual implementations.
ControlState readControlState() {
  ControlState c{};
  c.throttle = 0.0f;
  c.roll = 0.0f;
  c.pitch = 0.0f;
  c.yaw = 0.0f;
  c.stabilize = true;
  return c;
}

AttitudeState readAttitudeState() {
  AttitudeState a{};
  a.rollDeg = 0.0f;
  a.pitchDeg = 0.0f;
  a.rollRateDegPerSec = 0.0f;
  a.pitchRateDegPerSec = 0.0f;
  a.yawRateDegPerSec = 0.0f;
  a.valid = true;
  return a;
}

bool isArmed() {
  return false;
}

bool isAirborne() {
  return false;
}

float getLevelRollTrimDeg() {
  return 0.0f;
}

float getLevelPitchTrimDeg() {
  return 0.0f;
}

void writeMotors(const MotorMixOutput& m) {
  // Example:
  // writeMotorFL(m.frontLeft);
  // writeMotorFR(m.frontRight);
  // writeMotorRL(m.rearLeft);
  // writeMotorRR(m.rearRight);
}

void setup() {
  gFlightController.begin();
  gFlightController.setDebugEnabled(true);

  gDebugPrinter.begin(115200);
  delay(300);
  gDebugPrinter.printHeader();

  gLastLoopUs = micros();
}

void loop() {
  const unsigned long nowUs = micros();
  float dt = (nowUs - gLastLoopUs) * 1e-6f;
  gLastLoopUs = nowUs;
  if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.002f;
  }

  const ControlState control = readControlState();
  const AttitudeState attitude = readAttitudeState();
  const bool armed = isArmed();

  gFlightController.setAirborne(isAirborne());
  gFlightController.setLevelTrim(getLevelRollTrimDeg(), getLevelPitchTrimDeg());

  const MotorMixOutput motorOut = gFlightController.update(control, attitude, dt, armed);
  writeMotors(motorOut);

  const unsigned long nowMs = millis();
  if (nowMs - gLastDebugMs >= 50) {
    gLastDebugMs = nowMs;
    gDebugPrinter.printCsv(nowMs, gFlightController.debugData());
  }
}
