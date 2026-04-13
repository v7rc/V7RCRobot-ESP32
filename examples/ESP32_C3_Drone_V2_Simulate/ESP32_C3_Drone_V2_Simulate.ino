#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <V7RCDroneV2-esp32.h>
#include <protocol/V7RCProtocol.h>

namespace {

static const unsigned long kLoopIntervalMs = 50;
static const unsigned long kCalibrationDurationMs = 1000;

char serialLine[320];
size_t serialLineLength = 0;

V7RCDroneV2RuntimeOptions options = V7RCDroneV2DefaultRuntimeOptions();
V7RCDroneV2AngleController angleController;
V7RCDroneV2RateController rateController;
V7RCDroneV2MotorMixer motorMixer;
V7RCDroneV2SimulatedIcm20948 simulatedImu;
V7RCDroneV2Icm20948InputSample icmInput = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, false};
V7RCDroneV2ControlState controlState = {0.0f, 0.0f, 0.0f, 0.0f, true};
V7RCDroneV2DebugData debugData = {};
V7RCDroneV2MotorMix lastMotorMix = {};
unsigned long lastLoopMs = 0;

bool armed = false;
bool airborne = false;
bool trimsReady = false;
float trimRollDeg = 0.0f;
float trimPitchDeg = 0.0f;
float trimRollSumDeg = 0.0f;
float trimPitchSumDeg = 0.0f;
uint16_t trimSamples = 0;

float clampUnitSigned(float value) {
  return V7RCDroneV2Clamp(value, -1.0f, 1.0f);
}

float clampUnitUnsigned(float value) {
  return V7RCDroneV2Clamp(value, 0.0f, 1.0f);
}

bool parseJsonNumber(const char* json, const char* key, float* outValue) {
  if (!json || !key || !outValue) return false;

  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\":", key);
  const char* found = strstr(json, pattern);
  if (!found) {
    snprintf(pattern, sizeof(pattern), "\"%s\" :", key);
    found = strstr(json, pattern);
    if (!found) return false;
  }

  found = strchr(found, ':');
  if (!found) return false;
  ++found;
  while (*found == ' ' || *found == '\t') ++found;

  char* endPtr = nullptr;
  const float parsed = strtof(found, &endPtr);
  if (endPtr == found) return false;

  *outValue = parsed;
  return true;
}

bool parseJsonBool(const char* json, const char* key, bool* outValue) {
  if (!json || !key || !outValue) return false;

  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\":", key);
  const char* found = strstr(json, pattern);
  if (!found) {
    snprintf(pattern, sizeof(pattern), "\"%s\" :", key);
    found = strstr(json, pattern);
    if (!found) return false;
  }

  found = strchr(found, ':');
  if (!found) return false;
  ++found;
  while (*found == ' ' || *found == '\t') ++found;

  if (strncmp(found, "true", 4) == 0) {
    *outValue = true;
    return true;
  }
  if (strncmp(found, "false", 5) == 0) {
    *outValue = false;
    return true;
  }
  return false;
}

void applyIncomingIcmJson(const char* json) {
  float value = 0.0f;
  bool boolValue = false;

  if (parseJsonNumber(json, "accelXg", &value)) {
    icmInput.accelXg = value;
  }
  if (parseJsonNumber(json, "accelYg", &value)) {
    icmInput.accelYg = value;
  }
  if (parseJsonNumber(json, "accelZg", &value)) {
    icmInput.accelZg = value;
  }
  if (parseJsonNumber(json, "gyroXDegPerSec", &value)) {
    icmInput.gyroXDegPerSec = value;
  }
  if (parseJsonNumber(json, "gyroYDegPerSec", &value)) {
    icmInput.gyroYDegPerSec = value;
  }
  if (parseJsonNumber(json, "gyroZDegPerSec", &value)) {
    icmInput.gyroZDegPerSec = value;
  }
  if (parseJsonBool(json, "valid", &boolValue)) {
    icmInput.valid = boolValue;
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      serialLine[serialLineLength] = '\0';
      if (serialLineLength > 0 && serialLine[0] == '{') {
        applyIncomingIcmJson(serialLine);
      }
      serialLineLength = 0;
      continue;
    }

    if (serialLineLength < sizeof(serialLine) - 1) {
      serialLine[serialLineLength++] = ch;
    } else {
      serialLineLength = 0;
    }
  }
}

int channelFromSigned(float normalized) {
  return (int)lroundf(1500.0f + clampUnitSigned(normalized) * 500.0f);
}

int channelFromThrottle(float throttle) {
  return (int)lroundf(1000.0f + clampUnitUnsigned(throttle) * 1000.0f);
}

void buildSrtCommand(unsigned long nowMs, char* outCommand, size_t outSize, const char** outPhase) {
  const unsigned long phaseMs = nowMs % 14000UL;
  const char* phase = "idle";
  float throttle = 0.0f;
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;

  if (phaseMs < 2000UL) {
    phase = "idle";
    throttle = 0.00f;
  } else if (phaseMs < 4500UL) {
    phase = "takeoff";
    const float t = (float)(phaseMs - 2000UL) / 2500.0f;
    throttle = 0.18f + t * 0.20f;
  } else if (phaseMs < 6500UL) {
    phase = "yaw_right";
    throttle = 0.38f;
    yaw = 0.35f;
  } else if (phaseMs < 8500UL) {
    phase = "roll_left";
    throttle = 0.38f;
    roll = -0.28f;
  } else if (phaseMs < 10500UL) {
    phase = "roll_right";
    throttle = 0.38f;
    roll = 0.28f;
  } else if (phaseMs < 12250UL) {
    phase = "pitch_forward";
    throttle = 0.37f;
    pitch = 0.26f;
  } else {
    phase = "pitch_backward";
    throttle = 0.37f;
    pitch = -0.26f;
  }

  snprintf(
      outCommand,
      outSize,
      "SRT%04d%04d%04d%04d#",
      channelFromSigned(yaw),
      channelFromThrottle(throttle),
      channelFromSigned(pitch),
      channelFromSigned(roll));

  if (outPhase) {
    *outPhase = phase;
  }
}

float normalizeBidirectional(V7RC_ProtocolType protocolType, int16_t rawValue) {
  if (protocolType == V7RC_HEX || protocolType == V7RC_SRV || protocolType == V7RC_SS8) {
    return clampUnitSigned(((float)rawValue - 1500.0f) / 500.0f);
  }
  if (protocolType == V7RC_DEG) {
    return clampUnitSigned((float)rawValue / 127.0f);
  }
  return 0.0f;
}

float normalizeThrottle(V7RC_ProtocolType protocolType, int16_t rawValue) {
  if (protocolType == V7RC_HEX || protocolType == V7RC_SRV || protocolType == V7RC_SS8) {
    return clampUnitUnsigned(((float)rawValue - 1000.0f) / 1000.0f);
  }
  return clampUnitUnsigned(((float)rawValue + 127.0f) / 254.0f);
}

void updateControlStateFromCommand(const char* command) {
  const V7RC_Frame frame = V7RCProtocolDecoder::decode(command, strlen(command));
  if (!frame.valid) {
    controlState = {0.0f, 0.0f, 0.0f, 0.0f, true};
    return;
  }

  controlState.yaw = frame.channelPresent[0] ? normalizeBidirectional(frame.type, frame.values[0]) : 0.0f;
  controlState.throttle = frame.channelPresent[1] ? normalizeThrottle(frame.type, frame.values[1]) : 0.0f;
  controlState.pitch = frame.channelPresent[2] ? normalizeBidirectional(frame.type, frame.values[2]) : 0.0f;
  controlState.roll = frame.channelPresent[3] ? normalizeBidirectional(frame.type, frame.values[3]) : 0.0f;
  controlState.stabilize = true;
}

void updateCalibration(unsigned long nowMs, const V7RCDroneV2AttitudeState& attitude) {
  if (trimsReady || !attitude.valid) return;

  if (nowMs >= kCalibrationDurationMs) {
    if (trimSamples > 0) {
      trimRollDeg = trimRollSumDeg / trimSamples;
      trimPitchDeg = trimPitchSumDeg / trimSamples;
    }
    trimsReady = true;
    armed = true;
    return;
  }

  trimRollSumDeg += attitude.rollDeg;
  trimPitchSumDeg += attitude.pitchDeg;
  ++trimSamples;
}

float computeAuthority(float throttle) {
  return options.attitudeAuthorityMin + (1.0f - options.attitudeAuthorityMin) * clampUnitUnsigned(throttle);
}

void runControllerStep(const V7RCDroneV2AttitudeState& attitude, float dt) {
  debugData = {};
  debugData.throttle = controlState.throttle;
  debugData.stabilize = controlState.stabilize;

  if (!armed || !attitude.valid || !trimsReady || controlState.throttle <= 0.01f) {
    rateController.reset();
    airborne = false;
    lastMotorMix = {};
    return;
  }

  airborne = controlState.throttle >= options.airborneThrottleThreshold;

  V7RCDroneV2CorrectedAttitude corrected{};
  corrected.rollDeg = attitude.rollDeg - trimRollDeg;
  corrected.pitchDeg = attitude.pitchDeg - trimPitchDeg;
  corrected.rollRateDegPerSec = attitude.rollRateDegPerSec;
  corrected.pitchRateDegPerSec = attitude.pitchRateDegPerSec;
  corrected.yawRateDegPerSec = attitude.yawRateDegPerSec;

  debugData.desiredRollDeg = controlState.roll * options.angle.maxTiltDeg;
  debugData.desiredPitchDeg = controlState.pitch * options.angle.maxTiltDeg;
  debugData.correctedRollDeg = corrected.rollDeg;
  debugData.correctedPitchDeg = corrected.pitchDeg;

  V7RCDroneV2AngleLoopOutput angleOut = angleController.update(controlState, corrected);
  V7RCDroneV2RateLoopOutput rateOut = rateController.update(
      controlState,
      corrected,
      angleOut,
      dt,
      airborne,
      false,
      &debugData);

  const float authority = computeAuthority(controlState.throttle);
  rateOut.rollCmd *= authority;
  rateOut.pitchCmd *= authority;
  rateOut.yawCmd *= authority;

  debugData.rollCmd = rateOut.rollCmd;
  debugData.pitchCmd = rateOut.pitchCmd;
  debugData.yawCmd = rateOut.yawCmd;
  debugData.attitudeAuthority = authority;

  lastMotorMix = motorMixer.mix(controlState.throttle, rateOut);
  debugData.motorsSaturated = motorMixer.applySaturationAndDesaturation(lastMotorMix, 0.0f, 1.0f);
}

void emitJson(unsigned long nowMs, const char* phase, const char* command, const V7RCDroneV2AttitudeState& attitude) {
  Serial.print("{\"ts\":");
  Serial.print(nowMs);
  Serial.print(",\"phase\":\"");
  Serial.print(phase);
  Serial.print("\",\"icm20948\":{");
  Serial.print("\"accelXg\":");
  Serial.print(icmInput.accelXg, 4);
  Serial.print(",\"accelYg\":");
  Serial.print(icmInput.accelYg, 4);
  Serial.print(",\"accelZg\":");
  Serial.print(icmInput.accelZg, 4);
  Serial.print(",\"gyroXDegPerSec\":");
  Serial.print(icmInput.gyroXDegPerSec, 4);
  Serial.print(",\"gyroYDegPerSec\":");
  Serial.print(icmInput.gyroYDegPerSec, 4);
  Serial.print(",\"gyroZDegPerSec\":");
  Serial.print(icmInput.gyroZDegPerSec, 4);
  Serial.print(",\"filteredAccelXg\":");
  Serial.print(simulatedImu.filteredAccelXg(), 4);
  Serial.print(",\"filteredAccelYg\":");
  Serial.print(simulatedImu.filteredAccelYg(), 4);
  Serial.print(",\"filteredAccelZg\":");
  Serial.print(simulatedImu.filteredAccelZg(), 4);
  Serial.print(",\"filteredGyroXDegPerSec\":");
  Serial.print(simulatedImu.filteredGyroXDegPerSec(), 4);
  Serial.print(",\"filteredGyroYDegPerSec\":");
  Serial.print(simulatedImu.filteredGyroYDegPerSec(), 4);
  Serial.print(",\"filteredGyroZDegPerSec\":");
  Serial.print(simulatedImu.filteredGyroZDegPerSec(), 4);
  Serial.print(",\"valid\":");
  Serial.print(icmInput.valid ? "true" : "false");
  Serial.print("},\"imu\":{");
  Serial.print("\"rollDeg\":");
  Serial.print(attitude.rollDeg, 3);
  Serial.print(",\"pitchDeg\":");
  Serial.print(attitude.pitchDeg, 3);
  Serial.print(",\"rollRateDegPerSec\":");
  Serial.print(attitude.rollRateDegPerSec, 3);
  Serial.print(",\"pitchRateDegPerSec\":");
  Serial.print(attitude.pitchRateDegPerSec, 3);
  Serial.print(",\"yawRateDegPerSec\":");
  Serial.print(attitude.yawRateDegPerSec, 3);
  Serial.print(",\"valid\":");
  Serial.print(attitude.valid ? "true" : "false");
  Serial.print("},\"control\":{");
  Serial.print("\"command\":\"");
  Serial.print(command);
  Serial.print("\",\"armed\":");
  Serial.print(armed ? "true" : "false");
  Serial.print(",\"airborne\":");
  Serial.print(airborne ? "true" : "false");
  Serial.print(",\"stabilize\":");
  Serial.print(controlState.stabilize ? "true" : "false");
  Serial.print(",\"THR\":");
  Serial.print(controlState.throttle, 3);
  Serial.print(",\"YAW\":");
  Serial.print(controlState.yaw, 3);
  Serial.print(",\"YRT\":");
  Serial.print(attitude.yawRateDegPerSec, 3);
  Serial.print(",\"PIT\":");
  Serial.print(debugData.correctedPitchDeg, 3);
  Serial.print(",\"ROL\":");
  Serial.print(debugData.correctedRollDeg, 3);
  Serial.print("},\"motors\":{");
  Serial.print("\"frontLeft\":");
  Serial.print(lastMotorMix.frontLeft, 4);
  Serial.print(",\"frontRight\":");
  Serial.print(lastMotorMix.frontRight, 4);
  Serial.print(",\"rearLeft\":");
  Serial.print(lastMotorMix.rearLeft, 4);
  Serial.print(",\"rearRight\":");
  Serial.print(lastMotorMix.rearRight, 4);
  Serial.println("}}");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  options.debugEnabled = true;
  angleController.configure(options.angle);
  rateController.configure(options.rate);

  Serial.println("Drone_V2_Simulate ready.");
  Serial.println("Input simulated ICM20948 JSON line example:");
  Serial.println("{\"accelXg\":0.0,\"accelYg\":0.0,\"accelZg\":1.0,\"gyroXDegPerSec\":0.0,\"gyroYDegPerSec\":0.0,\"gyroZDegPerSec\":0.0,\"valid\":true}");
  Serial.println("Output JSON every 50ms with raw ICM20948 sample, computed attitude, simulated SRT command, telemetry, and motor outputs.");
}

void loop() {
  handleSerialInput();

  const unsigned long nowMs = millis();
  if ((nowMs - lastLoopMs) < kLoopIntervalMs) {
    return;
  }

  const float dt = lastLoopMs == 0 ? (float)kLoopIntervalMs / 1000.0f : (float)(nowMs - lastLoopMs) / 1000.0f;
  lastLoopMs = nowMs;

  simulatedImu.setInputSample(icmInput);
  simulatedImu.update(nowMs);
  const V7RCDroneV2AttitudeState attitude = simulatedImu.attitude();

  updateCalibration(nowMs, attitude);

  char command[21];
  const char* phase = "idle";
  buildSrtCommand(nowMs, command, sizeof(command), &phase);
  updateControlStateFromCommand(command);
  runControllerStep(attitude, dt);
  emitJson(nowMs, phase, command, attitude);
}
