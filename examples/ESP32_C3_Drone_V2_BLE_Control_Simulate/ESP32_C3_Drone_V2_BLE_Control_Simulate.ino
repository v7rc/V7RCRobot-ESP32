#include <Arduino.h>
#include <esp_mac.h>
#include <stdlib.h>
#include <string.h>
#include <V7RCDroneV2-esp32.h>
#include <protocol/V7RCProtocol.h>
#include <transport/V7RCBleTransportEsp32.h>

namespace {

static const unsigned long kLoopIntervalMs = 50;
static const unsigned long kCalibrationDurationMs = 1000;
static const unsigned long kSignalTimeoutMs = 300;
static const int16_t kStabilizeOffThreshold = 1300;
static const int16_t kStabilizeOnThreshold = 1700;

char serialLine[320];
size_t serialLineLength = 0;
char frameBuffer[48];
size_t frameLength = 0;
char lastCommand[21] = "SRT1500100015001500#";
char lastCommandPhase[16] = "ble_control";

V7RCDroneV2RuntimeOptions options = V7RCDroneV2DefaultRuntimeOptions();
V7RCDroneV2AngleController angleController;
V7RCDroneV2RateController rateController;
V7RCDroneV2MotorMixer motorMixer;
V7RCDroneV2SimulatedIcm20948 simulatedImu;
V7RCBleTransportEsp32 transport;
V7RCDroneV2Icm20948InputSample icmInput = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, false};
V7RCDroneV2ControlState controlState = {0.0f, 0.0f, 0.0f, 0.0f, true};
V7RCDroneV2DebugData debugData = {};
V7RCDroneV2MotorMix lastMotorMix = {};
unsigned long lastLoopMs = 0;

bool armed = false;
bool airborne = false;
bool trimsReady = false;
bool controlUnlocked = false;
bool signalTimedOut = false;
float trimRollDeg = 0.0f;
float trimPitchDeg = 0.0f;
float trimRollSumDeg = 0.0f;
float trimPitchSumDeg = 0.0f;
uint16_t trimSamples = 0;
unsigned long calibrationStartMs = 0;
unsigned long lastSignalMs = 0;

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

int channelFromSigned(float normalized) {
  return (int)lroundf(1500.0f + clampUnitSigned(normalized) * 500.0f);
}

int channelFromThrottle(float throttle) {
  return (int)lroundf(1000.0f + clampUnitUnsigned(throttle) * 1000.0f);
}

void rebuildCanonicalCommand() {
  snprintf(
      lastCommand,
      sizeof(lastCommand),
      "SRT%04d%04d%04d%04d#",
      channelFromSigned(controlState.yaw),
      channelFromThrottle(controlState.throttle),
      channelFromSigned(controlState.pitch),
      channelFromSigned(controlState.roll));
}

bool isUnlockGesture(const V7RC_Frame& frame) {
  if (!frame.channelPresent[0] || !frame.channelPresent[1] || !frame.channelPresent[2] || !frame.channelPresent[3]) {
    return false;
  }
  if (!(frame.type == V7RC_HEX || frame.type == V7RC_SRV || frame.type == V7RC_SS8)) {
    return false;
  }
  return frame.values[0] <= 1000 &&
         frame.values[1] <= 1000 &&
         frame.values[2] <= 1000 &&
         frame.values[3] >= 2000;
}

void resetSimulationState() {
  armed = false;
  airborne = false;
  trimsReady = false;
  controlUnlocked = false;
  signalTimedOut = false;
  trimRollDeg = 0.0f;
  trimPitchDeg = 0.0f;
  trimRollSumDeg = 0.0f;
  trimPitchSumDeg = 0.0f;
  trimSamples = 0;
  calibrationStartMs = 0;
  lastSignalMs = 0;
  controlState = {0.0f, 0.0f, 0.0f, 0.0f, true};
  rebuildCanonicalCommand();
  rateController.reset();
  simulatedImu.reset();
  lastMotorMix = {};
}

void applyFrame(const V7RC_Frame& frame) {
  if (!frame.valid || frame.type == V7RC_LED) return;

  lastSignalMs = millis();
  signalTimedOut = false;

  if (!controlUnlocked && isUnlockGesture(frame)) {
    controlUnlocked = true;
    calibrationStartMs = millis();
    trimRollSumDeg = 0.0f;
    trimPitchSumDeg = 0.0f;
    trimSamples = 0;
    strncpy(lastCommandPhase, "unlock", sizeof(lastCommandPhase) - 1);
    lastCommandPhase[sizeof(lastCommandPhase) - 1] = '\0';
    controlState = {0.0f, 0.0f, 0.0f, 0.0f, true};
    rebuildCanonicalCommand();
    return;
  }

  if (!controlUnlocked) {
    controlState = {0.0f, 0.0f, 0.0f, 0.0f, true};
    rebuildCanonicalCommand();
    return;
  }

  if (frame.channelPresent[0]) {
    controlState.yaw = normalizeBidirectional(frame.type, frame.values[0]);
  }
  if (frame.channelPresent[1]) {
    controlState.throttle = normalizeThrottle(frame.type, frame.values[1]);
  }
  if (frame.channelPresent[2]) {
    controlState.pitch = normalizeBidirectional(frame.type, frame.values[2]);
  }
  if (frame.channelPresent[3]) {
    controlState.roll = normalizeBidirectional(frame.type, frame.values[3]);
  }
  if (frame.channelPresent[4]) {
    if (frame.type == V7RC_HEX || frame.type == V7RC_SRV || frame.type == V7RC_SS8) {
      if (frame.values[4] <= kStabilizeOffThreshold) {
        controlState.stabilize = false;
      } else if (frame.values[4] >= kStabilizeOnThreshold) {
        controlState.stabilize = true;
      }
    } else {
      const float stabilizeSwitch = normalizeBidirectional(frame.type, frame.values[4]);
      if (stabilizeSwitch <= -0.5f) {
        controlState.stabilize = false;
      } else if (stabilizeSwitch >= 0.5f) {
        controlState.stabilize = true;
      }
    }
  }

  strncpy(lastCommandPhase, "ble_control", sizeof(lastCommandPhase) - 1);
  lastCommandPhase[sizeof(lastCommandPhase) - 1] = '\0';
  rebuildCanonicalCommand();
}

void handleDecodedFrame() {
  const V7RC_Frame frame = V7RCProtocolDecoder::decode(frameBuffer, frameLength);
  applyFrame(frame);
  frameLength = 0;
}

void handleTransportByte(uint8_t byte, void* context) {
  (void)context;
  if (frameLength >= sizeof(frameBuffer) - 1) {
    frameLength = 0;
  }
  frameBuffer[frameLength++] = (char)byte;
  frameBuffer[frameLength] = '\0';
  if (byte == '#') {
    handleDecodedFrame();
  }
}

void handleConnectionChanged(bool connected, void* context) {
  (void)context;
  if (!connected) {
    resetSimulationState();
    strncpy(lastCommandPhase, "disconnected", sizeof(lastCommandPhase) - 1);
    lastCommandPhase[sizeof(lastCommandPhase) - 1] = '\0';
    Serial.println("BLE disconnected. Simulation locked.");
    return;
  }
  Serial.println("BLE connected.");
}

uint32_t robotIdFromBleMac() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  return ((uint32_t)mac[4] << 8) | mac[5];
}

void updateCalibration(unsigned long nowMs, const V7RCDroneV2AttitudeState& attitude) {
  if (!controlUnlocked || trimsReady || !attitude.valid || calibrationStartMs == 0) return;

  if (nowMs >= (calibrationStartMs + kCalibrationDurationMs)) {
    if (trimSamples > 0) {
      trimRollDeg = trimRollSumDeg / trimSamples;
      trimPitchDeg = trimPitchSumDeg / trimSamples;
    }
    trimsReady = true;
    armed = true;
    strncpy(lastCommandPhase, "armed", sizeof(lastCommandPhase) - 1);
    lastCommandPhase[sizeof(lastCommandPhase) - 1] = '\0';
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

  V7RCDroneV2AngleLoopOutput angleOut{};
  if (controlState.stabilize) {
    angleOut = angleController.update(controlState, corrected);
  } else {
    angleOut.desiredRollRateDegPerSec = controlState.roll * options.angle.maxRollRateDegPerSec;
    angleOut.desiredPitchRateDegPerSec = controlState.pitch * options.angle.maxPitchRateDegPerSec;
  }

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

void emitJson(unsigned long nowMs, const V7RCDroneV2AttitudeState& attitude) {
  Serial.print("{\"ts\":");
  Serial.print(nowMs);
  Serial.print(",\"phase\":\"");
  Serial.print(lastCommandPhase);
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
  Serial.print(lastCommand);
  Serial.print("\",\"bleConnected\":");
  Serial.print(transport.isConnected() ? "true" : "false");
  Serial.print(",\"unlocked\":");
  Serial.print(controlUnlocked ? "true" : "false");
  Serial.print(",\"armed\":");
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

  char deviceName[20];
  snprintf(deviceName, sizeof(deviceName), "V7RC-DRV2SIM-%02lu", (unsigned long)(robotIdFromBleMac() & 0xFF));

  transport.setByteHandler(handleTransportByte, nullptr);
  transport.setConnectionHandler(handleConnectionChanged, nullptr);
  transport.begin(deviceName);

  rebuildCanonicalCommand();

  Serial.println("Drone_V2_BLE_Control_Simulate ready.");
  Serial.printf("BLE device: %s\n", deviceName);
  Serial.println("Serial simulated ICM20948 JSON input example:");
  Serial.println("{\"accelXg\":0.0,\"accelYg\":0.0,\"accelZg\":1.0,\"gyroXDegPerSec\":0.0,\"gyroYDegPerSec\":0.0,\"gyroZDegPerSec\":0.0,\"valid\":true}");
  Serial.println("Use V7RC App channels: ch0=Yaw, ch1=Throttle, ch2=Pitch, ch3=Roll, ch4=Stabilize.");
  Serial.println("Unlock gesture: ch0=1000, ch1=1000, ch2=1000, ch3=2000.");
  Serial.println("Output JSON every 50ms. No real IMU or motor output is used.");
}

void loop() {
  handleSerialInput();
  transport.poll();

  const unsigned long nowMs = millis();
  if (transport.isConnected() && lastSignalMs != 0 && (nowMs - lastSignalMs) > kSignalTimeoutMs) {
    controlState = {0.0f, 0.0f, 0.0f, 0.0f, controlState.stabilize};
    rebuildCanonicalCommand();
    signalTimedOut = true;
    armed = false;
    trimsReady = false;
    controlUnlocked = false;
    calibrationStartMs = 0;
    trimRollSumDeg = 0.0f;
    trimPitchSumDeg = 0.0f;
    trimSamples = 0;
    lastSignalMs = 0;
    rateController.reset();
    lastMotorMix = {};
    strncpy(lastCommandPhase, "signal_timeout", sizeof(lastCommandPhase) - 1);
    lastCommandPhase[sizeof(lastCommandPhase) - 1] = '\0';
  }

  if ((nowMs - lastLoopMs) < kLoopIntervalMs) {
    return;
  }

  const float dt = lastLoopMs == 0 ? (float)kLoopIntervalMs / 1000.0f : (float)(nowMs - lastLoopMs) / 1000.0f;
  lastLoopMs = nowMs;

  simulatedImu.setInputSample(icmInput);
  simulatedImu.update(nowMs);
  const V7RCDroneV2AttitudeState attitude = simulatedImu.attitude();

  updateCalibration(nowMs, attitude);
  runControllerStep(attitude, dt);
  emitJson(nowMs, attitude);
}
