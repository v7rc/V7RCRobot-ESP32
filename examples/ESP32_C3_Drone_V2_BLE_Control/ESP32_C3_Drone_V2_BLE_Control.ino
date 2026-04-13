#include <Arduino.h>
#include <V7RCDroneV2-esp32.h>
#include <esp_mac.h>
#include <protocol/V7RCProtocol.h>
#include <transport/V7RCBleTransportEsp32.h>

namespace {

static const uint8_t kImuSdaPin = 5;
static const uint8_t kImuSclPin = 6;
static const int16_t kStabilizeOffThreshold = 1300;
static const int16_t kStabilizeOnThreshold = 1700;
static const unsigned long kSignalTimeoutMs = 300;

V7RCDroneV2Runtime runtime;
V7RCIcm20948Imu icm20948Imu(kImuSdaPin, kImuSclPin);
V7RCDroneV2Icm20948Estimator estimator(&icm20948Imu);
V7RCBleTransportEsp32 transport;

V7RC_DCMotorConfig motors[] = {
  {20, 21, false},  // front-left
  {0, 10, true},    // front-right
  {2, 1, true},     // rear-left
  {3, 4, false},    // rear-right
};

V7RCDroneV2ControlState controlState = {0.0f, 0.0f, 0.0f, 0.0f, true};
bool controlUnlocked = false;
unsigned long lastSignalMs = 0;
char frameBuffer[48];
size_t frameLength = 0;

V7RCDroneV2RuntimeOptions options = V7RCDroneV2DefaultRuntimeOptions();

uint32_t robotIdFromBleMac() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  return ((uint32_t)mac[4] << 8) | mac[5];
}

float clampUnit(float value) {
  return V7RCDroneV2Clamp(value, -1.0f, 1.0f);
}

float clampZeroToOne(float value) {
  return V7RCDroneV2Clamp(value, 0.0f, 1.0f);
}

float normalizeBidirectional(V7RC_ProtocolType protocolType, int16_t rawValue) {
  float normalized = 0.0f;

  if (protocolType == V7RC_HEX || protocolType == V7RC_SRV || protocolType == V7RC_SS8) {
    normalized = ((float)rawValue - 1500.0f) / 500.0f;
  } else if (protocolType == V7RC_DEG) {
    normalized = (float)rawValue / 127.0f;
  }

  return clampUnit(normalized);
}

float normalizeThrottle(V7RC_ProtocolType protocolType, int16_t rawValue) {
  if (protocolType == V7RC_HEX || protocolType == V7RC_SRV || protocolType == V7RC_SS8) {
    return clampZeroToOne(((float)rawValue - 1000.0f) / 1000.0f);
  }

  return clampZeroToOne(((float)rawValue + 127.0f) / 254.0f);
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

void applyFrame(const V7RC_Frame& frame) {
  if (!frame.valid || frame.type == V7RC_LED) return;

  lastSignalMs = millis();

  if (!controlUnlocked && isUnlockGesture(frame)) {
    controlUnlocked = true;
    runtime.beginUnlock();
    return;
  }

  if (!controlUnlocked) {
    controlState = {0.0f, 0.0f, 0.0f, 0.0f, true};
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
    controlState = {0.0f, 0.0f, 0.0f, 0.0f, true};
    controlUnlocked = false;
    lastSignalMs = 0;
    runtime.disarm();
  }
}

void configureOptions() {
  options.outputMode = V7RC_DRONE_V2_OUTPUT_DC_MOTOR;
  options.dcMotors = motors;
  options.numDCMotors = sizeof(motors) / sizeof(motors[0]);
  options.motorPins[0] = 20;
  options.motorPins[1] = 10;
  options.motorPins[2] = 1;
  options.motorPins[3] = 3;
  options.debugEnabled = false;
  options.angle.maxTiltDeg = 18.0f;
  options.angle.rollKp = 4.0f;
  options.angle.pitchKp = 4.0f;
  options.angle.maxRollRateDegPerSec = 180.0f;
  options.angle.maxPitchRateDegPerSec = 180.0f;
  options.rate.roll = {0.015f, 0.010f, 0.0008f, 80.0f, 0.5f};
  options.rate.pitch = {0.015f, 0.010f, 0.0008f, 80.0f, 0.5f};
  options.rate.yaw = {0.010f, 0.005f, 0.0000f, 80.0f, 0.3f};
  options.rate.maxYawRateDegPerSec = 150.0f;
}

}  // namespace

void setup() {
  configureOptions();

  icm20948Imu.setAxisTransform(
    V7RC_ICM20948_AXIS_Y, -1,
    V7RC_ICM20948_AXIS_X, -1,
    V7RC_ICM20948_AXIS_Z, 1
  );

  runtime.begin(options, &estimator);

  char deviceName[20];
  snprintf(deviceName, sizeof(deviceName), "V7RC-DRV2-%02lu", (unsigned long)(robotIdFromBleMac() & 0xFF));

  transport.setByteHandler(handleTransportByte, nullptr);
  transport.setConnectionHandler(handleConnectionChanged, nullptr);
  transport.begin(deviceName);
}

void loop() {
  transport.poll();

  if (transport.isConnected() && lastSignalMs != 0 && (millis() - lastSignalMs) > kSignalTimeoutMs) {
    controlState = {0.0f, 0.0f, 0.0f, 0.0f, controlState.stabilize};
    lastSignalMs = 0;
  }

  runtime.update(controlState, millis());
  delay(10);
}
