#include <Arduino.h>
#include <V7RCDrone-esp32.h>
#include <esp_mac.h>
#include <protocol/V7RCProtocol.h>
#include <transport/V7RCBleTransportEsp32.h>

namespace {

V7RCDroneRuntime runtime;
V7RCBleTransportEsp32 transport;

V7RC_DCMotorConfig motors[] = {
  {20, 21, false},  // front-left
  {10, 0, false},   // front-right
  {1, 2, false},    // rear-left
  {3, 4, false},    // rear-right
};

V7RCDroneControlState controlState = {0.0f, 0.0f, 0.0f, 0.0f};
bool stabilizationEnabled = false;
bool controlUnlocked = false;

char frameBuffer[48];
size_t frameLength = 0;

V7RCDroneRuntimeOptions options = {
  .outputMode = V7RC_DRONE_OUTPUT_DC_MOTOR,
  .motorPins = {20, 10, 1, 3},
  .dcMotors = motors,
  .numDCMotors = sizeof(motors) / sizeof(motors[0]),
  .stabilizationEnabled = false,
  .maxTiltDeg = 18.0f,
  .rollKp = 0.85f,
  .pitchKp = 0.85f,
  .yawGain = 0.35f,
  .escMinUs = 1000,
  .escMaxUs = 2000,
  .escIdleUs = 1080,
};

uint32_t robotIdFromBleMac() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  return ((uint32_t)mac[4] << 8) | mac[5];
}

float clampUnit(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

float clampZeroToOne(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < 0.0f) return 0.0f;
  return value;
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
  // Map throttle so that:
  // 1000 -> 0.0f
  // 2000 -> 1.0f
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

  if (!controlUnlocked && isUnlockGesture(frame)) {
    controlUnlocked = true;
    runtime.beginUnlock();
    Serial.println("Unlock gesture accepted. Entering standby; keep throttle low briefly, then control is enabled.");
    return;
  }

  if (!controlUnlocked) {
    controlState = {0.0f, 0.0f, 0.0f, 0.0f};
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
    stabilizationEnabled = normalizeBidirectional(frame.type, frame.values[4]) > 0.0f;
    runtime.setStabilizationEnabled(stabilizationEnabled);
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
    controlState = {0.0f, 0.0f, 0.0f, 0.0f};
    controlUnlocked = false;
    runtime.disarm();
    Serial.println("BLE disconnected. Drone locked.");
    return;
  }

  Serial.println("BLE connected.");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  runtime.begin(options, nullptr);
  runtime.setStabilizationEnabled(stabilizationEnabled);

  char deviceName[20];
  snprintf(deviceName, sizeof(deviceName), "V7RC-DRONE-%02lu", (unsigned long)(robotIdFromBleMac() & 0xFF));

  transport.setByteHandler(handleTransportByte, nullptr);
  transport.setConnectionHandler(handleConnectionChanged, nullptr);
  transport.begin(deviceName);

  Serial.println("Drone BLE control ready (DC motor mode, IMU disabled, locked on boot).");
  Serial.println("V7RC App channels: ch0=Yaw, ch1=Throttle(1000->0, 2000->max), ch2=Pitch, ch3=Roll, ch4=Stabilize.");
  Serial.println("Unlock gesture: ch0=1000, ch1=1000, ch2=1000, ch3=2000.");
}

void loop() {
  transport.poll();
  runtime.update(controlState, millis());
  delay(10);
}
