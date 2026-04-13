#include <Arduino.h>
#include <V7RCDrone-esp32.h>
#include <esp_mac.h>
#include <protocol/V7RCProtocol.h>
#include <transport/V7RCBleTransportEsp32.h>

namespace {

enum V7RCDroneImuSelection : uint8_t {
  V7RC_DRONE_IMU_NONE = 0,
  V7RC_DRONE_IMU_ADXL345,
  V7RC_DRONE_IMU_ICM20948
};

static const uint8_t kImuSdaPin = 5;
static const uint8_t kImuSclPin = 6;
static const V7RCDroneImuSelection kImuSelection = V7RC_DRONE_IMU_ICM20948;

V7RCDroneRuntime runtime;
V7RCAdxl345Imu adxl345Imu(kImuSdaPin, kImuSclPin);
V7RCIcm20948Imu icm20948Imu(kImuSdaPin, kImuSclPin);
V7RCBleTransportEsp32 transport;

V7RC_DCMotorConfig motors[] = {
  {20, 21, false},  // front-left
  {0, 10, true},    // front-right
  {2, 1, true},     // rear-left
  {3, 4, false},    // rear-right
};

V7RCDroneControlState controlState = {0.0f, 0.0f, 0.0f, 0.0f};
bool stabilizationEnabled = false;
bool controlUnlocked = false;
bool imuReady = false;
bool gyroCalibrationOk = true;
unsigned long lastTelemetryMs = 0;

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

static const unsigned long kTelemetryIntervalMs = 100;

V7RCDroneImu* selectedImu() {
  if (kImuSelection == V7RC_DRONE_IMU_ADXL345) {
    return &adxl345Imu;
  }
  if (kImuSelection == V7RC_DRONE_IMU_ICM20948) {
    return &icm20948Imu;
  }
  return nullptr;
}

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

int telemetryInt(float value, float scale) {
  long scaled = lroundf(value * scale);
  if (scaled > 999) scaled = 999;
  if (scaled < -999) scaled = -999;
  return (int)scaled;
}

void formatTelemetryField(char* out, int value) {
  const int magnitude = abs(value);
  out[0] = value < 0 ? '-' : '+';
  out[1] = (char)('0' + ((magnitude / 100) % 10));
  out[2] = (char)('0' + ((magnitude / 10) % 10));
  out[3] = (char)('0' + (magnitude % 10));
}

void sendDebugTelemetry(unsigned long nowMs) {
  if (!transport.isConnected()) return;
  if ((nowMs - lastTelemetryMs) < kTelemetryIntervalMs) return;

  lastTelemetryMs = nowMs;

  const V7RCDroneAttitude attitude = runtime.attitude();

  char packet[21];
  memcpy(packet, "CMD", 3);
  formatTelemetryField(packet + 3, telemetryInt(controlState.throttle, 100.0f));
  formatTelemetryField(packet + 7, telemetryInt(controlState.yaw, 100.0f));
  formatTelemetryField(packet + 11, telemetryInt(attitude.yawRateDegPerSec, 1.0f));
  formatTelemetryField(packet + 15, telemetryInt(attitude.pitchDeg, 1.0f));
  packet[19] = '#';
  packet[20] = '\0';

  transport.send((const uint8_t*)packet, 20);
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
    if (kImuSelection == V7RC_DRONE_IMU_ICM20948 && imuReady) {
      Serial.println("Unlock gesture accepted. Calibrating ICM20948 gyro bias; keep the drone still...");
      gyroCalibrationOk = icm20948Imu.calibrateGyroBias(400, 2);
      if (!gyroCalibrationOk) {
        Serial.println("ICM20948 gyro calibration failed. Drone remains locked.");
        return;
      }
    }

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
    gyroCalibrationOk = true;
    lastTelemetryMs = 0;
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

  // IMU selection:
  //   V7RC_DRONE_IMU_NONE
  //   V7RC_DRONE_IMU_ADXL345
  //   V7RC_DRONE_IMU_ICM20948
  //
  // If you use ADXL345 and the mounting direction differs from the airframe logic,
  // adjust the axis transform here:
  // adxl345Imu.setAxisTransform(
  //   V7RC_ADXL345_AXIS_Y,  1,
  //   V7RC_ADXL345_AXIS_X, -1,
  //   V7RC_ADXL345_AXIS_Z,  1
  // );
  //
  // Active ICM20948 mounting compensation for this airframe:
  // logical X = sensor Y
  // logical Y = sensor X
  // logical Z = sensor Z
  //
  // If the tilt directions are still reversed after this change,
  // keep the axis order and only flip the sign of X or Y.
  icm20948Imu.setAxisTransform(
    V7RC_ICM20948_AXIS_Y,  1,
    V7RC_ICM20948_AXIS_X,  1,
    V7RC_ICM20948_AXIS_Z,  1
  );

  V7RCDroneImu* imu = selectedImu();
  imuReady = runtime.begin(options, imu);

  runtime.setStabilizationEnabled(stabilizationEnabled);

  char deviceName[20];
  snprintf(deviceName, sizeof(deviceName), "V7RC-DRONE-%02lu", (unsigned long)(robotIdFromBleMac() & 0xFF));

  transport.setByteHandler(handleTransportByte, nullptr);
  transport.setConnectionHandler(handleConnectionChanged, nullptr);
  transport.begin(deviceName);

  Serial.printf(
    "Drone BLE control ready (DC motor mode, IMU=%s, SDA=%u, SCL=%u, locked on boot).\n",
    imu ? (imuReady ? (gyroCalibrationOk ? imu->sensorName() : "CALIBRATION_FAILED") : "OFFLINE") : "DISABLED",
    kImuSdaPin,
    kImuSclPin
  );
  Serial.println("Motor rotation config: FL=normal, FR=invert, RL=invert, RR=normal.");
  Serial.println("V7RC App channels: ch0=Yaw, ch1=Throttle(1000->0, 2000->max), ch2=Pitch, ch3=Roll, ch4=Stabilize.");
  Serial.println("Unlock gesture: ch0=1000, ch1=1000, ch2=1000, ch3=2000.");
  Serial.println("ICM20948 gyro bias calibration now runs after the unlock gesture is received.");
  Serial.println("BLE debug telemetry: CMD + THR + YAW + YRT + PIT + #");
}

void loop() {
  transport.poll();
  runtime.update(controlState, millis());
  sendDebugTelemetry(millis());
  delay(10);
}
