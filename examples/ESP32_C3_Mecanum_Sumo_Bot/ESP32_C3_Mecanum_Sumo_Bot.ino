#include <Arduino.h>
#include <V7RCCar-esp32.h>
#include <esp_system.h>

namespace {

// -------------------- Configurable pins --------------------
const uint8_t kLeftLinePin = 7;
const uint8_t kRightLinePin = 8;
const uint8_t kUltrasonicTrigPin = 5;
const uint8_t kUltrasonicEchoPin = 6;

// -------------------- Configurable sensor parameters --------------------
const uint8_t kLineDetectedLevel = LOW;
const bool kUseLineSensorPullups = true;
const unsigned long kLineCheckIntervalMs = 10;
const unsigned long kUltrasonicCheckIntervalMs = 30;
const unsigned long kUltrasonicPulseTimeoutUs = 4000;
const float kStartDistanceCm = 5.0f;
const float kAttackDistanceCm = 50.0f;

// -------------------- Configurable debug parameters --------------------
const bool kSerialDebugEnabled = true;
const unsigned long kDebugPrintIntervalMs = 250;

// -------------------- Configurable motion parameters --------------------
const unsigned long kStartCountdownMs = 3000;
const unsigned long kOpeningForwardMs = 1000;
const unsigned long kOpeningStopMs = 100;
const unsigned long kOpeningTurnMs = 200;
const unsigned long kLineEscapeReverseMs = 200;
const unsigned long kLineEscapeTurnMs = 500;

const float kOpeningForwardSpeed = 0.85f;
const float kAttackForwardSpeed = 1.0f;
const float kSearchTurnSpeed = 0.35f;
const float kOpeningTurnSpeed = 0.55f;
const float kEscapeReverseSpeed = -0.65f;
const float kEscapeTurnSpeed = 0.65f;

// Update these pins and dirInvert flags to match your motor driver wiring.
V7RC_DCMotorConfig motors[] = {
  {20, 21, false},  // front-left
  {10, 0, false},   // front-right
  {1, 2, false},    // rear-left
  {3, 4, false},    // rear-right
};

V7RCCarRobot robot;

V7RCCarRobotOptions robotOptions = {
  .bleBaseName = "V7RC-SUMO-AUTO",
  .motors = motors,
  .numMotors = sizeof(motors) / sizeof(motors[0]),
  .customChannelMap = nullptr,
  .numCustomChannelMap = 0,
  .differentialThrottleChannel = 0,
  .differentialSteerChannel = 1,
  .mecanumVxChannel = 0,
  .mecanumVyChannel = 1,
  .mecanumOmegaChannel = 2,
  .servos = nullptr,
  .numServos = 0,
  .auxiliaryServo0Channel = 15,
  .auxiliaryServo1Channel = 15,
  .auxiliaryServo2Channel = 15,
  .ws2812Brightness = 0,
  .ws2812Enable = false,
  .ws2812Pin = 0,
  .ws2812Count = 0,
};

enum SumoState : uint8_t {
  SUMO_WAIT_FOR_TARGET = 0,
  SUMO_COUNTDOWN,
  SUMO_OPENING_FORWARD,
  SUMO_OPENING_STOP,
  SUMO_OPENING_TURN,
  SUMO_SEARCH,
  SUMO_ATTACK,
  SUMO_ESCAPE_REVERSE,
  SUMO_ESCAPE_TURN,
};

SumoState state = SUMO_WAIT_FOR_TARGET;
unsigned long stateStartedMs = 0;
unsigned long lastLineCheckMs = 0;
unsigned long lastUltrasonicCheckMs = 0;
unsigned long lastDebugPrintMs = 0;
float lastDistanceCm = -1.0f;
int8_t turnDirection = 1;
bool leftLineDetected = false;
bool rightLineDetected = false;

V7RCCarControlState control(float vx, float vy, float omega) {
  V7RCCarControlState state = V7RCCarControl::neutralState();
  state.vx = vx;
  state.vy = vy;
  state.omega = omega;
  return state;
}

void applyStop() {
  robot.applyControl(V7RCCarControl::neutralState());
}

void applyDrive(float forwardSpeed, float strafeSpeed, float turnSpeed) {
  robot.applyControl(control(strafeSpeed, forwardSpeed, turnSpeed));
}

int8_t randomTurnDirection() {
  return random(0, 2) == 0 ? -1 : 1;
}

int8_t escapeTurnDirection() {
  if (leftLineDetected && !rightLineDetected) {
    return 1;
  }
  if (rightLineDetected && !leftLineDetected) {
    return -1;
  }
  return randomTurnDirection();
}

const char* stateName(SumoState value) {
  switch (value) {
    case SUMO_WAIT_FOR_TARGET:
      return "WAIT_FOR_TARGET";
    case SUMO_COUNTDOWN:
      return "COUNTDOWN";
    case SUMO_OPENING_FORWARD:
      return "OPENING_FORWARD";
    case SUMO_OPENING_STOP:
      return "OPENING_STOP";
    case SUMO_OPENING_TURN:
      return "OPENING_TURN";
    case SUMO_SEARCH:
      return "SEARCH";
    case SUMO_ATTACK:
      return "ATTACK";
    case SUMO_ESCAPE_REVERSE:
      return "ESCAPE_REVERSE";
    case SUMO_ESCAPE_TURN:
      return "ESCAPE_TURN";
  }
  return "UNKNOWN";
}

void enterState(SumoState nextState) {
  state = nextState;
  stateStartedMs = millis();

  switch (state) {
    case SUMO_WAIT_FOR_TARGET:
    case SUMO_COUNTDOWN:
    case SUMO_OPENING_STOP:
      applyStop();
      break;
    case SUMO_OPENING_FORWARD:
      applyDrive(kOpeningForwardSpeed, 0.0f, 0.0f);
      break;
    case SUMO_OPENING_TURN:
      turnDirection = randomTurnDirection();
      applyDrive(0.0f, 0.0f, turnDirection * kOpeningTurnSpeed);
      break;
    case SUMO_SEARCH:
      turnDirection = randomTurnDirection();
      applyDrive(0.0f, 0.0f, turnDirection * kSearchTurnSpeed);
      break;
    case SUMO_ATTACK:
      applyDrive(kAttackForwardSpeed, 0.0f, 0.0f);
      break;
    case SUMO_ESCAPE_REVERSE:
      applyDrive(kEscapeReverseSpeed, 0.0f, 0.0f);
      break;
    case SUMO_ESCAPE_TURN:
      turnDirection = escapeTurnDirection();
      applyDrive(0.0f, 0.0f, turnDirection * kEscapeTurnSpeed);
      break;
  }
}

bool elapsed(unsigned long durationMs) {
  return millis() - stateStartedMs >= durationMs;
}

void readLineSensors() {
  leftLineDetected = digitalRead(kLeftLinePin) == kLineDetectedLevel;
  rightLineDetected = digitalRead(kRightLinePin) == kLineDetectedLevel;
}

float readUltrasonicCm() {
  digitalWrite(kUltrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(kUltrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(kUltrasonicTrigPin, LOW);

  const unsigned long echoUs = pulseIn(kUltrasonicEchoPin, HIGH, kUltrasonicPulseTimeoutUs);
  if (echoUs == 0) {
    return -1.0f;
  }

  return echoUs / 58.0f;
}

void updateSensors(unsigned long nowMs) {
  if (nowMs - lastLineCheckMs >= kLineCheckIntervalMs) {
    lastLineCheckMs = nowMs;
    readLineSensors();
  }

  if (nowMs - lastUltrasonicCheckMs >= kUltrasonicCheckIntervalMs) {
    lastUltrasonicCheckMs = nowMs;
    lastDistanceCm = readUltrasonicCm();
  }
}

bool hasTargetWithin(float distanceCm) {
  return lastDistanceCm > 0.0f && lastDistanceCm <= distanceCm;
}

bool shouldProtectLine() {
  return (leftLineDetected || rightLineDetected) &&
         state != SUMO_WAIT_FOR_TARGET &&
         state != SUMO_COUNTDOWN &&
         state != SUMO_ESCAPE_REVERSE &&
         state != SUMO_ESCAPE_TURN;
}

void updateDebug(unsigned long nowMs) {
  if (!kSerialDebugEnabled || nowMs - lastDebugPrintMs < kDebugPrintIntervalMs) {
    return;
  }

  lastDebugPrintMs = nowMs;

  Serial.print("state=");
  Serial.print(stateName(state));
  Serial.print(" distance_cm=");
  if (lastDistanceCm > 0.0f) {
    Serial.print(lastDistanceCm, 1);
  } else {
    Serial.print("none");
  }
  Serial.print(" line_left=");
  Serial.print(leftLineDetected ? "hit" : "clear");
  Serial.print(" line_right=");
  Serial.print(rightLineDetected ? "hit" : "clear");
  Serial.print(" turn=");
  Serial.println(turnDirection > 0 ? "right" : "left");
}

void updateSumoState() {
  if (shouldProtectLine()) {
    enterState(SUMO_ESCAPE_REVERSE);
    return;
  }

  switch (state) {
    case SUMO_WAIT_FOR_TARGET:
      if (hasTargetWithin(kStartDistanceCm)) {
        enterState(SUMO_COUNTDOWN);
      }
      break;
    case SUMO_COUNTDOWN:
      if (elapsed(kStartCountdownMs)) {
        enterState(SUMO_OPENING_FORWARD);
      }
      break;
    case SUMO_OPENING_FORWARD:
      if (elapsed(kOpeningForwardMs)) {
        enterState(SUMO_OPENING_STOP);
      }
      break;
    case SUMO_OPENING_STOP:
      if (elapsed(kOpeningStopMs)) {
        enterState(SUMO_OPENING_TURN);
      }
      break;
    case SUMO_OPENING_TURN:
      if (elapsed(kOpeningTurnMs)) {
        enterState(SUMO_SEARCH);
      }
      break;
    case SUMO_SEARCH:
      if (hasTargetWithin(kAttackDistanceCm)) {
        enterState(SUMO_ATTACK);
      }
      break;
    case SUMO_ATTACK:
      if (!hasTargetWithin(kAttackDistanceCm)) {
        enterState(SUMO_SEARCH);
      }
      break;
    case SUMO_ESCAPE_REVERSE:
      if (elapsed(kLineEscapeReverseMs)) {
        enterState(SUMO_ESCAPE_TURN);
      }
      break;
    case SUMO_ESCAPE_TURN:
      if (elapsed(kLineEscapeTurnMs)) {
        enterState(SUMO_SEARCH);
      }
      break;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(kLeftLinePin, kUseLineSensorPullups ? INPUT_PULLUP : INPUT);
  pinMode(kRightLinePin, kUseLineSensorPullups ? INPUT_PULLUP : INPUT);
  pinMode(kUltrasonicTrigPin, OUTPUT);
  pinMode(kUltrasonicEchoPin, INPUT);
  digitalWrite(kUltrasonicTrigPin, LOW);

  randomSeed((uint32_t)esp_random());

  robot.beginMecanumRuntime(robotOptions, 0, 1, 2, 3);
  enterState(SUMO_WAIT_FOR_TARGET);
}

void loop() {
  const unsigned long nowMs = millis();

  updateSensors(nowMs);
  updateSumoState();
  updateDebug(nowMs);
  robot.loop();
}
