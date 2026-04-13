#pragma once

#include <Arduino.h>
#include "../../../legacy/V7RCServoDriver.h"

enum V7RCDroneV2OutputMode : uint8_t {
  V7RC_DRONE_V2_OUTPUT_SERVO_PWM = 0,
  V7RC_DRONE_V2_OUTPUT_DC_MOTOR
};

struct V7RCDroneV2ControlState {
  float throttle;   // 0..1
  float roll;       // -1..1
  float pitch;      // -1..1
  float yaw;        // -1..1
  bool stabilize;   // true = angle loop, false = rate loop passthrough
};

struct V7RCDroneV2AttitudeState {
  float rollDeg;
  float pitchDeg;
  float rollRateDegPerSec;
  float pitchRateDegPerSec;
  float yawRateDegPerSec;
  bool valid;
};

struct V7RCDroneV2CorrectedAttitude {
  float rollDeg;
  float pitchDeg;
  float rollRateDegPerSec;
  float pitchRateDegPerSec;
  float yawRateDegPerSec;
};

struct V7RCDroneV2AngleLoopOutput {
  float desiredRollRateDegPerSec;
  float desiredPitchRateDegPerSec;
};

struct V7RCDroneV2RateLoopOutput {
  float rollCmd;
  float pitchCmd;
  float yawCmd;
};

struct V7RCDroneV2MotorMix {
  float frontLeft;
  float frontRight;
  float rearLeft;
  float rearRight;
};

struct V7RCDroneV2DebugData {
  float desiredRollDeg;
  float desiredPitchDeg;
  float correctedRollDeg;
  float correctedPitchDeg;
  float desiredRollRateDegPerSec;
  float desiredPitchRateDegPerSec;
  float desiredYawRateDegPerSec;
  float measuredRollRateDegPerSec;
  float measuredPitchRateDegPerSec;
  float measuredYawRateDegPerSec;
  float rollCmd;
  float pitchCmd;
  float yawCmd;
  float throttle;
  float attitudeAuthority;
  bool motorsSaturated;
  bool airborne;
  bool stabilize;
};

struct V7RCDroneV2PidConfig {
  float kp;
  float ki;
  float kd;
  float integralLimit;
  float outputLimit;
};

struct V7RCDroneV2AngleConfig {
  float maxTiltDeg;
  float rollKp;
  float pitchKp;
  float maxRollRateDegPerSec;
  float maxPitchRateDegPerSec;
};

struct V7RCDroneV2RateConfig {
  V7RCDroneV2PidConfig roll;
  V7RCDroneV2PidConfig pitch;
  V7RCDroneV2PidConfig yaw;
  float maxYawRateDegPerSec;
};

struct V7RCDroneV2RuntimeOptions {
  V7RCDroneV2OutputMode outputMode;
  uint8_t motorPins[4];
  V7RC_DCMotorConfig* dcMotors;
  uint8_t numDCMotors;

  uint16_t escMinUs;
  uint16_t escMaxUs;
  uint16_t escIdleUs;

  bool debugEnabled;
  bool autoDetectAirborne;
  float airborneThrottleThreshold;
  float attitudeAuthorityMin;
  float legacyRateFilterAlpha;

  uint16_t unlockHoldMs;
  uint16_t calibrationHoldMs;
  uint16_t readyCueMs;
  float readyCueThrottle;

  V7RCDroneV2AngleConfig angle;
  V7RCDroneV2RateConfig rate;
};

inline float V7RCDroneV2Clamp(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

inline V7RCDroneV2RuntimeOptions V7RCDroneV2DefaultRuntimeOptions() {
  V7RCDroneV2RuntimeOptions options{};
  options.outputMode = V7RC_DRONE_V2_OUTPUT_SERVO_PWM;
  options.motorPins[0] = 0;
  options.motorPins[1] = 0;
  options.motorPins[2] = 0;
  options.motorPins[3] = 0;
  options.dcMotors = nullptr;
  options.numDCMotors = 0;
  options.escMinUs = 1000;
  options.escMaxUs = 2000;
  options.escIdleUs = 1080;
  options.debugEnabled = false;
  options.autoDetectAirborne = true;
  options.airborneThrottleThreshold = 0.18f;
  options.attitudeAuthorityMin = 0.35f;
  options.legacyRateFilterAlpha = 0.35f;
  options.unlockHoldMs = 1500;
  options.calibrationHoldMs = 1500;
  options.readyCueMs = 2000;
  options.readyCueThrottle = 0.08f;

  options.angle.maxTiltDeg = 18.0f;
  options.angle.rollKp = 4.0f;
  options.angle.pitchKp = 4.0f;
  options.angle.maxRollRateDegPerSec = 180.0f;
  options.angle.maxPitchRateDegPerSec = 180.0f;

  options.rate.roll = {0.015f, 0.010f, 0.0008f, 80.0f, 0.5f};
  options.rate.pitch = {0.015f, 0.010f, 0.0008f, 80.0f, 0.5f};
  options.rate.yaw = {0.010f, 0.005f, 0.0000f, 80.0f, 0.3f};
  options.rate.maxYawRateDegPerSec = 150.0f;
  return options;
}
