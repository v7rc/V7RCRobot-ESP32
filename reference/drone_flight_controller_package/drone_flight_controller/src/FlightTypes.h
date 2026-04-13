#pragma once

#include <stdint.h>

struct ControlState {
  float throttle;   // 0..1
  float roll;       // -1..1
  float pitch;      // -1..1
  float yaw;        // -1..1
  bool stabilize;   // true = angle mode, false = rate/manual mode
};

struct AttitudeState {
  float rollDeg;
  float pitchDeg;

  float rollRateDegPerSec;
  float pitchRateDegPerSec;
  float yawRateDegPerSec;

  bool valid;
};

struct CorrectedAttitude {
  float rollDeg;
  float pitchDeg;

  float rollRateDegPerSec;
  float pitchRateDegPerSec;
  float yawRateDegPerSec;
};

struct AngleLoopOutput {
  float desiredRollRateDegPerSec;
  float desiredPitchRateDegPerSec;
};

struct RateLoopOutput {
  float rollCmd;
  float pitchCmd;
  float yawCmd;
};

struct MotorMixOutput {
  float frontLeft;
  float frontRight;
  float rearLeft;
  float rearRight;
};

struct FlightDebugData {
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

inline float clampf(float x, float minVal, float maxVal) {
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}
