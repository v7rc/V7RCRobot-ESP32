#include "V7RCQuadrupedRuntime.h"

#include <math.h>

namespace {

const float kPi = 3.14159265f;
const float kPhaseOffsets[4] = {0.0f, 0.5f, 0.5f, 0.0f};
const float kLegTurnSigns[4] = {-1.0f, 1.0f, -1.0f, 1.0f};

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float wrapUnit(float value) {
  while (value >= 1.0f) value -= 1.0f;
  while (value < 0.0f) value += 1.0f;
  return value;
}

}  // namespace

V7RCQuadrupedRuntime::V7RCQuadrupedRuntime()
  : started_(false),
    startMs_(0) {}

void V7RCQuadrupedRuntime::begin(const V7RCQuadrupedRuntimeOptions& options) {
  options_ = options;
  startMs_ = millis();
  started_ = true;

  for (uint8_t legIndex = 0; legIndex < 4; ++legIndex) {
    hipOutputs_[legIndex].attach(options_.legs[legIndex].hipPin, options_.servoMinUs, options_.servoMaxUs);
    kneeOutputs_[legIndex].attach(options_.legs[legIndex].kneePin, options_.servoMinUs, options_.servoMaxUs);
  }

  stand(options_.nominalBodyHeight);
}

void V7RCQuadrupedRuntime::update(const V7RCQuadrupedControlState& state, unsigned long nowMs) {
  if (!started_) return;

  const float cycleMs = (options_.gaitCycleMs > 1.0f) ? options_.gaitCycleMs : 1200.0f;
  const float gaitPhase = wrapUnit((float)(nowMs - startMs_) / cycleMs);
  const float forward = clampFloat(state.vx, -1.0f, 1.0f);
  const float turn = clampFloat(state.yaw, -1.0f, 1.0f);
  const float bodyHeight = (state.bodyHeight > 0.01f) ? state.bodyHeight : options_.nominalBodyHeight;
  const float stepLength = options_.stepLengthScale * forward;
  const float turnLength = options_.stepLengthScale * 0.6f * turn;
  const float swingPortion = 0.35f;

  for (uint8_t legIndex = 0; legIndex < 4; ++legIndex) {
    const float legPhase = wrapUnit(gaitPhase + kPhaseOffsets[legIndex]);
    const float turnOffset = turnLength * kLegTurnSigns[legIndex];

    V7RCQuadrupedFootPoint footPoint = {
      options_.nominalX + turnOffset,
      bodyHeight,
    };

    if (legPhase < swingPortion) {
      const float t = legPhase / swingPortion;
      footPoint.x += (-0.5f + t) * stepLength;
      footPoint.z -= sinf(t * kPi) * options_.stepHeight;
    } else {
      const float t = (legPhase - swingPortion) / (1.0f - swingPortion);
      footPoint.x += (0.5f - t) * stepLength;
    }

    writeLeg(legIndex, footPoint);
  }
}

void V7RCQuadrupedRuntime::stand(float bodyHeight) {
  if (!started_) return;

  const float targetHeight = (bodyHeight > 0.01f) ? bodyHeight : options_.nominalBodyHeight;
  const V7RCQuadrupedFootPoint neutralFoot = {
    options_.nominalX,
    targetHeight,
  };

  for (uint8_t legIndex = 0; legIndex < 4; ++legIndex) {
    writeLeg(legIndex, neutralFoot);
  }
}

void V7RCQuadrupedRuntime::writeLeg(uint8_t legIndex, const V7RCQuadrupedFootPoint& footPoint) {
  const V7RCQuadrupedLegIk ik = solveIk(footPoint);
  hipOutputs_[legIndex].writeMicroseconds(angleToPulse(ik.hipDeg, options_.legs[legIndex].hipInvert));
  kneeOutputs_[legIndex].writeMicroseconds(angleToPulse(ik.kneeDeg, options_.legs[legIndex].kneeInvert));
}

V7RCQuadrupedLegIk V7RCQuadrupedRuntime::solveIk(const V7RCQuadrupedFootPoint& footPoint) const {
  const float l1 = options_.upperLegLength;
  const float l2 = options_.lowerLegLength;
  const float minReach = fabsf(l1 - l2) + 0.002f;
  const float maxReach = l1 + l2 - 0.002f;

  float reach = sqrtf((footPoint.x * footPoint.x) + (footPoint.z * footPoint.z));
  reach = clampFloat(reach, minReach, maxReach);

  const float x = footPoint.x;
  const float z = footPoint.z;
  const float cosKnee = clampFloat(((l1 * l1) + (l2 * l2) - (reach * reach)) / (2.0f * l1 * l2), -1.0f, 1.0f);
  const float kneeInterior = acosf(cosKnee);

  const float cosHip = clampFloat(((l1 * l1) + (reach * reach) - (l2 * l2)) / (2.0f * l1 * reach), -1.0f, 1.0f);
  const float hipOffset = acosf(cosHip);
  const float lineAngle = atan2f(z, x);

  const float hipDeg = clampFloat((lineAngle - hipOffset) * 180.0f / kPi, -80.0f, 80.0f);
  const float kneeDeg = clampFloat((kPi - kneeInterior) * 180.0f / kPi, 10.0f, 170.0f);

  return {hipDeg, kneeDeg};
}

uint16_t V7RCQuadrupedRuntime::angleToPulse(float angleDeg, bool inverted) const {
  float normalized = (angleDeg + 90.0f) / 180.0f;
  normalized = clampFloat(normalized, 0.0f, 1.0f);
  if (inverted) {
    normalized = 1.0f - normalized;
  }

  const float span = (float)(options_.servoMaxUs - options_.servoMinUs);
  return (uint16_t)(options_.servoMinUs + (normalized * span));
}
