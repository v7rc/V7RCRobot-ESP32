#pragma once

#include "../../io/V7RCEsp32Outputs.h"
#include "V7RCRobotArmTypes.h"

class V7RCRobotArmRuntime {
public:
  V7RCRobotArmRuntime();

  void begin(const V7RCRobotArmRuntimeOptions& options);
  void moveTo(const V7RCRobotArmPose& pose);
  void home();

private:
  uint16_t angleToPulse(float angleDeg, bool inverted) const;

  bool started_;
  V7RCRobotArmRuntimeOptions options_;
  V7RCEsp32ServoOutput base_;
  V7RCEsp32ServoOutput shoulder_;
  V7RCEsp32ServoOutput elbow_;
  V7RCEsp32ServoOutput wrist_;
  V7RCEsp32ServoOutput gripper_;
};
