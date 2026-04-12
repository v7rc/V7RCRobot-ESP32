#pragma once

#include "../../legacy/V7RCServoDriver.h"
#include "V7RCCarTypes.h"

class V7RCCarRuntimeConfig {
public:
  V7RCCarRuntimeConfig();

  const V7RC_DriverConfig& buildDifferential(
    const V7RCCarRobotOptions& options,
    int leftMotorIndex,
    int rightMotorIndex
  );

  const V7RC_DriverConfig& buildMecanum(
    const V7RCCarRobotOptions& options,
    int frontLeftMotorIndex,
    int frontRightMotorIndex,
    int rearLeftMotorIndex,
    int rearRightMotorIndex
  );

private:
  void clearChannelMap();
  void fillCommonConfig(const V7RCCarRobotOptions& options);

  V7RC_DriverConfig config_;
  V7RC_SmoothConfig smooth_;
  V7RC_DriveConfig driveConfig_;
  V7RC_ChannelConfig channelMap_[16];
};
