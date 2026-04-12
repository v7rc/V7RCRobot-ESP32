#include "V7RCCarRuntimeConfig.h"

V7RCCarRuntimeConfig::V7RCCarRuntimeConfig() {
  smooth_.maxStepDeg = 45.0f;
  smooth_.deadbandDeg = 2.0f;
  smooth_.updateMs = 10;
  clearChannelMap();
}

void V7RCCarRuntimeConfig::clearChannelMap() {
  for (uint8_t i = 0; i < 16; ++i) {
    channelMap_[i].role = CH_NONE;
    channelMap_[i].targetIndex = -1;
  }
}

void V7RCCarRuntimeConfig::fillCommonConfig(const V7RCCarRobotOptions& options) {
  config_.bleBaseName = options.bleBaseName;
  config_.servos = nullptr;
  config_.numServos = 0;
  config_.smooth = smooth_;
  config_.waveDemoServoIndex = -1;
  config_.dcMotors = options.motors;
  config_.numDCMotors = options.numMotors;
  config_.drive = driveConfig_;
  config_.channelMap = channelMap_;
  config_.numChannelMap = 16;
  config_.ws2812Brightness = options.ws2812Brightness;
  config_.ws2812Enable = options.ws2812Enable;
  config_.ws2812Pin = options.ws2812Pin;
  config_.ws2812Count = options.ws2812Count;
}

const V7RC_DriverConfig& V7RCCarRuntimeConfig::buildDifferential(
  const V7RCCarRobotOptions& options,
  int leftMotorIndex,
  int rightMotorIndex
) {
  clearChannelMap();
  channelMap_[0].role = CH_DRIVE_THROTTLE;
  channelMap_[1].role = CH_DRIVE_STEER;

  driveConfig_.type = DRIVE_DIFF;
  driveConfig_.diffLeftMotor = leftMotorIndex;
  driveConfig_.diffRightMotor = rightMotorIndex;
  driveConfig_.mecFrontLeft = -1;
  driveConfig_.mecFrontRight = -1;
  driveConfig_.mecRearLeft = -1;
  driveConfig_.mecRearRight = -1;

  fillCommonConfig(options);
  return config_;
}

const V7RC_DriverConfig& V7RCCarRuntimeConfig::buildMecanum(
  const V7RCCarRobotOptions& options,
  int frontLeftMotorIndex,
  int frontRightMotorIndex,
  int rearLeftMotorIndex,
  int rearRightMotorIndex
) {
  clearChannelMap();
  channelMap_[0].role = CH_DRIVE_MEC_VX;
  channelMap_[1].role = CH_DRIVE_MEC_VY;
  channelMap_[2].role = CH_DRIVE_MEC_OMEGA;

  driveConfig_.type = DRIVE_MECANUM;
  driveConfig_.diffLeftMotor = -1;
  driveConfig_.diffRightMotor = -1;
  driveConfig_.mecFrontLeft = frontLeftMotorIndex;
  driveConfig_.mecFrontRight = frontRightMotorIndex;
  driveConfig_.mecRearLeft = rearLeftMotorIndex;
  driveConfig_.mecRearRight = rearRightMotorIndex;

  fillCommonConfig(options);
  return config_;
}
