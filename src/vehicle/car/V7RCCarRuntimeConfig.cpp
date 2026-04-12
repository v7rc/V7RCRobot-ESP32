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

void V7RCCarRuntimeConfig::applyCustomChannelMap(const V7RCCarRobotOptions& options) {
  const uint8_t count = (options.numCustomChannelMap < 16) ? options.numCustomChannelMap : 16;
  for (uint8_t i = 0; i < count; ++i) {
    channelMap_[i] = options.customChannelMap[i];
  }
}

void V7RCCarRuntimeConfig::fillCommonConfig(const V7RCCarRobotOptions& options) {
  config_.bleBaseName = options.bleBaseName;
  config_.servos = options.servos;
  config_.numServos = options.numServos;
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
  if (options.customChannelMap && options.numCustomChannelMap > 0) {
    applyCustomChannelMap(options);
  } else {
    if (options.differentialThrottleChannel < 16) {
      channelMap_[options.differentialThrottleChannel].role = CH_DRIVE_THROTTLE;
    }
    if (options.differentialSteerChannel < 16) {
      channelMap_[options.differentialSteerChannel].role = CH_DRIVE_STEER;
    }
    if (options.numServos > 0 && options.auxiliaryServo0Channel < 16) {
      channelMap_[options.auxiliaryServo0Channel].role = CH_SERVO;
      channelMap_[options.auxiliaryServo0Channel].targetIndex = 0;
    }
    if (options.numServos > 1 && options.auxiliaryServo1Channel < 16) {
      channelMap_[options.auxiliaryServo1Channel].role = CH_SERVO;
      channelMap_[options.auxiliaryServo1Channel].targetIndex = 1;
    }
    if (options.numServos > 2 && options.auxiliaryServo2Channel < 16) {
      channelMap_[options.auxiliaryServo2Channel].role = CH_SERVO;
      channelMap_[options.auxiliaryServo2Channel].targetIndex = 2;
    }
  }

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
  if (options.customChannelMap && options.numCustomChannelMap > 0) {
    applyCustomChannelMap(options);
  } else {
    if (options.mecanumVxChannel < 16) {
      channelMap_[options.mecanumVxChannel].role = CH_DRIVE_MEC_VX;
    }
    if (options.mecanumVyChannel < 16) {
      channelMap_[options.mecanumVyChannel].role = CH_DRIVE_MEC_VY;
    }
    if (options.mecanumOmegaChannel < 16) {
      channelMap_[options.mecanumOmegaChannel].role = CH_DRIVE_MEC_OMEGA;
    }
    if (options.numServos > 0 && options.auxiliaryServo0Channel < 16) {
      channelMap_[options.auxiliaryServo0Channel].role = CH_SERVO;
      channelMap_[options.auxiliaryServo0Channel].targetIndex = 0;
    }
    if (options.numServos > 1 && options.auxiliaryServo1Channel < 16) {
      channelMap_[options.auxiliaryServo1Channel].role = CH_SERVO;
      channelMap_[options.auxiliaryServo1Channel].targetIndex = 1;
    }
    if (options.numServos > 2 && options.auxiliaryServo2Channel < 16) {
      channelMap_[options.auxiliaryServo2Channel].role = CH_SERVO;
      channelMap_[options.auxiliaryServo2Channel].targetIndex = 2;
    }
  }

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
