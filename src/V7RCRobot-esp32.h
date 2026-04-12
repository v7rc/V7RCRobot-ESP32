#pragma once

// ESP32-only umbrella header for the V7RC multi-vehicle platform.
//
// During the bootstrap phase we still expose the legacy driver so
// existing sketches can keep working while transport/protocol/core
// modules are extracted into their own layers.

#if !defined(ARDUINO_ARCH_ESP32)
#error "V7RCRobot-ESP32 only supports ESP32 boards."
#endif

#include "legacy/V7RCServoDriver.h"
