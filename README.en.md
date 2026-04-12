# V7RCRobot-esp32

Languages: [繁中](README.md) | `English` | [简中](README.zh-CN.md) | [日本語](README.ja.md)

`V7RCRobot-esp32` is an ESP32-focused V7RC multi-vehicle control library bootstrap repository.

The goal of this repository is not to finish the full architecture all at once. Instead, it provides a clean starting point for ongoing refactoring:

- split transport, protocol, core, and io into separate layers
- reserve clear locations for vehicle-specific modules
- keep the existing `V7RCServoDriver` and legacy examples as references
- centralize architecture and bootstrap docs under `docs/architecture/`

## Current Status

Public entry headers:

- `V7RCRobot-esp32.h`
- `V7RCCar-esp32.h`
- `V7RCDrone-esp32.h`
- `V7RCQuadruped-esp32.h`
- `V7RCOtto-esp32.h`
- `V7RCRobotArm-esp32.h`

The legacy runtime is still an important compatibility base:

- `src/legacy/V7RCServoDriver.h`
- `src/legacy/V7RCServoDriver.cpp`

## Installation

Dependencies currently expected in Arduino IDE / Arduino CLI:

- `ESP32Servo`
- `NimBLE-Arduino`
- `Adafruit NeoPixel`

The library is ESP32-only and will fail fast on non-ESP32 boards.

## Public Entry

Default include:

```cpp
#include <V7RCRobot-esp32.h>
```

Module-specific entry headers:

```cpp
#include <V7RCCar-esp32.h>
#include <V7RCDrone-esp32.h>
#include <V7RCQuadruped-esp32.h>
#include <V7RCOtto-esp32.h>
#include <V7RCRobotArm-esp32.h>
```

## Examples

Legacy examples:

- `ESP32_C3_Differential_Drive`
- `ESP32_C3_Mini_Mecanum_V3`

Car examples:

- `ESP32_C3_Differential_Drive_Clean`
- `ESP32_C3_Car_Runtime_Demo`
- `ESP32_C3_Differential_Runtime_Demo`
- `ESP32_C3_Mecanum_Runtime_Demo`
- `ESP32_C3_Mecanum_V7RC_BLE_Control`
- `ESP32_C3_Mini_Mecanum_Clean`
- `ESP32_C3_Tank_V7RC_BLE_Control`
- `ESP32_C3_Tank_Runtime_Demo`

Drone examples:

- `ESP32_C3_Drone_Runtime_Demo`
- `ESP32_C3_Drone_V7RC_BLE_Control`

Other vehicle examples:

- `ESP32_C3_Dog_Runtime_Demo`
- `ESP32_C3_Dog_Runtime_Demo_V2`
- `ESP32_C3_OTTO_Runtime_Demo`
- `ESP32_C3_RobotARM_Runtime_Demo`

Suggested starting points:

- standalone car runtime: `ESP32_C3_Differential_Runtime_Demo`
- BLE mecanum control via V7RC App: `ESP32_C3_Mecanum_V7RC_BLE_Control`
- BLE tank control via V7RC App: `ESP32_C3_Tank_V7RC_BLE_Control`
- BLE drone control via V7RC App: `ESP32_C3_Drone_V7RC_BLE_Control`

## Project Layout

```text
src/
  V7RCRobot-esp32.h
  transport/
  protocol/
  core/
  io/
  legacy/

vehicle/
  car/
  drone/
  quadruped/
  otto/
  arm/

examples/
  ...

docs/
  architecture/
```

## Validation

```sh
ARDUINO_CLI=arduino-cli ./scripts/compile-examples.sh
ARDUINO_LINT=arduino-lint ./scripts/lint-library.sh
```

Default FQBN:

- `esp32:esp32:esp32c3`

## Release Notes

- [docs/RELEASE_CHECKLIST.md](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/docs/RELEASE_CHECKLIST.md)
- [CHANGELOG.md](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/CHANGELOG.md)
