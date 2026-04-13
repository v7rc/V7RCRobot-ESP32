# V7RCRobot-esp32

Languages: [繁中](README.md) | [English](README.en.md) | `简中` | [日本語](README.ja.md)

`V7RCRobot-esp32` 是一个明确面向 ESP32 的 V7RC 多载具控制库初始化仓库。

这个仓库的目标不是一次性完成全部新架构，而是先建立一个适合持续重构的基础：

- 将 transport、protocol、core、io 拆成独立层
- 预留不同 vehicle module 的固定位置
- 保留现有 `V7RCServoDriver` 与 legacy examples 作为参考
- 将架构与 bootstrap 文档集中到 `docs/architecture/`

## 当前状态

当前对外入口头文件：

- `V7RCRobot-esp32.h`
- `V7RCCar-esp32.h`
- `V7RCDrone-esp32.h`
- `V7RCQuadruped-esp32.h`
- `V7RCOtto-esp32.h`
- `V7RCRobotArm-esp32.h`

legacy runtime 仍然是重要的兼容基础：

- `src/legacy/V7RCServoDriver.h`
- `src/legacy/V7RCServoDriver.cpp`

## 安装

Arduino IDE / Arduino CLI 目前预期依赖：

- `ESP32Servo`
- `NimBLE-Arduino`
- `Adafruit NeoPixel`

本库只支持 ESP32，非 ESP32 板卡会直接报错。

## 公共入口

默认入口：

```cpp
#include <V7RCRobot-esp32.h>
```

模块入口：

```cpp
#include <V7RCCar-esp32.h>
#include <V7RCDrone-esp32.h>
#include <V7RCQuadruped-esp32.h>
#include <V7RCOtto-esp32.h>
#include <V7RCRobotArm-esp32.h>
```

## 示例

legacy 示例：

- `ESP32_C3_Differential_Drive`
- `ESP32_C3_Mini_Mecanum_V3`

车类示例：

- `ESP32_C3_Differential_Drive_Clean`
- `ESP32_C3_Car_Runtime_Demo`
- `ESP32_C3_Differential_Runtime_Demo`
- `ESP32_C3_Mecanum_Runtime_Demo`
- `ESP32_C3_Mecanum_V7RC_BLE_Control`
- `ESP32_C3_Mini_Mecanum_Clean`
- `ESP32_C3_Tank_V7RC_BLE_Control`
- `ESP32_C3_Tank_Runtime_Demo`

无人机示例：

- `ESP32_C3_Drone_Runtime_Demo`
- `ESP32_C3_Drone_V7RC_BLE_Control`

其他载具示例：

- `ESP32_C3_Dog_Runtime_Demo`
- `ESP32_C3_Dog_Runtime_Demo_V2`
- `ESP32_C3_OTTO_Runtime_Demo`
- `ESP32_C3_RobotARM_Runtime_Demo`

建议入口：

- 独立 car runtime：`ESP32_C3_Differential_Runtime_Demo`
- V7RC App 蓝牙麦克纳姆控制：`ESP32_C3_Mecanum_V7RC_BLE_Control`
- V7RC App 蓝牙坦克控制：`ESP32_C3_Tank_V7RC_BLE_Control`
- V7RC App 蓝牙无人机控制：`ESP32_C3_Drone_V7RC_BLE_Control`
- `ESP32_C3_Drone_V7RC_BLE_Control` 可在 sketch 顶部切换 `V7RC_DRONE_IMU_NONE`、`V7RC_DRONE_IMU_ADXL345`、`V7RC_DRONE_IMU_ICM20948`，当前默认是 `ICM20948`

## 项目结构

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

## 验证

```sh
ARDUINO_CLI=arduino-cli ./scripts/compile-examples.sh
ARDUINO_LINT=arduino-lint ./scripts/lint-library.sh
```

默认 FQBN：

- `esp32:esp32:esp32c3`

## 发版说明

- [docs/RELEASE_CHECKLIST.md](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/docs/RELEASE_CHECKLIST.md)
- [CHANGELOG.md](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/CHANGELOG.md)
