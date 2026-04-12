# V7RCRobot-esp32

`V7RCRobot-esp32` 是一個明確鎖定 ESP32 的 V7RC 多載具控制平台初始化 repo。

目前這個 repo 的重點不是一次把所有新架構實作完，而是先把專案整理成適合後續重構的骨架：

- 把 transport、protocol、core、io 拆成獨立層
- 把不同 vehicle module 的位置先固定下來
- 保留既有 `V7RCServoDriver` 與舊 examples 作為 legacy 參考
- 將架構與 bootstrap 文件收斂到 `docs/architecture/`

## Current Status

目前對外可用入口已分成兩層：

- `V7RCRobot-esp32.h`
  - 提供目前完整、相容性優先的 ESP32-only 入口
- `V7RCCar-esp32.h`
  - 提供 car module 的較乾淨入口與 facade
- `V7RCDrone-esp32.h`
  - 提供 drone runtime、IMU abstraction 與 MPU6050 實作
- `V7RCQuadruped-esp32.h`
  - 提供 quadruped runtime 與 dog demo 所需型別
- `V7RCOtto-esp32.h`
  - 提供 OTTO runtime 與基礎 gait demo 型別
- `V7RCRobotArm-esp32.h`
  - 提供 robot arm runtime 與 pose demo 型別

legacy driver 仍是目前主要 runtime 基底：

- `src/legacy/V7RCServoDriver.h`
- `src/legacy/V7RCServoDriver.cpp`
- `examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`
- `examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`

新的核心分層已經建立，但各層尚待從 legacy driver 中逐步拆出。

## Installation

在 Arduino IDE / Arduino CLI 中，這個 library 目前預期搭配以下依賴使用：

- `ESP32Servo`
- `NimBLE-Arduino`
- `Adafruit NeoPixel`

目前 `library.properties` 已宣告這些依賴，但首次開發仍建議先確認 ESP32 core 與相關 library 已安裝完成。

## Public Entry

對外公共 include 入口為：

```cpp
#include <V7RCRobot-esp32.h>
```

這個入口檔明確鎖定 ESP32；若在非 ESP32 架構下編譯，會直接報錯。

如果你要直接使用 car module 相關型別，則可使用：

```cpp
#include <V7RCCar-esp32.h>
```

如果你要直接使用 drone runtime 與 MPU6050 IMU，則可使用：

```cpp
#include <V7RCDrone-esp32.h>
```

如果你要直接使用 quadruped runtime，則可使用：

```cpp
#include <V7RCQuadruped-esp32.h>
```

如果你要直接使用 OTTO runtime，則可使用：

```cpp
#include <V7RCOtto-esp32.h>
```

如果你要直接使用 robot arm runtime，則可使用：

```cpp
#include <V7RCRobotArm-esp32.h>
```

## Example Tiers

目前 examples 可分成兩類：

- legacy examples
  - `ESP32_C3_Differential_Drive`
  - `ESP32_C3_Mini_Mecanum_V3`
- clean car examples
  - `ESP32_C3_Differential_Drive_Clean`
  - `ESP32_C3_Car_Runtime_Demo`
  - `ESP32_C3_Differential_Runtime_Demo`
  - `ESP32_C3_Mecanum_Runtime_Demo`
  - `ESP32_C3_Mecanum_V7RC_BLE_Control`
  - `ESP32_C3_Mini_Mecanum_Clean`
  - `ESP32_C3_Tank_V7RC_BLE_Control`
  - `ESP32_C3_Tank_Runtime_Demo`
- drone runtime example
  - `ESP32_C3_Drone_Runtime_Demo`
- quadruped runtime example
  - `ESP32_C3_Dog_Runtime_Demo`
  - `ESP32_C3_Dog_Runtime_Demo_V2`
- OTTO runtime example
  - `ESP32_C3_OTTO_Runtime_Demo`
- robot arm runtime example
  - `ESP32_C3_RobotARM_Runtime_Demo`

如果是新專案，建議優先從 clean examples 開始。
如果你要直接試 standalone car runtime，可從 `ESP32_C3_Differential_Runtime_Demo` 開始。
如果你要用 V7RC App 透過 BLE 實際控制麥克納姆車，請用 `ESP32_C3_Mecanum_V7RC_BLE_Control` 或 `ESP32_C3_Mini_Mecanum_Clean`。
目前 `ESP32_C3_Mecanum_V7RC_BLE_Control` 預設使用 `ch0=Vx`、`ch1=Vy`、`ch3=Omega`、`ch2=Servo(GPIO7)`、`ch5=Servo(GPIO6)`，也可在 sketch 內直接調整 channel mapping。
如果你要用 V7RC App 透過 BLE 控制 tank，請用 `ESP32_C3_Tank_V7RC_BLE_Control`。
如果你要試 drone runtime，可從 `ESP32_C3_Drone_Runtime_Demo` 開始。
如果你要試 quadruped runtime，可從 `ESP32_C3_Dog_Runtime_Demo` 開始。
如果你要試 OTTO runtime，可從 `ESP32_C3_OTTO_Runtime_Demo` 開始。
如果你要試 robot arm runtime，可從 `ESP32_C3_RobotARM_Runtime_Demo` 開始。

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
  ESP32_C3_Differential_Drive/
  ESP32_C3_Differential_Drive_Clean/
  ESP32_C3_Mini_Mecanum_Clean/
  ESP32_C3_Mini_Mecanum_V3/

docs/
  architecture/
```

## Architecture Docs

- `docs/architecture/MULTI_VEHICLE_LIBRARY_ARCHITECTURE.md`
- `docs/architecture/NEW_REPO_BOOTSTRAP_GUIDE.md`

## Validation

若本機已安裝工具，可用下列方式重跑目前的基線檢查：

```sh
ARDUINO_CLI=arduino-cli ./scripts/compile-examples.sh
ARDUINO_LINT=arduino-lint ./scripts/lint-library.sh
```

預設 compile 板型為 `esp32:esp32:esp32c3`，可透過 `FQBN=...` 覆蓋。

## Release Notes

發版前建議檢查：

- [docs/RELEASE_CHECKLIST.md](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/docs/RELEASE_CHECKLIST.md)
- [CHANGELOG.md](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/CHANGELOG.md)

## Next Steps

1. 把更多 runtime / output 從 legacy driver 遷移到 core / io abstraction
2. 讓 `V7RCCarRobot` 逐步減少對 legacy driver 的依賴
3. 規劃下一個 vehicle module 的 facade 與 examples
