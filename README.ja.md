# V7RCRobot-esp32

Languages: [繁中](README.md) | [English](README.en.md) | [简中](README.zh-CN.md) | `日本語`

`V7RCRobot-esp32` は、ESP32 向けに明確に設計された V7RC マルチビークル制御ライブラリのブートストラップ用リポジトリです。

このリポジトリの目的は、新しいアーキテクチャを一度に完成させることではありません。継続的なリファクタリングのための土台を整えることが主目的です。

- transport、protocol、core、io を独立した層に分離
- vehicle module ごとの配置を固定
- 既存の `V7RCServoDriver` と legacy examples を参照実装として保持
- アーキテクチャ文書を `docs/architecture/` に集約

## 現在の状態

公開エントリヘッダ：

- `V7RCRobot-esp32.h`
- `V7RCCar-esp32.h`
- `V7RCDrone-esp32.h`
- `V7RCQuadruped-esp32.h`
- `V7RCOtto-esp32.h`
- `V7RCRobotArm-esp32.h`

互換性の基盤として legacy runtime も残しています：

- `src/legacy/V7RCServoDriver.h`
- `src/legacy/V7RCServoDriver.cpp`

## インストール

Arduino IDE / Arduino CLI で現在想定している依存関係：

- `ESP32Servo`
- `NimBLE-Arduino`
- `Adafruit NeoPixel`

このライブラリは ESP32 専用です。非 ESP32 ボードではエラーになります。

## 公開エントリ

基本の include：

```cpp
#include <V7RCRobot-esp32.h>
```

モジュール別エントリ：

```cpp
#include <V7RCCar-esp32.h>
#include <V7RCDrone-esp32.h>
#include <V7RCQuadruped-esp32.h>
#include <V7RCOtto-esp32.h>
#include <V7RCRobotArm-esp32.h>
```

## サンプル

legacy examples：

- `ESP32_C3_Differential_Drive`
- `ESP32_C3_Mini_Mecanum_V3`

car examples：

- `ESP32_C3_Differential_Drive_Clean`
- `ESP32_C3_Car_Runtime_Demo`
- `ESP32_C3_Differential_Runtime_Demo`
- `ESP32_C3_Mecanum_Runtime_Demo`
- `ESP32_C3_Mecanum_V7RC_BLE_Control`
- `ESP32_C3_Mini_Mecanum_Clean`
- `ESP32_C3_Tank_V7RC_BLE_Control`
- `ESP32_C3_Tank_Runtime_Demo`

drone examples：

- `ESP32_C3_Drone_Runtime_Demo`
- `ESP32_C3_Drone_V7RC_BLE_Control`

その他の vehicle examples：

- `ESP32_C3_Dog_Runtime_Demo`
- `ESP32_C3_Dog_Runtime_Demo_V2`
- `ESP32_C3_OTTO_Runtime_Demo`
- `ESP32_C3_RobotARM_Runtime_Demo`

おすすめの開始点：

- 単体 car runtime: `ESP32_C3_Differential_Runtime_Demo`
- V7RC App から BLE で mecanum を操作: `ESP32_C3_Mecanum_V7RC_BLE_Control`
- V7RC App から BLE で tank を操作: `ESP32_C3_Tank_V7RC_BLE_Control`
- V7RC App から BLE で drone を操作: `ESP32_C3_Drone_V7RC_BLE_Control`
- `ESP32_C3_Drone_V7RC_BLE_Control` では sketch 冒頭で `V7RC_DRONE_IMU_NONE`、`V7RC_DRONE_IMU_ADXL345`、`V7RC_DRONE_IMU_ICM20948` を切り替えできます。現在の既定値は `ICM20948` です。

## プロジェクト構成

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

## 検証

```sh
ARDUINO_CLI=arduino-cli ./scripts/compile-examples.sh
ARDUINO_LINT=arduino-lint ./scripts/lint-library.sh
```

既定の FQBN：

- `esp32:esp32:esp32c3`

## リリース前チェック

- [docs/RELEASE_CHECKLIST.md](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/docs/RELEASE_CHECKLIST.md)
- [CHANGELOG.md](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/CHANGELOG.md)
