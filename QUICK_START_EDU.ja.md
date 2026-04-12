# Quick Start (Education) — 10 Minutes

Languages: [繁中](QUICK_START_EDU.md) | [English](QUICK_START_EDU.en.md) | [简中](QUICK_START_EDU.zh-CN.md) | `日本語`

目的：**ESP32-C3** の機体を 10 分以内に動かし、**V7RC App** から BLE 接続で制御できる状態にすることです。

## 0) 必要なもの

- ESP32-C3 開発ボード
- `TB6612FNG` または `DRV8833` などの DC モータードライバ
- 差動駆動なら 2 モーター、メカナムなら 4 モーター
- 外部電源
- ESP32 Core を導入済みの Arduino IDE

## 1) ライブラリをインストール

Arduino IDE で以下をインストールします：

- `ESP32Servo`
- `NimBLE-Arduino`
- `Adafruit NeoPixel`

## 2) サンプルを開く

推奨サンプル：

- 差動駆動: `examples/ESP32_C3_Differential_Drive_Clean/ESP32_C3_Differential_Drive_Clean.ino`
- BLE メカナム制御: `examples/ESP32_C3_Mecanum_V7RC_BLE_Control/ESP32_C3_Mecanum_V7RC_BLE_Control.ino`
- BLE タンク制御: `examples/ESP32_C3_Tank_V7RC_BLE_Control/ESP32_C3_Tank_V7RC_BLE_Control.ino`
- BLE ドローン制御: `examples/ESP32_C3_Drone_V7RC_BLE_Control/ESP32_C3_Drone_V7RC_BLE_Control.ino`

legacy examples：

- `ESP32_C3_Differential_Drive`
- `ESP32_C3_Mini_Mecanum_V3`

## 3) Include 入口

主な公開ヘッダ：

```cpp
#include <V7RCRobot-esp32.h>
#include <V7RCCar-esp32.h>
#include <V7RCDrone-esp32.h>
```

注意：

- `*_Runtime_Demo` はローカル runtime / mixer の確認向けです
- V7RC App で実際に操作したい場合は BLE control サンプルを使ってください

## 4) 配線の基本

- ESP32-C3 制御基板は USB 給電
- モーター用電源は外部バッテリーを使用
- すべての GND は共通化が必要

## 5) V7RC App の Channel 例

差動駆動：

- `ch0 = Throttle`
- `ch1 = Steering`

メカナム：

- `ch0 = Vx`
- `ch1 = Vy`
- `ch3 = Omega`

タンク BLE サンプル：

- `ch0 = Steering`
- `ch1 = Throttle`
- `ch2 = Barrel`
- `ch3 = Turret`
- `ch4 = Launch`

ドローン BLE サンプル：

- `ch0 = Yaw`
- `ch1 = Throttle`
- `ch2 = Pitch`
- `ch3 = Roll`
- アンロックジェスチャー: `1000 / 1000 / 1000 / 2000`

## 6) 最低限の確認項目

- sketch がコンパイル・書き込み成功
- スマートフォンから BLE デバイスを見つけられる
- V7RC App で接続できる
- 機体が期待通りに動く
