# Quick Start (Education) — 10 Minutes

Languages: [繁中](QUICK_START_EDU.md) | [English](QUICK_START_EDU.en.md) | `简中` | [日本語](QUICK_START_EDU.ja.md)

目标：在 10 分钟内让 **ESP32-C3** 载具动起来，并通过 **V7RC App** 完成蓝牙连接控制。

## 0) 你需要准备

- ESP32-C3 开发板
- 直流电机驱动板，例如 `TB6612FNG` 或 `DRV8833`
- 差速车 2 个电机，麦克纳姆 4 个电机
- 外接电源
- 已安装 ESP32 Core 的 Arduino IDE

## 1) 安装库

在 Arduino IDE 中安装：

- `ESP32Servo`
- `NimBLE-Arduino`
- `Adafruit NeoPixel`

## 2) 打开示例

推荐示例：

- 差速车：`examples/ESP32_C3_Differential_Drive_Clean/ESP32_C3_Differential_Drive_Clean.ino`
- BLE 麦克纳姆：`examples/ESP32_C3_Mecanum_V7RC_BLE_Control/ESP32_C3_Mecanum_V7RC_BLE_Control.ino`
- BLE 坦克：`examples/ESP32_C3_Tank_V7RC_BLE_Control/ESP32_C3_Tank_V7RC_BLE_Control.ino`
- BLE 无人机：`examples/ESP32_C3_Drone_V7RC_BLE_Control/ESP32_C3_Drone_V7RC_BLE_Control.ino`

legacy 示例：

- `ESP32_C3_Differential_Drive`
- `ESP32_C3_Mini_Mecanum_V3`

## 3) Include 入口

主要公共头文件：

```cpp
#include <V7RCRobot-esp32.h>
#include <V7RCCar-esp32.h>
#include <V7RCDrone-esp32.h>
```

说明：

- `*_Runtime_Demo` 更偏向本地 runtime / mixer 验证
- 如果要真的让 V7RC App 控制，请优先使用 BLE control 示例

## 4) 接线基础

- ESP32-C3 控制板可由 USB 供电
- 电机电源请使用外接电池
- 所有 GND 必须共地

## 5) V7RC App Channel 建议

差速车：

- `ch0 = Throttle`
- `ch1 = Steering`

麦克纳姆：

- `ch0 = Vx`
- `ch1 = Vy`
- `ch3 = Omega`

坦克 BLE 示例：

- `ch0 = Steering`
- `ch1 = Throttle`
- `ch2 = Barrel`
- `ch3 = Turret`
- `ch4 = Launch`

无人机 BLE 示例：

- `ch0 = Yaw`
- `ch1 = Throttle`
- `ch2 = Pitch`
- `ch3 = Roll`
- 解锁手势：`1000 / 1000 / 1000 / 2000`

## 6) 最小验收

- sketch 编译并上传成功
- 手机可以搜索到 BLE 设备
- V7RC App 能成功连接
- 载具动作符合预期
