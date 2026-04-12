# Quick Start (Education) — 10 Minutes

Languages: [繁中](QUICK_START_EDU.md) | `English` | [简中](QUICK_START_EDU.zh-CN.md) | [日本語](QUICK_START_EDU.ja.md)

Goal: get an **ESP32-C3** vehicle moving within 10 minutes and connect it with the **V7RC App**.

## 0) What You Need

- ESP32-C3 development board
- a DC motor driver such as `TB6612FNG` or `DRV8833`
- 2 DC motors for differential drive, or 4 for mecanum
- external motor power
- Arduino IDE with ESP32 core installed

## 1) Install Libraries

In Arduino IDE:

- install `ESP32Servo`
- install `NimBLE-Arduino`
- install `Adafruit NeoPixel`

## 2) Open an Example

Recommended examples:

- differential drive: `examples/ESP32_C3_Differential_Drive_Clean/ESP32_C3_Differential_Drive_Clean.ino`
- BLE mecanum control: `examples/ESP32_C3_Mecanum_V7RC_BLE_Control/ESP32_C3_Mecanum_V7RC_BLE_Control.ino`
- BLE tank control: `examples/ESP32_C3_Tank_V7RC_BLE_Control/ESP32_C3_Tank_V7RC_BLE_Control.ino`
- BLE drone control: `examples/ESP32_C3_Drone_V7RC_BLE_Control/ESP32_C3_Drone_V7RC_BLE_Control.ino`

Legacy examples still include:

- `ESP32_C3_Differential_Drive`
- `ESP32_C3_Mini_Mecanum_V3`

## 3) Includes

Main public headers:

```cpp
#include <V7RCRobot-esp32.h>
#include <V7RCCar-esp32.h>
#include <V7RCDrone-esp32.h>
```

Notes:

- `*_Runtime_Demo` sketches are local runtime/mixer demos
- BLE control examples are the right choice if you want real V7RC App control

## 4) Wiring Basics

- USB powers the ESP32-C3 control board
- motor power should come from an external battery
- all grounds must be shared

## 5) V7RC App Mapping Tips

Differential drive:

- `ch0 = Throttle`
- `ch1 = Steering`

Mecanum:

- `ch0 = Vx`
- `ch1 = Vy`
- `ch3 = Omega`

Tank BLE sample:

- `ch0 = Steering`
- `ch1 = Throttle`
- `ch2 = Barrel`
- `ch3 = Turret`
- `ch4 = Launch`

Drone BLE sample:

- `ch0 = Yaw`
- `ch1 = Throttle`
- `ch2 = Pitch`
- `ch3 = Roll`
- unlock gesture: `1000 / 1000 / 1000 / 2000`

## 6) Minimum Success Criteria

- sketch compiles and uploads
- phone can discover the BLE device
- the V7RC App connects successfully
- the vehicle responds as expected
