# Changelog

## Unreleased

- Bootstrap `V7RCRobot-esp32` 專案結構，建立 `transport`、`protocol`、`core`、`io`、`legacy` 與 `vehicle` 分層
- 將 legacy driver 與舊 examples 移到新結構
- 建立 ESP32-only 公共入口 `V7RCRobot-esp32.h`
- 建立 car-specific 公共入口 `V7RCCar-esp32.h`
- 抽出 BLE transport layer
- 抽出 V7RC protocol decode layer
- 建立 `V7RCRuntimeState` 與 IO abstraction 骨架
- 建立 car module 的 control state、mixer 與 facade
- 建立 drone runtime、MPU6050 IMU abstraction 與安全解鎖流程
- 建立 quadruped runtime 與 dog runtime demos
- 建立 OTTO runtime 與 OTTO runtime demo
- 建立 robot arm runtime 與 robot arm runtime demo
- 新增 clean examples：
  - `ESP32_C3_Differential_Drive_Clean`
  - `ESP32_C3_Mini_Mecanum_Clean`
- 新增 runtime demos：
  - `ESP32_C3_Car_Runtime_Demo`
  - `ESP32_C3_Differential_Runtime_Demo`
  - `ESP32_C3_Dog_Runtime_Demo`
  - `ESP32_C3_Dog_Runtime_Demo_V2`
  - `ESP32_C3_Drone_Runtime_Demo`
  - `ESP32_C3_Mecanum_Runtime_Demo`
  - `ESP32_C3_Mecanum_V7RC_BLE_Control`
  - `ESP32_C3_OTTO_Runtime_Demo`
  - `ESP32_C3_RobotARM_Runtime_Demo`
  - `ESP32_C3_Tank_Runtime_Demo`
- 建立 compile / lint 驗證腳本
