# Drone Module

預留給四軸飛行器控制模組。

預計涵蓋：

- throttle / roll / pitch / yaw
- arm / disarm safety
- mixer 與飛行狀態管理

目前已建立第一版檔案：

- `src/vehicle/drone/V7RCDroneTypes.h`
- `src/vehicle/drone/V7RCDroneImu.h`
- `src/vehicle/drone/V7RCMpu6050Imu.h`
- `src/vehicle/drone/V7RCMpu6050Imu.cpp`
- `src/vehicle/drone/V7RCDroneRuntime.h`
- `src/vehicle/drone/V7RCDroneRuntime.cpp`

目前 drone runtime 已具備：

- 4-motor X quad flight mixer
- arm / disarm / unlock flow
- MPU6050-based attitude estimate
- stabilization enable / disable switch

後續可再把 IMU 實作擴充為其他感測器。

## DroneV2

為了整理無人機演算法，但不影響目前已存在的 `V7RCDroneRuntime`，repo 內已另外加入 `DroneV2` 路線：

- 對外入口：`src/V7RCDroneV2-esp32.h`
- runtime：`src/vehicle/drone/v2/V7RCDroneV2Runtime.*`
- 設計文件：`vehicle/drone/DRONE_V2_DESIGN.md`
- simulate protocol：`vehicle/drone/DRONE_V2_SIM_PROTOCOL.md`
- demo：`examples/ESP32_C3_Drone_V2_Runtime_Demo`
- simulate demo：`examples/ESP32_C3_Drone_V2_Simulate`
- BLE simulate demo：`examples/ESP32_C3_Drone_V2_BLE_Control_Simulate`

`DroneV2` 採用：

- estimator / controller / runtime 分層
- angle loop + rate loop 雙迴路
- legacy IMU adapter，先相容現有 `V7RCDroneImu`
