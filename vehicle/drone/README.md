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
