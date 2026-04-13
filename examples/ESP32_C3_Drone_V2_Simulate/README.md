# Drone_V2_Simulate

這個 example 不使用真實 IMU 與馬達硬體。

它的用途是：

- 從 `Serial` 接收模擬的 `ICM20948` JSON
- 在板端套用與 `V7RCIcm20948Imu` 相同的姿態估算流程
- 在板端自動產生一段模擬飛行命令
- 使用 `DroneV2` controller 模組計算四軸輸出
- 每 `50ms` 輸出一行 JSON 結果

## Serial Input Protocol

每行一筆 JSON，結尾用換行：

```json
{"accelXg":0.0,"accelYg":0.0,"accelZg":1.0,"gyroXDegPerSec":0.0,"gyroYDegPerSec":0.0,"gyroZDegPerSec":0.0,"valid":true}
```

## Notes

- `accel*` 與 `gyro*` 欄位代表模擬的 `ICM20948` 原始量測
- example 內會用和 `V7RCIcm20948Imu.cpp` 相同的低通濾波與 complementary filter 算出姿態
- 輸出 JSON 內會同時包含：
  - `icm20948`: 原始與濾波後量測
  - `imu`: 計算出的姿態與角速度
