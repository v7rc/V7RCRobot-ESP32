# Drone_V2_BLE_Control_Simulate

這個 example 提供：

- `Serial` 輸入模擬的 `ICM20948` JSON
- `V7RC App` 經由 BLE 傳入控制命令
- 板端使用與 `V7RCIcm20948Imu.cpp` 相同的姿態估算流程
- `DroneV2` controller 在板端做計算
- 每 `50ms` 從 `Serial` 輸出 JSON 結果

這個範例：

- 不使用真實 IMU
- 不實際驅動馬達
- 不自己模擬 V7RC command

## Serial Input

每行一筆 JSON：

```json
{"accelXg":0.0,"accelYg":0.0,"accelZg":1.0,"gyroXDegPerSec":0.0,"gyroYDegPerSec":0.0,"gyroZDegPerSec":0.0,"valid":true}
```

## Output

輸出仍維持原本 simulate 系列的結構，但會額外帶出：

- `icm20948`
  - 原始量測值
  - 濾波後 accel / gyro
- `imu`
  - 由 ICM20948 計算流程得到的姿態

`control.command` 仍然會輸出 canonical `SRT....#` 字串。
