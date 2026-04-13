# DroneV2 Design

`DroneV2` 是為了在不動既有 `V7RCDroneRuntime` 的前提下，把無人機演算法整理成可擴充的雙迴路架構。

## 參考來源

- `reference/drone_flight_controller_package/drone_flight_controller/`
- 既有 `src/vehicle/drone/V7RCDroneRuntime.*`

## 為什麼要做 V2

目前 `V7RCDroneRuntime` 已經能完成：

- unlock / disarm safety flow
- level trim calibration
- 四軸 X mixer
- 基礎姿態補償

但演算法仍然是單體式流程，幾個地方之後會很難擴：

- angle correction、rate damping、motor mixing 都擠在同一個 runtime
- IMU 抽象目前只保留 `roll/pitch/yawRate`
- 想做 PID tuning、debug CSV、不同模式切換時，沒有乾淨的模組邊界

## V2 分層

`DroneV2` 目前拆成三層：

1. `Estimator`
   - 提供 `roll/pitch/rollRate/pitchRate/yawRate`
   - 先用 `V7RCDroneV2LegacyImuAdapter` 包住現有 `V7RCDroneImu`
   - 舊 IMU 沒有 `rollRate/pitchRate` 時，先由角度差分估計

2. `Controller`
   - `V7RCDroneV2AngleController`
   - `V7RCDroneV2RateController`
   - `V7RCDroneV2Pid`
   - `V7RCDroneV2MotorMixer`

3. `Runtime`
   - 保留 V1 已驗證過的 arm / unlock / calibration / ready cue 流程
   - 負責把 estimator、controller、motor output 串起來

## 目前 V2 的定位

這一版是可以開始整合與調參的第一版骨架，不是最終 flight stack。

目前已經具備：

- dual-loop 架構
- legacy IMU 相容轉接
- mixer saturation / desaturation
- runtime debug data
- 獨立 `DroneV2` facade 與 demo example

## 接下來建議的開發順序

1. 先用 `ESP32_C3_Drone_V2_Runtime_Demo` 驗證方向與正負號
2. 針對 `ICM20948` 或下一顆主力 IMU，做原生 `V7RCDroneV2Estimator`
3. 把 debug data 接成 serial CSV 輸出
4. 分別調整 angle loop 與 rate loop 參數
5. 視需要加入 altitude / heading hold，但不要先塞進 runtime 主流程

## 設計原則

- 不修改既有 `V7RCDroneRuntime` 對外行為
- 演算法模組化，方便替換與單獨調整
- 先相容現有 IMU，再逐步升級感測輸入品質
- 安全流程優先沿用既有邏輯，再慢慢優化控制器
