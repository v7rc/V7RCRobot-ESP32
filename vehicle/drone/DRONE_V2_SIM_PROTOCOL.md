# DroneV2 Simulation Protocol

這份文件定義 `DroneV2` 模擬相關範例使用的 `Serial JSON` 輸入與輸出格式。

適用範例：

- `examples/ESP32_C3_Drone_V2_Simulate`
- `examples/ESP32_C3_Drone_V2_BLE_Control_Simulate`

## 目的

這份 protocol 用來讓外部虛擬世界或上位機：

- 傳送模擬的 `ICM20948` 原始量測到 ESP32
- 接收 ESP32 端控制器算出的 telemetry 與四軸輸出

這些範例都：

- 不使用真實 IMU
- 不實際驅動馬達
- 只把控制計算結果經由 `Serial` 輸出

## Transport

- 傳輸介面：`Serial`
- 編碼：UTF-8 純文字
- framing：一行一筆 JSON
- 行結尾：`\n`

也就是說：

- 輸入時，每送一行 JSON，ESP32 解析一筆
- 輸出時，ESP32 每次輸出一行完整 JSON

## Input JSON

### Schema

每行輸入格式如下：

```json
{
  "accelXg": 0.0,
  "accelYg": 0.0,
  "accelZg": 1.0,
  "gyroXDegPerSec": 0.0,
  "gyroYDegPerSec": 0.0,
  "gyroZDegPerSec": 0.0,
  "valid": true
}
```

### Fields

- `accelXg`
  - 型別：`number`
  - 單位：g
  - 說明：ICM20948 X 軸加速度
- `accelYg`
  - 型別：`number`
  - 單位：g
  - 說明：ICM20948 Y 軸加速度
- `accelZg`
  - 型別：`number`
  - 單位：g
  - 說明：ICM20948 Z 軸加速度
- `gyroXDegPerSec`
  - 型別：`number`
  - 單位：degrees/sec
  - 說明：ICM20948 X 軸角速度
- `gyroYDegPerSec`
  - 型別：`number`
  - 單位：degrees/sec
  - 說明：ICM20948 Y 軸角速度
- `gyroZDegPerSec`
  - 型別：`number`
  - 單位：degrees/sec
  - 說明：ICM20948 Z 軸角速度
- `valid`
  - 型別：`boolean`
  - 說明：這筆 IMU 是否有效

### Notes

- 欄位名稱必須與上面一致。
- 目前 parser 只抓它需要的 key，不依賴欄位順序。
- 若某欄位缺少，ESP32 會保留上一個值。
- `valid=false` 時，控制器輸出會進入保守狀態，馬達輸出歸零。
- ESP32 會使用和 `src/vehicle/drone/V7RCIcm20948Imu.cpp` 相同的流程：
  - accel / gyro 低通濾波
  - complementary filter
  - 姿態推算

## Output JSON

### Schema

ESP32 每 `50ms` 輸出一行 JSON，格式如下：

```json
{
  "ts": 2450,
  "phase": "ble_control",
  "icm20948": {
    "accelXg": 0.0,
    "accelYg": 0.0,
    "accelZg": 1.0,
    "gyroXDegPerSec": 0.0,
    "gyroYDegPerSec": 0.0,
    "gyroZDegPerSec": 1.5,
    "filteredAccelXg": 0.0,
    "filteredAccelYg": 0.0,
    "filteredAccelZg": 1.0,
    "filteredGyroXDegPerSec": 0.0,
    "filteredGyroYDegPerSec": 0.0,
    "filteredGyroZDegPerSec": 1.5,
    "valid": true
  },
  "imu": {
    "rollDeg": 0.1,
    "pitchDeg": -0.2,
    "rollRateDegPerSec": 0.0,
    "pitchRateDegPerSec": 0.0,
    "yawRateDegPerSec": 1.5,
    "valid": true
  },
  "control": {
    "command": "SRT1500136001501500#",
    "armed": true,
    "airborne": true,
    "stabilize": true,
    "THR": 0.36,
    "YAW": 0.0,
    "YRT": 1.5,
    "PIT": -0.2,
    "ROL": 0.1
  },
  "motors": {
    "frontLeft": 0.36,
    "frontRight": 0.36,
    "rearLeft": 0.36,
    "rearRight": 0.36
  }
}
```

### Top-level Fields

- `ts`
  - 型別：`integer`
  - 單位：milliseconds
  - 說明：ESP32 `millis()`
- `phase`
  - 型別：`string`
  - 說明：目前模擬或控制狀態

## `imu` Object

這一段反映目前 ESP32 經由模擬 `ICM20948` 計算出來的姿態狀態。

- `rollDeg`
- `pitchDeg`
- `rollRateDegPerSec`
- `pitchRateDegPerSec`
- `yawRateDegPerSec`
- `valid`

## `icm20948` Object

這一段反映目前 ESP32 內部持有的模擬 `ICM20948` 狀態：

- `accelXg`
- `accelYg`
- `accelZg`
- `gyroXDegPerSec`
- `gyroYDegPerSec`
- `gyroZDegPerSec`
- `filteredAccelXg`
- `filteredAccelYg`
- `filteredAccelZg`
- `filteredGyroXDegPerSec`
- `filteredGyroYDegPerSec`
- `filteredGyroZDegPerSec`
- `valid`

## `control` Object

- `command`
  - 型別：`string`
  - 說明：canonical V7RC command 字串，格式固定為 `SRT....#`
- `armed`
  - 型別：`boolean`
  - 說明：控制器是否已進入可輸出狀態
- `airborne`
  - 型別：`boolean`
  - 說明：是否達到飛行推力門檻
- `stabilize`
  - 型別：`boolean`
  - 說明：是否啟用 angle stabilize 模式
- `THR`
  - 型別：`number`
  - 範圍：`0..1`
  - 說明：throttle normalized
- `YAW`
  - 型別：`number`
  - 範圍：`-1..1`
  - 說明：yaw control input
- `YRT`
  - 型別：`number`
  - 單位：degrees/sec
  - 說明：目前 yaw rate measurement
- `PIT`
  - 型別：`number`
  - 單位：degrees
  - 說明：目前修正後的 pitch angle
- `ROL`
  - 型別：`number`
  - 單位：degrees
  - 說明：目前修正後的 roll angle

### BLE simulate extra fields

在 `ESP32_C3_Drone_V2_BLE_Control_Simulate` 中，`control` 會多出：

- `bleConnected`
- `unlocked`

## `motors` Object

四個馬達輸出都使用 normalized 浮點數：

- `frontLeft`
- `frontRight`
- `rearLeft`
- `rearRight`

範圍：

- `0.0 .. 1.0`

意義：

- 這是控制器計算後的最終輸出
- 只作為模擬結果
- 不代表真的已寫到 PWM 或 DC motor driver

## Canonical V7RC Command

輸出中的 `control.command` 一律使用：

```text
SRTYYYYTTTTPPPPRRRR#
```

欄位定義：

- `YYYY`
  - yaw
  - 範圍：`1000..2000`
  - 中立：`1500`
- `TTTT`
  - throttle
  - 範圍：`1000..2000`
- `PPPP`
  - pitch
  - 範圍：`1000..2000`
  - 中立：`1500`
- `RRRR`
  - roll
  - 範圍：`1000..2000`
  - 中立：`1500`

normalized 對應：

- yaw / pitch / roll
  - `-1 -> 1000`
  - `0 -> 1500`
  - `+1 -> 2000`
- throttle
  - `0 -> 1000`
  - `1 -> 2000`

## Mode Differences

### `ESP32_C3_Drone_V2_Simulate`

- 控制命令由 example 內部自動產生
- `phase` 會是：
  - `idle`
  - `takeoff`
  - `yaw_right`
  - `roll_left`
  - `roll_right`
  - `pitch_forward`
  - `pitch_backward`

### `ESP32_C3_Drone_V2_BLE_Control_Simulate`

- 控制命令來自 `V7RC App` 的 BLE 封包
- `phase` 可能是：
  - `ble_control`
  - `unlock`
  - `armed`
  - `signal_timeout`
  - `disconnected`

## Timing

- 控制與輸出週期：`50ms`
- 也就是大約 `20Hz`

## Recommended Host Behavior

- 上位機每次送 IMU 時，都送完整欄位，不要只送局部更新
- 收到輸出 JSON 後，以 `motors.*` 當作控制器輸出結果
- 若要做閉環模擬，應用 `control.command` 與 `motors.*` 更新你的虛擬世界，再把新的 IMU 狀態回送給 ESP32
