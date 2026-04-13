# Drone_V2_BLE_Control

這個範例使用：

- 真實 `ICM20948`
- `V7RC App` 經由 BLE 控制
- `DroneV2` dual-loop controller
- 真實四馬達輸出

這個範例不使用 `Serial` 做 telemetry 輸出，因為控制馬達時不另外佔用 `Serial Port`。

## Hardware

### IMU

- 感測器：`ICM20948`
- I2C SDA：`GPIO5`
- I2C SCL：`GPIO6`

### Motor Output

目前設定為四路 DC motor 輸出：

- front-left: `dir=GPIO20`, `pwm=GPIO21`
- front-right: `dir=GPIO0`, `pwm=GPIO10`
- rear-left: `dir=GPIO2`, `pwm=GPIO1`
- rear-right: `dir=GPIO3`, `pwm=GPIO4`

方向設定：

- FL: `normal`
- FR: `invert`
- RL: `invert`
- RR: `normal`

## BLE

裝置名稱格式：

```text
V7RC-DRV2-XX
```

其中 `XX` 來自 BLE MAC 最後一個 byte。

## V7RC App Channel Mapping

- `ch0`: yaw
- `ch1`: throttle
- `ch2`: pitch
- `ch3`: roll
- `ch4`: stabilize switch

控制量對應：

- yaw / pitch / roll
  - `1000..2000 -> -1..1`
- throttle
  - `1000..2000 -> 0..1`

stabilize switch：

- `ch4 <= 1300`: stabilize OFF
- `ch4 >= 1700`: stabilize ON

## Unlock Gesture

沿用既有 drone BLE control 的解鎖手勢：

```text
ch0=1000, ch1=1000, ch2=1000, ch3=2000
```

解鎖後：

1. runtime 進入 unlock hold
2. 進行 gyro bias 校正與 level trim 校正
3. ready cue
4. 進入 armed 狀態

## ICM20948 Axis Transform

目前範例內使用：

```cpp
icm20948Imu.setAxisTransform(
  V7RC_ICM20948_AXIS_Y, -1,
  V7RC_ICM20948_AXIS_X, -1,
  V7RC_ICM20948_AXIS_Z,  1
);
```

也就是：

- logical X = sensor Y, sign `-1`
- logical Y = sensor X, sign `-1`
- logical Z = sensor Z, sign `+1`

如果你的安裝方向不同，優先調整這一段。

## Safety Behavior

- BLE 斷線時：
  - 清空控制量
  - 重設 unlock 狀態
  - `runtime.disarm()`
- BLE 訊號 timeout 時：
  - 控制量歸零
  - 保留目前 stabilize 狀態

## Related Files

- 範例主程式：`examples/ESP32_C3_Drone_V2_BLE_Control/ESP32_C3_Drone_V2_BLE_Control.ino`
- DroneV2 runtime：`src/vehicle/drone/v2/V7RCDroneV2Runtime.*`
- ICM20948 estimator：`src/vehicle/drone/v2/V7RCDroneV2Icm20948Estimator.*`
