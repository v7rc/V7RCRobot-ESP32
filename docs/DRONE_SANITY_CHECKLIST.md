# Drone Motor / Prop / IMU Sanity Checklist

這份清單是給目前 `V7RCRobot-ESP32` 的 drone sample 用的，目標是用最短路徑排掉三種最常見問題：

- 一加 throttle 就自轉
- 油門一大就翻覆
- IMU 軸向或控制方向相反

建議排查順序不要跳步，先做機械與電機方向，再做 IMU，最後才調 gain。

## 1. 先不要裝槳

第一次驗證時，先拆掉所有槳。

原因：

- 可以先確認馬達方向與混控，不會一上電就危險
- 如果 mixer / IMU 方向反了，不會直接翻機

## 2. 確認馬達編號和程式一致

目前 drone runtime 的馬達邏輯順序是：

- `motor[0] = front-left`
- `motor[1] = front-right`
- `motor[2] = rear-left`
- `motor[3] = rear-right`

先確認你接線的物理位置和 `.ino` 內的 `motors[]` 一致。

如果位置錯了，先改 `.ino` 的腳位，不要先改 mixer。

## 3. 確認馬達旋向設定

目前範例預設是交錯旋向：

- `front-left = normal`
- `front-right = invert`
- `rear-left = invert`
- `rear-right = normal`

在 sample 裡對應為：

```cpp
V7RC_DCMotorConfig motors[] = {
  {20, 21, false},  // front-left
  {10, 0, true},    // front-right
  {2, 1, true},     // rear-left
  {3, 4, false},    // rear-right
};
```

如果一加 throttle 就明顯持續自轉，先檢查：

1. `dirInvert` 是否真的和你的槳旋向需求一致
2. 物理接線是不是把某顆馬達插錯位置

## 4. 確認槳方向和馬達旋向配對

即使馬達旋向正確，只要槳裝反，也會推力錯誤。

確認方式：

- 每顆槳的迎風面方向要正確
- 順時針槳裝在順時針馬達
- 逆時針槳裝在逆時針馬達

如果你不確定，先只裝一顆馬達測風向，再比對其他三顆。

## 5. 先做純油門測試

在沒有裝槳或低風險條件下，先只測 `Throttle`：

- `Yaw = 0`
- `Roll = 0`
- `Pitch = 0`
- 慢慢增加 throttle

期望結果：

- 四顆馬達一起升速
- 聲音接近
- 沒有某兩顆特別慢或特別快

如果這一步就會自轉傾向，先不要測 IMU，先回頭檢查：

- 馬達位置
- `dirInvert`
- 槳方向

## 6. 先關掉 stabilization 測試

第一次測通道和馬達時，建議先關：

- `stabilizationEnabled = false`

原因：

- 可以把問題切成「基本 mixer / wiring」和「IMU/姿態修正」兩段
- 不然 IMU 方向錯誤時，控制器會主動把錯誤放大

## 7. 確認 IMU 本身讀值正常

先用單獨驗證 sample：

- `ADXL345`: [ESP32_C3_ADXL345_Integration_Check](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/examples/ESP32_C3_ADXL345_Integration_Check/ESP32_C3_ADXL345_Integration_Check.ino)
- `ICM20948`: [ESP32_C3_ICM20948_Integration_Check](/Users/louischuang/Documents/Arduino/libraries/V7RCRobot-ESP32/examples/ESP32_C3_ICM20948_Integration_Check/ESP32_C3_ICM20948_Integration_Check.ino)

期望結果：

- 平放時 `roll ≈ 0`, `pitch ≈ 0`
- 向右傾時 `roll` 方向一致
- 機頭朝下時 `pitch` 方向一致
- 旋轉機身時 `yawRate` 方向一致

如果方向不對：

- `ADXL345` 先調 `setAxisTransform(...)`
- `ICM20948` 若方向不對，先確認模組安裝方向，再考慮加同類型軸向映射

## 8. 開啟 stabilization 後做手持反應測試

這一步仍然建議先不要裝槳，或至少固定機身。

開啟 stabilization 後，手持機身輕微傾斜：

- 機身右傾時，控制器應該嘗試把左/右推力差調成「推回水平」
- 機頭下壓時，控制器應該嘗試把前/後推力差調成「拉回水平」

如果你一傾斜，馬達反應讓它更想翻：

- `roll` 或 `pitch` 的控制方向反了
- 先不要調 gain，先修正 IMU 軸向或控制符號

## 9. Yaw 單獨測試

在 `Throttle` 很低、且機身安全固定的前提下，單獨打 `Yaw`：

期望結果：

- 對角的兩組馬達會有相反方向的微調
- 不應該出現四顆同向一起暴衝

如果一打 yaw 就很怪：

- 檢查交錯旋向是否正確
- 檢查 `yawGain` 是否過大

## 10. 最後才調 gain

只有在以下都正確後，才開始調：

- 馬達位置正確
- `dirInvert` 正確
- 槳方向正確
- IMU 軸向正確
- stabilization 方向正確

建議順序：

1. 先把 `rollKp` / `pitchKp` 降低
2. 確認不會立即翻
3. 再逐步提高
4. 最後再調 `yawGain`

## 快速判斷表

### 症狀：一加 throttle 就持續自轉

優先檢查：

1. 馬達位置是否對應 `FL/FR/RL/RR`
2. `dirInvert` 是否正確
3. 槳是否裝反

### 症狀：油門一大就往單一方向翻

優先檢查：

1. 某顆馬達位置錯
2. 某顆槳方向錯
3. `roll/pitch` 控制方向反了

### 症狀：開 stabilization 之後更容易翻

優先檢查：

1. IMU 軸向錯
2. IMU 安裝方向和程式假設不一致
3. `rollKp/pitchKp` 過大

## 建議測試順序

1. 不裝槳，測馬達位置與旋向
2. 不裝槳，測 throttle / yaw mixer
3. 單獨跑 IMU integration check
4. 固定機身，開 stabilization 看補償方向
5. 低油門、低 gain、裝槳短測
6. 最後才做自由飛行
