# Quick Start (Education) — 10 Minutes

Languages: `繁中` | [English](QUICK_START_EDU.en.md) | [简中](QUICK_START_EDU.zh-CN.md) | [日本語](QUICK_START_EDU.ja.md)

目標：用 **ESP32-C3** 在 10 分鐘內把車「動起來」，並能用 **V7RC App（V7RCDOM V2.0）** 連線控制。

---
## 0) 你需要準備

- ESP32-C3 開發板（例如 ESP32-C3 SuperMini / DevKit）
- 直流馬達驅動板（二選一）
  - **TB6612FNG**（推薦，常見且穩）
  - **DRV8833**（也常見）
- 2 顆 DC 馬達（差速車）或 4 顆（麥克納姆）
- 最多6PWM訊號（如果4顆DC馬達，則只能外接2個Servo)
- 外接電源（馬達電源不要用 USB 5V 硬撐，建議 1S 鋰電或 3.7V 視馬達而定）
- Arduino IDE + ESP32 Core

---
## 1) 安裝套件（Arduino IDE）

Arduino IDE → Tools → Manage Libraries：
- 安裝 `ESP32Servo`
- 安裝 `NimBLE-Arduino`（by h2zero）
- 安裝 `Adafruit NeoPixel`

---
## 2) 下載 / 打開範例

- 麥克納姆：`examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
- 差速車：`examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`
- 乾淨版差速車：`examples/ESP32_C3_Differential_Drive_Clean/ESP32_C3_Differential_Drive_Clean.ino`
- Runtime Demo 差速車：`examples/ESP32_C3_Differential_Runtime_Demo/ESP32_C3_Differential_Runtime_Demo.ino`
- Runtime Demo 車體：`examples/ESP32_C3_Car_Runtime_Demo/ESP32_C3_Car_Runtime_Demo.ino`
- Runtime Demo 麥克納姆：`examples/ESP32_C3_Mecanum_Runtime_Demo/ESP32_C3_Mecanum_Runtime_Demo.ino`
- V7RC BLE 麥克納姆控制：`examples/ESP32_C3_Mecanum_V7RC_BLE_Control/ESP32_C3_Mecanum_V7RC_BLE_Control.ino`
- V7RC BLE 坦克控制：`examples/ESP32_C3_Tank_V7RC_BLE_Control/ESP32_C3_Tank_V7RC_BLE_Control.ino`
- 乾淨版麥克納姆：`examples/ESP32_C3_Mini_Mecanum_Clean/ESP32_C3_Mini_Mecanum_Clean.ino`
- Runtime Demo 坦克：`examples/ESP32_C3_Tank_Runtime_Demo/ESP32_C3_Tank_Runtime_Demo.ino`
- Drone Runtime Demo：`examples/ESP32_C3_Drone_Runtime_Demo/ESP32_C3_Drone_Runtime_Demo.ino`
- V7RC BLE Drone 控制：`examples/ESP32_C3_Drone_V7RC_BLE_Control/ESP32_C3_Drone_V7RC_BLE_Control.ino`
- Dog Runtime Demo：`examples/ESP32_C3_Dog_Runtime_Demo/ESP32_C3_Dog_Runtime_Demo.ino`
- Dog Runtime Demo V2：`examples/ESP32_C3_Dog_Runtime_Demo_V2/ESP32_C3_Dog_Runtime_Demo_V2.ino`
- OTTO Runtime Demo：`examples/ESP32_C3_OTTO_Runtime_Demo/ESP32_C3_OTTO_Runtime_Demo.ino`
- RobotARM Runtime Demo：`examples/ESP32_C3_RobotARM_Runtime_Demo/ESP32_C3_RobotARM_Runtime_Demo.ino`

舊版範例目前會透過：

```cpp
#include <V7RCRobot-esp32.h>
```

載入這個 library 的 ESP32-only 公共入口。

乾淨版 car 範例則可直接透過：

```cpp
#include <V7RCCar-esp32.h>
```

使用 car facade。

注意：

- `*_Runtime_Demo` 是本地 runtime / mixer 驗證用，不會等 V7RC App 的 BLE 指令
- 如果你要真的用 V7RC App 開車，請優先用 `ESP32_C3_Mecanum_V7RC_BLE_Control` 或 `ESP32_C3_Mini_Mecanum_Clean`

如果你要直接試四足 runtime，則可透過：

```cpp
#include <V7RCQuadruped-esp32.h>
```

使用 quadruped runtime。

如果你要直接試 OTTO / arm runtime，則可分別透過：

```cpp
#include <V7RCOtto-esp32.h>
#include <V7RCRobotArm-esp32.h>
```

> **WS2812 LED**
> 這個範例啟用了燈條（`.ws2812Enable = true`），使用 GPIO8、8 顆 LED；
> 開機時紅色閃爍，每秒亮/暗；BLE 連線成功後會自動變為恆亮綠色。
> 若不需要 LED 可將 `.ws2812Enable` 設為 `false`，那樣 pin/count 將被忽略；
> 啟用後可透過 `.ws2812Pin`、`.ws2812Count` 變更，如設為零將換成預設值（8顆、腳位8）。
>
> ### LE* 控制協議
> 若要從 V7RC App 傳送單顆燈控制，請使用文字命令：
> `LE?rrrrggggbbbbx#`，其中 `?` 是 `D`（0–3）、`2`（4–7）、`3`（8–11）……
> 各 LED 使用 4 個字元：紅/綠/藍/閃爍，前 3 個以 `0`–`F` 表示 16 級 (轉成 0–255，
> 乘 17)，第四個以 `0`–`A` 表示每秒亮的百毫秒份量 (0=關, A=恆亮)。
> 範例：`LED0123A4567B89C#` 設定前 4 顆。自訂後將取代預設的連線動畫。

---
## 3) 接線圖（文字版）

### A) 供電（重要）
- **ESP32-C3**：USB 供電（僅供控制板）
- **馬達驅動板 VM / VIN**：外接電池（供馬達）
- **GND 必須共地**：ESP32 GND ↔ 驅動板 GND ↔ 電池負極

### B) TB6612FNG（差速車最簡接法）
- 你需要兩組 H-Bridge：A 給左輪、B 給右輪
- `STBY` 必須拉高（接 3.3V 或由 GPIO 控制）

---
## 4) 接線 Pin 表（可直接用範例預設）

> ⚠️ 不同 ESP32-C3 板子腳位標示可能不同，請以你的板子實際 GPIO 為準。
> 若你已經有固定硬體腳位，直接改 `.ino` 的 pin 定義即可。

### TB6612FNG（Differential Drive / 2 motors）

| Function | TB6612FNG Pin | ESP32-C3 GPIO (default) |
|---------:|---------------|--------------------------|
| Left PWM | PWMA          | GPIO4                    |
| Left DIR | AIN1          | GPIO5                    |
| Left DIR | AIN2          | GPIO6                    |
| Right PWM| PWMB          | GPIO7                    |
| Right DIR| BIN1          | GPIO8                    |
| Right DIR| BIN2          | GPIO9                    |
| Enable   | STBY          | GPIO10 (or 3.3V)         |
| GND      | GND           | GND (common ground)      |
| Motor V  | VM            | Battery +                |

### 連線成功但車不動？
- 先確認 `STBY` 是否為 High
- 確認 VM 有外接電源
- 確認共地
- 如果方向反了：把該馬達的 `dirInvert=true` 或交換 AIN1/AIN2（右輪同理）

---
## 5) V7RC App（V7RCDOM V2.0）設定提示

- 建議先用「差速車」布局測試
  - ch0 = Throttle（前後）
  - ch1 = Steering（左右）
- 麥克納姆布局：
  - ch0 = Vx（前後）
  - ch1 = Vy（左右平移）
  - ch2 = Omega（旋轉）
- 如果你遇到「X/Y 對調」：交換 ch0/ch1 的角色即可

---
## 6) 最小驗收（10 分鐘目標）

✅ Arduino IDE 編譯/上傳成功  
✅ 手機能掃描到 BLE 裝置（例如 `V7RC-ROBOT-01`）  
✅ App 連線後推搖桿，車能前進/後退/轉向  

做到這裡，你就可以開始再往「麥克納姆」「伺服」「平滑」等功能擴充。
