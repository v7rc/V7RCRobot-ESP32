# TODO

這份文件用來追蹤 `V7RCRobot-ESP32` 的重構順序、每個步驟的驗收條件，以及完成後必做的測試工作。

原則：

- 依照這份清單由上而下執行
- 每完成一個步驟，就更新本文件的狀態與結果
- 每個步驟都要包含至少一次 compile 檢查
- 如果環境缺少工具，必須在該步驟的測試紀錄中註明阻塞原因

狀態標記：

- `[x]` 已完成
- `[ ]` 尚未開始
- `[-]` 進行中
- `[!]` 阻塞

## Phase 0: Repo Bootstrap

- [x] 建立多載具 repo 基本目錄骨架
  - 驗收：
    - `src/transport`
    - `src/protocol`
    - `src/core`
    - `src/io`
    - `src/legacy`
    - `vehicle/*`
    - `docs/architecture`
  - 測試：
    - 結構檢查：確認目錄已建立
    - compile：本步驟不涉及編譯，無需 compile
  - 結果：
    - 已完成

- [x] 將 legacy driver 與 examples 移到新位置
  - 驗收：
    - `src/legacy/V7RCServoDriver.h`
    - `src/legacy/V7RCServoDriver.cpp`
    - `examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`
    - `examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
  - 測試：
    - 結構檢查：確認檔案已移動
    - compile：尚未執行，待安裝 Arduino CLI 或等效工具後補跑
  - 結果：
    - 已完成

- [x] 建立 ESP32-only 公共入口與 metadata 初版
  - 驗收：
    - `src/V7RCRobot-esp32.h`
    - `library.properties` 使用 `includes=V7RCRobot-esp32.h`
    - examples 改為 `#include <V7RCRobot-esp32.h>`
    - public header 有 `ARDUINO_ARCH_ESP32` 保護
  - 測試：
    - 靜態檢查：確認 README、examples、`library.properties` 命名一致
    - compile：尚未執行，待安裝 Arduino CLI 或等效工具後補跑
  - 結果：
    - 已完成

## Phase 1: Release Readiness Baseline

- [x] 補齊 Arduino Library Manager 上架前基線
  - 工作：
    - 檢查 `keywords.txt`
    - 規劃 `.gitignore`
    - 檢查 `README.md`
    - 檢查 `LICENSE`
    - 檢查 `library.properties` 欄位完整性與文案
  - 驗收：
    - metadata、examples、public include 入口一致
    - 無明顯不適合上架的暫存檔被納入 repo
  - 測試：
    - 執行 `arduino-lint` 或等效檢查
    - 若可用，執行至少一次 library 結構檢查
    - compile：確認兩個 examples 都能進入編譯流程
  - 結果：
    - 已新增 `.gitignore`
    - 已更新 `keywords.txt`
    - 已補 README / Quick Start 的依賴與公共入口說明
    - `arduino-lint --project-type library --library-manager submit --compliance specification` 已通過

- [x] 建立 compile 測試基線
  - 工作：
    - 決定使用 `arduino-cli` 的 board fqbn
    - 建立最小 compile 測試流程
    - 記錄依賴安裝方式
  - 驗收：
    - 可重複執行 example compile
    - 測試命令可寫入 README 或後續 CI
  - 測試：
    - compile `examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`
    - compile `examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
  - 結果：
    - compile 基線板型使用 `esp32:esp32:esp32c3`
    - 已新增 `scripts/compile-examples.sh`
    - 已新增 `scripts/lint-library.sh`
    - 兩個 examples 均編譯成功

## Phase 2: Transport Extraction

- [x] 從 legacy driver 拆出 BLE transport layer
  - 工作：
    - 定義 transport 介面
    - 把 BLE 初始化、連線狀態、收包流程移出 legacy monolith
    - 保留舊功能可對照
  - 驗收：
    - `src/transport` 下有明確的 BLE transport 實作
    - legacy driver 不再承擔完整 transport 細節
  - 測試：
    - compile 全部 examples
    - 若有條件，增加最小 transport smoke test
  - 結果：
    - 已新增 `src/transport/V7RCTransport.h`
    - 已新增 `src/transport/V7RCBleTransportEsp32.h`
    - 已新增 `src/transport/V7RCBleTransportEsp32.cpp`
    - legacy driver 已改用 transport callback 接收 BLE bytes 與連線狀態
    - `./scripts/compile-examples.sh` 已通過
    - `./scripts/lint-library.sh` 已通過

## Phase 3: Protocol Extraction

- [x] 從 legacy driver 拆出 protocol decode layer
  - 工作：
    - 定義 frame / command model
    - 拆出 `HEX`、`DEG`、`SRV/SRT`、`SS8`、`LED` 解析
  - 驗收：
    - protocol layer 不依賴 BLE 細節
    - transport 與 vehicle 不直接處理 raw bytes
  - 測試：
    - compile 全部 examples
    - 若可行，新增 parser 單元測試或最小解碼測試
  - 結果：
    - 已新增 `src/protocol/V7RCProtocol.h`
    - 已新增 `src/protocol/V7RCProtocol.cpp`
    - protocol decode 已從 legacy driver 抽離
    - legacy driver 目前透過 `applyLegacyFrameDefaults()` 保留舊行為
    - `./scripts/compile-examples.sh` 已通過
    - `./scripts/lint-library.sh` 已通過

## Phase 4: Core And IO

- [-] 建立 core runtime 與 IO abstraction
  - 工作：
    - channel state
    - timeout / safety state
    - GPIO / PWM / Servo / ESC abstraction
    - LED / status abstraction
  - 驗收：
    - vehicle 層可以透過 core / io 存取共用能力
    - 不再把所有邏輯塞回單一 driver
  - 測試：
    - compile 全部 examples
    - 若可行，新增最小 runtime state 測試
  - 目前進度：
    - 已新增 `src/core/V7RCRuntimeState.h`
    - 已新增 `src/core/V7RCRuntimeState.cpp`
    - 已新增 `src/io/V7RCIOInterfaces.h`
    - 已新增 `src/io/V7RCEsp32Outputs.h`
    - 已新增 `src/io/V7RCEsp32Outputs.cpp`
    - 已補上 ESP32 的 servo / DC motor / WS2812 具體 IO 實作
    - legacy driver 已改用 `V7RCRuntimeState` 管理 frame timestamp 與 channel values
    - `./scripts/compile-examples.sh` 已通過
    - `./scripts/lint-library.sh` 已通過
    - 尚未把 legacy runtime / output 大量切到這些 abstraction

## Phase 5: Vehicle Module Migration

- [-] 整理 car module 第一版
  - 工作：
    - 將差速車與 mecanum 的控制語意逐步移往 `vehicle/car`
    - 重新確認 channel mapping 與範例定位
  - 驗收：
    - `vehicle/car` 有可辨識的模組骨架
    - examples 不再完全依賴 legacy monolith
  - 測試：
    - compile car examples
    - 若可行，補充一個較乾淨的新式 example
  - 目前進度：
    - 已新增 `vehicle/car/V7RCCarTypes.h`
    - 已新增 `vehicle/car/V7RCCarControl.h`
    - 已新增 `vehicle/car/V7RCCarControl.cpp`
    - 已新增 Arduino 可編譯版本：
      - `src/vehicle/car/V7RCCarTypes.h`
      - `src/vehicle/car/V7RCCarControl.h`
      - `src/vehicle/car/V7RCCarControl.cpp`
    - 已固定 differential / mecanum 的 mixer API 方向
    - legacy driver 的 differential / mecanum mixer 已改接 `V7RCCarControl`
    - legacy driver 的 drive control state 已改用 `V7RCCarControlState`
    - drive channel normalization / input axis 套用已開始由 `V7RCCarControl` 處理
    - 已新增 car-specific 公共入口 `src/V7RCCar-esp32.h`
    - 已新增 car facade：
      - `src/vehicle/car/V7RCCarRobot.h`
      - `src/vehicle/car/V7RCCarRobot.cpp`
    - 已新增 car runtime config builder：
      - `src/vehicle/car/V7RCCarRuntimeConfig.h`
      - `src/vehicle/car/V7RCCarRuntimeConfig.cpp`
    - 已新增 standalone car runtime：
      - `src/vehicle/car/V7RCCarRuntime.h`
      - `src/vehicle/car/V7RCCarRuntime.cpp`
    - 已新增較乾淨的新式範例 `examples/ESP32_C3_Differential_Drive_Clean/ESP32_C3_Differential_Drive_Clean.ino`
    - 已新增較乾淨的新式範例 `examples/ESP32_C3_Mini_Mecanum_Clean/ESP32_C3_Mini_Mecanum_Clean.ino`
    - 已新增 standalone runtime demo：
      - `examples/ESP32_C3_Differential_Runtime_Demo/ESP32_C3_Differential_Runtime_Demo.ino`
    - clean example 已改用 `V7RCCarRobot`，不再直接手組完整 `V7RC_DriverConfig`
    - `V7RCCarRobot` 已改由 `V7RCCarRuntimeConfig` 組裝 legacy driver config
    - `V7RCCar-esp32.h` 已匯出 `V7RCCarRuntime`
    - `./scripts/compile-examples.sh` 已通過
    - `ESP32_C3_Differential_Drive_Clean` 已單獨 compile 通過
    - `ESP32_C3_Mini_Mecanum_Clean` 已單獨 compile 通過
    - `ESP32_C3_Differential_Runtime_Demo` 已單獨 compile 通過
    - `./scripts/lint-library.sh` 已通過
    - 尚未建立完全脫離 legacy driver 的 car runtime implementation

## Phase 6: Release Preparation

- [x] 整理第一版對外發布內容
  - 工作：
    - 更新 README
    - 檢查版本號
    - 確認 release note / tag 策略
    - 檢查是否需要保留或移除某些暫存文件
  - 驗收：
    - repo 可供外部使用者安裝與閱讀
    - 上架前需要的資料完整
  - 測試：
    - `arduino-lint`
    - compile 全部對外 examples
    - 重新檢查 `library.properties`
  - 結果：
    - 已更新 README 的入口與 example 分層說明
    - 已新增 `docs/RELEASE_CHECKLIST.md`
    - 已新增 `CHANGELOG.md`
    - `scripts/compile-examples.sh` 已擴充為涵蓋 4 個對外 examples
    - `arduino-lint` 已通過
    - 所有對外 examples 已 compile 通過

## Current Notes

- 本地已透過 `/tmp/arduino-cli-bin/arduino-cli` 驗證 compile
- 本地已透過 `/tmp/arduino-lint-bin/arduino-lint` 驗證 lint
- 後續每推進一個 phase，都必須回到這份文件更新狀態與測試結果

## Requested Demo Backlog

- [x] 增加 `Mecanum_Runtime_Demo`
  - 需求：
    - 完全使用 `V7RCCarRobot`
    - 走 `V7RCCarRuntime`
    - 目錄與檔名：
      - `examples/ESP32_C3_Mecanum_Runtime_Demo/ESP32_C3_Mecanum_Runtime_Demo.ino`
  - 測試：
    - compile `examples/ESP32_C3_Mecanum_Runtime_Demo`
    - `arduino-lint`
  - 結果：
    - 已新增 `ESP32_C3_Mecanum_Runtime_Demo`
    - `V7RCCarRobot` 已支援 runtime mode：
      - `beginDifferentialRuntime(...)`
      - `beginMecanumRuntime(...)`
      - `applyControl(...)`
      - `stop()`
    - `ESP32_C3_Mecanum_Runtime_Demo` 已 compile 通過
    - `ESP32_C3_Differential_Runtime_Demo` 已改用 `V7RCCarRobot` + `V7RCCarRuntime`
    - `arduino-lint` 已通過

- [x] 增加 `Car_Runtime_Demo`
  - 需求：
    - 完全使用 `V7RCCarRobot`
    - 走 `V7RCCarRuntime`
    - 目錄與檔名：
      - `examples/ESP32_C3_Car_Runtime_Demo/ESP32_C3_Car_Runtime_Demo.ino`
  - 測試：
    - compile `examples/ESP32_C3_Car_Runtime_Demo`
    - `arduino-lint`
  - 結果：
    - 已實作 `ESP32_C3_Car_Runtime_Demo`
    - 使用 `V7RCCarRobot` runtime mode
    - `ESP32_C3_Car_Runtime_Demo` 已 compile 通過
    - `arduino-lint` 已通過

- [x] 增加 `Tank_Runtime_Demo`
  - 需求：
    - 完全使用 `V7RCCarRobot`
    - 走 `V7RCCarRuntime`
    - 目錄與檔名：
      - `examples/ESP32_C3_Tank_Runtime_Demo/ESP32_C3_Tank_Runtime_Demo.ino`
  - 測試：
    - compile `examples/ESP32_C3_Tank_Runtime_Demo`
    - `arduino-lint`
  - 結果：
    - 已實作 `ESP32_C3_Tank_Runtime_Demo`
    - 使用 `V7RCCarRobot` runtime mode
    - `ESP32_C3_Tank_Runtime_Demo` 已 compile 通過
    - `arduino-lint` 已通過

- [x] 增加 `Drone_Runtime_Demo`
  - 需求：
    - 已升級為 drone 專屬 runtime 路徑
    - 目錄與檔名：
      - `examples/ESP32_C3_Drone_Runtime_Demo/ESP32_C3_Drone_Runtime_Demo.ino`
  - 測試：
    - compile `examples/ESP32_C3_Drone_Runtime_Demo`
    - `arduino-lint`
  - 結果：
    - 已實作 `ESP32_C3_Drone_Runtime_Demo`
    - 已新增 `V7RCDrone-esp32.h`
    - 已新增 `V7RCDroneRuntime`
    - 已新增 `V7RCDroneImu` abstraction
    - 已新增 `V7RCMpu6050Imu`
    - 已補上 4-motor X quad flight mixer
    - 已補上 arm / disarm / unlock flow
    - 已補上 MPU6050 自穩設定，可開啟或關閉
    - 已為未來更換其他陀螺儀預留 `V7RCDroneImu` 介面
    - `ESP32_C3_Drone_Runtime_Demo` 已 compile 通過
    - `arduino-lint` 已通過

- [x] 增加 `Dog_Runtime_Demo`
  - 需求：
    - 已調整為 quadruped 專屬 runtime 路徑
    - 目錄與檔名：
      - `examples/ESP32_C3_Dog_Runtime_Demo/ESP32_C3_Dog_Runtime_Demo.ino`
  - 測試：
    - compile `examples/ESP32_C3_Dog_Runtime_Demo`
    - `arduino-lint`
  - 結果：
    - 已新增 `V7RCQuadruped-esp32.h`
    - 已新增 `V7RCQuadrupedRuntime`
    - 已新增 `V7RCQuadrupedTypes`
    - 已補上 quadruped gait phase -> foot target -> IK -> servo output 的 runtime flow
    - `ESP32_C3_Dog_Runtime_Demo` 已改為 dog / quadruped 專屬 demo
    - 控制分層有參考實際 quadruped controller 常見做法，包含 gait scheduler、stance / swing 與 inverse kinematics
    - `ESP32_C3_Dog_Runtime_Demo` 已 compile 通過
    - `arduino-lint` 已通過

- [x] 增加第二個 `Dog_Runtime_Demo` 項目
  - 需求：
    - 已調整為 quadruped runtime 的第二個 demo 變體
    - 目錄與檔名：
      - `examples/ESP32_C3_Dog_Runtime_Demo_V2/ESP32_C3_Dog_Runtime_Demo_V2.ino`
  - 測試：
    - compile `examples/ESP32_C3_Dog_Runtime_Demo_V2`
    - `arduino-lint`
  - 結果：
    - 已實作 `ESP32_C3_Dog_Runtime_Demo_V2`
    - 使用同一套 `V7RCQuadrupedRuntime`，但採用不同幾何 / gait 參數與動作節奏
    - `ESP32_C3_Dog_Runtime_Demo_V2` 已 compile 通過
    - `arduino-lint` 已通過

- [x] 增加 `OTTO_Runtime_Demo`
  - 需求：
    - 已調整為 OTTO 專屬 runtime 路徑
    - 目錄與檔名：
      - `examples/ESP32_C3_OTTO_Runtime_Demo/ESP32_C3_OTTO_Runtime_Demo.ino`
  - 測試：
    - compile `examples/ESP32_C3_OTTO_Runtime_Demo`
    - `arduino-lint`
  - 結果：
    - 已新增 `V7RCOtto-esp32.h`
    - 已新增 `V7RCOttoRuntime`
    - 已新增 `V7RCOttoTypes`
    - 已補上 walk / turn / bounce 的 OTTO runtime demo
    - `ESP32_C3_OTTO_Runtime_Demo` 已 compile 通過
    - `arduino-lint` 已通過

- [x] 增加 `RobotARM_Runtime_Demo`
  - 需求：
    - 已調整為 robot arm 專屬 runtime 路徑
    - 目錄與檔名：
      - `examples/ESP32_C3_RobotARM_Runtime_Demo/ESP32_C3_RobotARM_Runtime_Demo.ino`
  - 測試：
    - compile `examples/ESP32_C3_RobotARM_Runtime_Demo`
    - `arduino-lint`
  - 結果：
    - 已新增 `V7RCRobotArm-esp32.h`
    - 已新增 `V7RCRobotArmRuntime`
    - 已新增 `V7RCRobotArmTypes`
    - 已補上 robot arm 的 preset pose sequence demo
    - `ESP32_C3_RobotARM_Runtime_Demo` 已 compile 通過
    - `arduino-lint` 已通過
