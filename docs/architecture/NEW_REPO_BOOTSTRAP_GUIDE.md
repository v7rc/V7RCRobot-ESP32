# New Repo Bootstrap Guide

這份文件是給下一個獨立 repo 使用的開工指南。

目標是把目前的多載具平台重構，從既有的 `V7RCServoDriver` repo 中抽離，建立成一個新的、可長期演進的專案。

## 建議的新 Repo 定位

新的 repo 不建議再沿用目前「只像 servo driver」的定位。

比較適合的新定位是：

- 多載具控制平台
- 可替換通訊層
- 可擴充 protocol layer
- 可擴充 sensor / output abstraction
- 可掛載多種 vehicle module

可考慮的名稱方向例如：

- `V7RCPlatform`
- `V7RCMultiVehicle`
- `V7RCCore`
- `V7RCVehicleKit`

## 哪些檔案建議直接 Copy

這些檔案適合直接帶去新 repo，作為初期參考或重構起點。

### 1. 核心原始碼

- `src/V7RCServoDriver.h`
- `src/V7RCServoDriver.cpp`

用途：

- 當作現有功能的來源
- 方便拆出 transport / protocol / core / io 的邏輯
- 可作為 legacy compatibility layer 的初始版本

### 2. 現有 examples

- `examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`
- `examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`

用途：

- 作為 car 類 vehicle module 的最早測試案例
- 幫助確認舊功能在新架構下是否還能跑

### 3. 文件檔

- `README.md`
- `README.zh-TW.md`
- `QUICK_START_EDU.md`
- `library.properties`
- `keywords.txt`
- `LICENSE`

用途：

- 參考目前對外說明方式
- 方便保留 Arduino library 基本結構
- 作為之後重新撰寫 README 的素材

### 4. 架構草圖文件

- `MULTI_VEHICLE_LIBRARY_ARCHITECTURE.md`
- `NEW_REPO_BOOTSTRAP_GUIDE.md`

用途：

- 作為新 repo 的設計依據
- 避免開新專案後重複討論同一批架構問題

## 哪些檔案建議「帶過去參考，但不要直接沿用」

這些檔案可以 copy 過去，但建議視為素材，不要原封不動當成正式版本。

### 1. `README.ja.md`

原因：

- 現階段應先把主架構與主 README 定好
- 多語系文件可以等 API 比較穩定再補

### 2. `README.th.md`

原因同上。

### 3. `ARDUINO_LIBRARY_MANAGER_DESC.md`

原因：

- 新 repo 的定位會不同
- 描述內容之後應該重寫，不適合直接沿用

## 哪些內容建議先不要直接 Copy 成正式設計

以下內容可以參考，但不建議直接搬去當新 repo 的正式結構。

### 1. 目前的 class 命名

例如：

- `V7RCServoDriver`

原因：

- 新 repo 的定位已經不只是 servo driver
- 後續應考慮更中性的命名

### 2. 目前把 BLE / protocol / runtime 寫在同一個 `.cpp` 的做法

原因：

- 這正是新 repo 要拆開重構的核心問題

### 3. 目前 example 內一些不一致的設定

原因：

- 現有 examples 本身有歷史包袱
- 可以當測試素材，但不應直接當成新架構的標準寫法

## 我建議你在新 Repo 的起始檔案

新 repo 一建立後，我建議至少先有：

- `README.md`
- `LICENSE`
- `library.properties`
- `src/`
- `examples/`
- `docs/`

另外建議第一批就放入：

- `docs/architecture/MULTI_VEHICLE_LIBRARY_ARCHITECTURE.md`
- `docs/architecture/NEW_REPO_BOOTSTRAP_GUIDE.md`

## 建議的新 Repo 初始結構

```text
src/
  transport/
  protocol/
  core/
  io/
  legacy/

vehicle/
  car/
  drone/
  quadruped/
  otto/
  arm/

examples/
  legacy_car/
  legacy_mecanum/

docs/
  architecture/
```

## 建議第一批 Copy 過去後怎麼放

### 放進 `legacy/`

建議先把這兩個檔案放進 `src/legacy/`，而不是直接放在新核心根目錄：

- `V7RCServoDriver.h`
- `V7RCServoDriver.cpp`

原因：

- 這樣可以保留舊功能參考
- 同時避免它在語意上繼續占據新架構的核心位置

### 放進 `examples/legacy_*`

建議先把現有車類 examples 當成 legacy examples：

- `examples/legacy_car/`
- `examples/legacy_mecanum/`

原因：

- 可以保留行為驗證
- 但不會和未來新的 vehicle examples 混在一起

## 建議第一階段不要急著 Copy 的東西

如果你希望新 repo 一開始乾淨一點，下列可以稍後再決定是否搬：

- 多語系 README
- 舊版 library manager 描述
- 過度針對當前 BLE driver 的文案

## 建議第一階段一定要保留的歷史參考

即使未來不公開，也建議在新 repo 中保留以下參考來源：

- 舊版 `V7RCServoDriver.cpp`
- 舊版 `V7RCServoDriver.h`
- 至少兩個舊 examples

原因：

- 方便對照重構前後行為
- 方便日後補 legacy compatibility
- 方便快速驗證舊有 BLE / V7RC protocol 行為

## 建議你建立新 Repo 後的第一步

當你建立好新 repo 並在 Codex 開新專案後，我們可以直接照這個順序開始：

1. 建立新 repo 基本目錄
2. 複製 legacy source 與 architecture docs
3. 重新命名與定義 core / transport / protocol 邊界
4. 先把 BLE 從舊 driver 中拆成 transport layer
5. 再把 protocol decode 拆出來
6. 最後才開始整理 vehicle layers

## 建議你現在實際 Copy 的清單

最精簡但足夠開工的版本：

- `src/V7RCServoDriver.h`
- `src/V7RCServoDriver.cpp`
- `examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`
- `examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
- `README.md`
- `README.zh-TW.md`
- `QUICK_START_EDU.md`
- `library.properties`
- `keywords.txt`
- `LICENSE`
- `MULTI_VEHICLE_LIBRARY_ARCHITECTURE.md`
- `NEW_REPO_BOOTSTRAP_GUIDE.md`

## 不需要現在就 Copy 的清單

- `README.ja.md`
- `README.th.md`
- `ARDUINO_LIBRARY_MANAGER_DESC.md`

---

當新 repo 建好後，這份文件就可以作為第一天的施工藍圖。
