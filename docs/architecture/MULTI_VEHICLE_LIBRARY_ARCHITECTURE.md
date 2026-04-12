# Multi-Vehicle Library Architecture Draft

這份文件是未來重構 `V7RCServoDriver` 時的多載具架構草圖，目標不是只支援單一類型載具，而是把目前的 library 逐步整理成可支援多種應用的平台。

目前規劃納入的載具應用包含：

- 一般車輛
- 四軸飛行無人機
- 四足狗
- Otto 機器人
- 機器手臂

同時也把下列共通能力一起納入重構考量：

- 多種通訊層
- 感應器輸入
- GPIO / PWM / Servo / ESC 輸出
- 安全邏輯與狀態管理
- 不同載具的控制語意切換

## 目標

- 保留目前 repo 已有的 BLE 與 V7RC 協議能力
- 把目前寫死在 BLE 的通訊邏輯抽成可替換 transport layer
- 把 base library 整理成較穩定的 core
- 讓不同載具在上層以獨立 library 或 module 方式實作
- 避免所有 vehicle-specific 邏輯都塞進同一個 public API
- 讓未來再增加新載具時，維護成本仍可控

## 目前 Repo 的定位

目前這個 repo 比較像是：

- V7RC BLE receiver
- V7RC protocol decoder
- channel-based control runtime
- actuator mapper
- common safety / LED runtime

它已經具備幾個很有價值的基礎：

- NimBLE 連線與資料接收
- `HEX`、`DEG`、`SRV/SRT`、`SS8`、`LED` 協議解析
- 16-channel frame 模型
- Servo 與 DC motor 的基本控制
- timeout safety
- WS2812 LED 狀態輸出

這些能力適合當作共用底層，但不適合直接承擔所有載具邏輯。

另外，目前的「通訊層」也和 BLE 綁得很緊，這在未來要支援其他連線方式時會成為限制。

## 為什麼要改成多載具分層

如果之後直接把一般車輛、四足狗、Otto、機器手臂、四軸飛行器都持續往目前 `V7RCServoDriver` 裡面加，短期上手快，但長期會出現明顯問題：

- API 會變得混雜
- transport 與 protocol 會互相耦合
- actuator 模型會互相污染
- 不同載具的 safety policy 很難共存
- example 和 README 會變得很重
- 後續每加一種載具都要動 base library

這些載具的控制語意其實差很多：

- 一般車輛：`throttle / steer` 或 `vx / vy / omega`
- 四軸：`throttle / roll / pitch / yaw / arm`
- 四足狗：`gait / stride / turn / body tilt / stance`
- Otto：`walk / turn / dance / gesture / buzzer / LED`
- 機器手臂：`joint angle / gripper / preset pose / interpolation`

如果在設計上不先分層，base library 很快就會變成難以維護的集合體。

## 建議的分層

### Layer 1: Transport / Communication Layer

這一層負責「資料怎麼進來、怎麼出去」，不負責載具語意，也不應直接控制 actuator。

建議未來可支援的 transport 包含：

- Bluetooth Low Energy
- Wi-Fi UDP
- Wi-Fi MQTT
- UART

之後若有需要，也可擴充：

- Wi-Fi TCP
- ESP-NOW
- USB Serial

這一層的責任應包含：

- transport 初始化
- connect / disconnect 狀態
- 收包與發包
- 將收到的 bytes 或 message 交給 protocol parser
- 通知上層目前 transport 狀態

### Layer 2: Protocol Layer

這一層負責「資料格式怎麼解讀」。

目前已有的 V7RC 協議格式包含：

- `HEX`
- `DEG`
- `SRV / SRT`
- `SS8`
- `LED`

這層應負責：

- frame decode
- command validation
- channel extraction
- 將封包轉成統一的 command / frame model

重點是：

- transport layer 不需要知道 `HEX` 或 `DEG` 是什麼
- vehicle layer 也不應直接處理原始 bytes

### Layer 3: Core / Base Library

這一層應負責所有載具都會共用的能力：

- channel state storage
- common timeout handling
- LED / status indication
- generic output abstraction
- generic sensor abstraction
- hardware runtime state

這一層會吃到 protocol layer 解出的統一 channel / command 結果，並維持共用 runtime。

這一層盡量不要帶太多「某種載具專屬語意」。

### Layer 4: Vehicle Libraries

每一種載具各自有自己的 library 或 module，例如：

- `V7RCCar`
- `V7RCDrone`
- `V7RCQuadruped`
- `V7RCOtto`
- `V7RCArm`

這些 library 負責：

- channel interpretation
- control mapping
- mixer 或 motion solver
- actuator usage policy
- sensor usage policy
- vehicle-specific safety rules
- vehicle-specific examples

## 這次重構應一起納入的範圍

如果目標是做真正可延伸的平台，這次重構不應只看馬達輸出，還要一起考慮：

- 多種 transport / communication
- 感應器輸入
- GPIO / PWM / Digital 類型輸出
- Servo / ESC 類型輸出
- 狀態機與安全策略

原因是：

- 不同產品現場不一定都適合 BLE
- 某些場景會需要低延遲 UDP
- 某些場景會需要 MQTT 做雲端整合
- 某些場景可能只需要 UART 接外部主控
- 一般車輛可能需要編碼器、測距、循跡感測器
- 四軸需要 IMU、電壓監測、arm/disarm 狀態
- 四足狗需要多顆 servo、姿態或接地狀態資訊
- Otto 需要 servo、buzzer、LED、動作模式控制
- 機器手臂需要多關節 servo、limit switch、gripper 控制

如果 transport、input、output abstraction 沒有在 base 層先規劃好，後面每個 vehicle lib 都會重複做一次整合。

## 建議的責任邊界

### Transport Layer 應負責

- BLE / UDP / MQTT / UART 初始化
- transport connection state
- 收到原始資料後往 protocol layer 傳遞
- 發送遙測或狀態資料
- transport-specific reconnect policy

### Protocol Layer 應負責

- 原始 payload decode
- 封包格式驗證
- command / channel frame 轉換
- 對不同 transport 保持一致的資料模型

### Core / Base Library 應負責

- raw channel / normalized channel 保存
- timeout 與 signal valid 狀態
- 共用 LED / status 呈現
- 基礎 GPIO / PWM / Servo / ESC 輸出抽象
- 基礎 sensor adapter 介面
- pin ownership 與硬體初始化
- 提供 vehicle layer 查詢狀態與讀寫硬體的 API

### Vehicle Library 應負責

- 決定哪些 channel 對應哪些語意
- 決定如何把 channel 轉成動作
- 決定需要用哪些 output
- 決定需要讀哪些 sensor
- vehicle-specific safety policy
- vehicle-specific motion logic
- vehicle-specific example 與文件

## 感應器與 GPIO 為什麼一定要一起設計

未來多載具擴充時，幾乎每種應用都會同時用到輸入與輸出：

- 車：馬達輸出 + 距離感測器 + 編碼器
- 四軸：ESC 輸出 + IMU + battery monitor
- 四足狗：多路 servo + 姿態估測或接地資訊
- Otto：servo + buzzer + RGB LED + 互動輸入
- 機器手臂：多關節 servo + gripper + limit switch

如果只有 output abstraction，沒有 sensor abstraction，後續 vehicle layer 還是會分裂。

如果只有 sensor abstraction，沒有統一 output model，則 actuator 端也會失控。

所以這兩者應該在同一次重構裡一起設計。

## 建議加入的 Core 能力

### 1. Channel State Layer

這層負責保存來自 V7RC App 的控制值，提供：

- raw channel
- normalized channel
- last update time
- signal valid / timeout state

### 2. Output Abstraction Layer

這層負責描述各種輸出裝置，而不是只侷限在目前的 servo 與 DC motor。

建議至少考慮下列輸出類型：

- digital output
- pwm output
- servo output
- esc output
- rgb / led output
- buzzer output

概念草圖如下：

```cpp
enum V7RC_OutputType : uint8_t {
  OUT_NONE = 0,
  OUT_DIGITAL,
  OUT_PWM,
  OUT_SERVO,
  OUT_ESC,
  OUT_RGB,
  OUT_BUZZER
};

struct V7RC_OutputConfig {
  V7RC_OutputType type;
  uint8_t pinA;
  uint8_t pinB;
  bool invert;
  int16_t minValue;
  int16_t maxValue;
  int16_t initValue;
};
```

這不代表一開始要全部做完，而是重構時應保留這樣的方向。

### 3. Sensor Abstraction Layer

這層負責管理感應器讀值與有效性，而不是把 sensor 直接寫死在某個 vehicle library。

概念草圖如下：

```cpp
enum V7RC_SensorType : uint8_t {
  SENSOR_NONE = 0,
  SENSOR_DIGITAL_IN,
  SENSOR_ANALOG_IN,
  SENSOR_IMU,
  SENSOR_DISTANCE,
  SENSOR_BATTERY,
  SENSOR_ENCODER,
  SENSOR_LIMIT_SWITCH
};

struct V7RC_SensorReading {
  bool valid;
  float value0;
  float value1;
  float value2;
  uint32_t timestampMs;
};
```

第一版不需要所有 sensor 都完整支援，但至少應保留：

- sensor identity
- reading format
- validity
- timestamp

### 4. Runtime / Safety Layer

這層負責共通安全規則，例如：

- signal timeout
- default output state
- sensor stale timeout
- boot-time init state
- connection state

vehicle layer 再在上層決定自己的安全策略：

- drone 的 disarm
- 車類的 brake / coast
- 四足狗的站姿回正
- 機器手臂的 home pose / torque release

### 5. Transport Abstraction Layer

這層負責讓未來可以在不改 vehicle logic 的前提下，切換不同通訊方式。

建議至少先預留下列 transport 類型：

- BLE
- Wi-Fi UDP
- Wi-Fi MQTT
- UART

概念草圖如下：

```cpp
enum V7RC_TransportType : uint8_t {
  TRANSPORT_NONE = 0,
  TRANSPORT_BLE,
  TRANSPORT_WIFI_UDP,
  TRANSPORT_WIFI_MQTT,
  TRANSPORT_UART
};

struct V7RC_TransportConfig {
  V7RC_TransportType type;
  const char* name;
  uint16_t port;
  uint32_t baudRate;
};
```

實作上不一定要用這個結構，但文件上建議明確把 transport 視為獨立層，而不是核心類別裡的一段 if/else。

## 建議的 API 方向

### Core 提供較中性的資料介面

例如：

```cpp
class V7RCCore {
public:
  void begin(uint32_t robotId, const V7RC_CoreConfig& cfg);
  void loop();

  bool hasSignal() const;
  bool isConnected() const;
  uint32_t lastFrameMs() const;

  int16_t rawChannel(uint8_t ch) const;
  float normalizedChannel(uint8_t ch) const;

  bool sensorAvailable(uint8_t sensorId) const;
  V7RC_SensorReading readSensor(uint8_t sensorId) const;

  void writeDigital(uint8_t outputId, bool value);
  void writePwm(uint8_t outputId, float duty);
  void writeServoUs(uint8_t outputId, uint16_t us);
  void writeEscUs(uint8_t outputId, uint16_t us);
};
```

重點不是命名，而是 base library 應逐步提供：

- channel access
- signal / timeout access
- sensor access
- generic output access

而不是只做某一種載具的驅動器。

### Transport / Protocol 層可考慮的介面

例如：

```cpp
class V7RCTransport {
public:
  virtual bool begin(const V7RC_TransportConfig& cfg) = 0;
  virtual void loop() = 0;
  virtual bool connected() const = 0;
  virtual bool readPacket(uint8_t* data, size_t& len) = 0;
  virtual bool writePacket(const uint8_t* data, size_t len) = 0;
};

class V7RCProtocol {
public:
  virtual bool decode(const uint8_t* data, size_t len, V7RC_CommandFrame& out) = 0;
};
```

這樣的好處是：

- BLE、UDP、MQTT、UART 可以共用上層 core
- vehicle library 不用知道底層是什麼通訊
- protocol 可獨立測試

## 通訊層的設計原則

### Transport 與 Protocol 分開

不要把：

- BLE callback
- bytes parsing
- channel mapping

全部寫在同一層。

建議拆成：

- transport: 怎麼收到資料
- protocol: 怎麼解讀資料
- core: 怎麼保存狀態
- vehicle: 怎麼使用狀態

### Vehicle 不直接依賴 BLE

例如 `V7RCDrone` 不應該知道自己是透過 NimBLE 還是 UDP 控制。

它只應該知道：

- 是否有有效控制訊號
- 目前 channel 值是多少
- 是否 timeout

### 遙測回傳也應保留抽象

未來如果要回傳：

- battery
- arm state
- mode
- pose
- sensor data

也應該走 transport abstraction，而不是只綁在 BLE notify。

## 各載具應用的建議定位

### 1. 一般車輛 `V7RCCar`

適合承接：

- 差速車
- 阿克曼轉向車
- 麥克納姆車
- 履帶車

主要語意可能包含：

- throttle
- steer
- brake
- mode select
- light / horn

常見輸出：

- DC motor
- Servo
- PWM
- Digital output

常見感應器：

- distance sensor
- wheel encoder
- line sensor
- battery voltage

### 2. 四軸飛行器 `V7RCDrone`

主要語意可能包含：

- throttle
- roll
- pitch
- yaw
- arm / disarm
- flight mode

常見輸出：

- ESC PWM output
- status LED
- buzzer

常見感應器：

- IMU
- battery voltage
- optional altitude / distance sensor

### 3. 四足狗 `V7RCQuadruped`

主要語意可能包含：

- gait select
- forward / backward
- strafe 或 body shift
- turn
- stance height
- body tilt
- action trigger

常見輸出：

- 多顆 servo
- status LED
- buzzer

常見感應器：

- IMU
- foot contact sensor
- battery voltage

備註：

四足狗的上層通常需要 gait planner 或 posture solver，因此非常不適合直接塞進 base library。

### 4. Otto 機器人 `V7RCOtto`

主要語意可能包含：

- walk
- turn
- dance
- pose
- sound effect
- expression / light effect

常見輸出：

- 雙腿或多顆 servo
- buzzer
- RGB LED

常見感應器：

- ultrasonic distance
- button / touch input
- battery voltage

備註：

Otto 的應用通常偏教育、互動與動作編排，與車類或 drone 的控制模型差很大，建議獨立成專用 module。

### 5. 機器手臂 `V7RCArm`

主要語意可能包含：

- joint control
- gripper open / close
- speed scale
- preset pose
- record / playback

常見輸出：

- 多關節 servo
- gripper output
- status LED

常見感應器：

- limit switch
- potentiometer
- current / load monitor
- battery or supply voltage

備註：

機器手臂通常需要 pose management、動作插值與 home/limit 安全規則，這也應該放在專屬 vehicle layer。

## Sensor 與 GPIO 的分層原則

### Core 管理「能力」

例如：

- 哪些 pin 是 digital output
- 哪些 pin 是 servo / ESC output
- 哪些 sensor 可讀
- 哪些 sensor 資料有效

### Vehicle Layer 管理「策略」

例如：

- 車子距離過近是否自動限速
- drone timeout 是否立刻 disarm
- 四足狗在失聯時是否回站姿
- Otto 失聯時是否停止舞步
- 手臂失聯時是否回 home pose

這樣責任會比較清楚。

## 建議納入重構的 GPIO / Output 類型

建議至少先規劃：

- Digital input
- Digital output
- Analog input
- PWM output
- Servo output
- ESC PWM output
- WS2812 / status LED
- Buzzer output

其中 `Servo output` 與 `ESC PWM output` 建議明確分開：

- Servo 是角度裝置
- ESC 是推力裝置

語意分開之後，vehicle library 比較好維護。

## 建議納入重構的 Sensor 類型

第一階段建議先預留：

- 板上 GPIO digital sensor
- 板上 analog sensor
- battery voltage
- IMU
- distance sensor
- wheel encoder
- limit switch

## 建議優先支援的通訊方式

第一波最值得規劃的 transport：

- BLE
- Wi-Fi UDP
- UART

原因：

- BLE 是目前既有基礎
- UDP 適合低延遲控制
- UART 適合串接外部主控、飛控或上位機

`Wi-Fi MQTT` 也很重要，但它比較偏裝置管理、遠端監控、事件傳遞，若要做即時控制需額外評估延遲與 QoS 策略。

因此建議可以先把 MQTT 納入架構，但不一定放在第一個即時控制版本完成。

## 建議的演進步驟

### Phase 1

整理目前 base lib，讓它更像共用底層：

- 抽離 BLE 專屬通訊邏輯，規劃 transport layer
- 抽離 protocol decode，避免和 transport 綁死
- 補強 channel state access
- 把 vehicle-specific 邏輯與 common logic 分清楚
- 把 output model 從單一 servo / DC motor 擴充成更通用的 abstraction
- 預留 sensor abstraction
- 減少 example 中不一致的設定

### Phase 2

建立 vehicle libraries 的最小骨架：

- `V7RCCar`
- `V7RCDrone`
- `V7RCQuadruped`
- `V7RCOtto`
- `V7RCArm`

每個先只做到最小可用版本。

### Phase 3

依載具逐步補強：

- BLE / UDP / UART transport prototype
- car 的不同底盤模型
- drone 的 manual quad mixer
- quadruped 的 gait / posture
- Otto 的互動動作
- arm 的 pose / interpolation

### Phase 4

視需求再加入：

- IMU stabilization
- PID tuning
- more advanced sensor integration
- telemetry / feedback channel
- preset behavior engine

## 建議的目錄草圖

若先維持同 repo，可考慮往這種方向整理：

```text
src/
  transport/
    V7RCTransport.h
    V7RCBleTransport.cpp
    V7RCUdpTransport.cpp
    V7RCMqttTransport.cpp
    V7RCUartTransport.cpp
  protocol/
    V7RCProtocol.h
    V7RCProtocol.cpp
  core/
    V7RCCore.h
    V7RCCore.cpp
  io/
    V7RCOutput.h
    V7RCOutput.cpp
    V7RCSensor.h
    V7RCSensor.cpp
  legacy/
    V7RCServoDriver.h
    V7RCServoDriver.cpp

vehicle/
  car/
    V7RCCar.h
    V7RCCar.cpp
  drone/
    V7RCDrone.h
    V7RCDrone.cpp
  quadruped/
    V7RCQuadruped.h
    V7RCQuadruped.cpp
  otto/
    V7RCOtto.h
    V7RCOtto.cpp
  arm/
    V7RCArm.h
    V7RCArm.cpp

examples/
  ESP32_C3_Differential_Drive/
  ESP32_C3_Mini_Mecanum_V3/
  ESP32_C3_Quad_X_Drone/
  ESP32_C3_Quadruped/
  ESP32_C3_Otto/
  ESP32_C3_Robot_Arm/

docs/
  architecture/
```

如果 Arduino library 結構限制讓多 library 不好管理，也可以後續拆成多個 repo。

## 維護原則

- core 儘量穩定，少放載具特化邏輯
- transport、protocol、core、vehicle 四層要盡量分清楚
- vehicle library 可以較快演進
- sensor 與 output 的抽象盡量共用
- control strategy 不要過度共用
- 每種載具有自己的 example
- 每種載具有自己的 README 或章節文件
- GPIO 命名與 pin ownership 要集中管理
- 盡量避免 vehicle layer 直接繞過 core 自己處理底層資源

## 目前最推薦的方向

以目前這個專案的狀態，我最推薦：

1. 不再把這個 repo 只視為 servo driver
2. 先把通訊層從目前 BLE 實作中抽離
3. 再把它逐步整理成較中性的 core/base
4. 重構時同步把 sensor / GPIO / output abstraction 納入
5. 在上層規劃 `Car / Drone / Quadruped / Otto / Arm` 五類 vehicle library
6. 先讓每個 vehicle library 都有最小可用版本
7. 等架構穩定後，再決定是否拆 repo

## 後續可直接執行的工作

接下來可以從下列項目開始：

- 決定 core/base 新名稱是否要保留 `V7RCServoDriver`
- 定義 transport layer 與 protocol layer 的邊界
- 決定第一波要先落地的 transport：BLE / UDP / UART / MQTT 的優先序
- 整理 base lib 要暴露哪些 channel/state API
- 整理 base lib 要暴露哪些 sensor/output API
- 定義各 vehicle library 的最小 config 結構
- 定義 GPIO 與 sensor config 格式
- 為五種載具各自整理最小 example 需求

---

這份文件目前是架構草圖，不是最終規格。之後若方向確認，可以再把它升級成正式重構 spec，並拆成：

- core spec
- io spec
- car spec
- drone spec
- quadruped spec
- otto spec
- arm spec
