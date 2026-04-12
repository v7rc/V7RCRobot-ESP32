# Transport Layer

這一層負責資料收發與連線生命週期，不負責載具語意或 actuator 控制。

預計逐步放入：

- BLE transport
- Wi-Fi UDP transport
- MQTT transport
- UART transport

目前已建立第一版檔案：

- `V7RCTransport.h`
- `V7RCBleTransportEsp32.h`
- `V7RCBleTransportEsp32.cpp`

目前 BLE transport 已負責：

- NimBLE 初始化
- 廣播啟動
- 連線 / 斷線狀態通知
- RX bytes ingress callback
- TX notify 傳送

legacy driver 目前透過 callback 與 transport 銜接，後續會再把更多 runtime 相依逐步拆開。
