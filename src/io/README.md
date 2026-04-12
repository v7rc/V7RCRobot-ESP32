# IO Layer

這一層負責抽象化硬體輸入輸出能力，避免 vehicle logic 直接綁定底層實作。

預計內容：

- GPIO / digital output
- PWM output
- servo output
- ESC output
- sensor adapter interface

目前已建立第一版檔案：

- `V7RCIOInterfaces.h`
- `V7RCEsp32Outputs.h`
- `V7RCEsp32Outputs.cpp`

這一版先固定幾個抽象方向：

- `V7RCServoOutput`
- `V7RCDCMotorOutput`
- `V7RCStatusLedOutput`

並補上第一版 ESP32 具體實作：

- `V7RCEsp32ServoOutput`
- `V7RCEsp32DCMotorOutput`
- `V7RCWs2812StatusLedOutput`
