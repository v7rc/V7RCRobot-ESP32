# Car Module

預留給一般車輛控制模組。

預計涵蓋：

- differential drive
- mecanum drive
- throttle / steer 或 `vx / vy / omega` 映射

目前已建立第一版檔案：

- `V7RCCarTypes.h`
- `V7RCCarControl.h`
- `V7RCCarControl.cpp`
- `V7RCCarRobot.h`
- `V7RCCarRobot.cpp`
- `V7RCCarRuntime.h`
- `V7RCCarRuntime.cpp`

Arduino library 實際參與編譯的版本位於：

- `src/vehicle/car/V7RCCarTypes.h`
- `src/vehicle/car/V7RCCarControl.h`
- `src/vehicle/car/V7RCCarControl.cpp`
- `src/vehicle/car/V7RCCarRobot.h`
- `src/vehicle/car/V7RCCarRobot.cpp`
- `src/vehicle/car/V7RCCarRuntime.h`
- `src/vehicle/car/V7RCCarRuntime.cpp`

這一版先固定 car module 的控制語意與 mixer 方向，後續再把 legacy driver 中的差速 / mecanum 控制逐步遷移到這裡。
