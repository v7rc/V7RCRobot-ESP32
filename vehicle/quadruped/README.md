# Quadruped Module

四足機器人控制模組。

目前第一版已開始建立：

- quadruped runtime
- gait phase scheduling
- foot target generation
- simple 2-link inverse kinematics

控制分層方向會參考實際 quadruped controller 常見做法，例如 Stanford Pupper 文件中的：

- gait scheduler
- stance / swing controller
- inverse kinematics

目前先提供可編譯、可跑動作序列的第一版 dog runtime demo，後續再往 body pose、足端軌跡與更完整 gait 設計擴充。
