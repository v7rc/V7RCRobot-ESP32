#pragma once

class V7RCDroneV2Pid {
public:
  V7RCDroneV2Pid();

  void configure(float kp,
                 float ki,
                 float kd,
                 float integralLimit,
                 float outputLimit);

  void reset();

  float update(float setpoint,
               float measurement,
               float dt,
               bool allowIntegral = true);

private:
  float kp_;
  float ki_;
  float kd_;
  float integral_;
  float integralLimit_;
  float outputLimit_;
  float previousMeasurement_;
  bool hasPreviousMeasurement_;
};
