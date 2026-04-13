# Integration Guide

## What this package expects from your project

You already need these working pieces in your own firmware:

- BLE / RC input decoding
- IMU readout and attitude estimation
- arming / disarming logic
- level trim calibration
- motor write functions

This package does **not** replace those pieces.
It provides the control core and a cleaner structure for the stabilization algorithm.

## Required mappings

### 1. Control input mapping

Map your normalized inputs into:

```cpp
ControlState control;
control.throttle  = ...; // 0..1
control.roll      = ...; // -1..1
control.pitch     = ...; // -1..1
control.yaw       = ...; // -1..1
control.stabilize = ...; // true/false
```

### 2. Attitude mapping

Map your attitude estimator output into:

```cpp
AttitudeState attitude;
attitude.rollDeg = ...;
attitude.pitchDeg = ...;
attitude.rollRateDegPerSec = ...;
attitude.pitchRateDegPerSec = ...;
attitude.yawRateDegPerSec = ...;
attitude.valid = true;
```

### 3. Level trim

After your existing unlock / calibration flow determines trim:

```cpp
gFlightController.setLevelTrim(levelRollTrimDeg, levelPitchTrimDeg);
```

### 4. Airborne state

The provided controller supports an `airborne` flag for anti-windup behavior.
You may initially hardcode it, then later replace it with a real detector.

Example strategies:
- throttle above threshold for a minimum time
- vertical acceleration behavior
- motor output plus attitude response

### 5. Motor writes

The controller returns normalized motor outputs `0..1`.
Convert those to:
- ESC microseconds, or
- normalized DC motor driver commands

## Safety checklist

Before using props:

- sign test passed
- motor order confirmed
- mixer direction confirmed
- arming behavior confirmed
- motor idle handling confirmed
- emergency disarm path confirmed

## Recommended next upgrades

After this controller is working, the next useful upgrades are:

1. derivative low-pass filtering
2. configurable parameter storage
3. better airborne detection
4. motor idle floor
5. explicit landed / takeoff state machine
6. more advanced desaturation logic
