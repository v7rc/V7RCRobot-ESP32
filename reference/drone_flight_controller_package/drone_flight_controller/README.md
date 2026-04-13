# ESP32-C3 Drone Dual-Loop Flight Controller Skeleton

This package upgrades a simplified self-leveling controller into a cleaner **dual-loop architecture**:

- **Outer loop**: angle controller -> desired roll/pitch rates
- **Inner loop**: rate PID -> roll/pitch/yaw commands
- **Mixer**: X quad motor mixing
- **Output conditioning**: simple desaturation instead of raw clamp-only behavior
- **Debug support**: CSV serial output for tuning and sign checking

## Folder structure

```text
src/
  FlightTypes.h
  PidController.h / .cpp
  AngleController.h / .cpp
  RateController.h / .cpp
  MotorMixer.h / .cpp
  FlightController.h / .cpp
  DebugPrinter.h
examples/
  ESP32_C3_Drone_Demo/
    ESP32_C3_Drone_Demo.ino
PARAMETERS.md
TUNING_GUIDE.md
INTEGRATION_GUIDE.md
```

## Control flow

```text
Remote input -> normalized control -> attitude estimate -> level trim ->
angle loop -> rate loop -> motor mixer -> desaturation -> motor output
```

## Important notes

1. **Confirm sign direction first**
   - If the craft tilts right, the controller must command a corrective lefting moment.
   - Do not start PID tuning before sign verification is complete.

2. **Start without props**
   - Hand-tilt the frame.
   - Observe motor output direction.
   - Use serial CSV debug first.

3. **Start with P only**
   - Set `Ki = 0`, `Kd = 0` first.
   - Add D next.
   - Add I last.

4. **This is still a bring-up skeleton**
   - It is intentionally lighter than PX4 / Betaflight.
   - It is meant to be integrated into an existing ESP32-C3 codebase with your own IMU and motor drivers.

## Recommended first integration steps

- Map your normalized control inputs into `ControlState`
- Export roll/pitch angles and roll/pitch/yaw gyro rates into `AttitudeState`
- Feed your existing level trim values into `setLevelTrim()`
- Connect your motor output code in `writeMotors()`
- Enable serial CSV output and verify polarity before spinning props

## Initial tuning strategy

- Start with:
  - roll/pitch rate PID: `Kp > 0`, `Ki = 0`, `Kd = 0`
  - yaw rate PID: `Kp > 0`, `Ki = 0`, `Kd = 0`
- Then add a small D term
- Finally add a small I term to remove long-term bias

## Output ranges

- Input throttle: `0..1`
- Stick roll/pitch/yaw: `-1..1`
- Motor outputs: `0..1`

## Debug CSV columns

```text
t,thr,auth,stb,air,rollDeg,pitchDeg,dRollRate,dPitchRate,dYawRate,mRollRate,mPitchRate,mYawRate,rollCmd,pitchCmd,yawCmd,sat
```

These are suitable for:
- Arduino Serial Plotter
- serial logging to CSV
- spreadsheet-based tuning review

