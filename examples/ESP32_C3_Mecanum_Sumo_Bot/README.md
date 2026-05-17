# ESP32 C3 Mecanum SUMO-BOT

## MVP

- Autonomous mecanum SUMO-BOT sketch, no BLE control, Servo, or WS2812 LED.
- Two digital TCRT5000 line sensors:
  - GPIO7: left line sensor
  - GPIO8: right line sensor
- One HC-SR04 ultrasonic sensor:
  - GPIO5: trigger
  - GPIO6: echo
- Boot behavior:
  - Motors stay stopped after startup.
  - When HC-SR04 sees a target within 20 cm, wait for the target to be removed.
  - After the target is removed from the 20 cm start zone, wait 3 seconds.
  - Start the SUMO routine after the countdown.
- SUMO routine:
  - Drive forward quickly for 1 second.
  - Stop briefly.
  - Randomly rotate left or right for 0.2 seconds.
  - Check both line sensors every 10 ms.
  - If either sensor detects the ring line, immediately back up for 1 second, rotate away from the triggered side for 1 second, then drive forward for 1 second before returning to target search.
  - If HC-SR04 detects an object within 100 cm, charge forward at full power.
  - Line detection always has priority over target charging.
- Serial debug mode can print distance, left/right line sensor state, SUMO state, and turn direction.
- All timing, speed, pin, polarity, and distance thresholds are configurable constants at the top of the sketch.

## TODO

- Add sensor filtering or confirmation counts if the ring surface causes noisy line readings.
- Add a match timeout or kill-switch input for safer testing.

## Notes

- `kLineDetectedLevel` is currently confirmed for the test TCRT5000 modules and ring line.
- Motor direction is tuned by changing the N20 motor wiring pins instead of `dirInvert`.
- Sensor filtering, match timeout, and kill-switch are intentionally deferred.
