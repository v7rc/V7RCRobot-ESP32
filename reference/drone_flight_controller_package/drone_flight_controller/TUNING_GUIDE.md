# Tuning Guide

## 1. Before any tuning

Do these first:

- Remove props.
- Confirm IMU axis mapping is correct.
- Confirm roll angle sign and roll-rate sign are consistent.
- Confirm pitch angle sign and pitch-rate sign are consistent.
- Confirm positive controller output produces the intended corrective motor response.

If signs are wrong, PID tuning will not fix it.

## 2. First bench verification

Hand-tilt test:

- Tilt right -> controller should produce a lefting correction.
- Tilt nose-down -> controller should produce a nose-up correction.
- Command positive yaw -> output should produce the expected yaw torque.

## 3. First PID pass

Start with:

- roll/pitch: `Ki = 0`, `Kd = 0`
- yaw: `Ki = 0`, `Kd = 0`

Only tune `Kp` first.

### Goal
- Stable corrective tendency
- No violent oscillation
- No reversed feedback

## 4. Add D

Add a small D only after P behaves correctly.

Expected effect:
- better damping
- less overshoot
- cleaner stop after disturbance

If D is too high:
- noisy motors
- rapid buzz
- command chatter in logs

## 5. Add I

Add a small I only after P and D are acceptable.

Expected effect:
- removes long-term bias
- compensates for persistent asymmetry

If I is too high:
- slow oscillation
- “stuck” correction after disturbance
- bad ground behavior

## 6. Suggested practical order

1. Roll/Pitch rate P
2. Yaw rate P
3. Roll/Pitch rate D
4. Roll/Pitch rate I
5. Yaw rate I

## 7. Ground vs airborne

Do not judge tuning quality only on the ground.
Ground contact can distort apparent controller behavior.

Recommended:
- keep I limited or frozen before airborne
- use the provided `airborne` gate to reduce windup

## 8. What to log

Use the CSV debug output and review these fields:

- corrected roll/pitch angle
- desired roll/pitch rates
- measured roll/pitch/yaw rates
- roll/pitch/yaw command outputs
- saturation flag

If saturation is frequent, mixer authority or output limits may need adjustment before more PID gain.
