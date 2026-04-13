# Parameter Table

## Default angle loop parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `maxTiltDeg` | 18.0 | Maximum commanded roll/pitch angle in stabilize mode |
| `angleKpRoll` | 4.0 | Roll angle error to desired roll-rate gain |
| `angleKpPitch` | 4.0 | Pitch angle error to desired pitch-rate gain |
| `maxRollRateDegPerSec` | 180.0 | Outer-loop roll-rate limit |
| `maxPitchRateDegPerSec` | 180.0 | Outer-loop pitch-rate limit |

## Default rate loop parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `rollKp` | 0.015 | Roll rate proportional gain |
| `rollKi` | 0.010 | Roll rate integral gain |
| `rollKd` | 0.0008 | Roll rate derivative gain |
| `pitchKp` | 0.015 | Pitch rate proportional gain |
| `pitchKi` | 0.010 | Pitch rate integral gain |
| `pitchKd` | 0.0008 | Pitch rate derivative gain |
| `yawKp` | 0.010 | Yaw rate proportional gain |
| `yawKi` | 0.005 | Yaw rate integral gain |
| `yawKd` | 0.0000 | Yaw rate derivative gain |
| `integralLimit` | 80.0 | Anti-windup clamp |
| `rollPitchOutputLimit` | 0.5 | Max roll/pitch command magnitude |
| `yawOutputLimit` | 0.3 | Max yaw command magnitude |
| `maxYawRateDegPerSec` | 150.0 | Max commanded yaw rate |

## Authority scaling

```cpp
const float minAuthority = 0.35f;
authority = minAuthority + (1.0f - minAuthority) * throttle;
```

This avoids losing almost all stabilization authority at low throttle.

## What to adjust first

1. `rollKp`, `pitchKp`
2. `yawKp`
3. `rollKd`, `pitchKd`
4. `rollKi`, `pitchKi`
5. `yawKi`

## Warning signs

### Kp too low
- mushy correction
- weak return to level
- poor disturbance rejection

### Kp too high
- fast oscillation
- twitching or hot motors

### D too high
- noisy output
- harsh motor command chatter

### I too high
- slow overshoot build-up
- controller keeps “leaning into” old error
- worse behavior near saturation or on ground
