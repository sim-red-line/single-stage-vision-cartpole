**English** | [한국어](ko/capture240_review.md)

# 240 mm/s Expanded-Test Review and Inspection Record

Status: **User approval received. The 0.5-second velocity-and-stop check passed. Swing-up results are under review.**

Before the expanded test, the motor was stopped at 264.375 mm. Under the previous 160 mm/s and center ±120 mm limits, the physical automatic swing-up had switched into balance control, but the longest PID interval was approximately 0.78 seconds and a continuous five-second upright hold had not yet been achieved.

| Item | Previous | Approved test range |
| --- | ---: | ---: |
| Cart velocity limit | 160 mm/s | 240 mm/s |
| Travel limit about center | ±120 mm | ±160 mm |
| Swing-up acceleration | 1,500 mm/s² | 1,500 mm/s² |
| Balance-control acceleration | 2,000 mm/s² | 2,000 mm/s² |
| Independent RP2040 operating-velocity ceiling | 600 mm/s | 600 mm/s |

Candidate configuration `config.capture240.candidate.yaml` was applied to `config.local.yaml`. It contains the approved PID gains Kp=26000 and Kd=3300 for angle, and Kp=0.0000077 and Kd=0.0001875 for cart position. Both integral gains are zero. Camera and rail calibration coordinates, homing, limits, ESTOP, watchdog, and tracking protections remain unchanged.

## Braking Check

With the current conservative braking acceleration of 1,000 mm/s², latency of 80 ms, and margin of 25 mm unchanged, the stopping headroom required at 240 mm/s is:

`240²/(2×1000) + 240×0.08 + 25 = 73 mm`

The proposed center-relative travel range is 96.369–416.369 mm and is clipped to its intersection with the existing software rail range of 10–490 mm. At an outward velocity of 240 mm/s, the braking condition becomes active approximately 87 mm from the center. Because this calculation was not itself a validation of physical braking at 240 mm/s, a short physical check was performed first.

In physical check `20261031T062011Z-b59d0b8f`, the motor reached 240.0 mm/s. The distance from the recorded camera position at the braking threshold, 344.359 mm, to the final motor position, 368.213 mm, was 23.854 mm, within the calculated 73 mm allowance. This value includes camera and communications delay and is not a separate precision stopping-distance measurement. After stopping, the camera/motor position difference was 0.788 mm and both actual and target velocity were zero. Both markers had a 100% detection rate; 16 video frames were recorded with zero drops; and no power, limit, or watchdog fault occurred. Only the intended 0.5-second timeout stop was verified before it was cleared.

The cart then returned in 20 mm/s POS mode from 368.213 -> 312.263 -> 256.350 mm, with a confirmed stop after each segment. Coordinate calibration and homing were not performed.

## Approved Procedure

1. Confirm again that actual and target motor velocity are zero, the pendulum is down, and tracking, power, homing, and the watchdog are healthy; then apply the candidate configuration. Restarting the dashboard does not itself ARM the system.
2. Record a check lasting no more than 0.5 seconds. Use 1,000 mm/s² startup acceleration, a 0.24-second startup pulse, energy gain 0.1, and zero cart position/velocity correction gains. Use the same travel and braking protections, with a pre-start GO confirmation that expires after 60 seconds.
3. Verify achieved velocity, stopping distance, agreement between camera and motor position, tracking, power, limits, watchdog, and final actual and target velocity of zero. Do not proceed if any result is abnormal.
4. Return to center with the validated 20 mm/s POS mode. Split the move into two segments if necessary; each segment must remain at most 100 mm and point toward the center. Do not recalibrate rail coordinates or rehome.
5. If the check passes, record a swing-up test lasting no more than 30 seconds with energy gain 32, target margin 0.05, and cart correction gains 3.2/3.0. If the controller transitions, verify a continuous five-second hold within ±5°; up to 5.5 seconds of additional observation is allowed for this verification. Send STOP and confirm standstill at the end.

The expanded configuration requires an explicit `--commissioning-profile capture240`. The default profile continues to reject settings above 160 mm/s or ±120 mm. After the code change, **all 182 Raspberry Pi tests passed**. Candidate validation without motor actuation also confirmed rejection by the original default profile and dynamic braking in the narrower range.

The user explicitly approved this expanded test while the area was clear, the pendulum was down, 24 V power was on, and the apparatus was under direct observation. Declaring swing-up success requires a separate continuous five-second hold within ±5°; passing the velocity check alone is not claimed as success.
