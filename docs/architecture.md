# Architecture and control boundaries

The project separates the real-time control path from non-real-time recording and review.

```text
real-time path
camera -> HSV tracker -> state estimator -> cascaded PID -> safety limiter -> serial -> RP2040

recording path
camera/control samples -> bounded queues -> CSV/AVI writers -> measured JSON summary
```

The camera thread owns capture and image processing. It timestamps frames with a monotonic clock at acquisition, publishes no state when required markers are missing, and requires 15 consecutive good frames after a profile load. The control thread runs independently at the configured rate and stops on stale or invalid state. CSV and video writes use bounded queues and separate writer threads.

The cascaded controller uses a position PID as the slow outer loop. Its bounded output is a small angle setpoint for the inner angle PID, whose bounded output is signed cart velocity. Both loops use derivative-on-measurement, low-pass derivative filtering, back-calculation anti-windup, timestamp `dt`, and output limits. A final slew limiter enforces the Raspberry Pi acceleration limit. The RP2040 independently applies another acceleration and braking constraint.

The host safety states are `DISCONNECTED`, `UNHOMED`, `CALIBRATION_REQUIRED`, `IDLE`, `ARMED`, `RUNNING`, `FAULT`, and `ESTOP`. Reaching `RUNNING` requires a connected/homed RP2040, a structurally complete and live-verified calibration, and reliable current tracking. `FAULT` and `ESTOP` are latched. ESTOP clearing invalidates homing.

The Raspberry Pi compares camera position and the firmware's step-estimated position in every log row. Without an encoder this can reveal a discrepancy but cannot prove actual motor torque or guarantee that commanded steps were executed.

