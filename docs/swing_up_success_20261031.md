**English** | [한국어](ko/swing_up_success_20261031.md)

# Five-Second Balance Record After Automatic Swing-Up — 2026-10-31

This document records the five-second test at 20:30 KST and the power warning observed at that time. After a later 15-second test, the pendulum was found to remain upright without control and to stop at a tilted position even with power off, indicating mechanical resistance. The earlier five-second record therefore must not be used as proof of control-only performance. The latest interpretation and saved configuration are documented in the [15-second record and mechanical-resistance review](hold15_success_20261031.md).

**In the test that began at 20:30:48 KST on 2026-10-31, 5.08 continuous seconds within ±5° were confirmed after automatic swing-up. The test ended once the success condition was met, and zero actual and target motor velocity were confirmed.**

Experiment ID: `20261031T113048Z-2c07fdc8`.
This was one physical run that met the criterion; it did not validate long-term holding or repeatability. No further physical test was started immediately after the result.

## Physical Validation Results

- Downward start → first capture at approximately 6.81 s → balance mode for approximately 5.26 s.
- Continuous interval within ±5° in the raw CSV: approximately 6.97–12.05 s, **5.079998849 s**.
- The independent approximately 20 Hz mode log also confirmed **5.0461 continuous seconds** within ±5°.
- Angle RMS in balance mode was approximately **1.92°**; velocity saturation in balance mode was 0%.
- Total test duration was approximately 12.05 s. Maximum actual motor velocity was 240 mm/s.
- Maximum recorded absolute angular velocity was 14.778 rad/s, within the existing 16 rad/s limit.
- Color-marker detection was 100%, with zero tracking losses. The video contains 362 frames with zero recording drops.
- The original video confirmed the downward start and an upright interval from approximately 7 to 12 seconds, with no visible hand contact in the reviewed frames. The angle decision used raw video state, not predicted values.
- After completion: IDLE, cart position 169.312 mm, and zero actual and target velocity. Homing, the watchdog, and Raspberry Pi power diagnostics were healthy; there was no RP2040 fault, ESTOP, or limit input.

After the immediate standstill confirmation, a history of **undervoltage and throttling events** was newly detected on the Raspberry Pi while the records were being organized, and the system entered a power-protection FAULT. Actual and target velocity remained zero; this protection was not cleared and no reboot command was sent to the Raspberry Pi. The healthy power snapshot immediately after the run and the later warning snapshot were preserved separately.

The Raspberry Pi was subsequently found to have rebooted. The cause was not established, and a zero power-history bit in the new boot was not treated as evidence that the power issue had been repaired. The dashboard and `localhost:8000` connection were restored with a review lock applied before startup for the previous power warning. **At that point, the system was in RP2040 ESTOP, was not homed, and had zero actual and target velocity.** No ESTOP clear, homing, or further test was performed at this stage. The later lock review and rehoming, after the user's power inspection and readiness confirmation, are recorded separately in the 15-second document referenced above.

The decision jointly checked `balance` mode, valid raw tracking, camera-state age, and continuous samples. RMS over the complete swinging motion was kept distinct from RMS in balance mode.

## Saved Configuration

All six PID gains retained their existing values.

| Item | Value |
| --- | --- |
| Angle PID | 26000 / 0 / 3300 |
| Position PID | 0.0000077 / 0 / 0.0001875 |
| Swing-up energy gain | 44 |
| Target energy margin | 0.045 |
| Energy-calculation timing compensation | 0 s |
| Catch angle / angular velocity | 0.22 rad (±12.6°) / 1.5 rad/s |
| Control headroom at capture | 20% |
| Balance-only camera-delay compensation | 0.020 s |
| Balance-only angular-rate alignment | 0.030 s |
| Full swing-up prediction | Off |
| Model damping / friction | 0.06 / 0.36 |
| Startup acceleration | 200 mm/s² for 0.3 s |
| Cart return position/velocity gains | 3.2 / 3.0 |
| Swing-up / balance velocity | 240 / 240 mm/s |
| Swing-up / balance acceleration | 1500 / 2000 mm/s² |
| Center / allowed travel | 256.369 mm / ±160 mm |

The existing physical, tracking, limit, watchdog, time, and angular-velocity protections remained enabled. Existing camera-to-rail coordinates and the color-marker profile were retained. Before each test, the cart moved at low speed through 260 mm and approached 244.35 mm from the right. The actual initial position for this run was 244.313 mm.

SHA-256 of `config.local.yaml` on the Raspberry Pi and PC:
`fba0af4f74db2db1c5786776975bbfb1b7463783ad72da63f3654dfe45f96ab6`.

## Isolated Change Sequence

| Condition | Physical result |
| --- | --- |
| Gain 32, margin 0.025, catch 0.22 | Maximum rise 163.64°, no capture |
| Gain 44, margin 0.025 | Maximum rise 164.13°, no capture |
| Gain 44, margin 0.045 | Three captures; longest balance 0.849 s |
| Repeat under the same condition | Three captures; longest balance 0.372 s |
| Apply only 30 ms energy timing compensation | Maximum rise 163.85°, no capture → restored to zero |
| Zero energy compensation + balance-only compensation | One capture; successful 5.08 continuous seconds within ±5° |

Increasing gain alone did not materially resolve insufficient rise in this comparison. Raising the target energy margin produced three captures in each of two tests, but balance duration remained short. In those two records, the lag between angle and filtered angular velocity was estimated at approximately 29 ms.

Timing settings that had coupled energy calculation and balance feedback were separated. The new `balance_rate_lag_s` operates only during balance. If omitted or null, it follows `energy_rate_lag_s` as before, preserving existing configuration behavior. Swing-up retains the previous angle, angular velocity, and energy calculations. Only in balance are measurement times aligned before the controller predicts angle and angular velocity to the current time using previously permitted motor commands and the acceleration-limited model. Cart position, cart velocity, raw measurement timestamps, and safety-decision inputs are unchanged.

**All 278 tests passed.** New tests confirm that balance-only compensation does not alter energy output or capture eligibility, matches the existing predictor, and respects rate-lag and prediction-horizon limits. Independent reprocessing of the two preceding physical videos reduced angle-estimation RMS error from 2.47°/3.20° to 0.312°/0.330°, respectively. This reprocessing is separate from the physical success decision.

The preparation utility operates only when the explicitly reviewed configuration SHA-256 matches the configuration hash in the manifest. Four validations that reject configuration or control-source changes were also included. Existing low-speed position return, a single foreground test, unexpected-fault handling, and time limits were retained.

During the first preparation attempt, confirmation that the service had stopped timed out, but position moves remained at zero. After shutdown was confirmed, the server and preparation utility were restarted with their parent processes separated, and position return completed. No ARM/START or test actuation occurred during that recovery.

## Originals and Restoration Materials

- Success conditions and file hashes: `experiments/maintenance/balance-only-separated-20261031.manifest.json`.
- Raw CSV and mode log: `experiments/logs/20261031T113048Z-2c07fdc8.{csv,monitor.json}`.
- Summary: `experiments/summaries/20261031T113048Z-2c07fdc8.json`.
- Original video: `experiments/videos/20261031T113048Z-2c07fdc8.avi` on the Raspberry Pi.
- Standstill confirmation immediately after the result: `experiments/maintenance/successful-swingup-final-state-20261031.json`.
- Later power-protection state: `experiments/maintenance/successful-swingup-postcheck-power-fault-20261031.json`.
- Latest locked state after reboot: `experiments/maintenance/successful-swingup-after-reboot-locked-20261031.json`.
- Previous control code: `experiments/maintenance/pid_controller.before-balance-rate-separation-20261031.py`.
- Previous local configuration: `config.before-balance-only-separated.20261031.yaml` and dated backups on the Raspberry Pi.

A follow-up test should start from this condition and isolate each changed variable in a new record. No scheduled or repeated ARM/START run remains.
