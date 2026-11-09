**English** | [한국어](ko/position_drift_review_20261101.md)

# Position-Drift Comparison and Angle-Reference Correction — 2026-11-01

**The second test with position-dependent angle-reference correction recorded 15.03 continuous seconds within ±5°. During that interval, the cart moved from 215.1 to 324.075 mm in RP2040 coordinates and stopped normally at 238.313 mm.** The center reference is 256.369 mm. The experiment ID is `20261101T053856Z-adde3c2c`, started at 14:38:56 KST.

The first test under the same corrected condition did not maintain balance for long. This result does not establish repeatability or isolate the PID-only effect against an uncontrolled baseline. It is interpreted as a measured hold interval in which the cart did not drift into the left protective-braking region.

## Position-P Comparison

The user reported that the pendulum remained balanced while the cart kept moving toward one side. With the previous position P gain, a 100 mm displacement from center changed the target tilt by only approximately 0.044°. The first comparison kept the angle PID, swing-up gain, and protection limits unchanged and doubled only position P.

| Condition | Confirmed continuous hold within ±5° | Result |
| --- | --- | --- |
| Previous reference, October 31 test after shaft correction | 3.45 s in raw CSV | Reached the left protective-braking region |
| Position P 0.0000077 → 0.0000154 | Approximately 2.71 s in the independent mode log | Drift unresolved; restored original gain |
| Original gain + new angle reference, first test | 0 s | Two captures, each approximately 0.37 s |
| Repeat under the same condition | **15.03 s in raw CSV** | Met the criterion on the fourth capture |

The number of samples per condition is small, and the earlier baseline came from a different pre-reboot test. This table must not be used as a statistical comparison of gain quality. The active position P gain is the **original value, 0.0000077**. Angle PID `26000 / 0 / 3300`, position I/D `0 / 0.0001875`, energy gain 46, and energy margin 0.045 were retained.

## Position Dependence Found in the Angle Reference

During two low-speed homing runs, video state was recorded with the pendulum hanging down. In constant-velocity segments, the downward reference angle repeatedly changed with position at approximately **−0.4626° per 100 mm**. Because the result also differed by travel direction, paired data from both directions were averaged before estimating the position-dependent reference. Residual tilt after stopping was excluded from the fit.

The paired-average residual RMS between the first-record fit and the second, normal-length homing record was 0.0267°. **This measures agreement between the two records, not absolute accuracy against an external vertical reference.** The first homing distance was not adopted, and image-derived position was used for the angle fit.

An optional position-dependent term was added to the previous fixed angle reference.

```text
angle reference(x) = angle at reference position + slope × (x − 256.369 mm)
```

The angle at the reference position was adjusted by +0.1334° from the previous value. Relative to the old fixed reference, the maximum change across the rail is approximately 1.319°. The code permits no more than 3° of position-dependent correction and invalidates state when a non-finite value or an out-of-range correction is supplied. The default slope of zero preserves the previous angle calculation.

**Rail-position mapping, image coordinates for both rail ends, color-marker HSV values, and the brightness reference were not changed.** The new active profile is `angle-reference-20261101`. The correction is applied to angle before angular velocity is calculated, reducing the false angular velocity otherwise produced by cart motion.

## Independent Validation of the Final Test

- In the raw CSV, approximately 30.330–45.360 seconds: **15.029995731 seconds**, 1,504 samples.
- The independent approximately 20 Hz mode log also met the continuous condition for 15.0441 seconds.
- Angle RMS during the interval was **0.444°** and maximum absolute angle was **3.678°**.
- Cart range: RP2040 215.1–324.075 mm; camera 215.28–330.83 mm.
- Maximum actual velocity over the full test was 240 mm/s; maximum recorded angular velocity was 15.194 rad/s.
- Detection rate was 100%, with zero tracking losses. The video contains 1,361 frames with zero recording drops.
- After completion: IDLE, control inactive, zero actual and target motor velocity, and Raspberry Pi power diagnostics `0x0`.
- No RP2040 fault, ESTOP, or limit input; homing and the watchdog were healthy.

The angle decision jointly checked actual balance mode, valid raw tracking, video state no older than 120 ms, and sample continuity. The existing five-second success flag was not used as evidence for 15 seconds. During a separate ten-second observation after the test, motor velocity remained zero and the pendulum stayed at approximately 0.27–0.39°. The result therefore does not exclude the influence of manual support or static friction.

## Code Validation and Preservation

**All 322 tests passed** in the Raspberry Pi runtime environment, with one warning from the existing Starlette test client. New validation checks that a vertically oriented pendulum moving across the rail does not acquire a false angle or angular velocity under either image direction, upright/downward orientation, or angle wrapping; that rail-position mapping is unchanged; that existing profiles remain compatible; and that invalid correction is rejected. Production velocity, acceleration, angular-velocity, tracking, limit, and watchdog protections were retained.

- Actual log, mode log, summary, and original AVI/MP4: materials identified by `20261101T053856Z-adde3c2c` under `experiments/logs`, `experiments/summaries`, and `experiments/videos`.
- Revalidation: `experiments/verify_balance_recording.py` and `experiments/maintenance/20261101T053856Z-adde3c2c.verified.json`.
- Comparison data, angle fits, source backups, and restoration of approved gains: the `position-P2-*`, `position-angle-fit-*`, and `angle-correction-*` records in the maintenance directory.
- SHA-256 matched all 71 files in the PC copy. The original archive is `position-tuning-completed-20261101.tar.gz`.
- The previous configuration is `config.before-position-angle.20261101.yaml`; previous code is preserved in `*.before-position-angle-20261101.py`.

Current configuration SHA-256:
`3a84e16634d67251d5325f0b6eeecca14366fe86e7255f033af0350de9473f10`.
When restoring, match both the profile-selection configuration and code version.

After the user confirmed a manual Raspberry Pi reboot, the server was restored. The first homing result, 402.263 mm, differed from the previous approximately 513 mm and was rejected; the system was stopped with ESTOP. After the user explained the first-homing anomaly and the need to rehome, the second result of 513.113 mm and the image range were verified before testing. **The firmware cause of the first-homing problem was not corrected in this work.** No additional run or scheduled ARM/START task remains. Dashboard connectivity was restored at `http://localhost:8000/` on the PC.
