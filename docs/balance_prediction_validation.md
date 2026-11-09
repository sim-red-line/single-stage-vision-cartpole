# Balance prediction and room-light validation — 2026-10-31

**Historical validation note:** physical experiments were stopped after this
comparison. The 4.55-second baseline configuration was restored on Pi and PC;
balance prediction and the upright centering window were disabled, and both
mode speed limits were set to 240 mm/s. The candidates below are retained only
as historical experiments.

The physical acceptance criterion is an autonomous swing-up followed by at least
five continuous seconds within ±5°, with valid tracking and an actual motor stop.
The previously verified best was 4.549983689 s, ended by a lighting calibration
guard. Offline forecast accuracy alone does not satisfy this criterion.

## Camera and color observations

After the requested full rehome, the firmware measured 513.113 mm and stopped at
256.538 mm. The existing camera rail coordinates remain 512.738 mm; they were
not recalibrated. See `rehome_20261031.md` for the full-home record and abort
behavior.

The operator turned on room lighting. Background brightness still measured
38–40 at camera gain 0. Gain 64 raised it to 102–104 while the exposure remained
44 device units (configured 45), 640×480 MJPG and approximately 30 FPS.
`CameraCapture` now loads optional `camera.gain` and reports its readback.

Recorded moving pink tip masks measured 16–34.5 px² in frames rejected by the
old 35 px² minimum. The `roomlight-g64-20261031` profile uses a 16 px² tip
minimum and a measured background reference of 102. HSV ranges, coordinate
geometry, confidence threshold, ambiguity rejection and lighting protection
retain their previous values.

Validation:

- 2,158 frames across three recordings: zero missing/ambiguous detections and
  zero extra tip candidates using the revised area minimum.
- Centers exactly matched the original tracker wherever the original succeeded.
- Removing the real tip from 146 full-scene frames produced zero false tips.
- Sixty live checks passed before and after loading; gain 64 and the profile
  survived a stopped dashboard restart.
- Subsequent full 30 s physical trials retained tracking.

The profile, gain and source are synchronized between the Pi and PC. Backups
and hashes are retained in `experiments/maintenance/roomlight-config-sync.json`
on the Pi and in the PC review artifact directory.

## Identified response and causal angular forecast

Fitting the acceleration-limited admitted velocity targets against camera
position, with acquisition timestamps reconstructed as command timestamp minus
logged frame age, gave a consistent 2,000 mm/s² servo and 26 ms effective fixed
command-to-image timestamp delay. Position RMS errors were 0.49, 0.97 and
0.84 mm in three independent trials. The 26 ms parameter excludes frame age;
the angular rate estimator has a separate approximately 30 ms delay.

`balance_prediction_camera_latency_s` defaults to zero. The candidate uses
0.026 s with `energy_rate_lag_s=0.03`. It aligns measured angle history with
the filtered angular rate, then propagates to control time using previously
admitted targets, a bounded motor acceleration model, and nonlinear pendulum
dynamics. The motor model continues braking after a target STOP and records
the constant-speed remainder when a target is reached within a control tick.

The forecast is used after raw-vision capture, including derivative-filter
priming at handoff. Energy shaping, raw capture eligibility and release retain
their measured inputs. Rail position, cart velocity and measurement timestamp
remain measured values. Runtime safety continues to receive raw vision.
Forecasts beyond 140 ms or with invalid state latch a stopped fault. Full
swing-up prediction and balance-only prediction are mutually exclusive.

Later recorded frames, shifted by the independently identified effective delay,
provide the comparison reference. Only commands already admitted by each
control timestamp drive the forecast:

| Experiment | Balance samples | Raw angle RMS error | Forecast RMS error | Forecast 95th percentile absolute error |
| --- | ---: | ---: | ---: | ---: |
| `20261031T063236Z-f9688be6` | 453 | 1.446° | 0.278° | 0.532° |
| `20261031T071140Z-7b15ba24` | 382 | 1.668° | 0.276° | 0.549° |
| `20261031T080306Z-0c37b83b` | 144 | 2.853° | 0.339° | 0.612° |

Reproduce using `python -m experiments.replay_balance_prediction` with the CSV
paths. The analysis tool never connects to hardware. Its replay duration budget
is independent of physical trial limits.

## Physical candidate envelope

Maximum speed remains 240 mm/s, swing acceleration 1,500 mm/s², balance/motor
acceleration 2,000 mm/s², and centered excursion 160 mm. The conservative
1,000 mm/s² braking model, software/hardware limits, watchdog, tracking and
power guards remain active. Trials run in the supervised foreground wrapper,
with at most 30 s energy shaping and 5.5 s additional balance verification.

The current speed-reserve candidate targets at most **160 mm/s during swing-up**
and permits **240 mm/s during balance** via explicit
`balance_max_velocity_mm_s`. The default keeps both speeds equal. Handoff
preserves the admitted velocity, and a recovery from faster balance slews back
under the swing acceleration cap. Both balance and swing speed are checked
against the explicitly selected commissioning profile.

An optional `upright_centering_window_rad=0.7` smoothly multiplies the additive
centering term by `min(1, (abs(theta)/window)**2)`. This preserves energy
authority near upright while retaining full centering farther away. Independent
rail braking and hard excursion checks continue to apply. The default window
is zero, which retains the earlier centering behavior.

The raw capture rate candidate is 2.0 rad/s. A recorded pass at −7.13°, +1.741
rad/s and −217.9 mm/s commanded cart velocity required approximately −60.8
mm/s final velocity under the coupled capture check. This fits the existing
20% velocity and acceleration reserve. The same angular state at +240 mm/s
is still rejected by the regression test.

`run_scheduled_swing_up` also accepts an explicit `--kick-acceleration`, default
200 mm/s², bounded to 0–1,500 before any hardware request. The current comparison
candidate is 400 mm/s² for 0.3 s. This changes the initial trajectory while
retaining the commissioned speed, acceleration and travel envelope.

The source passed **240 tests**, with one existing Starlette deprecation warning.
Regression coverage includes unchanged energy commands, raw safety/release,
prediction horizon, retained measurement age and rail state, continuous motor
braking, capture authority, and rejection of invalid startup parameters before
contacting hardware. Additional checks cover separate balance speed authority,
bounded recovery deceleration and commissioning-profile enforcement for the
reserved balance speed.

## Physical results and persistence

| Experiment | Candidate | Maximum rise | Balance result |
| --- | --- | ---: | --- |
| `20261031T075629Z-58feade9` | Gain 48, margin .04, alignment .03, cart PD 3.2/3 | 174.32° | No capture; insufficient coupled velocity reserve |
| `20261031T075825Z-2f63b6f7` | Gain 32, margin .025, alignment 0 | 164.34° | No capture |
| `20261031T080116Z-d4bc1d58` | Gain 48, margin .04, alignment .03, cart PD 3.2/2 | 170.10° | No capture |
| `20261031T080306Z-0c37b83b` | Gain 64, margin .05, alignment .03, cart PD 3.2/3; forecast disabled | 179.95° | PID intervals .91 s and .51 s; continuous ±5° .3615 s |
| `20261031T080945Z-272c0776` | Same energy, balance forecast .026, capture rate 1.5 | 167.56° | No capture; forecast not physically exercised |
| `20261031T081305Z-be099fa1` | Same energy/forecast, capture rate 2.0 | 171.24° | No capture |
| `20261031T081644Z-abb88ea2` | Same settings with startup acceleration 400 | 175.17° | No capture |
| `20261031T082112Z-80e70ccc` | Same settings with .7 rad upright centering window | 176.54° | No capture |

All completed trials above ended with actual and target motor velocities zero.
The expected swing time-limit fault is cleared only after inspecting that
specific reason and stopped hardware. Unexpected faults are retained.

**Before final handoff after physical success:** independently verify the
continuous valid ±5° interval from CSV and mode monitor, inspect the recording,
persist all winning gains and energy/startup parameters on both Pi and PC, and
verify a stopped reload. Energy parameter API updates are memory-only; the
current conservative disk defaults are not yet a proven complete operating
preset. Retain the winning CSV, summary, monitor and locally playable video.
