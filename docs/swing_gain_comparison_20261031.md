**English** | [한국어](ko/swing_gain_comparison_20261031.md)

# Fixed-PID Swing-Up Gain Comparison — 2026-10-31

At the user's request, all six PID gains were held fixed while only the energy gain was compared at 32 → 40 → 48. **The target of five continuous seconds of balance within ±5° was not achieved.** Angular-velocity protection activated during the first gain-48 test, so repeat testing at that gain was stopped.

## Comparison Conditions and Physical Results

- PID: angle 26000 / 0 / 3300; position 0.0000077 / 0 / 0.0001875.
- 240 mm/s, center ±160 mm, swing-up acceleration 1500 mm/s², balance acceleration 2000 mm/s².
- Energy margin 0.025, startup acceleration 200 mm/s² for 0.3 seconds, position/velocity return gains 3.2/3.
- Prediction, energy timing compensation, and the near-upright centering option were all zero.
- Before every run, the cart passed through 260 mm and approached 244.35 mm from the right. The actual initial position was 244.313 mm in all seven tests. A stationary downward pendulum, live color markers, and power state were verified.
- The existing camera-to-rail mapping was retained. The current video start coordinate, 245.43–246.21 mm, differs from 243.54 mm in the previous best test, so these were not perfectly identical environments.
- Each test allowed up to 30 seconds, with a 35.5-second total limit including balance verification. Zero actual and target velocity were confirmed after every run. Video, CSV, and actual control-mode logs were preserved.

Rise angle is measured with downward as 0° and upright as 180°. Balance metrics use only valid video samples in actual `balance` mode. Mode boundaries have approximately 50 ms sample resolution.

| Gain | Experiment ID | Maximum rise | Capture | Longest balance interval | Continuous within ±5° |
| --- | --- | ---: | --- | ---: | ---: |
| 32 | 20261031T084851Z-1f73ccf4 | 168.56° | No | 0 | 0 |
| 32 | 20261031T085149Z-ea593eea | 179.96° | Yes | 6.15 s | 4.03 s |
| 32 | 20261031T085623Z-1a4aa5eb | 170.77° | No | 0 | 0 |
| 40 | 20261031T085919Z-68e16597 | 164.77° | No | 0 | 0 |
| 40 | 20261031T090243Z-e7a51200 | 178.66° | No | 0 | 0 |
| 40 | 20261031T090522Z-dfdc17d4 | 177.78° | No | 0 | 0 |
| 48 | 20261031T091208Z-4542d2ce | 179.66° | Yes | 0.59 s | 0.42 s |

The gain-32 capture test had 2.34° RMS during balance and approximately 5.1% velocity saturation. It ended at the 35.5-second overall limit while still in balance, but its continuous interval within ±5° was only 4.03 seconds. This must not be confused with the 4.55-second record from the previous best experiment.

Median rise angle increased from 170.77° at gain 32 to 177.78° at gain 40, but capture rates were 1/3 and 0/3, respectively. The sample is small and variable, so it does not establish that increasing gain is effective. In particular, near-upright passages at gain 40 were accompanied by residual cart velocity or insufficient rail headroom. The 1.5 rad/s capture angular-velocity condition was not widened. Runs without capture were not used to score PID quality.

## Gain-48 Stop and Camera-Timestamp Correction

The first gain-48 test stopped at approximately 28.75 seconds with `swing-up angular-speed limit exceeded`. The recorded angular velocity was −16.0405 rad/s, above the existing 16 rad/s limit. The run was not retried, the limit was not raised, and the host remained in FAULT.

When all 862 original frames were reprocessed at fixed 30 FPS intervals, there were zero detection losses and maximum absolute angular velocity was 15.1363 rad/s. Inverting the estimator equations gave 40.8148 ms and 27.3487 ms for the final two receive intervals. The final angle matched the video reprocessing. This indicates that receive-timestamp jitter may have affected angular velocity. **Fixed-interval video reprocessing is not evidence of the actual capture interval, so it does not establish that the original limit event was a false positive.** Color markers remained continuously detected and the pendulum was descending in the final video.

With the motor stationary, the camera was opened independently. `CAP_PROP_POS_MSEC` from OpenCV 4.14.0 / V4L2 agreed with the Raspberry Pi monotonic clock and preceded receive completion by approximately 5.49–10.35 ms. The capture interval itself varied by approximately 32/36 ms, so a fixed 30 FPS interval was not imposed.

`CameraCapture.read()` was changed to use the driver's capture timestamp on V4L2. Other backends retain the previous monotonic receive timestamp. Invalid or non-increasing driver timestamps are treated as frame errors. Long intervals from dropped frames and the true age of stale frames remain visible so the existing tracking and delay protections still apply.

The meaning of the value returned by V4L2 was checked against the [OpenCV implementation](https://github.com/opencv/opencv/blob/4.14.0/modules/videoio/src/cap_v4l.cpp#L1739) matching the installed version. Because timestamp clocks and exposure reference points can differ by driver, the behavior was separately validated on the physical device in accordance with the [Linux V4L2 documentation](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/buffer.html).

All 260 tests passed. After the change, 726 stationary frames were analyzed: capture intervals were 31.982–36.018 ms and receive latency was 5.564–10.304 ms. All 60 live-state checks reported healthy tracking, valid state, the FAULT still latched, and zero actual and target velocity. This does not demonstrate the effect of the change while the motor is moving.

## Stationary State and Validation Plan Immediately After Diagnosis

Immediately after the diagnosis, both the on-disk and runtime energy gain were 32, while all PID gains and physical limits were unchanged. The cart was at 387.225 mm, actual and target velocity were zero, and homing and the watchdog were healthy. When the dashboard was restarted during validation, the same FAULT was applied before connecting hardware; an ordinary restart did not clear the lock.

The next comparison was planned to record the capture-timestamp correction as a distinct condition. The new manifest is `experiments/maintenance/capture-clock-validation-20261031.manifest.json`. The plan was to check baseline gain 32 once, review the result after a normal stop, and compare gain 40 only if that review passed. No gain-48 retry was planned. Before a response confirmed observation and readiness, no FAULT clear, position return, or control start would occur.

The preparation utility's `--manifest` verifies file hashes for the requested condition. Existing manifests were not modified; requesting an old condition by default rejects the changed camera source. The existing behavior of not automatically clearing unexpected faults was also retained.

Original comparison materials are in the manifest/analysis files under `experiments/maintenance/{baseline-repeats,swing-gain40,swing-gain48}-20261031` and in the CSV, monitor, and summary files for each experiment ID. Camera diagnosis, reprocessing, and FAULT preservation are recorded in the `camera-clock-*` and `angular-fault-*` files in the same directory.

## Physical Comparison After the Timestamp Correction — 18:42–18:45 KST

After the user confirmed `area clear · pendulum down · 24 V ON · observing`, the review record for the existing angular-velocity fault was saved and the fault was explicitly cleared. Gain 48 was not retried. The same limits and PID gains were retained, and before every run the cart returned to the same position at no more than 20 mm/s. This did not establish that the earlier fault was caused by receive-timestamp error.

| Gain | Experiment ID | Actual initial position | Maximum rise | Capture | Maximum recorded angular velocity |
| --- | --- | ---: | ---: | --- | ---: |
| 32 | 20261031T094243Z-fbd28fe9 | 244.313 mm | 165.10° | No | 15.192 rad/s |
| 40 | 20261031T094511Z-45a3a9a3 | 244.313 mm | 165.65° | No | 15.424 rad/s |

Both tests stopped at the 30-second limit, with no new angular-velocity protection stop or tracking loss. Marker detection was 100% in both tests, and each video contained 901 frames with zero drops. Neither run entered actual balance mode, so there are no new data with which to evaluate PID quality, and the five-second balance target was not achieved. No effect is generalized from one run per condition.

Reconstructing only the energy-pumping term from the recorded state showed output at or above 95% of the acceleration ceiling for approximately 34.2% of the gain-32 run and 38.1% of the gain-40 run. Applying gains 32 and 40 to the same state arrays increased the mean absolute pumping command by approximately 6.9–7.2%. A 25% gain increase did not produce a proportional output increase because of `tanh` saturation. This is an analysis of the **calculated command**, not a measurement of actual motor force or improvement on another trajectory.

Requests above the velocity ceiling occurred approximately 4.6% and 4.7% of the time, respectively. Additional command limiting by rail braking then occurred approximately 0% and 0.1% of the time. Additional braking intervention was rare in these two tests. The results provide no evidence that simply increasing gain improves capture; energy target, injection timing, friction loss, and other causes require further identification. Those settings were not changed in this comparison.

After the comparison, only the expected 30-second timeout was cleared and the runtime gain was restored to 32. The dashboard was restarted while stationary to clear the old timeout display. **The final state was IDLE, cart position 201.825 mm, zero actual and target velocity, healthy tracking, and no error.** The six PID gains and on-disk configuration hash matched the baseline. No scheduled or repeated run remained.

Additional records are stored in `capture-clock-validation-20261031.{manifest,analysis}.json`, `capture-clock-fault-review-20261031.json`, `capture-clock-final-state-20261031.json`, and `capture-clock-pump-analysis-20261031.json`. The reproducibility script is `experiments/maintenance/analyze_capture_clock_pump_20261031.py` and does not open the hardware.

## Additional Request: Wider Catch-Angle Range

In response to the user's request to widen the catch range, only `controller.swing_up.capture_angle_rad` was changed in the local and Raspberry Pi `config.local.yaml` from **0.14 to 0.22 rad (±8.0° to ±12.6°)**. The dashboard was restarted while stationary and the applied value was confirmed. This extends capture to the existing PID-entry ceiling of 0.22 rad. The 1.5 rad/s angular-velocity condition, 20% control headroom, all six PID gains, energy gain 32, and every velocity, acceleration, travel, and release-angle limit remained unchanged.

Bidirectional regression tests were added to confirm capture for a state moving toward upright with adequate control headroom, and rejection for states outside the range, above the angular-velocity limit, or without sufficient headroom. All **130 related control tests passed**. After application, runtime `capture_angle_rad=0.22`; the system was IDLE with zero actual and target velocity, valid color-marker tracking, no error, and no active experiment. The before/after configuration and runtime verification are in `experiments/maintenance/catch022-applied-20261031.json`.

This request completed configuration application and stationary validation only; it does not mean that physical capture succeeded under the new range. The preceding two runs approached no closer than approximately 14.9°/14.3° from upright, so an identical trajectory would still need to move farther to enter the expanded 12.6° range. The earlier gain-comparison manifests and baseline configuration hashes were preserved as records of their original conditions.

## Subsequent Recorded Result

After additional swing-up comparisons, gain 44 and target energy margin 0.045 were used while energy timing compensation remained zero. In test `20261031T113048Z-2c07fdc8`, which applied angular-rate alignment and video-delay compensation separately in the balance stage, a **continuous 5.08-second interval within ±5°** was confirmed before stopping with zero actual and target velocity. All six PID gains and physical operating limits were retained. The settings and original validation materials are documented in the [recorded result](swing_up_success_20261031.md). Settings, failures, and stopping positions in the preceding sections remain records of their respective times.

After rehoming, a **continuous 15.01-second interval within ±5°** was recorded with gain 46 and margin 0.045. However, mechanical resistance was then found: the pendulum remained standing without control and at a manually tilted pose. The control effect was therefore reclassified as unconfirmed. See the [15-second record and mechanical-resistance review](hold15_success_20261031.md) for the latest interpretation and saved configuration.
