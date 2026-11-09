**English** | [한국어](ko/shaft_corrected_retest_20261031.md)

# Baseline Retest After Shaft Correction — 2026-10-31

This document records the 3.45-second test from October 31. The November 1 position-P comparison and 15.03-second record after position-dependent angle-reference correction are documented in the [latest drift review](position_drift_review_20261101.md).

**The test at 23:09:41 KST captured four times after automatic swing-up, and its longest continuous hold within ±5° was 3.45 seconds. The 15-second target was not achieved.** Experiment ID: `20261031T140941Z-608e2f0d`.

The run proceeded after the user reported correcting a misaligned shaft and confirmed free rotation, the pendulum down, the area clear, 24 V power on, and direct observation. The earlier observation that the pendulum could remain upright does not by itself establish shaft binding. The exact condition of the shaft was not inspected directly through remote access.

## Measurements

| Item | Result |
| --- | --- |
| First capture | Approximately 13.73 s |
| Number of captures | 4 |
| Longest balance-mode interval | Approximately 3.71 s |
| Longest continuous interval within ±5° in raw CSV | **3.449992680 s**, approximately 31.77–35.22 s |
| Continuous interval within ±5° in independent approximately 20 Hz log | 3.4418 s |
| Total test duration | Approximately 38.00 s |
| Maximum RP2040 velocity | 240 mm/s |
| Maximum recorded angular velocity | 15.386 rad/s, within the existing 16 rad/s limit |
| Tracking | 100% detection, zero losses |
| Video | 1,140 frames, zero recording drops |
| End condition | Cumulative swing-up timeout, actual and target motor velocity 0 |

The decision jointly used actual `balance` mode, valid raw tracking, video state no older than 120 ms, and continuous samples. Angle statistics over the entire run were not used as balance-hold duration.

## Observation Before Balance Loss

During both longer balance intervals, the cart moved left and reached the protective-braking region near 121 mm. Commands that continued to request motion left were limited to zero or a small velocity, after which the pendulum tilted and left balance mode. RP2040 positions near the end of these intervals were 120.825 mm and 121.088 mm. The other two captures were shorter.

The existing travel range was center 256.369 mm ±160 mm. Even within that range, the system decelerated according to a 25 mm margin plus velocity and delay. This protection was not relaxed. The next review targets were cart centering and the angle zero after shaft correction. This single record does not establish one root cause or demonstrate improvement from any particular gain.

## Retained Conditions and Post-Stop Observation

Swing-up gain 46, energy margin 0.045, all six PID gains, camera calibration, the color-marker profile, and protection limits were retained. The angle PID was `26000 / 0 / 3300`, and the position PID was `0.0000077 / 0 / 0.0001875`. The cart returned to the initial position of 246.263 mm at no more than 20 mm/s. Rail-image coordinates were not recalibrated.

The test utility's full-run limit was 45.5 seconds. The run ended on the internal cumulative swing-up limit of 30 seconds. Because time in balance mode is excluded from cumulative swing-up time, the full run lasted approximately 38 seconds. The final software state was `FAULT: swing-up time limit`, with control inactive. Cart position was 247.613 mm; actual and target velocity were zero; there was no RP2040 fault, ESTOP, or limit input; homing and the watchdog were healthy; and Raspberry Pi power diagnostics were `0x0`.

After the result returned, state was read for 10.01 seconds without further actuation. From the start of this observation, the pendulum was already near the downward position at approximately 179.66°, and it ended at approximately 179.70°. **This observation did not record the fall immediately after a STOP from upright and was not a PID-OFF/PID-ON comparison from the same initial state.** It does not substitute for such a comparison.

## Preserved Materials

- Conditions and source hashes: `experiments/maintenance/shaft-corrected-20261031.manifest.json`.
- Raw CSV and mode log: `experiments/logs/20261031T140941Z-608e2f0d.{csv,monitor.json}`.
- Summary: `experiments/summaries/20261031T140941Z-608e2f0d.json`.
- Original and playback video: `experiments/videos/20261031T140941Z-608e2f0d.{avi,mp4}`.
- Analysis, post-stop observation, and final-stop state: `shaft-corrected-20261031.analysis.json`, `shaft-corrected-post-stop-20261031.json`, and `shaft-corrected-final-state-20261031.json` in the maintenance directory.
- Sixteen files including preparation data, logs, and video were preserved in `shaft-corrected-completed-20261031.tar.gz`, and SHA-256 checks matched the PC copy.

Control-source and configuration hashes were compared with the previously validated versions. The configuration SHA-256 remained `3a2d7ee57c7092d791e5d86a2421b370eafe592da285e54f32df2ff1d2eafd40`. No additional ARM/START command or scheduled run remained after the test.
