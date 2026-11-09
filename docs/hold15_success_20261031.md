**English** | [한국어](ko/hold15_success_20261031.md)

# 15-Second Upright Record After Automatic Swing-Up — Control Effect Unconfirmed

In a later clarification, the user explained that when the pendulum was aligned well at upright, it remained standing after release. The user did not believe the shaft was jammed, but reported that it had been misaligned and was subsequently corrected. The earlier manual-hold observation alone does not establish shaft binding. In the [retest after shaft correction](shaft_corrected_retest_20261031.md), performed with the same settings after free rotation was confirmed, the longest continuous hold within ±5° was 3.45 seconds.

**The test that began at 21:26:27 KST recorded 15.01 continuous seconds within ±5°, but the pendulum was later found to remain standing without control and to stop at a manually tilted position. The earlier conclusion that this record demonstrated successful PID balance control is therefore withdrawn.** The experiment ID is `20261031T122627Z-cea4f757`. IDLE state and zero actual/target motor velocity were confirmed after completion. The angle and timing records are valid, but the contributions of mechanical resistance such as shaft friction or binding and of active control were not separated; control performance and repeatability therefore remain unconfirmed.

## Mechanical Resistance Found After the Test

After the test, the user pointed out that the pendulum was still standing upright. The API showed IDLE, `control.active=false`, zero actual and target motor velocity, and an angle near upright. The user then reported that **with 24 V motor power off, the pendulum remained at a slightly tilted position**. This shows that mechanical resistance could support the pose without motor control. The exact component and cause have not been identified.

Shaft clamping, bearing alignment, and nearby contact must be inspected, and gravity-driven free rotation must be confirmed before the control test is evaluated again. No ARM/START or motor-motion command was sent during this inspection. The evidence is stored in `experiments/maintenance/hold15-passive-binding-review-20261031.json`.

## Confirmed Results

- The raw video-angle CSV contains **15.009993926 continuous seconds** within ±5°, approximately from 16.16 to 31.17 seconds.
- Across 1,502 samples in the same interval, angle RMS was **0.650°** and maximum absolute angle was **4.376°**.
- The separate approximately 20 Hz mode log recorded a continuous hold of **15.0055 seconds**.
- First capture occurred at approximately 6.95 seconds. The first two captures were released; the third, at approximately 15.96 seconds, remained in balance mode for about 15.22 seconds and met the final criterion.
- The full test lasted approximately 31.17 seconds. Maximum actual RP2040 velocity was 240 mm/s and maximum recorded angular velocity was 15.787 rad/s. The existing 16 rad/s protection remained enabled.
- Color-marker detection was 100%, with zero tracking losses. The video contains 935 frames with zero recording drops. Review frames confirmed a downward start and upright hold, with no visible hand contact.
- Final cart position was 166.163 mm, actual and target velocity were zero, homing and the watchdog were healthy, and there was no RP2040 fault, ESTOP, or limit input. Power diagnostics after the test and video conversion were `0x0`.
- No additional ARM/START command or repeat test was run after the record.

The decision did not use predicted angle or the existing five-second success flag. It jointly checked raw angle within ±5°, actual `balance` mode, valid tracking, video state no older than 120 ms, and sample continuity. The revalidation script does not open the hardware or API.

## Configuration Change

Compared with the immediately preceding test from the same start position, **only the swing-up energy gain changed from 44 to 46**. The angle PID `26000 / 0 / 3300` and position PID `0.0000077 / 0 / 0.0001875` were unchanged. Target energy margin 0.045, zero energy timing compensation, capture at 0.22 rad and 1.5 rad/s, 20% capture headroom, balance-only 20 ms prediction, and 30 ms angular-rate alignment were also unchanged.

The limits remained 240 mm/s velocity, 1,500 mm/s² swing-up acceleration, 2,000 mm/s² firmware and balance acceleration, center 256.369 mm, and allowed travel ±160 mm. Return to the preparation position was performed at no more than 20 mm/s. Camera and color-marker profiles and rail-image coordinates were not recalibrated. The user also confirmed that the physical camera, rail, and pendulum conditions had not changed.

Gain 46 was saved to `config.local.yaml` on the Raspberry Pi and PC. Saving did not restart the system or actuate the hardware, and the value matched the already active runtime setting. This configuration is not considered validated on a freely rotating pendulum.

- Saved-configuration SHA-256: `3a2d7ee57c7092d791e5d86a2421b370eafe592da285e54f32df2ff1d2eafd40`.
- Previous configuration: `config.before-hold15-gain46.20261031.yaml`.
- The on-disk configuration during the test still contained gain 44; gain 46 was supplied as a test argument. The existing disk hash in the experiment manifest preserves this history.

## Comparison After Rehoming

After the earlier [5.08-second result](swing_up_success_20261031.md), a power warning and reboot occurred. Once the user confirmed that power connections had been checked, the pendulum was down, the area was clear, 24 V power was on, and observation was ready, the existing lock was reviewed and the system was rehomed. The new homed length was 513.263 mm, while the video-coordinate mapping was retained.

| Energy gain | Energy margin | Initial RP position | Result |
| --- | --- | --- | --- |
| 44 | 0.045 | 244.313 mm | Maximum rise 179.75°, no capture |
| 44 | 0.035 | 244.313 mm | Maximum rise 165.22°, no capture |
| 44 | 0.040 | 244.313 mm | Maximum rise 165.30°, no capture |
| 44 | 0.0425 | 244.313 mm | Maximum rise 169.13°, no capture |
| 44 | 0.045 | 246.263 mm | Maximum rise 170.02°, no capture |
| 46 | 0.045 | 246.263 mm | 15.01 seconds within ±5° recorded; mechanical-resistance effect not separated |

The initial position for the final two tests was chosen within the existing center ±20 mm preparation range to match the cart coordinate in the successful video before rehoming. This was not rail-coordinate calibration. Offline comparison of recorded commands and video positions gave a response delay of 22 ms for the earlier result and 21–22 ms after rehoming. These data alone do not identify the reboot or any particular mechanical change as the cause of the failures. One test per condition does not establish repeatable superiority of gain 46.

## Validation and Originals

In the actual Raspberry Pi runtime environment, **all 303 tests passed**, with one deprecation warning from the existing Starlette test client. The test utility that explicitly validates the 15-second target preserves the default five-second behavior; when 15 seconds is requested, it limits the complete run to 45.5 seconds and the parent process to 55 seconds. ARM/START is rejected after the allowed time expires.

- Original record: `experiments/logs/20261031T122627Z-cea4f757.{csv,monitor.json}`.
- Experiment summary: `experiments/summaries/20261031T122627Z-cea4f757.json`.
- Original video and MP4: `experiments/videos/20261031T122627Z-cea4f757.{avi,mp4}`.
- Conditions and results: `experiments/maintenance/hold15-gain46-20261031.{manifest,analysis}.json`.
- Independent revalidation: `experiments/maintenance/verify_hold15_success_20261031.py` and `hold15-independent-verification-20261031.json`.
- Saved configuration and final-stop confirmation: `hold15-saved-configuration-20261031.json` and `hold15-final-state-20261031.json` in the same maintenance directory.
- Preparation, rehoming, six tests, configuration, and original video are also archived in `hold15-completed-20261031.tar.gz`. SHA-256 checks matched all 72 files in the local copy.

The dashboard is available on the PC at `http://localhost:8000/`. The next physical run must be started explicitly by a prepared operator; no run is scheduled.
