# Supervised hardware validation procedure

Latest observation (2026-11-01): after a position-P comparison was reverted, an optional
position-dependent angle reference was fitted and tested with all six original gains.
One of two trials met 15.03 continuous seconds within ±5°, with cart position215..324mm
and a verified stop. See [the drift review](position_drift_review_20261101.md) for limits,
passive upright observations, the322 passing tests and the unresolved first-home issue.

Latest physical re-test (2026-10-31): after the operator reported correcting a misaligned
shaft and confirmed free rotation, the unchanged controller captured four times and
recorded at most 3.45 continuous seconds within ±5°. The 15-second target was not met.
See [the shaft correction re-test](shaft_corrected_retest_20261031.md) for the raw evidence.

Latest review (2026-10-31): a trial recorded 15.01 continuous seconds within ±5°,
but the pendulum then remained upright with control inactive and zero motor velocity.
The operator reports it also holds a tilted pose with 24V motor power off. Mechanical
friction or binding therefore prevents attributing the angle-duration result to control.
See [the record and mechanical review](hold15_success_20261031.md) before further testing.
The observations below retain their original dates and are historical.

Do not begin with balance tests. Each gate below must pass before proceeding, and the operator must remain at the machine with physical power removal available.

## Observed Raspberry Pi state — 2026-10-29

- Target: `Defotic` at `192.168.219.108`, Debian 13, `aarch64`, kernel `6.18.50+rpt-rpi-2712`.
- The `admin` account belongs to `dialout`, `video`, `gpio`, `i2c`, and `spi`.
- Logitech C922: `/dev/video0`; Waveshare RP2040 Zero: `/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6636825930C5527-if00`.
- The deployed source is `/home/admin/vision-cartpole`; its Raspberry Pi test run passed all 24 tests.
- The original YUYV path produced 1280x720 at 10.0 FPS. Selecting MJPEG changed the driver mode to 60 FPS and produced 39.52 decoded frames/s over a 120-frame OpenCV test.
- Before the camera load test, `vcgencmd get_throttled` returned `0x50000`, recording that undervoltage and throttling had occurred since boot. The low current-status bits were clear at that instant.
- During the subsequent multi-resolution camera test the Pi disappeared from the LAN, including its MAC address. Treat power stability as unresolved. No RP2040 command, firmware flash, homing, motor command, or PID run was performed.
- After a cold boot on 2026-10-30, the Pi reported `throttled=0x0` with both USB devices attached. The 1280x720 MJPEG stream then held 59.42–59.43 decoded frames/s with no errors for about 150 seconds.
- At approximately 150 seconds the C922 began returning empty frames and disappeared from `lsusb` and `/dev/video0`. The kernel recorded `usb 3-1: USB disconnect, device number 2`. The Pi stayed online at 38.4 °C with `throttled=0x0`, so this event is isolated to the camera USB path rather than a whole-Pi power loss. Camera cable, port, peripheral power, and long-run UVC stability remain unresolved.
- The first reconnect attempt logged USB descriptor error `-71`; the next attempt enumerated successfully. This further supports treating the high-rate USB path as unresolved rather than attributing the failure to vision code.
- A supervised 640x480 MJPEG test negotiated 30 FPS and completed 600.0 seconds with 17,977 frames, 29.96 average FPS, zero empty frames, `throttled=0x0`, and a final CPU temperature of 47.2 °C. This is the selected commissioning mode.
- The full camera/dashboard runtime subsequently produced kernel undervoltage events at 08:32:24 and 08:33:55. Device-tree reported `max_current=3000` mA, `usb_max_current_enable=0`, and the PMIC measured 4.82266 V immediately after load. The run was stopped; Gate 0 therefore remains failed until a suitable 5 V/5 A supply or a correctly rated powered USB arrangement is installed and the test passes from a clean boot.
- The Raspberry Pi runtime now polls `vcgencmd get_throttled`. When real hardware is enabled, an unavailable or non-zero result blocks the motor connection; a new power event during operation latches FAULT and requests a motor stop.
- After the reported power replacement and reboot, the Pi still negotiated only 3000 mA: `max_current=3000`, `usb_max_current_enable=0`, and the USB-PD PDO dump contained only zeros. A boot-time undervoltage event immediately restored `throttled=0x50000`; no load test was started. Do not force the USB current-limit override under this condition. Use a source that advertises the optional 5 V/5 A PDO and a 5 A-capable cable, or the official Raspberry Pi 27 W supply.
- On a later clean boot on 2026-10-30, a guarded 620-second camera/dashboard runtime test was run with `hardware.enabled=false` and every PID gain at zero. The C922 held 29.87–30.27 FPS at 640x480 MJPEG, the camera and RP2040 device paths remained present, `throttled=0x0` throughout, and no new undervoltage or USB-disconnect event appeared during the run. Sampled `EXT5V_V` reached a low of 4.84276 V; the post-test value was 4.90842 V and CPU temperature was 45.5 °C. The timeout produced a clean application shutdown and no runtime process remained.
- This limited non-actuating load test is a useful pass, but it does not clear the installation for motor operation: the source still negotiated only 3000 mA with `usb_max_current_enable=0`, and the same boot contained a pre-test `usb 1-2` disconnect at 14:35:19. Gate 0 remains unaccepted for actuator commissioning until the 5 A supply path is verified or an adequately powered USB arrangement is used and the test is repeated without a pre-test USB reset.

## Supervised RP2040 and actuator commissioning observations — 2026-10-30

- The installed controller is `cartpole-safe-7`. It boots with ESTOP latched, STEP stopped, target/current velocity zero, and ENA disabled. A locked calibration jog is rejected.
- The normally-closed limit inputs were checked by hand. Both report `false` at rest; the left and right inputs independently report `true` only while their corresponding switch is pressed.
- The original direction mapping was reversed on the machine. `DIR_RIGHT_LEVEL=LOW` was then verified physically: a `+20 mm/s` 250 ms jog moved right by an estimated 5.062 mm, and a `-20 mm/s` 250 ms jog moved left by an estimated 5.024 mm. Both tests ended in a verified ESTOP with zero target/current velocity.
- Homing seek/back-off speeds were reduced to 40/20 mm/s for first commissioning. Homing fault handling was changed to stop pulses and disable the driver.
- With 24 V disabled, a deliberately expired heartbeat during homing produced `HOME_WATCHDOG`, `home_state=failed`, and zero target/current velocity. ESTOP was re-latched afterward.
- A supervised full home traversed left, backed off, traversed right, measured 512.475 mm, and returned to 256.2 mm. It completed in 25.22 s without a firmware fault; ESTOP was applied immediately afterward.
- A second supervised full home repeated the same path, measured 512.550 mm, and stopped at 256.238 mm. The two rail measurements differed by only 0.075 mm. The operator confirmed the physical left/right/center sequence was normal; ESTOP was again applied immediately afterward.
- A camera synchronized home capture recorded 769 frames across the full 512.738 mm travel. Pivot marker pixel x fit was `x_px = 0.646416 * position_mm + 147.080`, with 1.458 px RMSE; this gives image endpoints 147.08 px and 478.51 px and positive camera x for physical right.
- With the pendulum held upright by the operator and motor ESTOP latched, 90/90 live frames detected pivot and tip. Median raw upright angle was 0.01037155 rad (0.59425°), with 0.05222° median absolute spread. The default profile stores this as `vertical_offset_rad`; the live runtime still requires its own 15-frame verification before arming.
- The operator reported a steady high-pitched tone only while the motor was moving; it disappeared when 24 V was removed. At the 40 mm/s homing speed and 0.0375 mm/step, the step rate is approximately 1.07 kHz, consistent with an audible stepper/driver tone. Treat this as provisional until the driver model, current setting, microstep setting, motor/driver temperature, and vibration are physically inspected. Grinding, irregular vibration, heating, or a tone that persists while stopped is a stop condition.
- On 2026-10-30, `cartpole-safe-8` was flashed and verified booting ESTOP-latched. The subsequent 5 mm calibration attempt completed homing at 512.400 mm and 256.200 mm center, then faulted `LEFT_LIMIT` while moving toward the 10 mm soft boundary; the reported position was 30.113 mm. Firmware then latched ESTOP with zero speed. In the synchronized video the left-limit contact during homing was at pivot x=146.09 px and the later fault stop was at x=146.25 px, effectively the same physical camera position. No 5 mm samples were accepted and the camera profile was not changed. Treat this as a true firmware-step-position versus physical-position disagreement; inspect pulse counting, drive coupling, belt tension, and switch wiring before any further rail sweep.
- After that failure, a single `CAL_JOG:20` dead-man jog was tried while 24 V was off. RP2040 telemetry changed from 30.113 to 35.175 mm (+5.062 mm) while the pivot remained at x≈146.3 px. This only confirmed that the firmware counted step pulses without motor power; it was not evidence of mechanical slip. ESTOP was re-latched, homing remained false, and the camera profile was unchanged.
- With 24 V on, a supervised `CAL_JOG:20` moved the firmware count +5.063 mm and the pivot +2.715 px (about 4.20 mm under the existing profile), then stopped and re-latched ESTOP. A subsequent 92-increment sweep recorded firmware 40.238→503.963 mm and pivot x=164.737→451.876 px. Linear fit was 0.615988 px per firmware mm (1.151 px RMSE), 4.71% below the earlier homing fit of 0.646416 px/mm. Both limits stayed inactive; the sweep stopped before the right endpoint and ended ESTOP-latched. No coordinate update was taken from this partial, unhomed sweep.
- A dashboard-controlled re-home completed at 512.588 mm rail length and 256.275 mm center; camera pivot x=312.22 px versus 312.7 px predicted by the existing profile. A second instrumented re-home completed at 512.775 mm and center 256.35 mm. After this valid home, a move to the 10 mm left soft boundary again faulted `LEFT_LIMIT` at firmware position 30.038 mm. The video ended at pivot x≈149.67 px, about 5.5 mm from the previously observed left-limit position x=146.09 px. The run stopped without completing the end-to-end coordinate sweep; final state was ESTOP, zero velocity, and unhomed. This reproduces a roughly 20 mm firmware-position versus target discrepancy after a successful home. Do not move the cart further until the step-to-distance scale, pulse counter, drive coupling, belt tension, and limit-trigger geometry are inspected.
- The operator clarified that the 5 mm sweep was intended for color-marker calibration. The 92-increment video contains 1,864 frames at 30 FPS; both pivot and tip markers tracked in all 1,864 frames under the previous HSV profile. New HSV ranges were derived from eroded marker-contour pixels across the sweep and replayed with 1,864/1,864 frames tracking both markers. The active profile is `color-sweep-20261030`; it changes marker HSV thresholds only and copies camera geometry unchanged from `default`. The Pi live-verified it over 254 frames. `default` remains available.
- After the operator requested swing-up without the 5 mm position correction, a full home was run only to restore the firmware reference. It completed at rail length 513.038 mm and center position 256.500 mm, with speed 0, limits inactive, and host state IDLE. Swing-up was not started: `controller.swing_up.enabled` is false and all configured PID gains are zero. The runtime start guard requires positive angle Kp and Kd for capture, so enabling swing-up now would be rejected. No balance gains were guessed or applied.
- Every sampled pre/post actuator check reported `throttled=0x0`. These supervised observations do not clear Gate 0: the Pi supply still advertises only 3000 mA and the prior USB reset remains unresolved.

## Gate 0: power and network stability

- Keep motor power and the driver disabled. Cold-boot the Pi with a suitable Raspberry Pi 5 power supply.
- If the C922 or RP2040 causes an undervoltage event, use a correctly rated powered USB hub rather than drawing the peripheral load from a marginal Pi supply.
- Confirm SSH remains reachable and monitor `vcgencmd get_throttled` while streaming the selected camera mode for at least ten minutes.
- Do not continue to RP2040 or motor validation if a current undervoltage/throttling bit appears, the historical bits reappear after a cold boot, USB devices reset, or the Pi leaves the network.

## Gate 1: static electrical inspection

- Confirm GPIO 6/7/8/9/10 wiring against ENA/STEP/DIR/left/right.
- Confirm the driver accepts RP2040 logic levels and shares signal ground.
- Confirm the 24 V supply and driver current setting suit the NEMA23 motor.
- Check physical end stops and provide a mechanical buffer beyond each switch.
- Keep the belt/cart path clear. Start unloaded or mechanically secured.

## Gate 2: inputs and emergency behavior

With motor power disabled, read `STATUS?` and actuate each limit switch by hand. Verify only the corresponding boolean changes. Send `ESTOP`; verify STEP PWM and ENA disable. ESTOP must remain latched across subsequent velocity commands.

## Gate 3: direction at commissioning speed

Enable motor power with the cart near the center. Send `HEARTBEAT`, then repeat `CAL_JOG:10`; every jog expires after 250 ms. Verify positive means physically right. If not, change only `DIR_RIGHT_LEVEL`, rebuild, and repeat. Test `CAL_JOG:-10`. Never proceed while direction is ambiguous.

## Gate 4: homing

Send `FULL_HOME` with a hand near physical power removal. Verify low-speed left seek, back-off, right seek, measured travel plausibility, and center return. Induce a switch failure with motor power removed and verify the timeout/fault path before relying on it.

## Gate 5: speed, acceleration, and stopping distance

Begin below 50 mm/s. Compare camera position with firmware step position, listen for step loss, and measure stopping distance. Increase in small supervised increments. Validate 2000 mm/s² rather than assuming the mechanism can achieve it. Keep the normal ceiling at 600 mm/s; 800 mm/s is an absolute command ceiling, not an operating target.

## Gate 6: camera and geometry

Create profiles under the actual bright and dark conditions. Move each marker through its full image region and introduce representative background objects. Ambiguous matches, large illumination changes, and marker occlusion must prevent arming. Measure rail endpoints and vertical reference; verify sign by tilting the pendulum in each direction.

## Gate 7: controller seed

The repository ships with all gains at zero. Establish a conservative seed from the physical units and direction sign; do not copy arbitrary gains from another apparatus. The optional energy-based swing-up hands off to the cascaded PID, but it must stay disabled until the balance gains and its energy/kick/capture settings have been validated on this apparatus. See `docs/swing_up_validation.md`. Use a short supervised experiment and conservative abort limits first.

## Gate 8: repeated fixed-gain comparison

Freeze hardware, camera profile, initial-angle procedure, duration, gains, and evaluation thresholds. Run repeated trials with the same configuration before changing one bounded parameter at a time. Compare distributions of continuous stable time, RMS angle, tracking validity, cart deviation, saturation, and failures. Keep the original run logs, and do not treat a single improved run as evidence of repeatable effectiveness.

