# Energy-based swing-up and balance handoff

The runtime can run a hybrid controller: normalized energy shaping while the
pendulum hangs or swings, followed by the approved cascaded PID near upright.
The pendulum angle convention is `theta=0` upright and `|theta|=pi` hanging.

For equivalent dynamic pendulum length `L`, angular velocity `omega`, gravity
`g`, and a small configured energy surplus `margin`, the dimensionless energy
error relative to the target is

`e = 0.5 * L * omega^2 / g + cos(theta) - 1 - margin`.

`energy_target_margin` defaults to zero and is bounded to 0..0.05. A small
positive value can compensate bearing losses that otherwise leave the swing
short of upright. It does not relax the velocity, acceleration, travel,
tracking, angular-speed or time limits, and does not itself establish capture.

With this angle convention, `e_dot = -a * omega * cos(theta) / g` for cart
acceleration `a` in m/s². The energy-shaping approach follows
[MIT's cart-pole notes](https://underactuated.mit.edu/acrobot.html), converted
to upright-zero angles. Centering, finite travel and saturation change the ideal
energy-only system, so the energy argument alone is not a convergence proof on
this rail.

The swing-up stage requests cart acceleration proportional to
`e * omega * cos(theta)`. Its sign increases pendulum energy below upright and
removes energy above the target. The controller applies a brief configurable
kick at startup because the exact hanging equilibrium has zero angular
velocity and cannot leave rest under energy feedback alone. It bounds the
acceleration and velocity target; the normal Pi rail/braking safety and the
RP2040 watchdog, speed limit, soft limits, and physical switches remain active.

When both angle and angular rate enter the capture window, control hands over
to the existing cascaded balance PID. If the angle leaves the release window,
energy shaping resumes. Tracking loss, power faults, rail limits, and the
configured maximum angular speed still stop the run.

The optional capture headroom check also examines the local divergent mode
`z = omega + sqrt(g/L)*theta`. It estimates a required cart velocity change
`L*z`, and reserves a configured fraction of available velocity and acceleration
before admitting the handoff. This local approximation screens out switches at
an unusable speed limit; it is not a nonlinear capture guarantee.

The 30 Hz backward-difference/rate filter was measured to lag the angle by
approximately 29 ms. `energy_rate_lag_s` aligns the potential and kinetic terms
using interpolated angle history, including across the pi wrap. A separate fit
of admitted velocity integrals against camera cart positions found 56..58 ms
of combined command/image/sample-hold delay, with fitted position scales
0.997..1.006. Raw acquisition timestamps alone do not include image latency.

An optional short predictor starts from the aligned angle/rate and propagates
the nonlinear pendulum with recent admitted acceleration history. The local
experimental values are 40 ms image latency, 0.06/s viscous damping and
0.36 rad/s² dry friction. The image latency is bounded to 60 ms, the total
prediction horizon to 140 ms, and predicted excessive angular speed stops the
controller. Rail coordinates and the original tracking-age checks still use
measured state. The predictor's uncertain plant parameters require physical
validation; the forecast is not treated as a new camera measurement.

Physical check `20261031T042717Z-fbba19e0` reached only 85.06° from down with
prediction enabled, versus 157.98° without it. Prediction and energy-time
alignment are therefore disabled in the current local configuration. Their
model tests remain experimental evidence only. Subsequent commissioning uses
the measured hardware response and records any changes to the speed envelope.

The feature is configured under `controller.swing_up`. It requires
`controller.output_mode: acceleration` and approved angle Kp/Kd gains for capture.
Acceleration-mode Kp must exceed 9806.65 mm/s²/rad; this necessary model condition
does not prove that real balance has been established. Initial values for
pendulum length, energy gain, kick, capture thresholds, and speed limits are
commissioning starting points only. The default keeps `enabled: false`; the
local configuration is enabled for the explicitly requested supervised trials.
Real swing-up and continuous balance must be established from the recording
and measurements rather than a simulation pass.

## Implementation and status

- A bounded startup kick occurs only near hanging rest, not during a fast
  upright pass. A bounded position/velocity PD term keeps the cart centered.
- Acceleration is integrated in mm/s² using the actual control interval and the
  last admitted velocity command. The controller does not repeatedly add an
  increment to a stale camera velocity at 100 Hz. Idle waiting time is not
  integrated as an initial impulse.
- Capture checks both angle and angular rate, and caps the angle by the balance
  engage window. Wrapped full rotations can be captured correctly. Handoff
  preserves the admitted velocity and primes derivative filters from estimated
  rates, avoiding a zero-speed step at the switch.
- The smaller host/swing-up velocity limit applies to the whole hybrid run.
  Swing acceleration cannot exceed the host acceleration limit. Normal rail
  braking remains active and its actual admitted command is fed back to the
  velocity integrator.
- Invalid, non-finite or stale measurements, excessive angular speed,
  `max_cart_excursion_mm` from the configured center, and `max_duration_s` spent
  energy shaping cause a latched host FAULT. A brief capture does not erase the
  accumulated swing time. FAULT clearing resets the controller explicitly.

Additional settings include `kick_downward_tolerance_rad`, `kick_omega_rad_s`,
`cart_position_gain_s2`, `cart_velocity_gain_s`,
`centering_acceleration_fraction`, `max_cart_excursion_mm`, `max_duration_s`,
and `maximum_state_age_s`. Invalid numerical settings are rejected at load time.
No rail position calibration or automatic homing is added by this controller.
Pivot-to-marker length is only an approximation to effective point-pendulum
length; mass distribution and moment of inertia matter for the real apparatus.

`/api/status` exposes `swing_up.enabled`, `mode`, `energy_error`,
`swing_elapsed_s`, `fault`, and effective limits. Modes are `idle`, `swing_up`,
`balance`, `fault`, or `balance_only` when disabled. The dashboard labels
disabled swing-up explicitly. Loading code does not ARM or START the motor.

While IDLE with a stopped, homed, healthy motor, `PUT
/api/control/swing-up/parameters` accepts bounded energy gain, energy surplus,
startup kick, centering gains, and a 0.5..30 s energy-shaping timeout. The
endpoint rejects changes while armed, running, faulted, moving, or recording an
experiment. It cannot change speed, acceleration, travel, watchdog or tracking
limits. These candidate values remain in memory, and every experiment summary
stores the starting swing-up settings, including the equivalent length.

`experiments/run_upright_direction_trial.py --swing-up --stage` performs full
preflight again on GO, records the trial and always requests STOP on exit. It
reports a successful capture only after five continuous seconds within 5° in
balance mode with valid tracking and healthy interlocks. Each run has a 30 s
trial bound with at most 5.5 s of bounded capture verification extension.

## Offline validation

Run from the repository root:

```sh
python -m pytest -q tests/test_swing_up.py tests/test_swing_up_parameters.py
python -m experiments.simulate_swing_up
```

The simulator has no serial or API calls. It uses a nonlinear point pendulum,
an acceleration-limited velocity servo, damping, and a 30 Hz camera with 20 ms
delay. Its full reference configuration is printed and never written to the
live configuration. The reference envelope is 600 mm/s and 2000 mm/s², not the
current hardware commissioning limits.

Observed on 2026-10-31: the reference model captured from exact hanging rest at
5.19 s, then balanced continuously for at least 3 s without a fault. Maximum
excursion was 184.75 mm, maximum speed 453.67 mm/s, and maximum acceleration
2000 mm/s². The same model with the initial conservative hardware parameters
(80 mm/s host speed limit, 250 mm/s² swing-up acceleration, energy gain 1) did
not capture in 30 s and stopped on its time limit. No live gain or limit was
increased to make a simulation pass.

The model omits belt compliance, motor lag, missed steps and camera noise.
`plant_length_m` may differ from the controller's length, and optional
`coulomb_friction_rad_s2` allows checks against measured passive decay. These
uncertain model parameters are not hardware measurements by themselves.
Simulation validates the software path, not real swing-up performance.
Operator-contact trials cannot establish hands-free control performance.

## Supervised hardware observations, 2026-10-31

All the following runs used an 80 mm/s velocity cap and an 80 mm centered
excursion limit, with recording enabled. None established upright capture.

| Experiment | Duration | Swing acceleration | Energy gain | Maximum angle from down |
| --- | ---: | ---: | ---: | ---: |
| `20261031T030904Z-60fe1378` | 5 s | 250 mm/s² | 1 | 0.60° |
| `20261031T034417Z-d0286f0d` | 10 s | 250 mm/s² | 8 | 32.99° |
| `20261031T035016Z-56c947f8` | 30 s | 250 mm/s² | 8 | 78.64° |
| `20261031T035339Z-03ff3844` | 30 s | 500 mm/s² | 16 | 126.46° |
| `20261031T040532Z-8904c19d` | 30 s | 750 mm/s² | 16 | 157.16° |
| `20261031T040728Z-8f7aec35` | 30 s | 750 mm/s² | 32 | 157.98° |
| `20261031T041411Z-5ea1f8a5` | 30 s | 750 mm/s² | 32 | 141.03° |
| `20261031T042717Z-fbba19e0` | 30 s | 750 mm/s² | 32 | 85.06° |

The last two 157° runs used energy surplus 0.02 without prediction. The 141°
run used angle/rate alignment, surplus zero, and no predictor. These results
demonstrate that a larger energy gain or an ideal model pass does not establish
physical capture. PID angle gains remained Kp=22000, Kd=3300. Cart velocity
damping was raised from 0.0000306 to 0.0002 through the validated proposal API;
a subsequent model-based candidate 0.00025 is a separate supervised trial.

The 30 s / 250 mm/s² run tracked both markers for all recorded samples, wrote
901 video frames without drops, and stopped on the intended time limit. The
motor was verified at zero actual and target velocity after each run before
clearing that specific timeout fault.

After a separate 5 s excitation (`20261031T035601Z-264a0cdb`), the motor remained
stopped throughout a 12 s passive measurement. Twenty-five overlapping full
cycles gave a median equivalent point-pendulum length of 0.186893 m, using
nonlinear period correction for the measured amplitude. Individual estimates
were approximately 0.1857..0.1880 m. The energy model now uses 0.187 m rather
than the 0.226 m marker length; no camera or rail coordinates were changed.
The recorded samples and individual fits are retained in `experiments/passive-*`.

## Later supervised commissioning results

The later trials below retained the existing rail coordinates and hardware
interlocks. They used the measured 0.187 m effective length, with both the
short-horizon predictor and angle/rate alignment disabled after the 85°
physical regression above. PID gains were Kp=22000, Kd=3300, and cart velocity
gain 0.00025. The updated gains have not yet established five seconds of
hands-free upright balance.

| Experiment | Speed cap | Swing acceleration | Centered excursion cap | Maximum angle from down | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| `20261031T043259Z-78938eb7` | 120 mm/s | 750 mm/s² | 80 mm | 159.73° | Relative displacement stop |
| `20261031T044119Z-8f6d2290` | 120 mm/s | 750 mm/s² | 80 mm | 165.56° | 30 s timeout |
| `20261031T044336Z-594f2857` | 120 mm/s | 750 mm/s² | 80 mm | 166.91° | Relative displacement stop |
| `20261031T045432Z-df691eb8` | 160 mm/s | 1500 mm/s² | 120 mm | 178.63° | 30 s timeout; no capture |
| `20261031T050200Z-13af5c17` | 160 mm/s | 1500 mm/s² | 120 mm | 179.37° | 30 s timeout; no capture |
| `20261031T053740Z-7132ca64` | 160 mm/s | 1500 mm/s² | 120 mm | 173.00° | 30 s complete; no capture |
| `20261031T054035Z-b4a9ed54` | 160 mm/s | 1500 mm/s² | 120 mm | 164.42° | 15.46 s centered-excursion fault; stopped |
| `20261031T054731Z-4beec237` | 160 mm/s | 1500 mm/s² | 120 mm | 169.88° | 30 s complete; no capture |
| `20261031T055138Z-f0f7c4cf` | 160 mm/s | 1500 mm/s² | 120 mm | 178.80° | 30 s complete; no capture |
| `20261031T055434Z-75ae9bb6` | 160 mm/s | 1500 mm/s² | 120 mm | 170.42° | 30 s complete; no capture |
| `20261031T055909Z-bc899a5d` | 160 mm/s | 1500 mm/s² | 120 mm | 179.57° | Two balance intervals, 0.51 s and 0.30 s; no five-second hold |

The host acceleration cap for the final two rows was 2000 mm/s²; independent
firmware limits and the conservative 1000 mm/s² rail braking model remained
active. Cart centering used stopped, guarded low-energy trials, and one
separate firmware POS operation capped at 20 mm/s. No homing or rail
coordinate calibration was performed.

Recorded data from the 178.63° trial contained a slow pass at 6.47° from
upright and 0.534 rad/s. The capture angle was therefore widened from 0.10 to
0.12 rad, matching the existing balance engage angle. The subsequent 179.37°
trial still never met both the 0.12 rad angle window and 0.8 rad/s rate window:
its closest pass was 0.625° from upright at 1.410 rad/s. Near that pass the
measured energy was about 0.019 above upright. Reducing the target surplus
from 0.05 to 0.03 produced a slow apex about 7° from upright. A recorded pass
at 6.997°, 0.235 rad/s had sufficient velocity and acceleration headroom, but
missed the 6.875° capture-angle window. The next candidate is surplus 0.035.
Near-upright passage by itself is not successful capture.

Capture settings were subsequently widened to 0.22 rad (also the engage
angle) and 1.5 rad/s, retaining 20% velocity/acceleration headroom and the
0.28 rad release threshold. Recorded approaches at about 11.9 degrees had
more usable headroom than later near-upright samples. An offline replay
accepted these approaches while still rejecting insufficient-headroom
states; this is a gate check, not proof of balance stability.

The first two actual autonomous balance intervals occurred in
`20261031T055909Z-bc899a5d`. Both reached the 160 mm/s velocity cap and then
returned to energy control. During the first interval the cart target angle
approached -0.04 rad; the second approached +0.04 rad. Cart velocity feedback
was therefore reduced by 25%, from 0.00025 to 0.0001875, through validated
proposal `9dfc8b47c9ea4c3eb7f1fdb4f527da41`. Angle gains were unchanged. The
follow-up hardware result is recorded separately; no five-second hands-free
balance has yet been established by the results above.

The 0.035-surplus trial drifted outward because energy acceleration could
overpower the additive centering PD. The motor stopped at 382.05 mm with both
physical limits clear. Runtime now intersects the physical rail with the
existing centered swing corridor before applying the established dynamic
braking calculation, including the same 25 mm margin, 80 ms latency and
1000 mm/s² braking acceleration. The original 120 mm hard excursion fault
remains active, and the admitted velocity still feeds the PID integrator.
The additional corridor cannot widen the physical rail limits.

Recovery used two separately checked, recorded POS operations at <=20 mm/s:
382.05 -> 300.00 -> 256.35 mm. The optional intermediate target must lie
between the stopped cart and its configured center; every segment retains
the original <=100 mm recovery bound. The configured center and calibration
coordinates were unchanged.

The scheduled runner performs full preflight before waiting and again at its
explicit start slot, rejects late ARM/START, holds an exclusive process lock,
and requests STOP on exit. It is a single supervised trial, not an unattended
retry loop. A trial requires current operator readiness and supervision;
missed preparation slots are cancelled before ARM. Recording and monitor
files retain the complete result even if the PC tool output is delayed.

Validation on the Pi after the centered braking and recovery changes: all
166 tests passed, including the recorded outward-drift regression and
two-step recovery guards. After adding explicit trial envelopes and stronger
preflight checks, the full suite passed 182 tests.

## Enlarged envelope and near-successful capture

Further supervised gain comparisons reduced cart Kd from 0.00025 to
0.0001875 (proposal `9dfc8b47c9ea4c3eb7f1fdb4f527da41`), then raised angle
Kp from 22000 to 26000 (proposal `97d0ef387568458abc2f325a95895256`). Each
change passed the existing proposal validator; angle Kd remained 3300.
`20261031T060850Z-b01c0258` reached a 0.78 s PID interval but not a five-second
upright hold.

The operator explicitly approved a 240 mm/s, centered +/-160 mm trial
envelope after reviewing `capture240_review.md`. Swing acceleration stayed
1500 mm/s² and balance acceleration 2000 mm/s². The independent firmware
limits, rail coordinates, homing, tracking, power and watchdog protection
were retained. The default runner still rejects settings above 160 mm/s and
120 mm; the larger envelope requires `--commissioning-profile capture240`.

A separate 0.5 s commissioning pulse (`20261031T062011Z-b59d0b8f`) reached
240.0 mm/s. The recorded camera brake reference to final motor position was
23.854 mm, inside the conservative 73 mm braking budget; this is not a
separate precision stopping-distance measurement. Stopped camera/motor
positions differed by 0.788 mm, with zero actual and target velocity,
100% marker detection and no power, limit or watchdog fault.

| Experiment | Capture angle | Energy surplus | Maximum rise | Outcome |
| --- | ---: | ---: | ---: | --- |
| `20261031T062641Z-ff450df4` | 0.22 rad | 0.05 | 173.32° | Early captures stalled below upright |
| `20261031T063034Z-b5a6178a` | 0.14 rad | 0.05 | 178.30° | Near-upright rates 1.6..1.9 rad/s; no capture |
| `20261031T063236Z-f9688be6` | 0.14 rad | 0.025 | 179.91° | 4.55 s continuously within +/-5°, then lighting-envelope stop |

The last run's PID interval was 4.58 s; its actual continuous +/-5° time was
4.549983689 s, independently recomputed from the 100 Hz CSV and matching the
experiment summary. This is below the five-second success criterion.
Sampled recording frames showed hands-free upright control. The existing
lighting guard stopped it and subsequently requalified when frames passed;
no guard was disabled to continue.

The previous profile brightness baseline was 147, while twelve stopped live
measurements were all 88. Replaying the 524 recorded frames with the old
profile produced 70 rejected frames, including 65 after 10 s, due to the
lighting envelope. Replaying those same compressed frames with a new 88
baseline produced zero invalid/tracking-rejected frames. Marker HSV ranges,
coordinate data, confidence thresholds and lighting safeguards were all
unchanged. The new `daylight-capture240-20261031` profile then passed the
required 15 consecutive live verification frames and was persisted in
`config.local.yaml`. The original profile is retained. This replay supports
the brightness diagnosis; it does not replace the next physical trial.

## Validation sequence

1. Verify image angle sign by hand tilting the pendulum both ways.
2. Measure pivot-to-tip length and enter it in meters.
3. Keep 24 V off while checking the normalized energy and command sign in
   software for several angles and angular rates.
4. Establish and approve balance PID gains separately, beginning with the
   pendulum manually held near upright.
5. Enable swing-up only after low-speed direction, limits, power, stopping,
   camera tracking, and gain capture checks pass.
6. Begin with the cart centered, a clear rail, a physical power cutoff at hand,
   and the conservative configured limits. Record every attempt and stop on
   abnormal noise, missed steps, tracking loss, temperature rise, or rail
   approach.
