**English** | [한국어](ko/control_algorithm.md)

# Swing-Up and Balance Control

The controller connects energy-based swing-up and cascaded position/angle PID control in one state machine. It uses only `θ, ω, x, v` calculated from the camera, and its final output is the signed velocity sent to the RP2040.

## Control Flow

~~~mermaid
flowchart TD
    A["Valid θ, ω, x, v"] --> B{"Current mode"}
    B -->|swing_up| C["Calculate energy error"]
    C --> D["Energy acceleration + centering PD"]
    D --> E{"Capture conditions met?"}
    E -->|No| H["Integrate velocity"]
    E -->|Yes| F["Preserve current velocity and enter balance"]
    F --> G["Position PID → target angle"]
    G --> M{"Release angle exceeded?"}
    M -->|No| I["Angle PID → cart acceleration"]
    M -->|Yes| C
    I --> H
    B -->|balance| G
    H --> J["Host velocity, acceleration, and stopping-distance limits"]
    J --> K["RP2040 local safety limits"]
    K --> L["STEP/DIR/ENA"]
~~~

## State and Output

Control inputs:

~~~text
θ : pendulum angle, with upright defined as zero
ω : pendulum angular velocity
x : cart position
v : cart velocity
~~~

The control output is the cart target velocity. In the physical-system configuration, the angle-PID output is interpreted as acceleration and integrated with the actual `dt`.

## Balance Control

The outer position loop produces a small target angle, and the inner angle loop accelerates the cart to track that angle.

~~~text
e_x = x_center - x
θ_set = clamp(PID_position(e_x), -θ_max, +θ_max)

e_θ = θ_set - θ
a_cmd = PID_angle(e_θ)

v_target[k] = v_applied[k-1] + output_sign × a_cmd × dt
~~~

The implementation includes:

- derivative on measurement
- filtered estimator `ω` and `v` for the derivative terms
- an 18 Hz low-pass filter on derivative terms
- conditional integration
- back-calculation anti-windup
- bumpless updates when gains change
- maximum-velocity and `a_max × dt` slew-rate limits

In acceleration mode, execution is rejected when angle `Kp` is below `g=9806.65 mm/s²/rad`. This check prevents accidental operation with a configuration that cannot generate the restoring acceleration required by the pendulum.

## Energy-Based Swing-Up

Upright is defined as `θ=0`, and the downward position as `θ=±π`. The normalized energy error, divided by pendulum mass, is:

~~~text
e = 0.5 × l × ω² / g + cos(θ) - 1 - e_margin
~~~

`e=0` is the upright separatrix. `e_margin` is a small allowance for energy lost to friction.

Energy-injection acceleration:

~~~text
phase = e × (ω / ω_ref) × cos(θ)
a_energy = a_max × tanh(k_E × phase)
~~~

The implementation uses `tanh` instead of a sign function so the command does not jump abruptly during direction reversal.

The PD term that keeps the cart near the center of the rail is:

~~~text
a_center = clamp(k_x(x_center - x) - k_v v,
                 -center_fraction × a_max,
                 +center_fraction × a_max)

a_total = clamp(a_energy + a_center, -a_max, +a_max)
~~~

When `|ω|` is very small near the downward position, the controller uses a short startup kick. The kick is allowed only inside the configured downward-angle region and is not retriggered while passing upright.

## Capture and Re-entry

Basic capture conditions:

~~~text
|θ| ≤ θ_capture
|ω| ≤ ω_capture
~~~

Switching on angle alone can leave no control authority when the cart is near its velocity limit. An optional headroom check uses the unstable mode of the upright equilibrium:

~~~text
z = ω + sqrt(g/l) × θ
Δv ≈ l × z
~~~

The controller enters balance only if the predicted velocity change and acceleration demand fit within the configured reserve.

The current allowable velocity is preserved at the transition. Forcing the first balance command to zero would abruptly brake the cart precisely as the pendulum passes upright.

If the pendulum moves outside `release_angle_rad` during balance, the controller returns to swing-up. Separate capture and release values prevent rapid mode chattering at the transition boundary.

## Video-Delay Compensation

Timing compensation for energy swing-up and balance control is configured separately. The same dynamics predictor can be applied during swing-up, but the successful record described above used zero swing-up prediction time and applied 20 ms only in the balance stage.

During balance, the controller can optionally:

1. Interpolate the past angle aligned with the filtered angular velocity.
2. Use the history of cart acceleration that was actually permitted.
3. Integrate a model that includes pendulum length, viscous damping, and Coulomb friction.
4. Predict the current `θ, ω` over a bounded horizon.

The predicted state is used only for the selected control calculation. Raw measurements remain the basis for cart position, rail boundaries, tracking state, and safety decisions. The capture conditions used for the recorded result also used measurements without prediction.

## Final Safety Limits

~~~mermaid
flowchart LR
    A["Controller velocity"] --> B["Host velocity limit"]
    B --> C["Host acceleration limit"]
    C --> D["Camera-position stopping distance"]
    D --> E["USB serial"]
    E --> F["RP2040 watchdog"]
    F --> G["RP2040 acceleration/deceleration and stopping distance"]
    G --> H["Software limits + physical limits"]
    H --> I["Hardware PWM STEP"]
~~~

Host stopping distance:

~~~text
d_stop = v²/(2a) + |v|t_latency + margin
~~~

When the remaining distance in the direction of travel is shorter than `d_stop`, the host blocks outward commands and permits only braking in the opposite direction.

Independently of the host, the RP2040 rechecks the watchdog, homing state, both limits, ESTOP, maximum velocity, and acceleration/deceleration.

## Physical-Test Configuration

| Item | Value |
| --- | ---: |
| Cart center | 256.369 mm |
| Angle PID | 26000 / 0 / 3300 |
| Position PID | 0.0000077 / 0 / 0.0001875 |
| Balance velocity limit | 240 mm/s |
| Balance acceleration limit | 2000 mm/s² |
| Swing-up velocity limit | 240 mm/s |
| Swing-up acceleration limit | 1500 mm/s² |
| Swing-up travel range | ±160 mm about center |
| Energy gain | 46 |
| Energy margin | 0.045 |
| Capture angle | 0.22 rad |
| Capture angular velocity | 1.5 rad/s |
| Capture headroom | 20% |
| Balance prediction horizon | 20 ms |
| Rate alignment | 30 ms |

These values document this particular apparatus; they are not intended as initial values for other hardware.

## Related Code

- [PID and swing-up state machine](../raspberry_pi/pid_controller.py)
- [Host safety state machine](../raspberry_pi/safety.py)
- [Runtime threads and mode transitions](../raspberry_pi/runtime.py)
- [RP2040 firmware](../firmware/main.ino)
- [Swing-up validation procedure](swing_up_validation.md)
- [Hardware validation procedure](hardware_validation.md)
