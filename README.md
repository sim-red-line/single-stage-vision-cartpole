**English** | [한국어](README.ko.md)

# Vision-Based Single-Stage Inverted Pendulum

This project measures the pendulum and cart positions with a USB camera and uses a Raspberry Pi and an RP2040 to control the system from swing-up through balance.

- Raspberry Pi: vision processing, state estimation, swing-up and balance control, and recording
- RP2040: STEP pulses, homing, limits, watchdog, and ESTOP
- Camera: Logitech C922, 640×480 MJPEG, 30 FPS

![Swing-up and 15-second balance hold](docs/assets/20261101T053856Z-adde3c2c-swingup-and-balance.gif)

## Physical-System Record

| Metric | Measurement |
| --- | ---: |
| Continuous hold within ±5° | 15.029995731 s |
| Samples in the stable interval | 1,504 |
| Angle RMS in the stable interval | 0.444° |
| Maximum absolute angle in the stable interval | 3.678° |
| Cart travel range | 215.1–324.075 mm |
| Marker detection rate | 100% |
| Dropped video frames | 0 |
| Final state | IDLE, motor speed 0 |

## Operating Flow

~~~mermaid
flowchart LR
    A["Camera frame"] --> B["HSV marker tracking"]
    B --> C["State estimation<br/>θ, ω, x, v"]
    C --> D{"Control mode"}
    D -->|Lower region| E["Energy-based swing-up"]
    D -->|Near upright| F["Position/angle PID"]
    E --> G["Host safety limits"]
    F --> G
    G --> H["USB serial"]
    H --> I["RP2040<br/>watchdog · limits · PWM"]
    I --> J["Stepper motor"]
    J -. "Next camera frame" .-> A
~~~

The Raspberry Pi output does not become a STEP signal directly. The host first limits velocity, acceleration, and stopping distance, and the RP2040 validates the same command again.

## OpenCV Processing Pipeline

The calibration values used in the experiment were applied to frame 216 of the successful recording.

![OpenCV vision-processing stages](docs/assets/vision-pipeline/vision-pipeline-overview.jpg)

1. Convert the original BGR frame to HSV.
2. Separate the pivot and tip color ranges with `cv2.inRange`.
3. Remove small noise with 3×3 `MORPH_OPEN` and `MORPH_CLOSE` operations.
4. Evaluate contour area, shape, and temporal continuity to select each center point.
5. Calculate cart position and pendulum angle from the centers.
6. Calculate velocities from timestamp differences and a 12 Hz low-pass filter.

If two or more candidates are similar or a marker disappears, the previous coordinate is not reused; that frame is marked invalid.

[Read the detailed vision pipeline with intermediate images](docs/vision_pipeline.md)

## Control Algorithm

~~~mermaid
flowchart TD
    A["θ, ω, x, v"] --> B{"Swing-up / Balance"}
    B -->|Swing-up| C["Energy error + centering PD"]
    C --> D{"Capture angle, rate, and control headroom"}
    D -->|Pass| E["Switch to balance"]
    D -->|Continue| F["Integrate velocity"]
    B -->|Balance| G["Position PID → target angle"]
    E --> G
    G --> H["Angle PID → acceleration"]
    H --> F
    F --> I["Velocity, acceleration, and rail limits"]
~~~

Swing-up accelerates the cart from the pendulum energy error and adds a PD term that keeps the cart near the rail center. Near upright, the controller switches to the balance PID while preserving the current cart velocity once angle, angular velocity, and control-headroom conditions are all satisfied.

Balance control consists of two loops:

- Outer loop: cart position error → small target pendulum angle
- Inner loop: pendulum angle error → cart acceleration

[Read the swing-up equations, PID structure, and capture conditions](docs/control_algorithm.md)

## Safety Architecture

### Raspberry Pi

~~~text
DISCONNECTED → UNHOMED → CALIBRATION_REQUIRED → IDLE → ARMED → RUNNING
~~~

The system can enter `RUNNING` only when serial communication, the watchdog, homing, current vision tracking, and power state are all healthy. Stale state, marker loss, approach to a rail boundary, or a power fault causes a stop.

### RP2040

- ESTOP latched and ENA disabled immediately after boot
- 400 ms heartbeat watchdog
- Maximum velocity and 2000 mm/s² acceleration/deceleration limits
- Software limits at both ends and normally closed physical limit switches
- Rejection of ordinary velocity commands before homing
- PWM stop before every direction reversal
- Non-blocking homing state machine

## Hardware

| Component | Role |
| --- | --- |
| Raspberry Pi 5 | Vision, state estimation, control, dashboard, and recording |
| Logitech C922 | 640×480 MJPEG at 30 FPS |
| Waveshare RP2040 Zero | STEP/DIR/ENA, homing, watchdog, and limits |
| NEMA23 + stepper driver | Cart actuation |
| Two NC limit switches | Detection of both rail ends |
| Color markers | Pendulum tip and pivot tracking |

The RP2040 uses pin 6 for ENA, 7 for STEP, 8 for DIR, 9 for LEFT LIMIT, and 10 for RIGHT LIMIT.

## Installation and Startup

Python 3.11 or newer is recommended.

~~~bash
sudo apt update
sudo apt install -y python3-venv libgl1 libglib2.0-0

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
~~~

For the first startup, disable the motor and verify the video and dashboard before enabling hardware.

~~~yaml
hardware:
  enabled: false

camera:
  enabled: true
  index: 0
  fourcc: MJPG
  width: 640
  height: 480
  fps: 30

controller:
  swing_up:
    enabled: false
  gains:
    kp_angle: 0.0
    ki_angle: 0.0
    kd_angle: 0.0
    kp_position: 0.0
    ki_position: 0.0
    kd_position: 0.0
~~~

Run:

~~~bash
python -m raspberry_pi.main --config config.local.yaml
~~~

The dashboard is available at `http://127.0.0.1:8000` by default.

## Repository Layout

~~~text
firmware/main.ino               RP2040 firmware
raspberry_pi/color_tracker.py   HSV marker detection
raspberry_pi/state_estimator.py θ, ω, x, v calculation
raspberry_pi/pid_controller.py  Swing-up and balance PID
raspberry_pi/safety.py          Host safety state machine
raspberry_pi/runtime.py         Camera, control, and recording threads
dashboard/                      Operations dashboard
calibration/profiles/           Device-specific calibration values
experiments/logs/               Experiment logs
experiments/videos/             Experiment videos
tests/                          Regression tests
~~~

## Documentation

| Document | Contents |
| --- | --- |
| [Vision pipeline](docs/vision_pipeline.md) | HSV ranges, morphology, contour scoring, coordinates, and state calculation |
| [Swing-up and balance control](docs/control_algorithm.md) | Energy equation, PID, capture, delay compensation, and safety limits |
| [System architecture](docs/architecture.md) | Runtime threads and the Raspberry Pi/RP2040 roles |
| [Hardware validation](docs/hardware_validation.md) | Power, limits, direction, homing, and physical-test sequence |
| [Swing-up validation](docs/swing_up_validation.md) | Staged swing-up commissioning |
| [Firmware analysis](docs/firmware_analysis.md) | RP2040 PWM, watchdog, and homing implementation |
| [Korean documentation index](docs/ko/README.md) | Preserved Korean versions of the originally Korean documents |

## Tests

~~~bash
python -m pytest
~~~

Regression tests cover the camera, tracking, state estimation, control, safety state machine, and web interface. Motor direction, limit-switch polarity, stopping distance, mechanical friction, and power quality still require separate validation on the physical system.
