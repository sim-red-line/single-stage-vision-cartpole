**English** | [한국어](ko/vision_pipeline.md)

# Vision Processing Pipeline

This document describes how pendulum angle and cart position are obtained from camera frames.

The images below apply the actual `angle-reference-20261101` profile to frame 216 at 7.176 seconds from experiment `20261101T053856Z-adde3c2c`.

![Complete vision-processing pipeline](assets/vision-pipeline/vision-pipeline-overview.jpg)

## Overview

~~~mermaid
flowchart LR
    A["Raw BGR frame"] --> B["BGR → HSV"]
    B --> C["Per-marker inRange"]
    C --> D["3×3 MORPH_OPEN"]
    D --> E["3×3 MORPH_CLOSE"]
    E --> F["Contour-area filter"]
    F --> G["Candidate scoring and ambiguity check"]
    G --> H["Tip / pivot centers"]
    H --> I["Calculate θ, x"]
    I --> J["Timestamp differentiation and low-pass filtering"]
    J --> K["θ, ω, x, v"]
~~~

## 1. Raw Frame

![Raw frame](assets/vision-pipeline/01-raw-frame.jpg)

The camera is a Logitech C922 configured for 640×480 MJPEG at 30 FPS. The buffer size is one, and frame arrival is recorded with a monotonic timestamp.

In this frame, the blue marker is the pivot and the pink marker is the pendulum tip.

## 2. HSV Thresholds

![HSV threshold result](assets/vision-pipeline/02-hsv-threshold.png)

The BGR image is converted with `cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)`, then each marker range is passed to `cv2.inRange`.

Ranges used in the experiment:

| Marker | H | S | V | Allowed area |
| --- | ---: | ---: | ---: | ---: |
| pivot | 96–115 | 160–255 | 50–230 | 30–240 px² |
| tip | 152–179 | 30–220 | 30–255 | 16–250 px² |

The frame background brightness was 103 and the profile reference was 102. The lower and upper V bounds were shifted by +1 for this difference.

Brightness compensation is deliberately bounded.

| Difference from reference brightness | Behavior |
| --- | --- |
| `|ΔV| < 28` | Keep the current range and shift V by at most ±18 |
| `28 ≤ |ΔV| < 55` | Apply the same bounded correction and record a warning state |
| `|ΔV| ≥ 55` | Stop detection and require recalibration |

At this stage, each marker occupied 144 pixels.

## 3. Morphology

![Morphology result](assets/vision-pipeline/03-morphology.png)

A 3×3 elliptical kernel is applied to each binary mask.

~~~python
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
~~~

`MORPH_OPEN` removes small point noise, and `MORPH_CLOSE` fills small holes inside a marker. After processing, 135 pixels remained. The dark purple points in the image are pixels removed by this stage.

## 4. Contours and Centers

![Contours and state calculation](assets/vision-pipeline/04-contours-state.jpg)

After contours are found with `cv2.findContours`, only candidates inside the profile's area range are retained.

The following values are calculated for each candidate:

~~~text
solidity   = contour area / convex hull area
extent     = contour area / bounding-box area
area_score = exp(-|ln(area / sqrt(min_area × max_area))|)
continuity = exp(-distance from previous center / 120)
~~~

Final score:

~~~text
score = 0.38 × solidity
      + 0.22 × extent
      + 0.22 × area_score
      + 0.18 × continuity
~~~

The default acceptance threshold is 0.55. If the second candidate has at least 72% of the first candidate's area and at least 85% of its score, the tracker does not select one arbitrarily; it reports `AMBIGUOUS`.

Centers in this frame:

| Marker | x | y |
| --- | ---: | ---: |
| pivot | 276.352 px | 249.994 px |
| tip | 376.997 px | 145.358 px |

## 5. Cart Position and Pendulum Angle

Let the image coordinates of the rail ends be `x_L` and `x_R`, and the physical length be `L`. Cart position is:

~~~text
r = (pivot_x - x_L) / (x_R - x_L)
x_mm = r × L
~~~

For this experiment, `x_L=147.08 px`, `x_R=478.51 px`, and `L=512.738 mm`. The cart position in the example frame is 199.989 mm.

Pendulum angle uses upright as 0 rad.

~~~text
dx    = camera_x_sign × (tip_x - pivot_x)
dy_up = pivot_y - tip_y
θ_raw = atan2(dx, dy_up)
θ     = θ_raw - θ_reference(x)
~~~

Because the camera is not perfectly parallel to the rail, a position-dependent vertical reference is applied.

~~~text
θ_reference(x) = θ0 + slope × (x - x_ref)
~~~

The corrected angle in the example frame is +42.898°.

## 6. Angular and Cart Velocity

The actual timestamp difference between consecutive frames is used.

~~~text
Δθ = wrap_to_pi(θ[k] - θ[k-1])
ω_raw = Δθ / dt
v_raw = (x[k] - x[k-1]) / dt
~~~

Derivatives pass through a 12 Hz first-order low-pass filter.

~~~text
τ = 1 / (2πf_c)
α = dt / (τ + dt)
y[k] = y[k-1] + α(y_raw - y[k-1])
~~~

The previous state is not reused as though it were a new measurement in any of these cases:

- A required marker disappears.
- Multiple similar candidates make the result ambiguous.
- The frame interval exceeds 0.12 seconds.
- Tip and pivot are closer than 5 pixels.
- Lighting is far outside the profile range.
- A coordinate-calibration value is invalid.

## Related Code

- [Camera input](../raspberry_pi/camera.py)
- [Calibration](../raspberry_pi/calibration.py)
- [Color-marker tracking](../raspberry_pi/color_tracker.py)
- [State estimation](../raspberry_pi/state_estimator.py)
