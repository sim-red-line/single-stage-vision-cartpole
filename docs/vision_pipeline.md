# 비전 처리 파이프라인

이 문서는 카메라 프레임에서 진자 각도와 카트 위치를 얻는 과정을 정리한 것입니다.

아래 이미지는 실험 `20261101T053856Z-adde3c2c`의 216번 프레임(7.176초)에 실제 프로파일 `angle-reference-20261101`을 적용한 결과입니다.

![비전 처리 전체 과정](assets/vision-pipeline/vision-pipeline-overview.jpg)

## 전체 흐름

~~~mermaid
flowchart LR
    A["BGR 원본 프레임"] --> B["BGR → HSV"]
    B --> C["마커별 inRange"]
    C --> D["3×3 MORPH_OPEN"]
    D --> E["3×3 MORPH_CLOSE"]
    E --> F["contour 면적 필터"]
    F --> G["후보 점수·모호성 검사"]
    G --> H["tip / pivot 중심점"]
    H --> I["θ, x 계산"]
    I --> J["timestamp 미분·저역통과"]
    J --> K["θ, ω, x, v"]
~~~

## 1. 원본 프레임

![원본 프레임](assets/vision-pipeline/01-raw-frame.jpg)

카메라는 Logitech C922를 640×480 MJPEG, 30 FPS로 사용했습니다. 버퍼 크기는 1로 두고, 영상이 들어온 시각을 monotonic timestamp로 기록합니다.

이 프레임에서 파란색 마커는 피벗, 분홍색 마커는 진자 끝점입니다.

## 2. HSV 임계값

![HSV 임계값 결과](assets/vision-pipeline/02-hsv-threshold.png)

BGR 영상을 `cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)`로 변환한 뒤 마커별 범위를 `cv2.inRange`에 넣습니다.

실험에 사용한 범위:

| 마커 | H | S | V | 허용 면적 |
| --- | ---: | ---: | ---: | ---: |
| pivot | 96~115 | 160~255 | 50~230 | 30~240 px² |
| tip | 152~179 | 30~220 | 30~255 | 16~250 px² |

프레임의 배경 밝기는 103, 프로파일 기준은 102였습니다. 두 값의 차이에 따라 V 하한과 상한에 +1을 적용했습니다.

밝기 보정은 제한적으로만 동작합니다.

| 기준 밝기와의 차이 | 동작 |
| --- | --- |
| `|ΔV| < 28` | 현재 범위를 유지하며 최대 ±18 안에서 V 이동 |
| `28 ≤ |ΔV| < 55` | 같은 제한 보정을 적용하고 경고 상태 기록 |
| `|ΔV| ≥ 55` | 검출을 중단하고 재보정 요구 |

이 단계에서 두 마커에 해당하는 픽셀은 모두 144개였습니다.

## 3. Morphology

![Morphology 결과](assets/vision-pipeline/03-morphology.png)

이진 마스크에 3×3 타원 커널을 사용합니다.

~~~python
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
~~~

`MORPH_OPEN`은 작은 점 노이즈를 제거하고, `MORPH_CLOSE`는 마커 내부의 작은 빈 공간을 메웁니다. 처리 후 남은 픽셀은 135개였습니다. 이미지에서 어두운 자주색 점은 이 과정에서 제거된 픽셀입니다.

## 4. Contour와 중심점

![Contour와 상태 계산](assets/vision-pipeline/04-contours-state.jpg)

`cv2.findContours`로 외곽선을 찾은 뒤 프로파일의 면적 범위에 들어오는 후보만 남깁니다.

후보마다 다음 값을 계산합니다.

~~~text
solidity   = contour area / convex hull area
extent     = contour area / bounding-box area
area_score = exp(-|ln(area / sqrt(min_area × max_area))|)
continuity = exp(-이전 중심점과의 거리 / 120)
~~~

최종 점수:

~~~text
score = 0.38 × solidity
      + 0.22 × extent
      + 0.22 × area_score
      + 0.18 × continuity
~~~

기본 통과 기준은 0.55입니다. 두 번째 후보의 면적이 첫 번째 후보의 72% 이상이고 점수도 85% 이상이면 하나를 임의로 고르지 않고 `AMBIGUOUS`로 처리합니다.

이 프레임의 중심점은 다음과 같습니다.

| 마커 | x | y |
| --- | ---: | ---: |
| pivot | 276.352 px | 249.994 px |
| tip | 376.997 px | 145.358 px |

## 5. 카트 위치와 진자 각도

레일의 영상 좌표를 `x_L`, `x_R`, 실제 길이를 `L`이라고 하면 카트 위치는 다음과 같습니다.

~~~text
r = (pivot_x - x_L) / (x_R - x_L)
x_mm = r × L
~~~

이 실험의 좌표는 `x_L=147.08 px`, `x_R=478.51 px`, `L=512.738 mm`입니다. 예시 프레임의 카트 위치는 199.989 mm입니다.

진자 각도는 수직 위쪽을 0 rad로 둡니다.

~~~text
dx    = camera_x_sign × (tip_x - pivot_x)
dy_up = pivot_y - tip_y
θ_raw = atan2(dx, dy_up)
θ     = θ_raw - θ_reference(x)
~~~

카메라가 레일과 완전히 평행하지 않아 위치에 따른 수직 기준도 적용합니다.

~~~text
θ_reference(x) = θ0 + slope × (x - x_ref)
~~~

예시 프레임의 보정된 각도는 +42.898°입니다.

## 6. 각속도와 카트 속도

연속 프레임의 실제 timestamp 차이를 사용합니다.

~~~text
Δθ = wrap_to_pi(θ[k] - θ[k-1])
ω_raw = Δθ / dt
v_raw = (x[k] - x[k-1]) / dt
~~~

미분값은 12 Hz 1차 저역통과 필터를 통과합니다.

~~~text
τ = 1 / (2πf_c)
α = dt / (τ + dt)
y[k] = y[k-1] + α(y_raw - y[k-1])
~~~

다음 상황에서는 이전 상태를 새 측정처럼 재사용하지 않습니다.

- 필수 마커가 사라짐
- 비슷한 후보가 여러 개라 모호함
- 프레임 간격이 0.12초를 넘음
- tip과 pivot 사이가 5 px보다 가까움
- 조명이 프로파일 범위를 크게 벗어남
- 좌표 보정값이 유효하지 않음

## 관련 코드

- [카메라 입력](../raspberry_pi/camera.py)
- [캘리브레이션](../raspberry_pi/calibration.py)
- [색상 마커 추적](../raspberry_pi/color_tracker.py)
- [상태 추정](../raspberry_pi/state_estimator.py)
