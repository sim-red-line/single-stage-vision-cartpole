# 스윙업과 균형 제어

제어기는 에너지 기반 스윙업과 직렬형 위치/각도 PID를 하나의 상태기로 연결합니다. 카메라에서 계산한 `θ, ω, x, v`만 사용하며, 최종 출력은 RP2040에 보낼 signed velocity입니다.

## 제어 흐름

~~~mermaid
flowchart TD
    A["유효한 θ, ω, x, v"] --> B{"현재 모드"}
    B -->|swing_up| C["에너지 오차 계산"]
    C --> D["에너지 가속도 + 중앙복귀 PD"]
    D --> E{"포착 조건 충족?"}
    E -->|아니오| H["속도 적분"]
    E -->|예| F["현재 속도를 유지하고 balance 전환"]
    F --> G["위치 PID → 목표 각도"]
    G --> M{"release 각도 초과?"}
    M -->|아니오| I["각도 PID → 카트 가속도"]
    M -->|예| C
    I --> H
    B -->|balance| G
    H --> J["호스트 속도·가속도·제동거리 제한"]
    J --> K["RP2040 로컬 안전 제한"]
    K --> L["STEP/DIR/ENA"]
~~~

## 상태와 출력

제어 입력:

~~~text
θ : 수직 위쪽을 0으로 한 진자 각도
ω : 진자 각속도
x : 카트 위치
v : 카트 속도
~~~

제어 출력은 카트의 목표 속도입니다. 실물 설정에서는 angle PID 출력을 가속도로 해석한 뒤 실제 `dt`로 적분합니다.

## 균형 제어

외부 위치 루프가 작은 목표 각도를 만들고, 내부 각도 루프가 그 각도를 따라가도록 카트를 가속합니다.

~~~text
e_x = x_center - x
θ_set = clamp(PID_position(e_x), -θ_max, +θ_max)

e_θ = θ_set - θ
a_cmd = PID_angle(e_θ)

v_target[k] = v_applied[k-1] + output_sign × a_cmd × dt
~~~

구현에 포함된 처리:

- derivative-on-measurement
- estimator의 필터된 `ω`와 `v`를 D항에 사용
- 18 Hz D항 저역통과 필터
- 조건부 적분
- back-calculation anti-windup
- 게인 변경 시 bumpless update
- 최대 속도와 `a_max × dt` slew-rate 제한

Acceleration mode에서는 angle `Kp`가 `g=9806.65 mm/s²/rad`보다 작으면 실행을 거부합니다. 진자 복원 가속도를 만들 수 없는 설정을 실수로 구동하지 않기 위한 검사입니다.

## 에너지 스윙업

수직 위를 `θ=0`, 아래쪽을 `θ=±π`로 정의합니다. 진자 질량으로 나눈 정규화 에너지 오차는 다음과 같습니다.

~~~text
e = 0.5 × l × ω² / g + cos(θ) - 1 - e_margin
~~~

`e=0`은 수직 separatrix입니다. `e_margin`은 마찰로 사라지는 에너지를 보충하기 위한 작은 여유입니다.

에너지 주입 가속도:

~~~text
phase = e × (ω / ω_ref) × cos(θ)
a_energy = a_max × tanh(k_E × phase)
~~~

부호 함수 대신 `tanh`를 사용해 방향 전환 시 명령이 갑자기 뛰지 않게 했습니다.

레일 중앙을 유지하기 위한 PD 항:

~~~text
a_center = clamp(k_x(x_center - x) - k_v v,
                 -center_fraction × a_max,
                 +center_fraction × a_max)

a_total = clamp(a_energy + a_center, -a_max, +a_max)
~~~

아래쪽에서 `|ω|`가 매우 작을 때는 짧은 startup kick을 사용합니다. kick은 하향 허용각 안에서만 실행되며 수직 통과 중에는 다시 발생하지 않습니다.

## 포착과 재진입

기본 포착 조건:

~~~text
|θ| ≤ θ_capture
|ω| ≤ ω_capture
~~~

단순히 각도만 보고 전환하면 카트가 속도 한계 가까이에 있을 때 제어 여유가 사라질 수 있습니다. 선택적인 headroom 검사는 수직 평형점의 발산 모드를 이용합니다.

~~~text
z = ω + sqrt(g/l) × θ
Δv ≈ l × z
~~~

예상 속도 변화와 가속도 요구량이 설정한 reserve 안에 있을 때만 균형 모드로 넘깁니다.

전환할 때 현재 허용 속도를 보존합니다. 첫 balance 명령을 0으로 만들면 진자가 수직을 통과하는 순간 카트가 갑자기 제동되기 때문입니다.

균형 중 `release_angle_rad`를 벗어나면 다시 스윙업으로 돌아갑니다. capture와 release 값이 달라 전환 지점에서 모드가 빠르게 왕복하지 않습니다.

## 영상 지연 보정

에너지 스윙업과 균형 제어의 시간 보정은 별도 설정으로 분리되어 있습니다. 스윙업에도 같은 동역학 예측기를 적용할 수 있지만, 위 성공 기록에서는 스윙업 예측 시간을 0으로 두고 균형 단계에만 20 ms를 적용했습니다.

균형 단계에서는 선택적으로:

1. 필터된 각속도와 같은 시점의 과거 각도를 보간
2. 실제로 허용된 카트 가속도 이력을 사용
3. 진자 길이, 점성 감쇠와 Coulomb 마찰을 포함한 모델을 적분
4. 제한된 horizon 안에서 현재 `θ, ω`를 예측

예측 상태는 선택된 제어 계산에만 사용합니다. 카트 위치, 레일 경계, 추적 상태와 안전 판단은 원시 측정을 그대로 사용하며, 성공 기록의 포착 조건도 예측을 거치지 않은 측정값을 사용했습니다.

## 최종 안전 제한

~~~mermaid
flowchart LR
    A["Controller velocity"] --> B["호스트 속도 제한"]
    B --> C["호스트 가속도 제한"]
    C --> D["카메라 위치 기반 제동거리"]
    D --> E["USB serial"]
    E --> F["RP2040 watchdog"]
    F --> G["RP2040 가감속·제동거리"]
    G --> H["soft limit + 물리 limit"]
    H --> I["hardware PWM STEP"]
~~~

호스트 제동거리:

~~~text
d_stop = v²/(2a) + |v|t_latency + margin
~~~

진행 방향의 남은 거리가 `d_stop`보다 짧으면 바깥쪽 명령을 막고 반대 방향 제동만 허용합니다.

RP2040은 호스트와 별개로 watchdog, 호밍 상태, 양쪽 리미트, ESTOP, 최대 속도와 가감속을 다시 검사합니다.

## 실물 시험 설정

| 항목 | 값 |
| --- | ---: |
| 카트 중앙 | 256.369 mm |
| angle PID | 26000 / 0 / 3300 |
| position PID | 0.0000077 / 0 / 0.0001875 |
| 균형 속도 상한 | 240 mm/s |
| 균형 가속도 상한 | 2000 mm/s² |
| 스윙업 속도 상한 | 240 mm/s |
| 스윙업 가속도 상한 | 1500 mm/s² |
| 스윙업 이동 범위 | 중앙에서 ±160 mm |
| 에너지 게인 | 46 |
| 에너지 여유 | 0.045 |
| 포착 각도 | 0.22 rad |
| 포착 각속도 | 1.5 rad/s |
| 포착 headroom | 20% |
| balance 예측 horizon | 20 ms |
| rate 정렬 | 30 ms |

이 값은 해당 장비의 기록이며 다른 장비의 초기값으로 바로 사용하기 위한 값이 아닙니다.

## 관련 코드

- [PID와 스윙업 상태기](../raspberry_pi/pid_controller.py)
- [호스트 안전 상태기](../raspberry_pi/safety.py)
- [실행 스레드와 모드 전환](../raspberry_pi/runtime.py)
- [RP2040 펌웨어](../firmware/main.ino)
- [스윙업 검증 절차](swing_up_validation.md)
- [하드웨어 검증 절차](hardware_validation.md)
