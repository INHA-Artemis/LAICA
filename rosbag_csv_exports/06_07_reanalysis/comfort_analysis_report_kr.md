# LAICA Admittance vs Keyboard Comfort 분석 보고서

날짜: 2026-06-07  
데이터셋: `/home/artemis/Documents/rosbags/06_07`

## 1. 분석 목적

본 분석의 목적은 **admittance control**과 **keyboard control**을 사용자 comfort 관점에서 비교하는 것이다.

핵심 질문은 다음과 같다.

> Admittance control이 keyboard control에 비해 사용자가 가해야 하는 힘을 줄여주는가?

Comfort는 주로 force burden으로 평가하였다. 편한 controller라면 사용자가 오랫동안 큰 힘을 주지 않아도 로봇이 적절한 속도로 따라가야 한다.

## 2. 분석 데이터

분석한 rosbag은 다음과 같다.

- `JH_scene1_admit` to `JH_scene5_admit`
- `JH_scene1_key` to `JH_scene5_key`
- `ANDY_scene1_admit` to `ANDY_scene5_admit`
- `ANDY_scene1_key` to `ANDY_scene5_key`
- `ANDY_sceneIntend_admit`, `ANDY_sceneIntend_key`
- `JH_sceneIntend_admit`, `JH_sceneIntend_key`

사용한 주요 topic:

- Admittance force: `/laica/debug/zeroed_force`
- Keyboard force: `/laica/teleop_debug/zeroed_force`
- Command velocity: `/cmd_vel` 또는 debug command velocity
- Robot velocity: `/odom`

## 3. Comfort 평가 지표

주요 comfort 지표는 다음으로 설정하였다.

```text
time(|force| > 20 N) [%]
```

이유는 다음과 같다.

- `10 N`은 작은 접촉, 센서 offset, 보행 진동까지 포함될 수 있어 너무 민감하다.
- `20 N` 이상은 사용자가 의도적으로 밀거나 당기고 있다는 신호로 보기 좋다.
- `30 N` 이상은 high-effort 또는 uncomfortable 구간으로 볼 수 있다.

보조 지표는 다음을 사용하였다.

- 평균 `|force|`
- p95 `|force|`
- force variance
- `10, 15, 20, 25, 30, 35, 40 N` threshold 이상 force 지속 시간 비율
- odom jerk p95
- force-command lagged correlation

## 4. JH Scene 1-5 결과

| Metric | Admittance | Keyboard | 해석 |
|---|---:|---:|---|
| 평균 `|force|` | 10.53 N | 15.97 N | admittance가 낮음 |
| p95 `|force|` | 28.50 N | 40.29 N | admittance가 낮음 |
| force variance | 158.18 N^2 | 249.74 N^2 | admittance가 낮음 |
| `|force| > 10 N` | 42.11% | 57.43% | admittance가 낮음 |
| `|force| > 20 N` | 13.52% | 30.17% | admittance가 훨씬 낮음 |
| `|force| > 30 N` | 5.27% | 16.00% | admittance가 훨씬 낮음 |
| `|force| > 40 N` | 1.53% | 7.03% | admittance가 훨씬 낮음 |
| odom jerk p95 | 213.24 | 210.08 | 거의 비슷함 |

JH 결과 요약:

```text
일반 scene에서는 admittance control이 force comfort를 명확히 개선하였다.
```

특히 Scene 3과 Scene 4에서 개선이 크게 나타났다.  
다만 Scene 5는 예외였다. Scene 5에서는 keyboard가 평균 force와 high-force duration 측면에서 더 낮았다.

## 5. ANDY Scene 1-5 결과

| Metric | Admittance | Keyboard | 해석 |
|---|---:|---:|---|
| 평균 `|force|` | 8.21 N | 11.16 N | admittance가 낮음 |
| p95 `|force|` | 23.13 N | 28.19 N | admittance가 낮음 |
| force variance | 113.78 N^2 | 171.46 N^2 | admittance가 낮음 |
| `|force| > 10 N` | 31.76% | 44.69% | admittance가 낮음 |
| `|force| > 20 N` | 7.44% | 15.89% | admittance가 낮음 |
| `|force| > 30 N` | 2.17% | 4.33% | admittance가 낮음 |
| `|force| > 40 N` | 0.47% | 1.38% | admittance가 낮음 |
| odom jerk p95 | 207.79 | 213.89 | 거의 비슷하거나 admittance가 약간 낮음 |

ANDY 결과 요약:

```text
ANDY의 일반 scene에서도 admittance control이 전반적으로 comfort를 개선하였다.
```

하지만 Scene 3과 Scene 4는 mixed result이다.

- Scene 3: admittance는 평균 force와 force variance가 낮지만, keyboard가 `>20 N` 시간과 odom jerk에서 약간 더 좋다.
- Scene 4: keyboard가 force metric에서 더 좋다.

따라서 admittance는 전체적으로 효과가 있지만, 모든 scene에서 항상 최적은 아니다.

## 6. SceneIntend 결과

| Bag | 평균 `|force|` | p95 `|force|` | `|force| > 20 N` | `|force| > 30 N` |
|---|---:|---:|---:|---:|
| ANDY admit | 11.64 N | 40.91 N | 19.86% | 11.08% |
| ANDY key | 14.29 N | 38.34 N | 29.59% | 8.06% |
| JH admit | 19.13 N | 52.78 N | 38.80% | 24.45% |
| JH key | 12.09 N | 31.52 N | 20.81% | 7.11% |

SceneIntend 결과 요약:

```text
사용자 의도가 크게 변하는 상황에서는 admittance가 일관되게 좋다고 보기 어렵다.
```

ANDY의 경우 admittance가 평균 force와 `>20 N` 시간은 줄였지만, p95 force와 `>30 N` 시간은 keyboard보다 높았다. 즉, 평소 force 부담은 줄었지만 순간적으로 큰 force peak가 발생한 것으로 볼 수 있다.

JH의 경우 SceneIntend에서 admittance가 keyboard보다 명확히 나빴다.

- 평균 force가 더 큼
- p95 force가 더 큼
- `>20 N`, `>30 N`, `>40 N` 시간이 모두 더 큼

이는 현재 admittance parameter가 사용자의 intent 변화가 빠르거나 강한 상황에서는 충분히 잘 대응하지 못할 수 있음을 의미한다.

## 7. 해석

일반 scene에서는 admittance control이 성공적으로 동작하였다.

- 평균 force가 감소하였다.
- force variance가 감소하였다.
- high-force duration이 감소하였다.
- odom jerk는 keyboard 대비 크게 증가하지 않았다.

즉, keyboard control에 비해 사용자가 지속적으로 큰 힘을 가해야 하는 부담이 줄었다.

하지만 controller가 아직 완전히 robust하다고 보기는 어렵다.

- 일부 scene에서는 keyboard가 admittance보다 좋았다.
- SceneIntend에서는 특히 약점이 나타났다.
- 사용자의 의도가 빠르게 바뀔 때 controller response가 충분히 빠르지 않거나, tuning이 상황에 맞지 않을 가능성이 있다.

## 8. 추천 평가 기준

대표 comfort score는 다음을 추천한다.

```text
Comfort score = time(|force| > 20 N) [%]
```

보조 지표는 다음을 함께 사용한다.

```text
mean |force|
force variance
time(|force| > 30 N) [%]
p95 |force|
odom jerk p95
```

좋은 controller는 force 관련 지표를 낮추면서 odom jerk를 현재 수준 이하로 유지해야 한다.

## 9. Tuning 제안

Mass와 damping은 추가 tuning이 필요하다.

현재 결과는 다음을 보여준다.

- 일반 scene: controller가 잘 동작함
- intention-change scene: controller 성능이 불안정함

튜닝 방향은 다음과 같다.

```text
force가 오래 남는 경우:
  admittance_mass를 낮춘다
  또는 admittance_damping을 약간 낮춘다

robot motion이 너무 jerky한 경우:
  damping을 높인다
  또는 max_accel_mps2를 낮춘다

force noise에 의해 불필요한 velocity가 발생하는 경우:
  force_deadband_n을 높인다
  또는 force_filter_tau_sec을 높인다
```

다음 실험 후보:

```text
현재 baseline:
  M = 20
  B = 80

Candidate 1:
  M = 15
  B = 70

Candidate 2:
  M = 12
  B = 70

Candidate 3:
  M = 15
  B = 60
```

각 후보에 대해 다음을 비교한다.

- `time(|force| > 20 N)`
- `time(|force| > 30 N)`
- force variance
- p95 force
- odom jerk p95

## 10. 결론

Admittance controller는 JH와 ANDY의 일반 scene에서 keyboard control보다 comfort를 개선하였다.

가장 강한 근거는 high-force duration 감소이다.

- JH `|force| > 20 N`: keyboard 30.17%에서 admittance 13.52%로 감소
- ANDY `|force| > 20 N`: keyboard 15.89%에서 admittance 7.44%로 감소

따라서 현재 admittance control은 일반적인 walking/control scene에서는 잘 작동한다고 볼 수 있다.

남은 문제는 사용자 의도가 강하게 바뀌는 상황에서의 robustness이다. `sceneIntend` bag에서는 현재 tuning이 큰 force를 요구하는 경우가 나타났다. 다음 단계는 `|force| > 20 N`과 `|force| > 30 N`을 주요 comfort metric으로 사용하여 mass/damping을 튜닝하는 것이다.

