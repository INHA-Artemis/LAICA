# LAICA Admittance vs Keyboard Comfort Analysis Report

Date: 2026-06-07  
Dataset: `/home/artemis/Documents/rosbags/06_07`

## 1. Goal

This analysis compares **admittance control** and **keyboard control** in terms of user comfort.

The main question is:

> Does admittance control reduce the amount of force that the user must apply compared with keyboard control?

The analysis focuses on force burden, because a comfortable controller should not require the user to push or pull with large force for a long time.

## 2. Data

The analyzed rosbag set contains:

- `JH_scene1_admit` to `JH_scene5_admit`
- `JH_scene1_key` to `JH_scene5_key`
- `ANDY_scene1_admit` to `ANDY_scene5_admit`
- `ANDY_scene1_key` to `ANDY_scene5_key`
- `ANDY_sceneIntend_admit`, `ANDY_sceneIntend_key`
- `JH_sceneIntend_admit`, `JH_sceneIntend_key`

Important topics:

- Admittance force: `/laica/debug/zeroed_force`
- Keyboard force: `/laica/teleop_debug/zeroed_force`
- Command velocity: `/cmd_vel` or debug command velocity
- Robot velocity: `/odom`

## 3. Comfort Metrics

The primary comfort metric is:

```text
time(|force| > 20 N) [%]
```

Reason:

- `10 N` is useful but can include small contact, bias, or vibration.
- `20 N` is a clearer sign that the user is intentionally pushing or pulling.
- `30 N` and above represent high-effort or uncomfortable periods.

Secondary metrics:

- mean `|force|`
- p95 `|force|`
- force variance
- time above `10, 15, 20, 25, 30, 35, 40 N`
- odom jerk p95
- lagged force-command correlation

## 4. JH Scene 1-5 Result

| Metric | Admittance | Keyboard | Interpretation |
|---|---:|---:|---|
| mean `|force|` | 10.53 N | 15.97 N | Admittance lower |
| p95 `|force|` | 28.50 N | 40.29 N | Admittance lower |
| force variance | 158.18 N^2 | 249.74 N^2 | Admittance lower |
| `|force| > 10 N` | 42.11% | 57.43% | Admittance lower |
| `|force| > 20 N` | 13.52% | 30.17% | Admittance much lower |
| `|force| > 30 N` | 5.27% | 16.00% | Admittance much lower |
| `|force| > 40 N` | 1.53% | 7.03% | Admittance much lower |
| odom jerk p95 | 213.24 | 210.08 | Similar |

JH result:

```text
Admittance clearly improves force comfort in normal scenes.
```

The strongest improvement appears in Scene 3 and Scene 4.  
Scene 5 is the main exception: keyboard has lower mean force and lower high-force duration than admittance.

## 5. ANDY Scene 1-5 Result

| Metric | Admittance | Keyboard | Interpretation |
|---|---:|---:|---|
| mean `|force|` | 8.21 N | 11.16 N | Admittance lower |
| p95 `|force|` | 23.13 N | 28.19 N | Admittance lower |
| force variance | 113.78 N^2 | 171.46 N^2 | Admittance lower |
| `|force| > 10 N` | 31.76% | 44.69% | Admittance lower |
| `|force| > 20 N` | 7.44% | 15.89% | Admittance lower |
| `|force| > 30 N` | 2.17% | 4.33% | Admittance lower |
| `|force| > 40 N` | 0.47% | 1.38% | Admittance lower |
| odom jerk p95 | 207.79 | 213.89 | Similar |

ANDY result:

```text
Admittance also improves comfort for ANDY in normal scenes.
```

However, Scene 3 and Scene 4 are mixed:

- Scene 3: admittance has lower mean force and variance, but keyboard has slightly lower `>20 N` time and lower odom jerk.
- Scene 4: keyboard is better in force metrics.

This suggests that admittance is generally beneficial, but not uniformly optimal across all scene conditions.

## 6. SceneIntend Result

| Bag | mean `|force|` | p95 `|force|` | `|force| > 20 N` | `|force| > 30 N` |
|---|---:|---:|---:|---:|
| ANDY admit | 11.64 N | 40.91 N | 19.86% | 11.08% |
| ANDY key | 14.29 N | 38.34 N | 29.59% | 8.06% |
| JH admit | 19.13 N | 52.78 N | 38.80% | 24.45% |
| JH key | 12.09 N | 31.52 N | 20.81% | 7.11% |

SceneIntend result:

```text
Admittance is not consistently better in intention-change scenarios.
```

For ANDY, admittance reduces mean force and `>20 N` time, but it has higher p95 force and higher `>30 N` time.

For JH, admittance is clearly worse than keyboard in SceneIntend:

- higher mean force
- higher p95 force
- higher `>20 N`, `>30 N`, and `>40 N` time

This indicates that the current admittance tuning may struggle when the user's intended speed or motion intent changes strongly.

## 7. Interpretation

For normal scenes, admittance control is successful:

- It reduces average force.
- It reduces force variance.
- It reduces high-force duration.
- It does not noticeably increase odom jerk.

This means the robot requires less sustained human effort compared with keyboard control.

However, the controller is not fully robust yet:

- Some scenes show keyboard outperforming admittance.
- SceneIntend especially shows a weakness.
- The controller may respond too slowly or with insufficient adaptation when user intention changes quickly.

## 8. Recommended Evaluation Criterion

Use this as the primary comfort score:

```text
Comfort score = time(|force| > 20 N) [%]
```

Use these as supporting metrics:

```text
mean |force|
force variance
time(|force| > 30 N) [%]
p95 |force|
odom jerk p95
```

A better controller should reduce force metrics while keeping odom jerk similar or lower.

## 9. Tuning Recommendation

Yes, mass and damping should be tuned.

Current behavior suggests:

- Normal scenes: controller works well.
- Intention-change scenes: controller may not respond appropriately.

Suggested tuning direction:

```text
If force remains high for too long:
  reduce admittance_mass
  or reduce admittance_damping slightly

If robot becomes jerky:
  increase damping
  or reduce max_accel_mps2

If force noise causes unwanted velocity:
  increase force_deadband_n
  or increase force_filter_tau_sec
```

A reasonable next experiment:

```text
Baseline current:
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

For each candidate, compare:

- `time(|force| > 20 N)`
- `time(|force| > 30 N)`
- force variance
- p95 force
- odom jerk p95

## 10. Conclusion

The admittance controller improves comfort for both JH and ANDY in normal scenes.

The strongest evidence is the reduction in high-force duration:

- JH `|force| > 20 N`: 30.17% keyboard to 13.52% admittance
- ANDY `|force| > 20 N`: 15.89% keyboard to 7.44% admittance

Therefore, the current admittance control is working well for normal walking/control scenes.

The remaining issue is robustness during intention-change scenarios. The `sceneIntend` bags show that the current tuning can require large forces when the user's intended motion changes. The next step should be mass/damping tuning using `|force| > 20 N` and `|force| > 30 N` as the main comfort metrics.

