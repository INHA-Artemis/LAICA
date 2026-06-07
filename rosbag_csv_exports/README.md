# LAICA Rosbag Experiment Analysis Results

This directory contains analysis outputs generated from LAICA admittance-control
experiments. The raw rosbags are stored outside this workspace under:

```text
/home/artemis/Documents/rosbags/06_03
/home/artemis/Documents/rosbags/06_04
/home/artemis/Documents/rosbags/06_07
```

The purpose of this analysis is to evaluate whether force-based admittance
control can improve user comfort compared with keyboard control.

## High-Level Research Question

The experiments do **not** try to find one universal admittance parameter for
all users.

Instead, the main research question is:

```text
Can a force-based admittance control pipeline reduce sustained user force
burden, expose user-specific interaction preferences, and provide measurable
comfort metrics for future personalization?
```

This distinction matters because each user may prefer a different physical
interaction style. A blind or visually impaired person may also develop a
unique walking interaction pattern with their own guide dog. Therefore, the
goal is not a one-size-fits-all controller, but a user-adaptable control and
evaluation platform.

## Directory Overview

```text
rosbag_csv_exports/
├── README.md
├── convert_rosbag_all_topics.py
├── force_cmd_metrics_jh/
├── 06_07_comfort_analysis/
├── 06_07_reanalysis/
└── 06_07_preferred_param_analysis/
```

### `force_cmd_metrics_jh/`

Analysis of 06_04 JH repeated trials.

Important files:

```text
force_cmd_lagged_correlation.csv
force_variance.csv
force_metrics_grouped_summary.csv
force_cmd_lagged_correlation.png
force_variance.png
```

This directory is mainly used to evaluate:

- force variance
- force-command lagged correlation
- slow vs fast trial behavior
- admittance vs keyboard comparison

### `06_07_comfort_analysis/`

Initial 06_07 comfort analysis for JH scene 1-5.

Important files:

```text
06_07_comfort_trial_metrics.csv
06_07_comfort_mode_summary.csv
06_07_force_threshold_trial_metrics.csv
06_07_force_threshold_mode_summary.csv
force_threshold_sweep.png
force_threshold_sweep_by_scene.png
```

This directory focuses on threshold-based force burden, especially:

```text
time(|force| > 10, 15, 20, 25, 30, 35, 40 N)
```

### `06_07_reanalysis/`

Expanded 06_07 analysis including JH, ANDY, and sceneIntend bags.

Important files:

```text
06_07_trial_metrics.csv
06_07_JH_scene1_5_mode_summary.csv
06_07_ANDY_scene1_5_mode_summary.csv
06_07_sceneIntend_all_trial_metrics.csv
comfort_analysis_report.md
comfort_analysis_report_kr.md
```

This directory contains the main English and Korean experiment reports before
the preferred-parameter experiments were added.

### `06_07_preferred_param_analysis/`

Most recent analysis including additional user-preferred parameter experiments.

Important files:

```text
06_07_all_trial_metrics_with_params.csv
06_07_explicit_param_scene_summary.csv
06_07_group_summary_by_person_mode_param.csv
06_07_preferred_admittance_trials.csv
```

This is the most important directory for the latest interpretation of user
specific admittance tuning.

## Metrics

### Primary Comfort Metric

The primary metric used in the analysis is:

```text
time(|force| > 20 N) [%]
```

Rationale:

- `10 N` can include small contact, bias, or walking vibration.
- `20 N` is a clearer sign that the user is actively pushing or pulling.
- `30 N` and above indicate high-effort or uncomfortable interaction periods.

### Supporting Metrics

```text
mean |force|
p95 |force|
force variance
time(|force| > 30 N) [%]
time(|force| > 40 N) [%]
odom jerk p95
force-command lagged correlation
```

Interpretation:

- Lower mean force means lower overall physical effort.
- Lower high-force duration means the user does not need to push or pull hard
  for a long time.
- Lower force variance means a more stable physical interaction.
- Odom jerk checks whether the robot motion became rougher.
- Force-command correlation checks whether user force is meaningfully reflected
  in velocity command.

## 06_03 Experiments

The 06_03 data was the early comparison stage.

Available bags:

```text
3: MH_admittance
4: ANDY_keyboard
5: ANDY_admittance
6: JH_admittance
```

Main role of 06_03:

- verify that admittance debug topics were useful
- inspect force, admittance velocity, and odom trends
- identify that keyboard and admittance need comparable debug logging
- motivate force and odom based comfort metrics

The early interpretation was:

```text
Admittance appeared to work, but the logging was not yet clean enough for fair
keyboard-vs-admittance comparison.
```

This led to adding compact debug topics and later keyboard force logging.

## 06_04 Experiments

The 06_04 data added repeated trials.

Example structure:

```text
ad_jh_01 ... ad_jh_20
key_jh_01 ... key_jh_20
ad_mh_01 ... ad_mh_20
key_mh_01 ... key_mh_20
```

Odd trial numbers represent slow trials. Even trial numbers represent fast
trials.

### JH Repeated-Trial Findings

The JH 06_04 bags had force logs for both admittance and keyboard, so they are
the strongest repeated-trial comparison in this set.

Grouped result from `force_cmd_metrics_jh/force_metrics_grouped_summary.csv`:

| Person | Mode | Speed | Force variance | Force mean abs | Best lag corr |
|---|---|---|---:|---:|---:|
| JH | admittance | slow | 95.48 | 10.07 | 0.68 |
| JH | keyboard | slow | 260.63 | 19.05 | 0.37 |
| JH | admittance | fast | 453.01 | 14.85 | 0.72 |
| JH | keyboard | fast | 958.15 | 24.21 | 0.24 |

Interpretation:

```text
In repeated JH trials, admittance reduced force variance and mean force compared
with keyboard control.
```

The force-command lagged correlation was also higher under admittance:

```text
admittance slow: about 0.68
admittance fast: about 0.72
keyboard slow: about 0.37
keyboard fast: about 0.24
```

This means admittance created a clearer force-to-velocity coupling.

### 06_04 Limitation

The MH keyboard bags originally lacked comparable force debug logs. Therefore,
06_04 MH data is less reliable for direct force-comfort comparison than 06_04
JH data.

## 06_07 Normal-Scene Experiments

The 06_07 data added more structured scene-level comparisons for JH and ANDY.

Normal scene bags include:

```text
JH_scene1_admit ... JH_scene5_admit
JH_scene1_key ... JH_scene5_key
ANDY_scene1_admit ... ANDY_scene5_admit
ANDY_scene1_key ... ANDY_scene5_key
```

### JH Scene 1-5 Summary

From `06_07_reanalysis/06_07_JH_scene1_5_mode_summary.csv`:

| Metric | Admittance | Keyboard |
|---|---:|---:|
| mean `|force|` | 10.53 N | 15.97 N |
| p95 `|force|` | 28.50 N | 40.29 N |
| force variance | 158.18 N^2 | 249.74 N^2 |
| `|force| > 10 N` | 42.11% | 57.43% |
| `|force| > 20 N` | 13.52% | 30.17% |
| `|force| > 30 N` | 5.27% | 16.00% |
| `|force| > 40 N` | 1.53% | 7.03% |
| odom jerk p95 | 213.24 | 210.08 |

Interpretation:

```text
For JH normal scenes, admittance clearly reduced force burden. The average odom
jerk was close to keyboard, so comfort improvement did not come with a large
average smoothness penalty.
```

This average comparison should not be confused with the preferred-parameter
result. In the parameter-tuning trials, jerk differences became important for
explaining why JH preferred `B=80`.

Scene-level note:

- Scene 1-4: admittance was generally better.
- Scene 5: keyboard was better in some force metrics, suggesting scene-specific
  tuning issues.

### ANDY Scene 1-5 Summary

From `06_07_reanalysis/06_07_ANDY_scene1_5_mode_summary.csv`:

| Metric | Admittance | Keyboard |
|---|---:|---:|
| mean `|force|` | 8.21 N | 11.16 N |
| p95 `|force|` | 23.13 N | 28.19 N |
| force variance | 113.78 N^2 | 171.46 N^2 |
| `|force| > 10 N` | 31.76% | 44.69% |
| `|force| > 20 N` | 7.44% | 15.89% |
| `|force| > 30 N` | 2.17% | 4.33% |
| `|force| > 40 N` | 0.47% | 1.38% |
| odom jerk p95 | 207.79 | 213.89 |

Interpretation:

```text
For ANDY normal scenes, admittance also reduced force burden. The average odom
jerk was close to keyboard, so the normal-scene comparison does not show a large
smoothness penalty.
```

However, in the preferred-parameter trials, ANDY's preferred `B=60` had much
lower jerk than the default admittance setting. Therefore, jerk still matters
for explaining individual preference even if the mode-level average looks
similar.

Scene-level note:

- Scene 1, 2, and 5 supported admittance clearly.
- Scene 3 and 4 were mixed, showing that a single fixed parameter is not always
  best for every situation.

## 06_07 SceneIntend Experiments

The `sceneIntend` bags were used to test stronger intention changes.

From `06_07_reanalysis/06_07_sceneIntend_all_trial_metrics.csv`:

| Bag | mean `|force|` | p95 `|force|` | `|force| > 20 N` | `|force| > 30 N` |
|---|---:|---:|---:|---:|
| ANDY admit | 11.64 N | 40.91 N | 19.86% | 11.08% |
| ANDY key | 14.29 N | 38.34 N | 29.59% | 8.06% |
| JH admit | 19.13 N | 52.78 N | 38.80% | 24.45% |
| JH key | 12.09 N | 31.52 N | 20.81% | 7.11% |

Interpretation:

```text
Admittance was not consistently better during intention-change scenes.
```

For ANDY, admittance reduced mean force and `|force| > 20 N`, but p95 force and
`|force| > 30 N` were higher than keyboard.

For JH, admittance was worse than keyboard in sceneIntend. This suggests that
rapid or strong user-intention changes require better tuning or adaptive
control.

## Preferred-Parameter Experiments

Additional experiments tested user-preferred parameters:

```text
JH preferred:   B = 80
ANDY preferred: B = 60
MH preferred:   B = 50, M = 15
```

These results are in:

```text
06_07_preferred_param_analysis/
```

The key finding is:

```text
User preference does not always match the parameter with the lowest force
metric. Comfort appears to combine force burden, motion smoothness, response
feeling, and subjective preference.
```

This means that the preferred parameter should be interpreted as a
multi-objective comfort choice, not as the winner of one metric.

The factors used to interpret preference are:

```text
1. Force burden:
   mean |force|, p95 |force|, time(|force| > 20 N), time(|force| > 30 N)

2. Interaction stability:
   force variance

3. Motion smoothness:
   odom jerk p95

4. Response feeling:
   inferred from the trade-off between force reduction and smoothness
```

Important caution:

```text
The rosbags provide objective evidence, but they do not fully explain subjective
preference. Therefore, the preference explanation below should be read as a
data-grounded interpretation, not as a final psychological conclusion.
```

### JH Preferred Parameter

JH preferred `B=80`.

Scene 3:

| Setting | mean force | `>20 N` | `>30 N` | variance | odom jerk |
|---|---:|---:|---:|---:|---:|
| B60 | 11.27 | 15.40% | 5.82% | 212.35 | 247.81 |
| B80 | 9.54 | 14.26% | 6.68% | 169.91 | 229.40 |
| default | 9.38 | 11.88% | 4.51% | 153.77 | 274.91 |

Interpretation:

```text
JH's preference for B80 is reasonable because B80 reduces odom jerk compared
with default, even though default has slightly better force metrics.
```

This suggests JH may prioritize stable and less jerky motion over absolute
minimum force.

Detailed preference factor:

```text
Main factor for JH: motion smoothness / stability
Secondary factor: avoiding high force without making the robot feel too reactive
```

Why:

- Compared with `B60`, `B80` has lower mean force, lower `|force| > 20 N`, lower
  force variance, and lower odom jerk.
- Compared with `default`, `B80` has slightly worse force metrics but much lower
  odom jerk.
- Therefore, JH's preference is best explained by a balance between force burden
  and smoother robot motion.

In other words:

```text
JH did not appear to choose the most force-minimizing setting. JH appeared to
prefer the setting that made the robot motion feel more stable while still
keeping force burden reasonably low.
```

### ANDY Preferred Parameter

ANDY preferred `B=60`.

Scene 5:

| Setting | mean force | `>20 N` | `>30 N` | variance | odom jerk |
|---|---:|---:|---:|---:|---:|
| B60 | 10.10 | 13.74% | 1.91% | 168.56 | 135.59 |
| B70 | 10.48 | 9.79% | 2.84% | 175.23 | 126.57 |
| B90 | 14.22 | 23.54% | 15.74% | 408.75 | 180.32 |
| default | 9.49 | 5.74% | 1.83% | 138.51 | 210.80 |

Interpretation:

```text
ANDY's B60 preference may be related to smoother robot motion. B60 has much
lower odom jerk than default.
```

However, B70 should be tested again because it had even lower odom jerk and
lower `|force| > 20 N` than B60 in scene 5.

Detailed preference factor:

```text
Main factor for ANDY: smoother motion / lower jerk compared with default
Secondary factor: avoiding very overdamped response
```

Why:

- `default` has the best force metrics in scene 5, but its odom jerk is much
  higher than `B60`.
- `B60` greatly reduces odom jerk compared with `default`.
- `B90` performs poorly in force burden and variance, suggesting too much
  damping or a less responsive feel.
- `B70` is objectively promising because it has lower `|force| > 20 N` and
  lower odom jerk than `B60` in scene 5, but the user reported `B60` as
  preferred.

This suggests:

```text
ANDY's preference cannot be explained by force minimization alone. ANDY may have
preferred the response feel of B60, even though B70 looked slightly better in
some objective metrics.
```

The correct next step is not to declare `B60` universally best for ANDY, but to
repeat `B60` vs `B70` with subjective ratings.

### MH Preferred Parameter

MH preferred `B=50, M=15`.

Scene 3:

| Setting | mean force | `>20 N` | `>30 N` | variance | odom jerk |
|---|---:|---:|---:|---:|---:|
| B50 M15 | 12.64 | 21.51% | 10.58% | 201.40 | 140.75 |
| B60 | 11.76 | 21.10% | 9.46% | 218.26 | 151.19 |
| B80 | 13.18 | 22.99% | 10.56% | 274.45 | 234.70 |

Interpretation:

```text
MH's preferred setting has the lowest odom jerk and lowest force variance among
the tested admittance parameters.
```

This suggests MH may prefer a smoother and more stable interaction, even if the
high-force duration is not the absolute minimum.

Detailed preference factor:

```text
Main factor for MH: low jerk and low interaction variability
Secondary factor: responsive feel from lower mass and damping
```

Why:

- `B50 M15` has the lowest odom jerk among MH admittance settings.
- `B50 M15` also has the lowest force variance among MH admittance settings.
- `B60` has slightly better `|force| > 20 N` and `|force| > 30 N`, but its force
  variance and jerk are higher than `B50 M15`.
- `B80` has the highest force variance and highest odom jerk among the tested
  MH admittance settings.

This suggests:

```text
MH likely preferred the setting that felt smooth and consistent, even though it
was not the absolute minimum in high-force duration.
```

The addition of `M=15` may have also changed response feel. Lower mass can make
the admittance velocity respond more readily to force input, while lower damping
can reduce the effort needed to influence velocity. For MH, this combination
appears to have produced a smoother and more stable interaction than `B80`.

### Cross-User Preference Difference

The preferred settings show three different comfort profiles:

| User | Preferred setting | Best-supported preference factor | Evidence |
|---|---|---|---|
| JH | `B=80` | smoother/stable motion with moderate force burden | lower jerk than default, lower variance than B60 |
| ANDY | `B=60` | smoother motion and preferred response feel | much lower jerk than default, but not minimum force |
| MH | `B=50, M=15` | low jerk and low force variability | lowest jerk and variance among MH admittance settings |

The main conclusion from this table is:

```text
Each user appears to weight comfort factors differently.
```

JH appears to prefer stability without too much reactivity. ANDY appears to care
strongly about smooth motion and response feel, but the current data cannot
fully distinguish `B60` from `B70`. MH appears to prefer a more responsive
low-mass setting when it also produces low jerk and low force variance.

This supports the platform-level claim:

```text
The important contribution is not one fixed parameter. The important
contribution is the ability to measure and tune force-based interaction
according to each user's comfort profile.
```

## Smoothing Filter vs Force-Based Control

A velocity smoothing filter and force-based admittance control solve different
problems.

| Method | Input | Main effect | Limitation |
|---|---|---|---|
| Velocity smoothing | command velocity | makes command changes smoother | does not know user force or intent |
| Force-based admittance | human force | adapts velocity based on physical interaction | needs user-specific tuning |

Key distinction:

```text
A velocity smoothing filter improves command smoothness.
Force-based admittance closes the physical interaction loop between the user
and robot.
```

If the robot is too slow and the user keeps pushing:

- smoothing only smooths the existing command
- admittance can convert the user's force into velocity adaptation

This is why force-based control is needed for comfort-oriented physical
interaction.

## Overall Conclusion

Across the available rosbags, LAICA admittance control can be featured as:

```text
A comfort-oriented force-to-velocity interface that reduces sustained user force
burden and force variability in normal scenes, while exposing user-specific
admittance preferences.
```

Main evidence:

1. 06_04 JH repeated trials showed lower force variance and stronger
   force-command coupling under admittance.
2. 06_07 JH and ANDY normal scenes showed lower mean force, lower high-force
   duration, and lower force variance under admittance.
3. Odom jerk was close to keyboard on average in normal scenes, meaning
   admittance did not add a large average jerk penalty.
4. Preferred-parameter experiments showed that jerk and force variance can still
   strongly influence individual preference.
5. Preferred-parameter experiments showed that different users prefer different
   admittance responses.

Important limitation:

```text
Admittance should not be presented as universally better in every scene.
Intention-change scenarios and user-specific preferences require additional
tuning and subjective evaluation.
```

### Final Detailed Conclusion

The experiments support three levels of conclusion.

First, at the controller level:

```text
Force-based admittance control is meaningfully different from keyboard control
or velocity smoothing because it uses the user's physical force as part of the
control loop.
```

The 06_04 and 06_07 results show that this can reduce force burden in normal
walking scenes. JH and ANDY both showed lower mean force, lower high-force
duration, and lower force variance under admittance than keyboard control in
normal scenes. This supports the claim that admittance can reduce sustained
physical effort.

Second, at the comfort level:

```text
Comfort is not explained by force magnitude alone.
```

The preferred-parameter experiments show that the user-preferred setting was not
always the one with the lowest `|force| > 20 N`. For example, JH's `B80` setting
reduced odom jerk compared with default, even though default had slightly better
force metrics. ANDY preferred `B60`, which greatly reduced jerk compared with
default, even though default had lower force burden. MH preferred `B50 M15`,
which had the lowest jerk and force variance among MH admittance settings.

Therefore, the best current definition of comfort is:

```text
comfort = force burden + interaction stability + motion smoothness + response
feeling + subjective preference
```

Third, at the platform level:

```text
LAICA should be presented as a user-adaptable admittance-control and evaluation
pipeline, not as a one-size-fits-all controller.
```

This is especially important for blind and visually impaired mobility contexts,
where each person may have a distinct interaction style and may also be used to
the behavior of their own guide dog. A single fixed admittance parameter is not
expected to satisfy every person. The stronger contribution is that the LAICA
pipeline can record force, velocity, odom, and debug signals, compute comfort
metrics, and reveal how different users prefer different force-to-velocity
responses.

The final finding is:

```text
Admittance control is promising because it reduces force burden in normal
scenes, but its real value is personalization. Future work should tune and
evaluate admittance parameters per user using both objective metrics and
subjective ratings.
```

## Recommended Next Experiments

The next experiments should test the platform, not just a single parameter.

Recommended conditions:

```text
1. Keyboard
2. Keyboard + velocity smoothing
3. Current admittance
4. User-preferred admittance
```

Recommended scenes:

```text
1. Normal walking
2. Slow-fast speed changes
3. Stop-go transitions
4. Intention-change/random reference speed
```

Recommended subjective ratings after each trial:

```text
Effort:       1 easy - 7 hard
Smoothness:   1 jerky - 7 smooth
Delay:        1 no delay - 7 strong delay
Naturalness:  1 unnatural - 7 natural
Overall:      1 uncomfortable - 7 comfortable
```

Why subjective ratings are needed:

```text
The preferred-parameter experiments show that objective force metrics alone do
not fully explain user preference.
```

## Reproducing Analysis

The reusable script for force-command correlation and force variance is:

```text
../src/laica_bringup/scripts/analyze_force_cmd_metrics.py
```

Example:

```bash
cd /home/artemis/Documents/LAICA_ws
src/laica_bringup/scripts/analyze_force_cmd_metrics.py \
  /home/artemis/Documents/rosbags/06_04 \
  --pattern '*_jh_*' \
  --output-dir rosbag_csv_exports/force_cmd_metrics_jh
```
