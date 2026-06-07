# LAICA ROS 2 Bringup

This workspace contains the LAICA robot dog bringup, sensor publisher, Unitree
driver, and force-based admittance controller.

## Packages

```text
src/
├── LAICA_enclc/      # Arduino loadcell/encoder messages and publisher
├── unitree_ros/      # Unitree Go1 ROS 2 driver
└── laica_bringup/    # Integrated launch files and admittance controller
```

## Build

```bash
cd /home/artemis/Documents/LAICA_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Files

### Real Robot

Use this for real robot dog operation:

```bash
ros2 launch laica_bringup bringup_LAICA.launch
```

This starts:

```text
unitree driver
arduino_sensor_pub
laica_velocity_predictor
```

Important behavior:

```text
arduino_sensor_pub waits 5 s for loadcell startup calibration
-> publishes /load_cell/calibration_done
-> laica_velocity_predictor starts only after calibration_done == true
```

The current predictor config sends output directly to the robot command topic:

```text
/cmd_vel
```

Do not run another `/cmd_vel` publisher at the same time.

To drive with the simplified keyboard forward/stop controller:

```bash
ros2 launch laica_bringup bringup_LAICA_keyboard_teleop.launch.py
```

This starts:

```text
unitree driver
arduino_sensor_pub
keyboard_forward_stop.py
```

The keyboard node is the only motion-command source to `/cmd_vel` in this mode.
It does not use full twist teleop. Press `w` once to start following the selected
forward reference-speed scenario, press `s` to stop, and press `q` to quit.
Angular velocity is always zero.

To choose one of the deterministic speed scenarios:

```bash
ros2 launch laica_bringup bringup_LAICA_keyboard_teleop.launch.py \
  scenario:=3
```

To wait longer for the robot to come online:

```bash
ros2 launch laica_bringup bringup_LAICA_keyboard_teleop.launch.py \
  robot_connection_timeout_sec:=30.0
```

Keyboard defaults live in:

```text
src/laica_bringup/config/keyboard_forward_stop_params.yaml
```

To run the admittance controller with the same deterministic reference-speed
scenarios:

```bash
ros2 launch laica_bringup bringup_LAICA_random_reference.launch.py
```

To choose a scenario from the command line:

```bash
ros2 launch laica_bringup bringup_LAICA_random_reference.launch.py \
  scenario:=3
```

This starts:

```text
unitree driver
arduino_sensor_pub
laica_velocity_predictor
```

The predictor publishes directly to `/cmd_vel` in this launch. Do not run the
keyboard controller and admittance controller at the same time unless they publish
to different command topics.

### Rosbag / Plot Test

Use this for rosbag replay and visual checking:

```bash
ros2 launch laica_bringup live_admittance_plot.launch.py
```

Then play a bag:

```bash
ros2 bag play /home/artemis/Documents/rosbags/05_15/MH_8/MH_8_0.db3
```

This launch starts both:

```text
laica_velocity_predictor
live_plot_admittance.py
```

It plots:

```text
/load_cell/data
/laica/admittance_cmd_vel
/odom
```

For rosbag use, this launch disables calibration flag waiting because old bags do
not contain `/load_cell/calibration_done`.

## Main Topics

```text
/load_cell/data              Loadcell input
/load_cell/calibration_done  Startup calibration flag
/encoder/data                Encoder data, not used by 1D admittance by default
/odom                        Robot feedback
/cmd_vel                     Unitree command input
/laica/predicted_cmd_vel     Optional non-robot predictor output
/laica/admittance_cmd_vel    Plot-test admittance output
/laica/debug/*               Admittance debug values
/laica/teleop_debug/*        Keyboard/reference debug values
```

## Major Parameters

### `bringup_LAICA.launch`

| Argument | Default | Meaning |
|---|---:|---|
| `start_unitree_driver` | `true` | Start Unitree driver |
| `start_arduino_sensor_node` | `true` | Start Arduino sensor publisher |
| `arduino_port` | `/dev/ttyACM0` | Arduino serial port |
| `load_cell_startup_calibration_sec` | `5.0` | Sensor calibration time |
| `predicted_cmd_vel_topic_name` | `/cmd_vel` | Predictor output topic from config |
| `require_load_cell_calibration_done` | `true` | Wait for calibration flag |
| `load_cell_input_field` | `force_n` | Loadcell field used by controller |
| `force_deadband_n` | `5.0` | Ignore small force around zero |
| `force_velocity_sign` | `1.0` | Force-to-velocity sign |
| `base_velocity_mps` | `0.5` | Fallback/reference velocity offset |
| `min_velocity_mps` | `0.3` | Min `linear.x` command |
| `max_velocity_mps` | `1.0` | Max `linear.x` command |
| `max_accel_mps2` | `1.00` | Command acceleration limit |
| `sensor_timeout_sec` | `0.25` | Stop if loadcell data is stale |

### `bringup_LAICA_keyboard_teleop.launch.py`

| Argument | Default | Meaning |
|---|---:|---|
| `scenario` | `5` | Reference-speed scenario number, `1` to `5` |
| `start_unitree_driver` | `true` | Start Unitree driver |
| `start_arduino_sensor_node` | `true` | Start Arduino sensor publisher |
| `robot_connection_timeout_sec` | `10.0` | Wait time for robot IP before exiting |
| `unitree_robot_ip` | `192.168.123.161` | Robot IP address |
| `keyboard_params_file` | `keyboard_forward_stop_params.yaml` | Keyboard controller config |
| `random_reference_params_file` | `random_reference_speed_params.yaml` | Shared scenario config |

### `bringup_LAICA_random_reference.launch.py`

| Argument | Default | Meaning |
|---|---:|---|
| `scenario` | `5` | Reference-speed scenario number, `1` to `5` |
| `start_unitree_driver` | `true` | Start Unitree driver |
| `start_arduino_sensor_node` | `true` | Start Arduino sensor publisher |
| `predictor_params_file` | `velocity_predictor_params.yaml` | Admittance controller config |
| `random_reference_params_file` | `random_reference_speed_params.yaml` | Shared scenario config |

### `live_admittance_plot.launch.py`

| Argument | Default | Meaning |
|---|---:|---|
| `window_sec` | `20.0` | Plot window length |
| `update_hz` | `10.0` | Plot update rate |
| `force_topic` | `/load_cell/data` | Force input topic |
| `cmd_vel_topic` | `/laica/admittance_cmd_vel` | Admittance output topic |
| `odom_topic` | `/odom` | Odom feedback topic |
| `show_odom` | `true` | Show odom subplot |

Internal rosbag-test defaults:

```text
require_load_cell_calibration_done = false
base_velocity_mps = 0.5
max_velocity_mps = 0.80
force_velocity_sign = -1.0
```

## Predictor Config

Default predictor parameters are in:

```text
laica_bringup/config/velocity_predictor_params.yaml
```

Main values:

```yaml
load_cell_topic_name: "/load_cell/data"
load_cell_calibration_done_topic_name: "/load_cell/calibration_done"
predicted_cmd_vel_topic_name: "/cmd_vel"
require_load_cell_calibration_done: true
require_encoder: false
auto_zero_force: true
zero_force_duration_sec: 3.0
force_filter_tau_sec: 0.15
force_deadband_n: 5.0
force_velocity_sign: 1.0
admittance_mass: 15.0
admittance_damping: 50.0
base_velocity_mps: 0.5
min_velocity_mps: 0.3
max_velocity_mps: 1.0
max_accel_mps2: 1.00
sensor_timeout_sec: 0.25
```

## Deterministic Reference Scenarios

Shared reference-speed scenarios are in:

```text
laica_bringup/config/random_reference_speed_params.yaml
```

The YAML uses a shared ROS 2 wildcard block, so the same scenario definitions are
loaded by both `keyboard_forward_stop` and `laica_velocity_predictor`.

```yaml
"/**":
  ros__parameters:
    random_reference_speed_enabled: true
    random_reference_speed_scenario_id: 5
    random_reference_speed_loop: true
```

Each scenario runs for up to 30 s and stays inside the configured safety range.
The current scenario ranges are:

```text
scenario 1: 0.30 - 0.90 m/s, 6 levels
scenario 2: 0.35 - 1.00 m/s, 6 levels
scenario 3: 0.30 - 1.00 m/s, 6 levels
scenario 4: 0.30 - 0.95 m/s, 10 levels
scenario 5: 0.30 - 1.00 m/s, 6 levels
```

Use `scenario:=N` at launch time to override the YAML-selected scenario.

## Debug Topics

Keyboard forward/stop debug topics:

```text
/laica/teleop_debug/raw_force
/laica/teleop_debug/zeroed_force
/laica/teleop_debug/filtered_force
/laica/teleop_debug/reference_velocity
/laica/teleop_debug/command_velocity
```

Admittance debug topics:

```text
/laica/debug/raw_force
/laica/debug/zeroed_force
/laica/debug/filtered_force
/laica/debug/effective_force
/laica/debug/admittance_velocity
/laica/debug/control_dt
/laica/debug/admittance_accel
/laica/debug/reference_velocity
/laica/debug/command_velocity
```

## Useful Checks

```bash
ros2 topic echo /load_cell/calibration_done
ros2 topic echo /load_cell/data
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

## Experiment Summary

Detailed rosbag analysis results are documented in:

```text
/home/artemis/Documents/LAICA_ws/rosbag_csv_exports/README.md
```

Raw experiment bags:

```text
/home/artemis/Documents/rosbags/06_03
/home/artemis/Documents/rosbags/06_04
/home/artemis/Documents/rosbags/06_07
```

### Main Finding

LAICA admittance control should be described as a **user-adaptable
force-to-velocity interface**, not as a one-size-fits-all controller.

Normal-scene results show that admittance reduced sustained force burden:

```text
JH scene 1-5:
  mean |force|:       10.53 N admittance vs 15.97 N keyboard
  |force| > 20 N:     13.52% admittance vs 30.17% keyboard
  force variance:     158.18 admittance vs 249.74 keyboard

ANDY scene 1-5:
  mean |force|:        8.21 N admittance vs 11.16 N keyboard
  |force| > 20 N:      7.44% admittance vs 15.89% keyboard
  force variance:     113.78 admittance vs 171.46 keyboard
```

The 06_04 JH repeated trials also showed lower force variance under admittance:

```text
slow trials:  95.5 admittance vs 260.6 keyboard
fast trials: 453.0 admittance vs 958.2 keyboard
```

### Odom Jerk Interpretation

In the 06_07 normal-scene mode averages, odom jerk p95 was close between
admittance and keyboard:

```text
JH:   213.24 admittance vs 210.08 keyboard
ANDY: 207.79 admittance vs 213.89 keyboard
```

This means admittance did not create a large average jerk penalty in normal
scenes.

However, jerk was still important in the preferred-parameter experiments:

```text
JH preferred B80:
  B80 had lower jerk than default, while force stayed reasonable.

ANDY preferred B60:
  B60 had much lower jerk than default, even though default minimized force.

MH preferred B50 M15:
  B50 M15 had the lowest jerk and lowest force variance among MH admittance
  settings.
```

Correct interpretation:

```text
Average keyboard-vs-admittance jerk was similar in normal scenes, but jerk was a
key factor for explaining individual parameter preference.
```

### Preferred Parameters

Additional 06_07 trials showed different user preferences:

```text
JH preferred:   B = 80
ANDY preferred: B = 60
MH preferred:   B = 50, M = 15
```

The preference difference was not explained by force minimization alone.

```text
JH:   likely valued stable, less-jerky motion with moderate force burden.
ANDY: likely valued smoother response feel; B60 lowered jerk substantially.
MH:   likely valued low jerk and low interaction variability.
```

Therefore:

```text
comfort = force burden + force variance + motion smoothness + response feeling
          + subjective preference
```

Future experiments should include subjective ratings after each trial, because
objective force metrics alone do not fully explain preference.
