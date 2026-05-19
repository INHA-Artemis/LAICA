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

By default, predictor output is safe:

```text
/laica/predicted_cmd_vel
```

To command the robot directly:

```bash
ros2 launch laica_bringup bringup_LAICA.launch \
  predicted_cmd_vel_topic_name:=/cmd_vel
```

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
/laica/predicted_cmd_vel     Safe predictor output
/laica/admittance_cmd_vel    Plot-test admittance output
```

## Major Parameters

### `bringup_LAICA.launch`

| Argument | Default | Meaning |
|---|---:|---|
| `start_unitree_driver` | `true` | Start Unitree driver |
| `start_arduino_sensor_node` | `true` | Start Arduino sensor publisher |
| `arduino_port` | `/dev/ttyACM0` | Arduino serial port |
| `load_cell_startup_calibration_sec` | `5.0` | Sensor calibration time |
| `predicted_cmd_vel_topic_name` | `/laica/predicted_cmd_vel` | Predictor output topic |
| `require_load_cell_calibration_done` | `true` | Wait for calibration flag |
| `load_cell_input_field` | `force_n` | Loadcell field used by controller |
| `force_deadband_n` | `10.0` | Ignore small force around zero |
| `force_velocity_sign` | `1.0` | Force-to-velocity sign |
| `base_velocity_mps` | `0.0` | Constant velocity offset |
| `max_velocity_mps` | `0.40` | Max `linear.x` command |
| `sensor_timeout_sec` | `0.25` | Stop if loadcell data is stale |

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
predicted_cmd_vel_topic_name: "/laica/predicted_cmd_vel"
require_load_cell_calibration_done: true
require_encoder: false
auto_zero_force: true
zero_force_duration_sec: 3.0
force_filter_tau_sec: 0.25
force_deadband_n: 10.0
force_velocity_sign: 1.0
admittance_mass: 40.0
admittance_damping: 160.0
base_velocity_mps: 0.0
min_velocity_mps: 0.0
max_velocity_mps: 0.40
max_accel_mps2: 0.50
sensor_timeout_sec: 0.25
```

## Useful Checks

```bash
ros2 topic echo /load_cell/calibration_done
ros2 topic echo /load_cell/data
ros2 topic echo /laica/predicted_cmd_vel
ros2 topic echo /odom
```
