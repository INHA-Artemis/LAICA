# LAICA ROS 2 Bringup

This workspace is organized around a ROS 2 integrated bringup for LAICA.

Related package documents:

- [LAICA_enclc README](./LAICA_enclc/README.md)
- [unitree_ros README](./unitree_ros/README.md)

## Packages

```text
src/
├── LAICA_enclc/      # Arduino loadcell/encoder ROS 2 messages and publisher
├── unitree_ros/      # Unitree Go1 ROS 2 driver and robot interfaces
└── laica_bringup/    # Integrated bringup and LAICA predictor node
```

## Main Bringup

The integrated launch file is:

```text
laica_bringup/launch/bringup_LAICA.launch
```

Build:

```bash
cd /home/artemis/Documents/LAICA_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run:

```bash
ros2 launch laica_bringup bringup_LAICA.launch
```

Run with Wi-Fi Unitree connection:

```bash
ros2 launch laica_bringup bringup_LAICA.launch \
  wifi:=true \
  unitree_robot_ip:=192.168.12.1
```

Run with a different Arduino port:

```bash
ros2 launch laica_bringup bringup_LAICA.launch arduino_port:=/dev/ttyACM0
```

## Data Flow

```text
Arduino
  -> arduino_sensor_pub
    -> /load_cell/data
    -> /encoder/data

Unitree driver
  -> /imu
  -> /odom
  <- /cmd_vel

LAICA predictor
  <- /load_cell/data
  <- /encoder/data
  -> /laica/predicted_cmd_vel
```

By default, the predictor does **not** publish directly to `/cmd_vel`.
This avoids topic-name conflicts with the Unitree command input.

When the predictor is ready to control the robot directly:

```bash
ros2 launch laica_bringup bringup_LAICA.launch predicted_cmd_vel_topic_name:=/cmd_vel
```

Use this only when no other controller is publishing to `/cmd_vel`.

## Current Predictor Behavior

The current node:

```text
laica_bringup/src/laica_velocity_predictor.cpp
```

subscribes:

```text
/load_cell/data    enc_lc/msg/LoadCellData
/encoder/data      enc_lc/msg/EncoderData
```

publishes:

```text
/laica/predicted_cmd_vel    geometry_msgs/msg/Twist
```

It implements a conservative 1D force admittance controller:

```text
M * d(v_adm)/dt + B * v_adm = F_eff
cmd_vel.linear.x = base_velocity_mps + v_adm
```

Only `linear.x` is commanded. `linear.y` and `angular.z` stay zero for the
first force-only controller. Encoder data is subscribed for compatibility, but
is not required by default.

The force signal is zeroed at startup, low-pass filtered, passed through a
deadband, integrated by the admittance model, then clamped and rate-limited.

## Hyperparameter And Topic Settings

Predictor parameters are defined in:

```text
laica_bringup/config/velocity_predictor_params.yaml
```

Current parameters:

```yaml
laica_velocity_predictor:
  ros__parameters:
    load_cell_topic_name: "/load_cell/data"
    encoder_topic_name: "/encoder/data"
    predicted_cmd_vel_topic_name: "/laica/predicted_cmd_vel"
    publish_rate: 50.0
    load_cell_input_field: "force_n"
    admittance_enabled: true
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

The admittance controller should use `LoadCellData.force_n`. Keep the loadcell
relaxed during the startup zeroing window so the predictor can subtract a local
force offset before computing velocity.

Use `force_velocity_sign` to choose whether positive calibrated force should
increase or decrease `linear.x`. Start by publishing to `/laica/predicted_cmd_vel`
and inspect the output before remapping to `/cmd_vel`.

Launch arguments can override these values:

| Argument | Default | Meaning |
|---|---:|---|
| `start_unitree_driver` | `true` | Start Unitree driver |
| `start_arduino_sensor_node` | `true` | Start Arduino sensor publisher |
| `wifi` | `false` | Use Unitree Wi-Fi IP |
| `unitree_robot_ip` | `192.168.123.161` | Unitree robot IP address |
| `arduino_port` | `/dev/ttyACM0` | Arduino serial port |
| `arduino_baud` | `115200` | Arduino serial baud rate |
| `arduino_publish_rate` | `100.0` | Arduino sensor publish rate |
| `load_cell_topic_name` | `/load_cell/data` | Loadcell topic |
| `encoder_topic_name` | `/encoder/data` | Encoder topic |
| `predicted_cmd_vel_topic_name` | `/laica/predicted_cmd_vel` | Predictor output velocity topic |
| `predictor_publish_rate` | `50.0` | Predictor output publish rate |
| `load_cell_input_field` | `force_n` | Loadcell field used by the controller |
| `admittance_enabled` | `true` | Enable force admittance output |
| `force_deadband_n` | `10.0` | Ignore small force around zero |
| `force_velocity_sign` | `1.0` | Sign from force to forward velocity |
| `base_velocity_mps` | `0.0` | Constant forward velocity offset |
| `max_velocity_mps` | `0.40` | Maximum `linear.x` output |

Example:

```bash
ros2 launch laica_bringup bringup_LAICA.launch \
  arduino_port:=/dev/ttyACM0 \
  predictor_publish_rate:=100.0 \
  predicted_cmd_vel_topic_name:=/laica/predicted_cmd_vel \
  force_deadband_n:=10.0 \
  force_velocity_sign:=1.0
```

## Topic Naming Rule

Keep these meanings separate:

```text
/odom                     Unitree measured state and velocity
/cmd_vel                  Command velocity received by Unitree
/laica/predicted_cmd_vel  LAICA model output velocity
```

Do not use `/cmd_vel` as model input. If current velocity is needed for the model,
subscribe to `/odom` and read `twist.twist`.

## Loadcell Calibration

The loadcell message keeps the ADC raw value unchanged:

```text
LoadCellData.raw_count
```

This raw value is not normalized or mapped to `[-1, 1]`.

The calibrated force field is computed from loadcell voltage with linear regression.
Change these values in `LAICA_enclc/config/params.yaml`:

```text
force_n = force_gradient_n_per_mv * voltage_mv + force_bias_n
```

Current values:

```yaml
force_gradient_n_per_mv: 283.07
force_bias_n: 0.05122
```

Reference experimental pair kept as comments:

```yaml
# force_gradient_n_per_mv: 1.318e-03
# force_bias_n: -14.5
```

## Useful Checks

List topics:

```bash
ros2 topic list
```

Check Arduino data:

```bash
ros2 topic echo /load_cell/data
ros2 topic echo /encoder/data
```

Check predictor output:

```bash
ros2 topic echo /laica/predicted_cmd_vel
```

## Recording Rosbags

Create or enter a directory for bag files:

```bash
mkdir -p /home/artemis/Documents/rosbags
cd /home/artemis/Documents/rosbags
source /home/artemis/Documents/LAICA_ws/install/setup.bash
```

Record the main LAICA topics:

```bash
ros2 bag record /load_cell/data /encoder/data /imu /odom /laica/predicted_cmd_vel
```

Record to a timestamped folder:

```bash
ros2 bag record -o laica_$(date +%Y%m%d_%H%M%S) \
  /load_cell/data \
  /encoder/data \
  /imu \
  /odom \
  /laica/predicted_cmd_vel
```

Stop recording with `Ctrl+C`.

Inspect a saved bag:

```bash
ros2 bag info laica_YYYYMMDD_HHMMSS
```

## Aligned Odom Dataset

For adaptive-control analysis, build a timestamp-aligned dataset with force,
encoder angle, IMU, and robot odometry:

```bash
cd /home/artemis/Documents/LAICA_ws
source install/setup.bash
python3 src/laica_bringup/scripts/build_odom_dataset.py \
  --output-dir /home/artemis/Documents/LAICA_ws/plots/odomDataset
```

The script reads all bags under:

```text
/home/artemis/Documents/rosbags
```

and writes:

```text
plots/odomDataset/laica_aligned_odom_dataset.csv
plots/odomDataset/laica_bag_summary.csv
plots/odomDataset/laica_group_summary.csv
```

The aligned dataset uses encoder timestamps as the reference and keeps rows
only when force, IMU, and odom samples are within `30 ms`.

Force is normalized per bag:

```text
force_dev  = force_n - median(force_n)
force_norm = force_dev / MAD(force_n)
```

Interaction labels are based on this normalized force:

```text
pull        force_norm <= -1
push        force_norm >=  1
strong_pull force_norm <= -2
strong_push force_norm >=  2
neutral     otherwise
```

Use `/odom` velocity fields for walking-pace and adaptive-control analysis:

```text
odom.twist.twist.linear.x
odom.twist.twist.linear.y
odom.twist.twist.angular.z
odom.speed_xy
```

## Docker

The Dockerfile is:

```text
src/Dockerfile
```

Build the image from the `src` directory:

```bash
cd /home/artemis/Documents/LAICA_ws/src
docker build -t laica_ros2:humble .
```

Allow local X11 access before starting the container:

```bash
xhost +local:
```

Create and enter an interactive container with host network, privileged mode,
USB access, and X11 display forwarding:

```bash
docker run -it \
  --name laica_ros2 \
  --privileged \
  --network host \
  --ipc host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/artemis/Documents/LAICA_ws:/home/laica/LAICA_ws \
  laica_ros2:humble
```

If you want to expose only known Arduino serial devices instead of mounting all
of `/dev`, use this stricter form after checking the actual port:

```bash
ls /dev/ttyUSB* /dev/ttyACM*

docker run -it \
  --name laica_ros2 \
  --privileged \
  --network host \
  --ipc host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device=/dev/ttyACM0 \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/artemis/Documents/LAICA_ws:/home/laica/LAICA_ws \
  laica_ros2:humble
```

Start the existing container later:

```bash
xhost +local:
docker start -ai laica_ros2
```

Open another terminal inside the running container:

```bash
docker exec -it laica_ros2 bash
```

Build inside the container:

```bash
cd /home/laica/LAICA_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Run bringup inside the container:

```bash
ros2 launch laica_bringup bringup_LAICA.launch arduino_port:=/dev/ttyACM0
```
