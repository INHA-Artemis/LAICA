import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def wait_for_robot(context):
    start_unitree_driver = LaunchConfiguration("start_unitree_driver").perform(context)
    if not as_bool(start_unitree_driver):
        return True

    robot_ip = LaunchConfiguration("unitree_robot_ip").perform(context)
    timeout_sec = float(
        LaunchConfiguration("robot_connection_timeout_sec").perform(context)
    )
    deadline = time.monotonic() + timeout_sec

    while True:
        result = subprocess.run(
            ["ping", "-c1", "-W1", "-s1", robot_ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode == 0:
            return True

        if time.monotonic() >= deadline:
            return False

        time.sleep(1.0)


def launch_after_robot_ready(context):
    robot_ip = LaunchConfiguration("unitree_robot_ip").perform(context)
    if not wait_for_robot(context):
        return [
            LogInfo(
                msg=(
                    f"Robot connection unavailable at {robot_ip}; "
                    "not starting LAICA bringup."
                )
            ),
            EmitEvent(event=Shutdown(reason="Robot connection unavailable")),
        ]

    start_unitree_driver = LaunchConfiguration("start_unitree_driver")
    start_arduino_sensor_node = LaunchConfiguration("start_arduino_sensor_node")
    unitree_params_file = LaunchConfiguration("unitree_params_file")
    arduino_params_file = LaunchConfiguration("arduino_params_file")
    predictor_params_file = LaunchConfiguration("predictor_params_file")
    keyboard_params_file = LaunchConfiguration("keyboard_params_file")
    wifi = LaunchConfiguration("wifi")
    unitree_robot_ip = LaunchConfiguration("unitree_robot_ip")
    arduino_port = LaunchConfiguration("arduino_port")
    arduino_baud = LaunchConfiguration("arduino_baud")
    arduino_publish_rate = LaunchConfiguration("arduino_publish_rate")
    load_cell_topic_name = LaunchConfiguration("load_cell_topic_name")
    encoder_topic_name = LaunchConfiguration("encoder_topic_name")
    switch_topic_name = LaunchConfiguration("switch_topic_name")
    robot_odom_topic_name = LaunchConfiguration("robot_odom_topic_name")
    load_cell_calibration_done_topic_name = LaunchConfiguration(
        "load_cell_calibration_done_topic_name"
    )
    load_cell_startup_calibration_sec = LaunchConfiguration(
        "load_cell_startup_calibration_sec"
    )
    predicted_cmd_vel_topic_name = LaunchConfiguration("predicted_cmd_vel_topic_name")
    predictor_publish_rate = LaunchConfiguration("predictor_publish_rate")
    predictor_debug_publish_enabled = LaunchConfiguration(
        "predictor_debug_publish_enabled"
    )
    sensor_qos_reliability = LaunchConfiguration("sensor_qos_reliability")
    load_cell_input_field = LaunchConfiguration("load_cell_input_field")
    admittance_enabled = LaunchConfiguration("admittance_enabled")
    require_encoder = LaunchConfiguration("require_encoder")
    require_robot_odom = LaunchConfiguration("require_robot_odom")
    require_load_cell_calibration_done = LaunchConfiguration(
        "require_load_cell_calibration_done"
    )
    force_deadband_n = LaunchConfiguration("force_deadband_n")
    force_filter_tau_sec = LaunchConfiguration("force_filter_tau_sec")
    force_velocity_sign = LaunchConfiguration("force_velocity_sign")
    admittance_mass = LaunchConfiguration("admittance_mass")
    admittance_damping = LaunchConfiguration("admittance_damping")
    base_velocity_mps = LaunchConfiguration("base_velocity_mps")
    min_velocity_mps = LaunchConfiguration("min_velocity_mps")
    max_velocity_mps = LaunchConfiguration("max_velocity_mps")
    max_accel_mps2 = LaunchConfiguration("max_accel_mps2")
    sensor_timeout_sec = LaunchConfiguration("sensor_timeout_sec")
    robot_odom_timeout_sec = LaunchConfiguration("robot_odom_timeout_sec")
    stop_switch_enabled = LaunchConfiguration("stop_switch_enabled")
    debug_topic_prefix = LaunchConfiguration("debug_topic_prefix")
    keyboard_cmd_vel_topic_name = LaunchConfiguration("keyboard_cmd_vel_topic_name")
    keyboard_odom_topic_name = LaunchConfiguration("keyboard_odom_topic_name")
    keyboard_robot_ready_timeout_sec = LaunchConfiguration(
        "keyboard_robot_ready_timeout_sec"
    )
    keyboard_connection_check_period_sec = LaunchConfiguration(
        "keyboard_connection_check_period_sec"
    )
    keyboard_max_missed_pings = LaunchConfiguration("keyboard_max_missed_pings")
    keyboard_repeat_rate_hz = LaunchConfiguration("keyboard_repeat_rate_hz")
    keyboard_target_velocity_mps = LaunchConfiguration("keyboard_target_velocity_mps")
    keyboard_max_accel_mps2 = LaunchConfiguration("keyboard_max_accel_mps2")
    ros_env = {"RMW_FASTRTPS_USE_SHM": "0", "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"}

    actions = [LogInfo(msg=f"Robot connection available at {robot_ip}.")]

    unitree_driver_node = None
    if as_bool(LaunchConfiguration("start_unitree_driver").perform(context)):
        unitree_driver_node = Node(
            package="unitree_ros",
            executable="unitree_driver",
            output="screen",
            additional_env=ros_env,
            parameters=[
                unitree_params_file,
                {
                    "wifi": wifi,
                    "robot_ip": unitree_robot_ip,
                },
            ],
        )
        actions.append(unitree_driver_node)

    if as_bool(LaunchConfiguration("start_arduino_sensor_node").perform(context)):
        actions.append(
            Node(
                package="enc_lc",
                executable="arduino_sensor_pub",
                name="arduino_sensor_pub",
                output="screen",
                additional_env=ros_env,
                parameters=[
                    arduino_params_file,
                    {
                        "port": arduino_port,
                        "baud": ParameterValue(arduino_baud, value_type=int),
                        "publish_rate": ParameterValue(
                            arduino_publish_rate, value_type=float
                        ),
                        "load_cell_topic_name": load_cell_topic_name,
                        "encoder_topic_name": encoder_topic_name,
                        "switch_topic_name": switch_topic_name,
                        "load_cell_calibration_done_topic_name": load_cell_calibration_done_topic_name,
                        "load_cell_startup_calibration_sec": ParameterValue(
                            load_cell_startup_calibration_sec, value_type=float
                        ),
                    },
                ],
            )
        )

    actions.extend(
        [
            Node(
                package="laica_bringup",
                executable="keyboard_forward_stop.py",
                name="keyboard_forward_stop",
                output="screen",
                additional_env=ros_env,
                parameters=[
                    keyboard_params_file,
                    {
                        "cmd_vel_topic_name": keyboard_cmd_vel_topic_name,
                        "odom_topic_name": keyboard_odom_topic_name,
                        "robot_ip": unitree_robot_ip,
                        "robot_ready_timeout_sec": ParameterValue(
                            keyboard_robot_ready_timeout_sec, value_type=float
                        ),
                        "connection_check_period_sec": ParameterValue(
                            keyboard_connection_check_period_sec, value_type=float
                        ),
                        "max_missed_pings": ParameterValue(
                            keyboard_max_missed_pings, value_type=int
                        ),
                        "repeat_rate_hz": ParameterValue(
                            keyboard_repeat_rate_hz, value_type=float
                        ),
                        "target_velocity_mps": ParameterValue(
                            keyboard_target_velocity_mps, value_type=float
                        ),
                        "max_accel_mps2": ParameterValue(
                            keyboard_max_accel_mps2, value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="laica_bringup",
                executable="laica_velocity_predictor",
                name="laica_velocity_predictor",
                output="screen",
                additional_env=ros_env,
                parameters=[
                    predictor_params_file,
                    {
                        "load_cell_topic_name": load_cell_topic_name,
                        "encoder_topic_name": encoder_topic_name,
                        "switch_topic_name": switch_topic_name,
                        "robot_odom_topic_name": robot_odom_topic_name,
                        "load_cell_calibration_done_topic_name": load_cell_calibration_done_topic_name,
                        "predicted_cmd_vel_topic_name": predicted_cmd_vel_topic_name,
                        "publish_rate": ParameterValue(
                            predictor_publish_rate, value_type=float
                        ),
                        "sensor_qos_reliability": sensor_qos_reliability,
                        "load_cell_input_field": load_cell_input_field,
                        "admittance_enabled": ParameterValue(
                            admittance_enabled, value_type=bool
                        ),
                        "require_encoder": ParameterValue(
                            require_encoder, value_type=bool
                        ),
                        "require_robot_odom": ParameterValue(
                            require_robot_odom, value_type=bool
                        ),
                        "require_load_cell_calibration_done": ParameterValue(
                            require_load_cell_calibration_done, value_type=bool
                        ),
                        "force_deadband_n": ParameterValue(
                            force_deadband_n, value_type=float
                        ),
                        "force_filter_tau_sec": ParameterValue(
                            force_filter_tau_sec, value_type=float
                        ),
                        "force_velocity_sign": ParameterValue(
                            force_velocity_sign, value_type=float
                        ),
                        "admittance_mass": ParameterValue(
                            admittance_mass, value_type=float
                        ),
                        "admittance_damping": ParameterValue(
                            admittance_damping, value_type=float
                        ),
                        "base_velocity_mps": ParameterValue(
                            base_velocity_mps, value_type=float
                        ),
                        "min_velocity_mps": ParameterValue(
                            min_velocity_mps, value_type=float
                        ),
                        "max_velocity_mps": ParameterValue(
                            max_velocity_mps, value_type=float
                        ),
                        "max_accel_mps2": ParameterValue(
                            max_accel_mps2, value_type=float
                        ),
                        "sensor_timeout_sec": ParameterValue(
                            sensor_timeout_sec, value_type=float
                        ),
                        "robot_odom_timeout_sec": ParameterValue(
                            robot_odom_timeout_sec, value_type=float
                        ),
                        "stop_switch_enabled": ParameterValue(
                            stop_switch_enabled, value_type=bool
                        ),
                        "debug_publish_enabled": ParameterValue(
                            predictor_debug_publish_enabled, value_type=bool
                        ),
                        "debug_topic_prefix": debug_topic_prefix,
                    }
                ],
            ),
        ]
    )

    if unitree_driver_node is not None:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=unitree_driver_node,
                    on_exit=[
                        LogInfo(
                            msg=(
                                "Unitree driver exited; shutting down LAICA "
                                "bringup."
                            )
                        ),
                        EmitEvent(event=Shutdown(reason="Unitree driver exited")),
                    ],
                )
            )
        )

    return actions


def generate_launch_description():
    laica_dir = get_package_share_directory("laica_bringup")
    unitree_dir = get_package_share_directory("unitree_ros")
    enc_lc_dir = get_package_share_directory("enc_lc")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_unitree_driver", default_value="true"),
            DeclareLaunchArgument("start_arduino_sensor_node", default_value="true"),
            DeclareLaunchArgument(
                "unitree_params_file",
                default_value=os.path.join(unitree_dir, "config", "params.yaml"),
            ),
            DeclareLaunchArgument(
                "arduino_params_file",
                default_value=os.path.join(enc_lc_dir, "config", "params.yaml"),
            ),
            DeclareLaunchArgument(
                "predictor_params_file",
                default_value=os.path.join(laica_dir, "config", "velocity_predictor_params.yaml"),
            ),
            DeclareLaunchArgument(
                "keyboard_params_file",
                default_value=os.path.join(laica_dir, "config", "keyboard_forward_stop_params.yaml"),
            ),
            DeclareLaunchArgument("wifi", default_value="false"),
            DeclareLaunchArgument("unitree_robot_ip", default_value="192.168.123.161"),
            DeclareLaunchArgument("robot_connection_timeout_sec", default_value="10.0"),
            DeclareLaunchArgument("arduino_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("arduino_baud", default_value="115200"),
            DeclareLaunchArgument("arduino_publish_rate", default_value="100.0"),
            DeclareLaunchArgument("load_cell_topic_name", default_value="/load_cell/data"),
            DeclareLaunchArgument("encoder_topic_name", default_value="/encoder/data"),
            DeclareLaunchArgument("switch_topic_name", default_value="/switch/data"),
            DeclareLaunchArgument("robot_odom_topic_name", default_value="/odom"),
            DeclareLaunchArgument(
                "load_cell_calibration_done_topic_name",
                default_value="/load_cell/calibration_done",
            ),
            DeclareLaunchArgument("load_cell_startup_calibration_sec", default_value="5.0"),
            DeclareLaunchArgument(
                "predicted_cmd_vel_topic_name",
                default_value="/laica/predicted_cmd_vel",
            ),
            DeclareLaunchArgument("predictor_publish_rate", default_value="50.0"),
            DeclareLaunchArgument("predictor_debug_publish_enabled", default_value="false"),
            DeclareLaunchArgument("sensor_qos_reliability", default_value="best_effort"),
            DeclareLaunchArgument("load_cell_input_field", default_value="force_n"),
            DeclareLaunchArgument("admittance_enabled", default_value="false"),
            DeclareLaunchArgument("require_encoder", default_value="false"),
            DeclareLaunchArgument("require_robot_odom", default_value="true"),
            DeclareLaunchArgument(
                "require_load_cell_calibration_done",
                default_value="true",
            ),
            DeclareLaunchArgument("force_deadband_n", default_value="5.0"),
            DeclareLaunchArgument("force_filter_tau_sec", default_value="0.15"),
            DeclareLaunchArgument("force_velocity_sign", default_value="1.0"),
            DeclareLaunchArgument("admittance_mass", default_value="20.0"),
            DeclareLaunchArgument("admittance_damping", default_value="80.0"),
            DeclareLaunchArgument("base_velocity_mps", default_value="0.5"),
            DeclareLaunchArgument("min_velocity_mps", default_value="-1.0"),
            DeclareLaunchArgument("max_velocity_mps", default_value="1.0"),
            DeclareLaunchArgument("max_accel_mps2", default_value="0.50"),
            DeclareLaunchArgument("sensor_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("robot_odom_timeout_sec", default_value="0.50"),
            DeclareLaunchArgument("stop_switch_enabled", default_value="true"),
            DeclareLaunchArgument("debug_topic_prefix", default_value="/laica/debug"),
            DeclareLaunchArgument("keyboard_cmd_vel_topic_name", default_value="/cmd_vel"),
            DeclareLaunchArgument("keyboard_odom_topic_name", default_value="/odom"),
            DeclareLaunchArgument(
                "keyboard_robot_ready_timeout_sec",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "keyboard_connection_check_period_sec",
                default_value="5.0",
            ),
            DeclareLaunchArgument("keyboard_max_missed_pings", default_value="3"),
            DeclareLaunchArgument("keyboard_repeat_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("keyboard_target_velocity_mps", default_value="0.4"),
            DeclareLaunchArgument("keyboard_max_accel_mps2", default_value="0.1"),
            OpaqueFunction(function=launch_after_robot_ready),
        ]
    )
