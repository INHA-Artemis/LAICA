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
                    "not starting LAICA keyboard bringup."
                )
            ),
            EmitEvent(event=Shutdown(reason="Robot connection unavailable")),
        ]

    start_unitree_driver = LaunchConfiguration("start_unitree_driver")
    start_arduino_sensor_node = LaunchConfiguration("start_arduino_sensor_node")
    unitree_params_file = LaunchConfiguration("unitree_params_file")
    arduino_params_file = LaunchConfiguration("arduino_params_file")
    keyboard_params_file = LaunchConfiguration("keyboard_params_file")
    wifi = LaunchConfiguration("wifi")
    unitree_robot_ip = LaunchConfiguration("unitree_robot_ip")
    arduino_port = LaunchConfiguration("arduino_port")
    arduino_baud = LaunchConfiguration("arduino_baud")
    arduino_publish_rate = LaunchConfiguration("arduino_publish_rate")
    load_cell_topic_name = LaunchConfiguration("load_cell_topic_name")
    encoder_topic_name = LaunchConfiguration("encoder_topic_name")
    switch_topic_name = LaunchConfiguration("switch_topic_name")
    load_cell_calibration_done_topic_name = LaunchConfiguration(
        "load_cell_calibration_done_topic_name"
    )
    load_cell_startup_calibration_sec = LaunchConfiguration(
        "load_cell_startup_calibration_sec"
    )
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
                        "load_cell_calibration_done_topic_name": (
                            load_cell_calibration_done_topic_name
                        ),
                        "load_cell_startup_calibration_sec": ParameterValue(
                            load_cell_startup_calibration_sec, value_type=float
                        ),
                    },
                ],
            )
        )

    actions.append(
        Node(
            package="laica_bringup",
            executable="keyboard_forward_stop.py",
            name="keyboard_forward_stop",
            output="screen",
            additional_env=ros_env,
            parameters=[
                keyboard_params_file,
                {
                    "robot_ip": unitree_robot_ip,
                },
            ],
        )
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
                                "keyboard bringup."
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
                "keyboard_params_file",
                default_value=os.path.join(
                    laica_dir, "config", "keyboard_forward_stop_params.yaml"
                ),
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
            DeclareLaunchArgument(
                "load_cell_calibration_done_topic_name",
                default_value="/load_cell/calibration_done",
            ),
            DeclareLaunchArgument(
                "load_cell_startup_calibration_sec",
                default_value="5.0",
            ),
            OpaqueFunction(function=launch_after_robot_ready),
        ]
    )
