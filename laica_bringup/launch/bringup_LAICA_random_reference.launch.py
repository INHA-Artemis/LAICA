import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    laica_dir = get_package_share_directory("laica_bringup")
    unitree_dir = get_package_share_directory("unitree_ros")
    enc_lc_dir = get_package_share_directory("enc_lc")

    start_unitree_driver = LaunchConfiguration("start_unitree_driver")
    start_arduino_sensor_node = LaunchConfiguration("start_arduino_sensor_node")
    unitree_params_file = LaunchConfiguration("unitree_params_file")
    arduino_params_file = LaunchConfiguration("arduino_params_file")
    predictor_params_file = LaunchConfiguration("predictor_params_file")
    random_reference_params_file = LaunchConfiguration("random_reference_params_file")
    scenario = LaunchConfiguration("scenario")
    wifi = LaunchConfiguration("wifi")
    unitree_robot_ip = LaunchConfiguration("unitree_robot_ip")
    enable_battery_check = LaunchConfiguration("enable_battery_check")
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
                default_value=os.path.join(
                    laica_dir, "config", "velocity_predictor_params.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "random_reference_params_file",
                default_value=os.path.join(
                    laica_dir, "config", "random_reference_speed_params.yaml"
                ),
            ),
            DeclareLaunchArgument("scenario", default_value="5"),
            DeclareLaunchArgument("wifi", default_value="false"),
            DeclareLaunchArgument("unitree_robot_ip", default_value="192.168.123.161"),
            DeclareLaunchArgument("enable_battery_check", default_value="true"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(unitree_dir, "launch", "unitree_driver_launch.py")
                ),
                condition=IfCondition(start_unitree_driver),
                launch_arguments={
                    "params_file": unitree_params_file,
                    "wifi": wifi,
                    "robot_ip": unitree_robot_ip,
                    "enable_battery_check": enable_battery_check,
                }.items(),
            ),
            Node(
                package="enc_lc",
                executable="arduino_sensor_pub",
                name="arduino_sensor_pub",
                output="screen",
                condition=IfCondition(start_arduino_sensor_node),
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
            ),
            Node(
                package="laica_bringup",
                executable="laica_velocity_predictor",
                name="laica_velocity_predictor",
                output="screen",
                parameters=[
                    predictor_params_file,
                    random_reference_params_file,
                    {
                        "random_reference_speed_scenario_id": ParameterValue(
                            scenario, value_type=int
                        ),
                    },
                ],
            ),
        ]
    )
