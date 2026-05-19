from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    robot_target_port = LaunchConfiguration("robot_target_port")
    start_arduino_sensor_node = LaunchConfiguration("start_arduino_sensor_node")
    arduino_params_file = LaunchConfiguration("arduino_params_file")
    arduino_port = LaunchConfiguration("arduino_port")
    arduino_baud = LaunchConfiguration("arduino_baud")
    arduino_publish_rate = LaunchConfiguration("arduino_publish_rate")
    load_cell_topic_name = LaunchConfiguration("load_cell_topic_name")
    encoder_topic_name = LaunchConfiguration("encoder_topic_name")
    load_cell_startup_calibration_sec = LaunchConfiguration(
        "load_cell_startup_calibration_sec"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.123.161",
            description="Go1 robot IP address. Use 192.168.12.1 for Wi-Fi.",
        ),
        DeclareLaunchArgument(
            "robot_target_port",
            default_value="8082",
            description="Go1 high-level UDP target port.",
        ),
        DeclareLaunchArgument(
            "start_arduino_sensor_node",
            default_value="true",
            description="Start Arduino load cell and encoder sensor node.",
        ),
        DeclareLaunchArgument(
            "arduino_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("enc_lc"),
                "config",
                "params.yaml",
            ]),
            description="Arduino sensor node parameter file.",
        ),
        DeclareLaunchArgument(
            "arduino_port",
            default_value="/dev/ttyACM0",
            description="Arduino serial port.",
        ),
        DeclareLaunchArgument(
            "arduino_baud",
            default_value="115200",
            description="Arduino serial baud rate.",
        ),
        DeclareLaunchArgument(
            "arduino_publish_rate",
            default_value="100.0",
            description="Arduino sensor publish rate in Hz.",
        ),
        DeclareLaunchArgument(
            "load_cell_topic_name",
            default_value="/load_cell/data",
            description="Load cell topic name.",
        ),
        DeclareLaunchArgument(
            "encoder_topic_name",
            default_value="/encoder/data",
            description="Encoder topic name.",
        ),
        DeclareLaunchArgument(
            "load_cell_startup_calibration_sec",
            default_value="5.0",
            description="Seconds to average loadcell startup baseline before logging.",
        ),
        Node(
            package="unitree_ros",
            executable="unitree_driver",
            name="unitree_ros_node",
            output="screen",
            parameters=[{
                "robot_ip": robot_ip,
                "robot_target_port": robot_target_port,
                "odom_topic_name": "/odom",
                "imu_topic_name": "/imu",
                "bms_state_topic_name": "/bms_state",
                "joint_states_topic_name": "/joint_states",
                "use_obstacle_avoidance": False,
                "auto_stand_up": False,
                "enable_battery_check": False,
                "enable_motion_commands": False,
            }],
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("enc_lc"),
                "launch",
                "sensors.launch",
            ])),
            condition=IfCondition(start_arduino_sensor_node),
            launch_arguments={
                "params_file": arduino_params_file,
                "port": arduino_port,
                "baud": arduino_baud,
                "publish_rate": arduino_publish_rate,
                "load_cell_topic_name": load_cell_topic_name,
                "encoder_topic_name": encoder_topic_name,
                "load_cell_startup_calibration_sec": load_cell_startup_calibration_sec,
            }.items(),
        ),
    ])
