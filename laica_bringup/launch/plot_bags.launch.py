from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("laica_bringup"),
        "config",
        "plot_bags.yaml",
    ])

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YAML config containing bag paths, topics, fields, and output paths.",
        ),
        ExecuteProcess(
            cmd=[
                PathJoinSubstitution([
                    FindPackageShare("laica_bringup"),
                    "scripts",
                    "plot_rosbag_sensors.py",
                ]),
                "--config",
                config_file,
            ],
            output="screen",
        ),
    ])
