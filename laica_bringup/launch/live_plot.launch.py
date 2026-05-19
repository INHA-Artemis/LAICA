from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    window_sec = LaunchConfiguration("window_sec")
    update_hz = LaunchConfiguration("update_hz")

    encoder_topic = LaunchConfiguration("encoder_topic")
    force_topic = LaunchConfiguration("force_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    switch_topic = LaunchConfiguration("switch_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "window_sec",
            default_value="20.0",
            description="Rolling plot window length in seconds.",
        ),
        DeclareLaunchArgument(
            "update_hz",
            default_value="10.0",
            description="Matplotlib redraw rate in Hz.",
        ),
        DeclareLaunchArgument(
            "encoder_topic",
            default_value="/encoder/data",
            description="Encoder topic to plot.",
        ),
        DeclareLaunchArgument(
            "force_topic",
            default_value="/load_cell/data",
            description="Load cell topic to plot.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/imu",
            description="IMU topic to plot.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/laica/predicted_cmd_vel",
            description="Predicted cmd_vel topic to plot.",
        ),
        DeclareLaunchArgument(
            "switch_topic",
            default_value="/switch/data",
            description="Switch marker topic used to highlight action intervals.",
        ),
        Node(
            package="laica_bringup",
            executable="live_plot_sensors.py",
            name="live_plot_sensors",
            output="screen",
            parameters=[{
                "window_sec": window_sec,
                "update_hz": update_hz,
                "encoder_topic": encoder_topic,
                "force_topic": force_topic,
                "imu_topic": imu_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "switch_topic": switch_topic,
            }],
        ),
    ])
