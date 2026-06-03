from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    window_sec = LaunchConfiguration("window_sec")
    update_hz = LaunchConfiguration("update_hz")

    encoder_topic = LaunchConfiguration("encoder_topic")
    force_topic = LaunchConfiguration("force_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    reference_speed_mps = LaunchConfiguration("reference_speed_mps")

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
            default_value="/cmd_vel",
            description="Updated cmd_vel topic to plot.",
        ),
        DeclareLaunchArgument(
            "reference_speed_mps",
            default_value="0.5",
            description="Reference forward speed shown as a horizontal line.",
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
                "reference_speed_mps": ParameterValue(
                    reference_speed_mps, value_type=float
                ),
            }],
        ),
    ])
