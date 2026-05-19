from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    window_sec = LaunchConfiguration("window_sec")
    update_hz = LaunchConfiguration("update_hz")
    force_topic = LaunchConfiguration("force_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    show_odom = LaunchConfiguration("show_odom")
    time_source = LaunchConfiguration("time_source")

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
            "force_topic",
            default_value="/load_cell/data",
            description="Loadcell force topic.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/laica/predicted_cmd_vel",
            description="Admittance/predicted cmd_vel topic.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odom",
            description="Robot odom topic.",
        ),
        DeclareLaunchArgument(
            "show_odom",
            default_value="true",
            description="Whether to plot odom feedback.",
        ),
        DeclareLaunchArgument(
            "time_source",
            default_value="arrival",
            description=(
                "Time axis source: arrival for rosbag replay with live cmd output, "
                "or header for recorded sensor header stamps."
            ),
        ),
        Node(
            package="laica_bringup",
            executable="live_plot_admittance.py",
            name="live_plot_admittance",
            output="screen",
            parameters=[{
                "window_sec": window_sec,
                "update_hz": update_hz,
                "force_topic": force_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "odom_topic": odom_topic,
                "show_odom": show_odom,
                "time_source": time_source,
            }],
        ),
    ])
