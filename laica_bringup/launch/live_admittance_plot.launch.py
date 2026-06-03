from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    window_sec = LaunchConfiguration("window_sec")
    update_hz = LaunchConfiguration("update_hz")
    force_topic = LaunchConfiguration("force_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    switch_topic = LaunchConfiguration("switch_topic")
    show_odom = LaunchConfiguration("show_odom")
    predictor_params_file = PathJoinSubstitution([
        FindPackageShare("laica_bringup"),
        "config",
        "velocity_predictor_params.yaml",
    ])

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
            default_value="/cmd_vel",
            description="Admittance/predicted cmd_vel topic.",
        ),
        DeclareLaunchArgument(
            "switch_topic",
            default_value="/switch/data",
            description="Switch topic used by the predictor shutdown input.",
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
        Node(
            package="laica_bringup",
            executable="laica_velocity_predictor",
            name="laica_velocity_predictor",
            output="screen",
            parameters=[
                predictor_params_file,
                {
                    "load_cell_topic_name": force_topic,
                    "switch_topic_name": switch_topic,
                    "robot_odom_topic_name": odom_topic,
                    "load_cell_calibration_done_topic_name": "/load_cell/calibration_done",
                    "predicted_cmd_vel_topic_name": cmd_vel_topic,
                    "publish_rate": 50.0,
                    "load_cell_input_field": "force_n",
                    "sensor_qos_reliability": "best_effort",
                    "load_cell_subscription_mode": "typed",
                    "admittance_enabled": True,
                    "require_encoder": False,
                    "require_robot_odom": True,
                    "require_load_cell_calibration_done": True,
                    "stop_switch_enabled": True,
                    "auto_zero_force": True,
                    "zero_force_duration_sec": 3.0,
                    "force_filter_tau_sec": 0.15,
                    "force_deadband_n": 5.0,
                    "force_velocity_sign": 1.0,
                    "admittance_mass": 20.0,
                    "admittance_damping": 80.0,
                    "base_velocity_mps": 0.5,
                    "min_velocity_mps": -1.0,
                    "max_velocity_mps": 1.0,
                    "max_accel_mps2": 0.50,
                    "sensor_timeout_sec": 0.25,
                    "robot_odom_timeout_sec": 0.50,
                },
            ],
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
                "time_source": "arrival",
            }],
        ),
    ])
