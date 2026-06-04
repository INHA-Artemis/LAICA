#!/usr/bin/env python3

import select
import signal
import subprocess
import termios
import time
import tty

import rclpy
from enc_lc.msg import LoadCellData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


HELP_TEXT = """
LAICA forward/stop keyboard control
-----------------------------------
w : hold forward target speed
s : stop
q : quit
"""


class KeyboardForwardStop(Node):
    def __init__(self):
        super().__init__("keyboard_forward_stop")
        self.declare_parameter("cmd_vel_topic_name", "/cmd_vel")
        self.declare_parameter("odom_topic_name", "/odom")
        self.declare_parameter("robot_ip", "192.168.123.161")
        self.declare_parameter("robot_ready_timeout_sec", 1.0)
        self.declare_parameter("connection_check_period_sec", 1.0)
        self.declare_parameter("enable_runtime_ping_check", False)
        self.declare_parameter("max_missed_pings", 3)
        self.declare_parameter("repeat_rate_hz", 20.0)
        self.declare_parameter("target_velocity_mps", 0.5)
        self.declare_parameter("max_accel_mps2", 0.1)
        self.declare_parameter("load_cell_topic_name", "/load_cell/data")
        self.declare_parameter("debug_publish_enabled", True)
        self.declare_parameter("debug_topic_prefix", "/laica/teleop_debug")

        self.cmd_vel_topic_name = self.get_parameter("cmd_vel_topic_name").value
        odom_topic_name = self.get_parameter("odom_topic_name").value
        self.robot_ip = self.get_parameter("robot_ip").value
        self.robot_ready_timeout_sec = float(
            self.get_parameter("robot_ready_timeout_sec").value
        )
        self.connection_check_period_sec = float(
            self.get_parameter("connection_check_period_sec").value
        )
        self.enable_runtime_ping_check = bool(
            self.get_parameter("enable_runtime_ping_check").value
        )
        self.max_missed_pings = int(self.get_parameter("max_missed_pings").value)
        self.repeat_rate_hz = float(self.get_parameter("repeat_rate_hz").value)
        self.target_velocity_mps = float(
            self.get_parameter("target_velocity_mps").value
        )
        self.max_accel_mps2 = max(
            0.0, float(self.get_parameter("max_accel_mps2").value)
        )
        load_cell_topic_name = self.get_parameter("load_cell_topic_name").value
        self.debug_publish_enabled = bool(
            self.get_parameter("debug_publish_enabled").value
        )
        self.debug_topic_prefix = self.normalize_topic_prefix(
            self.get_parameter("debug_topic_prefix").value
        )

        self.publisher = None
        self.debug_publishers = {}
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic_name, self.odom_callback, 10
        )
        self.load_cell_sub = self.create_subscription(
            LoadCellData, load_cell_topic_name, self.load_cell_callback, 10
        )
        self.target_command_velocity_mps = 0.0
        self.current_command_velocity_mps = 0.0
        self.last_publish_time = None
        self.last_odom_time = None
        self.last_ping_time = 0.0
        self.last_ping_ok = False
        self.missed_ping_count = 0
        self.was_ready = False
        self.repeat_timer = None
        self.force_zero_started = False
        self.force_zero_sum = 0.0
        self.force_zero_samples = 0
        self.force_zero_offset = 0.0
        self.filtered_force = 0.0
        self.has_filtered_force = False
        if self.debug_publish_enabled:
            self.create_debug_publishers()

        self.get_logger().info(
            f"Waiting for robot readiness: odom={odom_topic_name}, ip={self.robot_ip}"
        )
        self.get_logger().info(
            f"Subscribing to load-cell force for keyboard logs: {load_cell_topic_name}"
        )

    def normalize_topic_prefix(self, prefix):
        prefix = str(prefix or "/laica/teleop_debug")
        if not prefix.startswith("/"):
            prefix = "/" + prefix
        return prefix.rstrip("/") or "/"

    def odom_callback(self, _msg):
        self.last_odom_time = time.monotonic()

    def create_debug_publishers(self):
        for topic in ["raw_force", "zeroed_force", "filtered_force"]:
            self.debug_publishers[topic] = self.create_publisher(
                Float64, f"{self.debug_topic_prefix}/{topic}", 10
            )
        self.get_logger().info(
            f"Publishing keyboard force debug topics under {self.debug_topic_prefix}"
        )

    def publish_debug_float(self, topic, value):
        publisher = self.debug_publishers.get(topic)
        if publisher is None:
            return
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    def load_cell_callback(self, msg):
        force = float(msg.force_n)
        if not self.force_zero_started:
            self.force_zero_started = True
            self.force_zero_sum = 0.0
            self.force_zero_samples = 0

        if self.force_zero_samples < 100:
            self.force_zero_sum += force
            self.force_zero_samples += 1
            self.force_zero_offset = self.force_zero_sum / self.force_zero_samples

        zeroed_force = force - self.force_zero_offset
        alpha = 0.12
        if not self.has_filtered_force:
            self.filtered_force = zeroed_force
            self.has_filtered_force = True
        else:
            self.filtered_force += alpha * (zeroed_force - self.filtered_force)

        if self.debug_publish_enabled:
            self.publish_debug_float("raw_force", force)
            self.publish_debug_float("zeroed_force", zeroed_force)
            self.publish_debug_float("filtered_force", self.filtered_force)

    def handle_key(self, key):
        if key == "w":
            self.target_command_velocity_mps = self.target_velocity_mps
        elif key == "s":
            self.target_command_velocity_mps = 0.0
        else:
            return

        if self.ready_to_command():
            self.publish_cmd()
        else:
            self.get_logger().info(
                "Stored keyboard command; waiting for robot readiness before publishing."
            )
        self.get_logger().info(
            f"target velocity={self.target_command_velocity_mps:.2f} m/s"
        )

    def publish_cmd(self):
        if self.publisher is None:
            return

        self.update_current_velocity()

        msg = Twist()
        msg.linear.x = self.current_command_velocity_mps
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def stop(self):
        self.target_command_velocity_mps = 0.0
        self.current_command_velocity_mps = 0.0
        self.publish_cmd()

    def repeat_cmd(self):
        if self.ready_to_command():
            self.publish_cmd()

    def update_current_velocity(self):
        now = time.monotonic()
        if self.last_publish_time is None:
            dt = 1.0 / max(self.repeat_rate_hz, 1.0)
        else:
            dt = max(now - self.last_publish_time, 0.0)
        self.last_publish_time = now

        if self.max_accel_mps2 <= 0.0:
            self.current_command_velocity_mps = self.target_command_velocity_mps
            return

        delta = self.target_command_velocity_mps - self.current_command_velocity_mps
        max_delta = self.max_accel_mps2 * dt
        if abs(delta) <= max_delta:
            self.current_command_velocity_mps = self.target_command_velocity_mps
        elif delta > 0.0:
            self.current_command_velocity_mps += max_delta
        else:
            self.current_command_velocity_mps -= max_delta

    def robot_ready(self):
        now = time.monotonic()
        odom_ok = (
            self.last_odom_time is not None
            and now - self.last_odom_time <= self.robot_ready_timeout_sec
        )

        if (
            self.enable_runtime_ping_check
            and now - self.last_ping_time >= self.connection_check_period_sec
        ):
            self.last_ping_ok = self.ping_robot()
            if self.last_ping_ok:
                self.missed_ping_count = 0
            else:
                self.missed_ping_count += 1
            self.last_ping_time = now

        ping_ok = (
            not self.enable_runtime_ping_check
            or self.last_ping_ok
            or self.missed_ping_count < self.max_missed_pings
        )
        return odom_ok and ping_ok

    def ready_to_command(self):
        return self.was_ready and self.publisher is not None and self.robot_ready()

    def ping_robot(self):
        try:
            result = subprocess.run(
                ["ping", "-c1", "-W1", "-s1", self.robot_ip],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            return result.returncode == 0
        except KeyboardInterrupt:
            raise
        except OSError as exc:
            self.get_logger().error(f"Robot ping check failed: {exc}")
            return False

    def check_robot_or_exit(self):
        ready = self.robot_ready()

        if ready and not self.was_ready:
            self.publisher = self.create_publisher(Twist, self.cmd_vel_topic_name, 10)
            period_sec = 1.0 / max(self.repeat_rate_hz, 1.0)
            self.repeat_timer = self.create_timer(period_sec, self.repeat_cmd)
            self.was_ready = True
            self.get_logger().info(
                f"Robot is ready. Press w for {self.target_velocity_mps:.2f} m/s, "
                "s to stop."
            )
            self.publish_cmd()
            return False

        if not ready and self.was_ready:
            self.get_logger().error(
                "Robot connection/odom heartbeat lost. Stopping forward command."
            )
            self.stop()
            return True

        return False


def read_key(input_stream, timeout_sec=0.01):
    settings = termios.tcgetattr(input_stream)
    try:
        tty.setcbreak(input_stream.fileno())
        ready, _, _ = select.select([input_stream], [], [], timeout_sec)
        if ready:
            return input_stream.read(1)
        return None
    finally:
        termios.tcsetattr(input_stream, termios.TCSADRAIN, settings)


def main():
    input_stream = open("/dev/tty", "r")
    settings = termios.tcgetattr(input_stream)
    terminal_restored = False

    def restore_terminal():
        nonlocal terminal_restored
        if terminal_restored:
            return
        termios.tcsetattr(input_stream, termios.TCSADRAIN, settings)
        terminal_restored = True

    def shutdown_from_signal(_signum, _frame):
        restore_terminal()
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown_from_signal)
    signal.signal(signal.SIGHUP, shutdown_from_signal)

    rclpy.init()
    node = KeyboardForwardStop()

    print(HELP_TEXT, flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.check_robot_or_exit():
                break
            key = read_key(input_stream)
            if key == "q":
                break
            if key:
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        restore_terminal()
        input_stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
