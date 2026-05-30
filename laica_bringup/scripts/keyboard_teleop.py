#!/usr/bin/env python3

import select
import subprocess
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


HELP_TEXT = """
Keyboard teleop for LAICA
-------------------------
w/s : increase/decrease forward speed
a/d : increase/decrease turn speed
x   : stop
q   : quit
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter("cmd_vel_topic_name", "/cmd_vel")
        self.declare_parameter("odom_topic_name", "/odom")
        self.declare_parameter("robot_ip", "192.168.123.161")
        self.declare_parameter("robot_ready_timeout_sec", 1.0)
        self.declare_parameter("connection_check_period_sec", 1.0)
        self.declare_parameter("enable_runtime_ping_check", False)
        self.declare_parameter("max_missed_pings", 3)
        self.declare_parameter("repeat_rate_hz", 20.0)
        self.declare_parameter("linear_step", 0.05)
        self.declare_parameter("angular_step", 0.1)
        self.declare_parameter("max_linear", 0.4)
        self.declare_parameter("max_angular", 1.0)

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
        self.linear_step = float(self.get_parameter("linear_step").value)
        self.angular_step = float(self.get_parameter("angular_step").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.publisher = None
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic_name, self.odom_callback, 10
        )
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_odom_time = None
        self.last_ping_time = 0.0
        self.last_ping_ok = False
        self.missed_ping_count = 0
        self.was_ready = False
        self.repeat_timer = None
        self.get_logger().info(
            f"Waiting for robot readiness: odom={odom_topic_name}, ip={self.robot_ip}"
        )

    def odom_callback(self, _msg):
        self.last_odom_time = time.monotonic()

    def clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def handle_key(self, key):
        if not self.ready_to_command():
            return

        if key == "w":
            self.linear_x = self.clamp(self.linear_x + self.linear_step, self.max_linear)
        elif key == "s":
            self.linear_x = self.clamp(self.linear_x - self.linear_step, self.max_linear)
        elif key == "a":
            self.angular_z = self.clamp(self.angular_z + self.angular_step, self.max_angular)
        elif key == "d":
            self.angular_z = self.clamp(self.angular_z - self.angular_step, self.max_angular)
        elif key == "x":
            self.linear_x = 0.0
            self.angular_z = 0.0
        else:
            return

        self.publish_cmd()
        print(f"linear.x={self.linear_x:.2f} angular.z={self.angular_z:.2f}", flush=True)

    def publish_cmd(self):
        if self.publisher is None:
            return

        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher.publish(msg)

    def stop(self):
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.publish_cmd()

    def repeat_cmd(self):
        if self.ready_to_command():
            self.publish_cmd()

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
                f"Robot is ready. Publishing keyboard commands to "
                f"{self.cmd_vel_topic_name} at {self.repeat_rate_hz:.1f} Hz"
            )
            return False

        if not ready and self.was_ready:
            self.get_logger().error(
                "Robot connection/odom heartbeat lost. Stopping keyboard teleop."
            )
            self.stop()
            return True

        return False


def read_key(input_stream, timeout_sec=0.01):
    ready, _, _ = select.select([input_stream], [], [], timeout_sec)
    if ready:
        return input_stream.read(1)
    return None


def main():
    input_stream = open("/dev/tty", "r")
    settings = termios.tcgetattr(input_stream)
    rclpy.init()
    node = KeyboardTeleop()

    print(HELP_TEXT, flush=True)
    try:
        tty.setcbreak(input_stream.fileno())
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
        node.stop()
        termios.tcsetattr(input_stream, termios.TCSADRAIN, settings)
        input_stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
