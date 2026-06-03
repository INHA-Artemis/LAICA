#!/usr/bin/env python3
"""Live plot for checking force-only admittance output."""

from collections import deque
import math

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from enc_lc.msg import LoadCellData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class LiveAdmittancePlot(Node):
    def __init__(self):
        super().__init__("live_plot_admittance")

        self.declare_parameter("window_sec", 20.0)
        self.declare_parameter("update_hz", 10.0)
        self.declare_parameter("force_topic", "/load_cell/data")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("show_odom", True)
        self.declare_parameter("time_source", "arrival")

        self.window_sec = float(self.get_parameter("window_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)
        self.show_odom = bool(self.get_parameter("show_odom").value)
        self.time_source = str(self.get_parameter("time_source").value).lower()
        if self.time_source not in ("arrival", "header"):
            self.get_logger().warn(
                f"Unsupported time_source={self.time_source}; using arrival"
            )
            self.time_source = "arrival"

        self.start_time = None
        self.wall_start_time = None
        self.latest_plot_time = 0.0
        self.series = {
            "force_n": deque(),
            "cmd_vx": deque(),
            "odom_vx": deque(),
            "odom_speed": deque(),
        }

        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        self.fig.canvas.manager.set_window_title("LAICA Force Admittance Live")
        self.lines = {}
        self.status_texts = []
        self._setup_axes()

        self.create_subscription(
            LoadCellData,
            self.get_parameter("force_topic").value,
            self.force_callback,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.cmd_vel_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            10,
        )

        period = 1.0 / max(self.update_hz, 1.0)
        self.create_timer(period, self.update_plot)
        self.get_logger().info(
            "Live admittance plot running: "
            f"force={self.get_parameter('force_topic').value} "
            f"cmd={self.get_parameter('cmd_vel_topic').value} "
            f"odom={self.get_parameter('odom_topic').value} "
            f"time_source={self.time_source}"
        )

    def _setup_axes(self):
        axis_specs = [
            ("Force input", self.axes[0], ["force_n"]),
            ("Admittance output", self.axes[1], ["cmd_vx"]),
            ("Robot odom feedback", self.axes[2], ["odom_vx", "odom_speed"]),
        ]

        for title, ax, fields in axis_specs:
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
            for field in fields:
                line, = ax.plot([], [], label=field, linewidth=1.5)
                self.lines[field] = line
            self.status_texts.append(
                ax.text(
                    0.02,
                    0.90,
                    "",
                    transform=ax.transAxes,
                    fontsize=9,
                    bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"),
                )
            )
            ax.legend(loc="best")

        self.axes[0].set_ylabel("N")
        self.axes[1].set_ylabel("cmd")
        self.axes[2].set_ylabel("m/s")
        self.axes[-1].set_xlabel("time [s]")
        self.fig.tight_layout()
        plt.ion()
        plt.show(block=False)

    def message_time(self, msg):
        if self.time_source == "arrival":
            return self.arrival_time()

        stamp = getattr(msg, "header", None)
        if stamp is not None and (stamp.stamp.sec != 0 or stamp.stamp.nanosec != 0):
            now = float(stamp.stamp.sec) + float(stamp.stamp.nanosec) * 1e-9
            if self.start_time is None:
                self.start_time = now
            time_value = now - self.start_time
        else:
            now_msg = self.get_clock().now().to_msg()
            now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
            if self.wall_start_time is None:
                self.wall_start_time = now
            # Twist has no header. During rosbag replay, align it with the most
            # recent stamped sensor/odom time instead of mixing wall time with
            # recorded header stamps.
            if self.start_time is not None:
                time_value = self.latest_plot_time
            else:
                time_value = now - self.wall_start_time

        self.latest_plot_time = max(self.latest_plot_time, time_value)
        return time_value

    def arrival_time(self):
        now_msg = self.get_clock().now().to_msg()
        now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
        if self.wall_start_time is None:
            self.wall_start_time = now
        time_value = now - self.wall_start_time
        self.latest_plot_time = max(self.latest_plot_time, time_value)
        return time_value

    def now_time(self):
        if self.latest_plot_time > 0.0:
            return self.latest_plot_time
        if self.wall_start_time is None:
            now_msg = self.get_clock().now().to_msg()
            self.wall_start_time = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
        now_msg = self.get_clock().now().to_msg()
        now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
        return now - self.wall_start_time

    def append(self, field, time_value, value):
        self.latest_plot_time = max(self.latest_plot_time, time_value)
        self.series[field].append((time_value, float(value)))

    def force_callback(self, msg):
        self.append("force_n", self.message_time(msg), msg.force_n)

    def cmd_vel_callback(self, msg):
        time_value = self.message_time(msg)
        self.append("cmd_vx", time_value, msg.linear.x)

    def odom_callback(self, msg):
        if not self.show_odom:
            return
        time_value = self.message_time(msg)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.append("odom_vx", time_value, vx)
        self.append("odom_speed", time_value, math.hypot(vx, vy))

    def prune(self, min_time):
        for values in self.series.values():
            while values and values[0][0] < min_time:
                values.popleft()

    def update_plot(self):
        now = self.now_time()
        min_time = max(0.0, now - self.window_sec)
        self.prune(min_time)

        for field, values in self.series.items():
            times = [item[0] for item in values]
            y_values = [item[1] for item in values]
            self.lines[field].set_data(times, y_values)

        self.status_texts[0].set_text(
            "" if self.series["force_n"] else "No /load_cell/data messages"
        )
        self.status_texts[1].set_text(
            ""
            if self.series["cmd_vx"]
            else f"No cmd messages on {self.get_parameter('cmd_vel_topic').value}"
        )
        self.status_texts[2].set_text(
            ""
            if (self.series["odom_vx"] or not self.show_odom)
            else "No /odom messages"
        )

        for ax in self.axes:
            ax.set_xlim(min_time, max(self.window_sec, now))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = LiveAdmittancePlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
