#!/usr/bin/env python3

from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from enc_lc.msg import EncoderData, LoadCellData, SwitchData
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class LiveSensorPlot(Node):
    def __init__(self):
        super().__init__("live_plot_sensors")

        self.declare_parameter("window_sec", 20.0)
        self.declare_parameter("update_hz", 10.0)
        self.declare_parameter("encoder_topic", "/encoder/data")
        self.declare_parameter("force_topic", "/load_cell/data")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("reference_speed_mps", 0.5)
        self.declare_parameter("switch_topic", "/switch/data")
        self.declare_parameter("highlight_color", "tab:orange")
        self.declare_parameter("highlight_alpha", 0.18)

        self.window_sec = float(self.get_parameter("window_sec").value)
        self.update_hz = float(self.get_parameter("update_hz").value)
        self.reference_speed_mps = float(
            self.get_parameter("reference_speed_mps").value
        )
        self.highlight_color = self.get_parameter("highlight_color").value
        self.highlight_alpha = float(self.get_parameter("highlight_alpha").value)

        self.start_time = None
        self.last_switch_pressed = False
        self.active_interval_start = None
        self.completed_intervals = deque()

        self.encoder = {
            "angle_deg": deque(),
            "rev": deque(),
            "rpm": deque(),
        }
        self.force = {
            "force_n": deque(),
        }
        self.imu = {
            "linear_acceleration.x": deque(),
            "linear_acceleration.z": deque(),
        }
        self.cmd_vel = {
            "cmd_vel.linear.x": deque(),
        }

        self.fig, self.axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        self.fig.canvas.manager.set_window_title("LAICA Live Sensors")
        self.lines = {}
        self._setup_axes()

        self.create_subscription(
            EncoderData,
            self.get_parameter("encoder_topic").value,
            self.encoder_callback,
            10,
        )
        self.create_subscription(
            LoadCellData,
            self.get_parameter("force_topic").value,
            self.force_callback,
            10,
        )
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.imu_callback,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.cmd_vel_callback,
            10,
        )
        self.create_subscription(
            SwitchData,
            self.get_parameter("switch_topic").value,
            self.switch_callback,
            10,
        )

        period = 1.0 / max(self.update_hz, 1.0)
        self.create_timer(period, self.update_plot)
        self.get_logger().info("Live sensor plot running")

    def _setup_axes(self):
        groups = [
            ("Encoder", self.axes[0], self.encoder),
            ("Force", self.axes[1], self.force),
            ("IMU", self.axes[2], self.imu),
            ("Velocity command", self.axes[3], self.cmd_vel),
        ]

        for title, ax, series in groups:
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
            for field in series:
                line, = ax.plot([], [], label=field, linewidth=1.0)
                self.lines[field] = line
            ax.legend(loc="best")

        reference_line, = self.axes[3].plot(
            [],
            [],
            label="reference_vx",
            linewidth=1.5,
            linestyle="--",
            color="tab:gray",
        )
        self.lines["reference_vx"] = reference_line
        self.axes[3].legend(loc="best")

        self.axes[1].set_ylabel("N")
        self.axes[3].set_ylabel("m/s")
        self.axes[-1].set_xlabel("time [s]")
        self.fig.tight_layout()
        plt.ion()
        plt.show(block=False)

    def message_time(self, msg):
        stamp = getattr(msg, "header", None)
        if stamp is not None and (stamp.stamp.sec != 0 or stamp.stamp.nanosec != 0):
            now = float(stamp.stamp.sec) + float(stamp.stamp.nanosec) * 1e-9
        else:
            now_msg = self.get_clock().now().to_msg()
            now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9

        if self.start_time is None:
            self.start_time = now
        return now - self.start_time

    def append_value(self, series, field, time_value, value):
        series[field].append((time_value, float(value)))

    def prune_series(self, series, min_time):
        for values in series.values():
            while values and values[0][0] < min_time:
                values.popleft()

    def prune_intervals(self, min_time):
        while self.completed_intervals and self.completed_intervals[0][1] < min_time:
            self.completed_intervals.popleft()

    def encoder_callback(self, msg):
        time_value = self.message_time(msg)
        self.append_value(self.encoder, "angle_deg", time_value, msg.angle_deg)
        self.append_value(self.encoder, "rev", time_value, msg.rev)
        self.append_value(self.encoder, "rpm", time_value, msg.rpm)

    def force_callback(self, msg):
        time_value = self.message_time(msg)
        self.append_value(self.force, "force_n", time_value, msg.force_n)

    def imu_callback(self, msg):
        time_value = self.message_time(msg)
        self.append_value(self.imu, "linear_acceleration.x", time_value, msg.linear_acceleration.x)
        self.append_value(self.imu, "linear_acceleration.z", time_value, msg.linear_acceleration.z)

    def cmd_vel_callback(self, msg):
        time_value = self.message_time(msg)
        self.append_value(self.cmd_vel, "cmd_vel.linear.x", time_value, msg.linear.x)

    def switch_callback(self, msg):
        time_value = self.message_time(msg)
        pressed = bool(msg.switch_1) or bool(msg.switch_2)

        if pressed and not self.last_switch_pressed:
            if self.active_interval_start is None:
                self.active_interval_start = time_value
            else:
                self.completed_intervals.append((self.active_interval_start, time_value))
                self.active_interval_start = None

        self.last_switch_pressed = pressed

    def current_time(self):
        if self.start_time is None:
            return 0.0
        now_msg = self.get_clock().now().to_msg()
        now = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
        return now - self.start_time

    def update_plot(self):
        now = self.current_time()
        min_time = max(0.0, now - self.window_sec)

        for series in (self.encoder, self.force, self.imu, self.cmd_vel):
            self.prune_series(series, min_time)
        self.prune_intervals(min_time)

        plot_end = max(self.window_sec, now)

        for series in (self.encoder, self.force, self.imu, self.cmd_vel):
            for field, values in series.items():
                times = [item[0] for item in values]
                y_values = [item[1] for item in values]
                self.lines[field].set_data(times, y_values)

        self.lines["reference_vx"].set_data(
            [min_time, plot_end],
            [self.reference_speed_mps, self.reference_speed_mps],
        )

        for ax in self.axes:
            for patch in list(ax.patches):
                patch.remove()

            for start, end in self.completed_intervals:
                if end >= min_time and start <= now:
                    ax.axvspan(
                        max(start, min_time),
                        min(end, now),
                        color=self.highlight_color,
                        alpha=self.highlight_alpha,
                        linewidth=0,
                    )

            if self.active_interval_start is not None:
                ax.axvspan(
                    max(self.active_interval_start, min_time),
                    now,
                    color=self.highlight_color,
                    alpha=self.highlight_alpha,
                    linewidth=0,
                )

            ax.set_xlim(min_time, plot_end)
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = LiveSensorPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
