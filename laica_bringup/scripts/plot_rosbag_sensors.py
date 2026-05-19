#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/laica_matplotlib")

import matplotlib
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


DEFAULT_CONFIG = {
    "bag_paths": [],
    "output_dir": "plots",
    "png_dir": "",
    "csv_dir": "",
    "output_prefix": "rosbag_plot",
    "encoder_topic": "/encoder/data",
    "force_topic": "/load_cell/data",
    "imu_topic": "/imu",
    "odom_topic": "/odom",
    "cmd_vel_topic": "/laica/predicted_cmd_vel",
    "switch_topic": "/switch/data",
    "encoder_fields": ["angle_deg", "rev", "rpm"],
    "force_fields": ["force_n"],
    "imu_fields": [
        "linear_acceleration.x",
        "linear_acceleration.z",
    ],
    "odom_fields": [
        "twist.twist.linear.x",
        "twist.twist.linear.y",
        "twist.twist.angular.z",
    ],
    "cmd_vel_fields": ["linear.x", "linear.y", "angular.z"],
    "switch_fields": ["switch_1", "switch_2"],
    "combined_motion_stream": "imu",
    "use_header_time": True,
    "relative_time": True,
    "save_csv": True,
    "save_png": True,
    "save_individual_pngs": True,
    "save_combined_png": True,
    "split_combined_odom_pngs": False,
    "show_plots": False,
    "highlight_switch_intervals": True,
    "switch_highlight_fields": ["switch_1"],
    "switch_highlight_color": "tab:orange",
    "switch_highlight_alpha": 0.18,
    "switch_press_debounce_sec": 0.25,
    "switch_min_interval_sec": 0.5,
    "wrap_encoder_angle_deg": True,
    "filter_plot_outliers": True,
    "outlier_mad_threshold": 8.0,
    "outlier_min_samples": 12,
}


def load_config(path):
    with open(path, "r", encoding="utf-8") as config_file:
        raw = yaml.safe_load(config_file) or {}

    params = raw
    if "plot_rosbag_sensors" in raw:
        params = raw["plot_rosbag_sensors"].get("ros__parameters", {})

    config = DEFAULT_CONFIG.copy()
    config.update(params)
    return config


def get_nested_attr(message, field):
    value = message
    for part in field.split("."):
        value = getattr(value, part)
    return value


def message_time_sec(message, bag_time_nsec, use_header_time):
    if use_header_time and hasattr(message, "header"):
        stamp = message.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(bag_time_nsec) * 1e-9


def open_reader(bag_path):
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def read_bags(config):
    topics = {
        config["encoder_topic"]: ("encoder", config["encoder_fields"]),
        config["force_topic"]: ("force", config["force_fields"]),
        config["imu_topic"]: ("imu", config["imu_fields"]),
        config["odom_topic"]: ("odom", config["odom_fields"]),
        config["cmd_vel_topic"]: ("cmd_vel", config["cmd_vel_fields"]),
        config["switch_topic"]: ("switch", config["switch_fields"]),
    }
    data = {
        "encoder": [],
        "force": [],
        "imu": [],
        "odom": [],
        "cmd_vel": [],
        "switch": [],
    }

    for bag_path_text in config["bag_paths"]:
        bag_path = Path(bag_path_text).expanduser()
        if not bag_path.exists():
            raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

        reader = open_reader(bag_path)
        topic_types = {
            topic.name: topic.type for topic in reader.get_all_topics_and_types()
        }
        message_types = {
            topic: get_message(type_name)
            for topic, type_name in topic_types.items()
            if topic in topics
        }

        while reader.has_next():
            topic, serialized, timestamp = reader.read_next()
            if topic not in topics or topic not in message_types:
                continue

            stream_name, fields = topics[topic]
            message = deserialize_message(serialized, message_types[topic])
            row = {
                "bag": str(bag_path),
                "time": message_time_sec(message, timestamp, config["use_header_time"]),
            }
            for field in fields:
                row[field] = get_nested_attr(message, field)
            data[stream_name].append(row)

    if config["relative_time"]:
        all_times = [
            row["time"]
            for rows in data.values()
            for row in rows
        ]
        if all_times:
            start_time = min(all_times)
            for rows in data.values():
                for row in rows:
                    row["time"] -= start_time

    return data


def ensure_dirs(config):
    output_dir = Path(config["output_dir"]).expanduser()
    png_dir = Path(config["png_dir"]).expanduser() if config["png_dir"] else output_dir / "png"
    csv_dir = Path(config["csv_dir"]).expanduser() if config["csv_dir"] else output_dir / "csv"

    png_dir.mkdir(parents=True, exist_ok=True)
    csv_dir.mkdir(parents=True, exist_ok=True)
    return png_dir, csv_dir


def write_stream_csv(path, rows, fields):
    columns = ["bag", "time"] + fields
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def nearest_rows_by_time(data):
    encoder_rows = data["encoder"]
    force_rows = data["force"]
    imu_rows = data["imu"]
    odom_rows = data["odom"]
    cmd_vel_rows = data["cmd_vel"]
    switch_rows = data["switch"]
    if not encoder_rows:
        return []

    def nearest(sorted_rows, time_value, start_index):
        if not sorted_rows:
            return None, start_index
        index = start_index
        while index + 1 < len(sorted_rows) and sorted_rows[index + 1]["time"] <= time_value:
            index += 1
        best = sorted_rows[index]
        if index + 1 < len(sorted_rows):
            next_row = sorted_rows[index + 1]
            if abs(next_row["time"] - time_value) < abs(best["time"] - time_value):
                best = next_row
        return best, index

    combined = []
    force_index = 0
    imu_index = 0
    odom_index = 0
    cmd_vel_index = 0
    switch_index = 0
    force_sorted = sorted(force_rows, key=lambda row: row["time"])
    imu_sorted = sorted(imu_rows, key=lambda row: row["time"])
    odom_sorted = sorted(odom_rows, key=lambda row: row["time"])
    cmd_vel_sorted = sorted(cmd_vel_rows, key=lambda row: row["time"])
    switch_sorted = sorted(switch_rows, key=lambda row: row["time"])

    for encoder in sorted(encoder_rows, key=lambda row: row["time"]):
        force, force_index = nearest(force_sorted, encoder["time"], force_index)
        imu, imu_index = nearest(imu_sorted, encoder["time"], imu_index)
        odom, odom_index = nearest(odom_sorted, encoder["time"], odom_index)
        cmd_vel, cmd_vel_index = nearest(
            cmd_vel_sorted,
            encoder["time"],
            cmd_vel_index,
        )
        switch, switch_index = nearest(
            switch_sorted,
            encoder["time"],
            switch_index,
        )
        combined.append((encoder, force, imu, odom, cmd_vel, switch))
    return combined


def write_combined_csv(path, data, config):
    columns = ["bag", "time"]
    columns += [f"encoder.{field}" for field in config["encoder_fields"]]
    columns += [f"force.{field}" for field in config["force_fields"]]
    columns += [f"imu.{field}" for field in config["imu_fields"]]
    columns += [f"odom.{field}" for field in config["odom_fields"]]
    columns += [f"cmd_vel.{field}" for field in config["cmd_vel_fields"]]
    columns += [f"switch.{field}" for field in config["switch_fields"]]

    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=columns)
        writer.writeheader()
        for encoder, force, imu, odom, cmd_vel, switch in nearest_rows_by_time(data):
            row = {
                "bag": encoder["bag"],
                "time": encoder["time"],
            }
            for field in config["encoder_fields"]:
                row[f"encoder.{field}"] = encoder.get(field)
            if force:
                for field in config["force_fields"]:
                    row[f"force.{field}"] = force.get(field)
            if imu:
                for field in config["imu_fields"]:
                    row[f"imu.{field}"] = imu.get(field)
            if odom:
                for field in config["odom_fields"]:
                    row[f"odom.{field}"] = odom.get(field)
            if cmd_vel:
                for field in config["cmd_vel_fields"]:
                    row[f"cmd_vel.{field}"] = cmd_vel.get(field)
            if switch:
                for field in config["switch_fields"]:
                    row[f"switch.{field}"] = switch.get(field)
            writer.writerow(row)


def wrap_degrees_180(angle_deg):
    return ((float(angle_deg) + 180.0) % 360.0) - 180.0


def remove_extreme_outliers(times, values, mad_threshold, min_samples):
    if len(values) < min_samples:
        return times, values

    median = statistics.median(values)
    deviations = [abs(value - median) for value in values]
    mad = statistics.median(deviations)

    if mad == 0.0:
        nonzero_deviations = [deviation for deviation in deviations if deviation > 0.0]
        if not nonzero_deviations:
            return times, values
        mad = statistics.median(nonzero_deviations)

    limit = mad_threshold * mad
    filtered_times = []
    filtered_values = []
    for time_value, value in zip(times, values):
        if abs(value - median) <= limit:
            filtered_times.append(time_value)
            filtered_values.append(value)

    return filtered_times, filtered_values


def numeric_series(rows, field, stream_name, config):
    times = []
    values = []
    for row in rows:
        value = row.get(field)
        if isinstance(value, bool):
            value = int(value)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            if (
                config["wrap_encoder_angle_deg"]
                and stream_name == "encoder"
                and field == "angle_deg"
            ):
                value = wrap_degrees_180(value)
            times.append(row["time"])
            values.append(value)

    if config["filter_plot_outliers"]:
        times, values = remove_extreme_outliers(
            times,
            values,
            float(config["outlier_mad_threshold"]),
            int(config["outlier_min_samples"]),
        )

    return times, values


def switch_press_intervals(rows, fields, debounce_sec, min_interval_sec):
    press_times = []
    previous_pressed = False
    last_press_time = None

    for row in sorted(rows, key=lambda item: item["time"]):
        pressed = any(bool(row.get(field)) for field in fields)
        if pressed and not previous_pressed:
            press_time = row["time"]
            if last_press_time is None or press_time - last_press_time >= debounce_sec:
                press_times.append(press_time)
                last_press_time = press_time
        previous_pressed = pressed

    intervals = []
    for index in range(0, len(press_times) - 1, 2):
        start_time = press_times[index]
        end_time = press_times[index + 1]
        if end_time - start_time >= min_interval_sec:
            intervals.append((start_time, end_time))
    return intervals


def shade_switch_intervals(ax, intervals, config, show_label):
    label = "switch interval" if show_label else None
    for start_time, end_time in intervals:
        ax.axvspan(
            start_time,
            end_time,
            color=config["switch_highlight_color"],
            alpha=float(config["switch_highlight_alpha"]),
            linewidth=0,
            label=label,
        )
        label = None


def plot_stream(path, title, rows, fields, stream_name, config):
    fig, ax = plt.subplots(figsize=(12, 6))
    for field in fields:
        times, values = numeric_series(rows, field, stream_name, config)
        if times:
            ax.plot(times, values, label=field, linewidth=1.2)

    ax.set_title(title)
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    if ax.lines:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_combined(path, data, config, motion_fields=None, motion_title=None):
    motion_stream_name = config.get("combined_motion_stream", "imu")
    if motion_stream_name not in ("imu", "odom"):
        motion_stream_name = "imu"
    if motion_title is None:
        motion_title = "Odom" if motion_stream_name == "odom" else "IMU"
    if motion_fields is None:
        motion_fields = config[f"{motion_stream_name}_fields"]

    groups = [
        ("Encoder", "encoder", data["encoder"], config["encoder_fields"]),
        ("Force", "force", data["force"], config["force_fields"]),
        (motion_title, motion_stream_name, data[motion_stream_name], motion_fields),
    ]

    switch_intervals = configured_switch_intervals(data, config)

    fig_height = 3.2 * len(groups) + 1.2
    fig, axes = plt.subplots(len(groups), 1, figsize=(12, fig_height), sharex=True)
    for ax, (title, stream_name, rows, fields) in zip(axes, groups):
        if switch_intervals:
            shade_switch_intervals(
                ax,
                switch_intervals,
                config,
                show_label=(ax is axes[0]),
            )
        for field in fields:
            times, values = numeric_series(rows, field, stream_name, config)
            if times:
                ax.plot(times, values, label=field, linewidth=1.0)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        if ax.lines:
            ax.legend(loc="best")
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def configured_switch_intervals(data, config):
    if not config["highlight_switch_intervals"]:
        return []

    switch_highlight_fields = config.get("switch_highlight_fields") or config["switch_fields"]
    return switch_press_intervals(
        data["switch"],
        switch_highlight_fields,
        float(config["switch_press_debounce_sec"]),
        float(config["switch_min_interval_sec"]),
    )


def combined_output_stem(prefix, config):
    motion_stream_name = config.get("combined_motion_stream", "imu")
    if motion_stream_name == "odom":
        return f"{prefix}_time_encoder_force_odom_cmd_vel"
    return f"{prefix}_time_encoder_force_imu_cmd_vel"


def combined_odom_split_stems(prefix):
    return {
        "xy": f"{prefix}_time_encoder_force_odom_xy_cmd_vel",
        "z": f"{prefix}_time_encoder_force_odom_z_cmd_vel",
    }


def save_outputs(data, config):
    png_dir, csv_dir = ensure_dirs(config)
    prefix = config["output_prefix"]

    if config["save_csv"]:
        write_stream_csv(
            csv_dir / f"{prefix}_time_encoder.csv",
            data["encoder"],
            config["encoder_fields"],
        )
        write_stream_csv(
            csv_dir / f"{prefix}_time_force.csv",
            data["force"],
            config["force_fields"],
        )
        write_stream_csv(
            csv_dir / f"{prefix}_time_imu.csv",
            data["imu"],
            config["imu_fields"],
        )
        write_stream_csv(
            csv_dir / f"{prefix}_time_odom.csv",
            data["odom"],
            config["odom_fields"],
        )
        write_stream_csv(
            csv_dir / f"{prefix}_time_cmd_vel.csv",
            data["cmd_vel"],
            config["cmd_vel_fields"],
        )
        write_stream_csv(
            csv_dir / f"{prefix}_time_switch.csv",
            data["switch"],
            config["switch_fields"],
        )
        write_combined_csv(
            csv_dir / f"{combined_output_stem(prefix, config)}.csv",
            data,
            config,
        )

    if config["save_png"] and config["save_individual_pngs"]:
        plot_stream(
            png_dir / f"{prefix}_time_encoder.png",
            "Time - Encoder",
            data["encoder"],
            config["encoder_fields"],
            "encoder",
            config,
        )
        plot_stream(
            png_dir / f"{prefix}_time_force.png",
            "Time - Force",
            data["force"],
            config["force_fields"],
            "force",
            config,
        )
        plot_stream(
            png_dir / f"{prefix}_time_imu.png",
            "Time - IMU",
            data["imu"],
            config["imu_fields"],
            "imu",
            config,
        )
        plot_stream(
            png_dir / f"{prefix}_time_odom.png",
            "Time - Odom",
            data["odom"],
            config["odom_fields"],
            "odom",
            config,
        )
        plot_stream(
            png_dir / f"{prefix}_time_switch.png",
            "Time - Switch",
            data["switch"],
            config["switch_fields"],
            "switch",
            config,
        )

    if (
        config["save_png"]
        and config["save_combined_png"]
        and config.get("split_combined_odom_pngs")
        and config.get("combined_motion_stream") == "odom"
    ):
        odom_fields = config["odom_fields"]
        xy_fields = [
            field for field in odom_fields if field.endswith(".x") or field.endswith(".y")
        ]
        z_fields = [field for field in odom_fields if field.endswith(".z")]
        split_stems = combined_odom_split_stems(prefix)
        if xy_fields:
            plot_combined(
                png_dir / f"{split_stems['xy']}.png",
                data,
                config,
                motion_fields=xy_fields,
                motion_title="Odom X/Y",
            )
        if z_fields:
            plot_combined(
                png_dir / f"{split_stems['z']}.png",
                data,
                config,
                motion_fields=z_fields,
                motion_title="Odom Z",
            )
    elif config["save_png"] and config["save_combined_png"]:
        plot_combined(
            png_dir / f"{combined_output_stem(prefix, config)}.png",
            data,
            config,
        )

    if config["show_plots"]:
        plt.show()

    return png_dir, csv_dir


def main():
    parser = argparse.ArgumentParser(description="Plot selected ROS 2 bag sensor topics.")
    parser.add_argument("--config", required=True, help="Path to plot_bags.yaml.")
    args = parser.parse_args()

    config = load_config(args.config)
    data = read_bags(config)
    png_dir, csv_dir = save_outputs(data, config)

    print("Loaded messages:")
    print(f"  encoder: {len(data['encoder'])}")
    print(f"  force:   {len(data['force'])}")
    print(f"  imu:     {len(data['imu'])}")
    print(f"  odom:    {len(data['odom'])}")
    print(f"  cmd_vel: {len(data['cmd_vel'])}")
    print(f"  switch:  {len(data['switch'])}")
    switch_intervals = configured_switch_intervals(data, config)
    print(f"Detected switch intervals: {len(switch_intervals)}")
    for start_time, end_time in switch_intervals:
        print(f"  {start_time:.3f}s -> {end_time:.3f}s")
    print(f"Saved PNG files to: {png_dir}")
    print(f"Saved CSV files to: {csv_dir}")


if __name__ == "__main__":
    main()
