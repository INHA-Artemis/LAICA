#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics
from pathlib import Path

import numpy as np

import plot_rosbag_sensors as prs


DEFAULT_OUTPUT_DIR = "/home/artemis/Documents/LAICA_ws/plots/odomDataset"
DEFAULT_ROSBAGS_ROOT = "/home/artemis/Documents/rosbags"


ALIGNED_COLUMNS = [
    "bag_relative",
    "bag_name",
    "person_group",
    "time",
    "encoder_time",
    "force_time",
    "imu_time",
    "odom_time",
    "cmd_vel_time",
    "switch_time",
    "dt_force_ms",
    "dt_imu_ms",
    "dt_odom_ms",
    "dt_cmd_vel_ms",
    "dt_switch_ms",
    "encoder.angle_deg",
    "encoder.angle_unwrapped_deg",
    "encoder.angle_velocity_deg_s",
    "encoder.rev",
    "encoder.rpm",
    "force.force_n",
    "force.baseline_n",
    "force.mad_n",
    "force.dev_n",
    "force.norm_mad",
    "interaction_label",
    "interaction_strength",
    "imu.angular_velocity.x",
    "imu.angular_velocity.z",
    "imu.linear_acceleration.x",
    "imu.linear_acceleration.z",
    "odom.twist.twist.linear.x",
    "odom.twist.twist.linear.y",
    "odom.twist.twist.angular.z",
    "odom.speed_xy",
    "cmd_vel.linear.x",
    "cmd_vel.linear.y",
    "cmd_vel.angular.z",
    "switch.switch_1",
    "switch.switch_2",
]


SUMMARY_COLUMNS = [
    "bag_relative",
    "bag_name",
    "person_group",
    "status",
    "encoder_rows",
    "force_rows",
    "imu_rows",
    "odom_rows",
    "cmd_vel_rows",
    "switch_rows",
    "aligned_rows",
    "aligned_coverage_pct",
    "encoder_duration_s",
    "aligned_duration_s",
    "aligned_start_s",
    "aligned_end_s",
    "force_baseline_n",
    "force_mad_n",
    "force_min_n",
    "force_max_n",
    "force_norm_min",
    "force_norm_max",
    "pull_pct",
    "push_pct",
    "strong_pull_pct",
    "strong_push_pct",
    "odom_vx_mean",
    "odom_vx_std",
    "odom_vx_min",
    "odom_vx_max",
    "odom_speed_xy_mean",
    "odom_speed_xy_std",
    "angle_velocity_abs_mean",
    "imu_ax_std",
    "imu_az_std",
    "dt_force_ms_p95",
    "dt_imu_ms_p95",
    "dt_odom_ms_p95",
]


GROUP_COLUMNS = [
    "person_group",
    "bag_count",
    "aligned_rows",
    "force_baseline_median_n",
    "force_mad_median_n",
    "pull_pct_mean",
    "push_pct_mean",
    "odom_vx_mean",
    "odom_speed_xy_mean",
    "angle_velocity_abs_mean",
    "imu_ax_std_mean",
    "imu_az_std_mean",
]


def person_group_from_name(name):
    if name.startswith("ANDY_"):
        return "ANDY"
    if name.startswith("JH_"):
        return "JH"
    if name.startswith("MH_"):
        return "MH"
    if "_" in name:
        return name.split("_", 1)[0]
    return name


def finite_median(values, fallback=0.0):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return fallback
    return statistics.median(finite)


def finite_std(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if len(finite) < 2:
        return 0.0
    return statistics.pstdev(finite)


def robust_baseline(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return 0.0, 1.0
    baseline = statistics.median(finite)
    deviations = [abs(value - baseline) for value in finite]
    mad = statistics.median(deviations)
    if mad <= 1e-9:
        mad = statistics.pstdev(finite)
    if mad <= 1e-9:
        mad = 1.0
    return baseline, mad


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


def unwrap_degrees(angle_values):
    if not angle_values:
        return []

    unwrapped = [float(angle_values[0])]
    previous_raw = float(angle_values[0])
    for raw_value in angle_values[1:]:
        raw_value = float(raw_value)
        delta = raw_value - previous_raw
        while delta > 180.0:
            delta -= 360.0
        while delta < -180.0:
            delta += 360.0
        unwrapped.append(unwrapped[-1] + delta)
        previous_raw = raw_value
    return unwrapped


def add_angle_velocity(rows):
    if not rows:
        return

    unwrapped = unwrap_degrees([row["encoder.angle_deg"] for row in rows])
    previous_time = None
    previous_angle = None
    for row, angle in zip(rows, unwrapped):
        row["encoder.angle_unwrapped_deg"] = angle
        if previous_time is None:
            row["encoder.angle_velocity_deg_s"] = ""
        else:
            dt = row["time"] - previous_time
            if dt > 0.0:
                row["encoder.angle_velocity_deg_s"] = (angle - previous_angle) / dt
            else:
                row["encoder.angle_velocity_deg_s"] = ""
        previous_time = row["time"]
        previous_angle = angle


def interaction_label(force_norm, threshold, strong_threshold):
    if force_norm <= -strong_threshold:
        return "strong_pull", abs(force_norm)
    if force_norm >= strong_threshold:
        return "strong_push", abs(force_norm)
    if force_norm <= -threshold:
        return "pull", abs(force_norm)
    if force_norm >= threshold:
        return "push", abs(force_norm)
    return "neutral", abs(force_norm)


def align_bag(data, bag_path, rosbags_root, max_dt_sec, threshold, strong_threshold):
    bag_name = bag_path.name
    bag_relative = os.path.relpath(str(bag_path), str(rosbags_root))
    person_group = person_group_from_name(bag_name)

    encoder_rows = sorted(data["encoder"], key=lambda row: row["time"])
    force_rows = sorted(data["force"], key=lambda row: row["time"])
    imu_rows = sorted(data["imu"], key=lambda row: row["time"])
    odom_rows = sorted(data["odom"], key=lambda row: row["time"])
    cmd_vel_rows = sorted(data["cmd_vel"], key=lambda row: row["time"])
    switch_rows = sorted(data["switch"], key=lambda row: row["time"])

    counts = {
        "encoder_rows": len(encoder_rows),
        "force_rows": len(force_rows),
        "imu_rows": len(imu_rows),
        "odom_rows": len(odom_rows),
        "cmd_vel_rows": len(cmd_vel_rows),
        "switch_rows": len(switch_rows),
    }

    force_baseline, force_mad = robust_baseline(row["force_n"] for row in force_rows)

    if not encoder_rows or not force_rows or not imu_rows or not odom_rows:
        return [], make_summary(
            bag_relative,
            bag_name,
            person_group,
            "missing_required_stream",
            counts,
            [],
            force_baseline,
            force_mad,
            encoder_rows,
        )

    indices = {
        "force": 0,
        "imu": 0,
        "odom": 0,
        "cmd_vel": 0,
        "switch": 0,
    }
    aligned = []

    for encoder in encoder_rows:
        time_value = encoder["time"]
        force, indices["force"] = nearest(force_rows, time_value, indices["force"])
        imu, indices["imu"] = nearest(imu_rows, time_value, indices["imu"])
        odom, indices["odom"] = nearest(odom_rows, time_value, indices["odom"])
        cmd_vel, indices["cmd_vel"] = nearest(cmd_vel_rows, time_value, indices["cmd_vel"])
        switch, indices["switch"] = nearest(switch_rows, time_value, indices["switch"])

        required = (force, imu, odom)
        if any(row is None for row in required):
            continue

        dt_force = abs(force["time"] - time_value)
        dt_imu = abs(imu["time"] - time_value)
        dt_odom = abs(odom["time"] - time_value)
        if dt_force > max_dt_sec or dt_imu > max_dt_sec or dt_odom > max_dt_sec:
            continue

        force_value = float(force["force_n"])
        force_dev = force_value - force_baseline
        force_norm = force_dev / force_mad
        label, strength = interaction_label(force_norm, threshold, strong_threshold)

        odom_vx = float(odom.get("twist.twist.linear.x", 0.0))
        odom_vy = float(odom.get("twist.twist.linear.y", 0.0))
        row = {
            "bag_relative": bag_relative,
            "bag_name": bag_name,
            "person_group": person_group,
            "time": time_value,
            "encoder_time": time_value,
            "force_time": force["time"],
            "imu_time": imu["time"],
            "odom_time": odom["time"],
            "cmd_vel_time": "",
            "switch_time": "",
            "dt_force_ms": dt_force * 1000.0,
            "dt_imu_ms": dt_imu * 1000.0,
            "dt_odom_ms": dt_odom * 1000.0,
            "dt_cmd_vel_ms": "",
            "dt_switch_ms": "",
            "encoder.angle_deg": encoder.get("angle_deg", ""),
            "encoder.angle_unwrapped_deg": "",
            "encoder.angle_velocity_deg_s": "",
            "encoder.rev": encoder.get("rev", ""),
            "encoder.rpm": encoder.get("rpm", ""),
            "force.force_n": force_value,
            "force.baseline_n": force_baseline,
            "force.mad_n": force_mad,
            "force.dev_n": force_dev,
            "force.norm_mad": force_norm,
            "interaction_label": label,
            "interaction_strength": strength,
            "imu.angular_velocity.x": imu.get("angular_velocity.x", ""),
            "imu.angular_velocity.z": imu.get("angular_velocity.z", ""),
            "imu.linear_acceleration.x": imu.get("linear_acceleration.x", ""),
            "imu.linear_acceleration.z": imu.get("linear_acceleration.z", ""),
            "odom.twist.twist.linear.x": odom_vx,
            "odom.twist.twist.linear.y": odom_vy,
            "odom.twist.twist.angular.z": odom.get("twist.twist.angular.z", ""),
            "odom.speed_xy": math.hypot(odom_vx, odom_vy),
            "cmd_vel.linear.x": "",
            "cmd_vel.linear.y": "",
            "cmd_vel.angular.z": "",
            "switch.switch_1": "",
            "switch.switch_2": "",
        }

        if cmd_vel is not None:
            dt_cmd_vel = abs(cmd_vel["time"] - time_value)
            if dt_cmd_vel <= max_dt_sec:
                row["cmd_vel_time"] = cmd_vel["time"]
                row["dt_cmd_vel_ms"] = dt_cmd_vel * 1000.0
                row["cmd_vel.linear.x"] = cmd_vel.get("linear.x", "")
                row["cmd_vel.linear.y"] = cmd_vel.get("linear.y", "")
                row["cmd_vel.angular.z"] = cmd_vel.get("angular.z", "")

        if switch is not None:
            dt_switch = abs(switch["time"] - time_value)
            if dt_switch <= max_dt_sec:
                row["switch_time"] = switch["time"]
                row["dt_switch_ms"] = dt_switch * 1000.0
                row["switch.switch_1"] = switch.get("switch_1", "")
                row["switch.switch_2"] = switch.get("switch_2", "")

        aligned.append(row)

    add_angle_velocity(aligned)
    summary = make_summary(
        bag_relative,
        bag_name,
        person_group,
        "ok" if aligned else "no_aligned_rows",
        counts,
        aligned,
        force_baseline,
        force_mad,
        encoder_rows,
    )
    return aligned, summary


def percentile(values, percent):
    finite = [float(value) for value in values if value != "" and math.isfinite(float(value))]
    if not finite:
        return ""
    return float(np.percentile(np.asarray(finite, dtype=float), percent))


def mean_value(values):
    finite = [float(value) for value in values if value != "" and math.isfinite(float(value))]
    if not finite:
        return ""
    return float(statistics.fmean(finite))


def min_value(values):
    finite = [float(value) for value in values if value != "" and math.isfinite(float(value))]
    return min(finite) if finite else ""


def max_value(values):
    finite = [float(value) for value in values if value != "" and math.isfinite(float(value))]
    return max(finite) if finite else ""


def std_value(values):
    finite = [float(value) for value in values if value != "" and math.isfinite(float(value))]
    if len(finite) < 2:
        return ""
    return float(statistics.pstdev(finite))


def make_summary(
    bag_relative,
    bag_name,
    person_group,
    status,
    counts,
    aligned,
    force_baseline,
    force_mad,
    encoder_rows,
):
    encoder_duration = ""
    if len(encoder_rows) >= 2:
        encoder_duration = max(row["time"] for row in encoder_rows) - min(
            row["time"] for row in encoder_rows
        )

    aligned_duration = ""
    aligned_start = ""
    aligned_end = ""
    if len(aligned) >= 2:
        aligned_start = aligned[0]["time"]
        aligned_end = aligned[-1]["time"]
        aligned_duration = aligned_end - aligned_start

    coverage = ""
    if counts["encoder_rows"]:
        coverage = len(aligned) * 100.0 / counts["encoder_rows"]

    labels = [row["interaction_label"] for row in aligned]
    force_values = [row["force.force_n"] for row in aligned]
    force_norm = [row["force.norm_mad"] for row in aligned]
    angle_velocity = [
        abs(float(row["encoder.angle_velocity_deg_s"]))
        for row in aligned
        if row["encoder.angle_velocity_deg_s"] != ""
    ]

    summary = {
        "bag_relative": bag_relative,
        "bag_name": bag_name,
        "person_group": person_group,
        "status": status,
        **counts,
        "aligned_rows": len(aligned),
        "aligned_coverage_pct": coverage,
        "encoder_duration_s": encoder_duration,
        "aligned_duration_s": aligned_duration,
        "aligned_start_s": aligned_start,
        "aligned_end_s": aligned_end,
        "force_baseline_n": force_baseline,
        "force_mad_n": force_mad,
        "force_min_n": min_value(force_values),
        "force_max_n": max_value(force_values),
        "force_norm_min": min_value(force_norm),
        "force_norm_max": max_value(force_norm),
        "pull_pct": labels.count("pull") * 100.0 / len(labels) if labels else "",
        "push_pct": labels.count("push") * 100.0 / len(labels) if labels else "",
        "strong_pull_pct": labels.count("strong_pull") * 100.0 / len(labels) if labels else "",
        "strong_push_pct": labels.count("strong_push") * 100.0 / len(labels) if labels else "",
        "odom_vx_mean": mean_value(row["odom.twist.twist.linear.x"] for row in aligned),
        "odom_vx_std": std_value(row["odom.twist.twist.linear.x"] for row in aligned),
        "odom_vx_min": min_value(row["odom.twist.twist.linear.x"] for row in aligned),
        "odom_vx_max": max_value(row["odom.twist.twist.linear.x"] for row in aligned),
        "odom_speed_xy_mean": mean_value(row["odom.speed_xy"] for row in aligned),
        "odom_speed_xy_std": std_value(row["odom.speed_xy"] for row in aligned),
        "angle_velocity_abs_mean": mean_value(angle_velocity),
        "imu_ax_std": std_value(row["imu.linear_acceleration.x"] for row in aligned),
        "imu_az_std": std_value(row["imu.linear_acceleration.z"] for row in aligned),
        "dt_force_ms_p95": percentile((row["dt_force_ms"] for row in aligned), 95),
        "dt_imu_ms_p95": percentile((row["dt_imu_ms"] for row in aligned), 95),
        "dt_odom_ms_p95": percentile((row["dt_odom_ms"] for row in aligned), 95),
    }
    return summary


def group_summaries(bag_summaries):
    groups = {}
    for row in bag_summaries:
        if row["status"] != "ok":
            continue
        groups.setdefault(row["person_group"], []).append(row)

    summaries = []
    for group_name, rows in sorted(groups.items()):
        summaries.append(
            {
                "person_group": group_name,
                "bag_count": len(rows),
                "aligned_rows": sum(int(row["aligned_rows"]) for row in rows),
                "force_baseline_median_n": finite_median(
                    row["force_baseline_n"] for row in rows
                ),
                "force_mad_median_n": finite_median(row["force_mad_n"] for row in rows),
                "pull_pct_mean": mean_value(row["pull_pct"] for row in rows),
                "push_pct_mean": mean_value(row["push_pct"] for row in rows),
                "odom_vx_mean": mean_value(row["odom_vx_mean"] for row in rows),
                "odom_speed_xy_mean": mean_value(
                    row["odom_speed_xy_mean"] for row in rows
                ),
                "angle_velocity_abs_mean": mean_value(
                    row["angle_velocity_abs_mean"] for row in rows
                ),
                "imu_ax_std_mean": mean_value(row["imu_ax_std"] for row in rows),
                "imu_az_std_mean": mean_value(row["imu_az_std"] for row in rows),
            }
        )
    return summaries


def write_csv(path, columns, rows):
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def build_dataset(args):
    rosbags_root = Path(args.rosbags_root).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    bag_paths = sorted(path.parent for path in rosbags_root.glob("**/metadata.yaml"))
    if args.limit:
        bag_paths = bag_paths[: args.limit]

    config = prs.DEFAULT_CONFIG.copy()
    config.update(
        {
            "imu_fields": [
                "angular_velocity.x",
                "angular_velocity.z",
                "linear_acceleration.x",
                "linear_acceleration.z",
            ],
            "odom_fields": [
                "twist.twist.linear.x",
                "twist.twist.linear.y",
                "twist.twist.angular.z",
            ],
            "relative_time": True,
            "use_header_time": True,
        }
    )

    aligned_rows = []
    bag_summaries = []

    print(f"Found {len(bag_paths)} bag(s).")
    for index, bag_path in enumerate(bag_paths, start=1):
        print(f"[{index}/{len(bag_paths)}] {bag_path}")
        bag_config = config.copy()
        bag_config["bag_paths"] = [str(bag_path)]
        data = prs.read_bags(bag_config)
        aligned, summary = align_bag(
            data,
            bag_path,
            rosbags_root,
            args.max_dt_sec,
            args.force_threshold_mad,
            args.strong_force_threshold_mad,
        )
        aligned_rows.extend(aligned)
        bag_summaries.append(summary)

    group_rows = group_summaries(bag_summaries)

    aligned_path = output_dir / "laica_aligned_odom_dataset.csv"
    bag_summary_path = output_dir / "laica_bag_summary.csv"
    group_summary_path = output_dir / "laica_group_summary.csv"

    write_csv(aligned_path, ALIGNED_COLUMNS, aligned_rows)
    write_csv(bag_summary_path, SUMMARY_COLUMNS, bag_summaries)
    write_csv(group_summary_path, GROUP_COLUMNS, group_rows)

    print(f"Wrote aligned dataset: {aligned_path}")
    print(f"Wrote bag summary:     {bag_summary_path}")
    print(f"Wrote group summary:   {group_summary_path}")
    print(f"Aligned rows:          {len(aligned_rows)}")
    print(f"Bag summaries:         {len(bag_summaries)}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build force/encoder/IMU/odom aligned LAICA datasets."
    )
    parser.add_argument("--rosbags-root", default=DEFAULT_ROSBAGS_ROOT)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--max-dt-sec", type=float, default=0.03)
    parser.add_argument("--force-threshold-mad", type=float, default=1.0)
    parser.add_argument("--strong-force-threshold-mad", type=float, default=2.0)
    parser.add_argument("--limit", type=int, default=0)
    return parser.parse_args()


def main():
    build_dataset(parse_args())


if __name__ == "__main__":
    main()
