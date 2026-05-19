#!/usr/bin/env python3

import argparse
import csv
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DEFAULT_ACTIVE_DATASET = "/home/artemis/Documents/LAICA_ws/plots/odomDataset/laica_active_odom_dataset.csv"
DEFAULT_ROSBAGS_ROOT = "/home/artemis/Documents/rosbags"
DEFAULT_OUTPUT_DIR = "/home/artemis/Documents/LAICA_ws/plots/odomDataset"


def format_bytes(num_bytes):
    value = float(num_bytes)
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if abs(value) < 1024.0 or unit == "TB":
            return f"{value:.1f} {unit}" if unit != "B" else f"{value:.0f} {unit}"
        value /= 1024.0
    return f"{value:.1f} TB"


def format_duration(seconds):
    seconds = int(round(float(seconds)))
    hours, rem = divmod(seconds, 3600)
    minutes, secs = divmod(rem, 60)
    if hours:
        return f"{hours}h {minutes:02d}m {secs:02d}s"
    return f"{minutes}m {secs:02d}s"


def bag_duration_from_metadata(metadata_path):
    text = metadata_path.read_text(encoding="utf-8", errors="replace")
    match = re.search(r"rosbag2_bagfile_information:.*?duration:\s*\n\s*nanoseconds:\s*(\d+)", text, re.S)
    if not match:
        return 0.0
    return int(match.group(1)) / 1e9


def summarize_rosbags(root):
    root = Path(root).expanduser()
    bag_dirs = sorted(path.parent for path in root.glob("**/metadata.yaml"))
    total_size_bytes = sum(path.stat().st_size for path in root.rglob("*") if path.is_file())
    total_duration_s = sum(bag_duration_from_metadata(path / "metadata.yaml") for path in bag_dirs)
    return {
        "rosbags_root": str(root),
        "bag_count": len(bag_dirs),
        "total_size_bytes": total_size_bytes,
        "total_duration_s": total_duration_s,
    }


def movement_split(active_dataset, turn_change_threshold_rad_s, change_window_s, max_dt_s):
    usecols = ["bag_relative", "time", "odom.twist.twist.angular.z"]
    df = pd.read_csv(active_dataset, usecols=usecols)

    turn_s = 0.0
    straight_s = 0.0
    valid_bags = 0
    for _, bag_df in df.groupby("bag_relative", sort=False):
        bag_df = bag_df.sort_values("time")
        times = bag_df["time"].to_numpy(dtype=float)
        wz = bag_df["odom.twist.twist.angular.z"].to_numpy(dtype=float)
        if len(times) < 2:
            continue

        dt = np.diff(times, append=times[-1])
        time_steps = np.diff(times)
        median_dt = np.nanmedian(time_steps[np.isfinite(time_steps) & (time_steps > 0.0)])
        if math.isfinite(median_dt) and median_dt > 0.0:
            dt[-1] = median_dt

        previous_times = times - change_window_s
        previous_wz = np.full(len(wz), np.nan)
        in_window = previous_times >= times[0]
        previous_wz[in_window] = np.interp(previous_times[in_window], times, wz)
        wz_change = np.abs(wz - previous_wz)

        valid = (
            np.isfinite(dt)
            & np.isfinite(wz_change)
            & (dt > 0.0)
            & (dt <= max_dt_s)
        )
        if not valid.any():
            continue

        turn = valid & (wz_change >= turn_change_threshold_rad_s)
        straight = valid & ~turn
        turn_s += float(dt[turn].sum())
        straight_s += float(dt[straight].sum())
        valid_bags += 1

    active_s = turn_s + straight_s
    return {
        "active_bag_count": valid_bags,
        "active_duration_s": active_s,
        "turn_duration_s": turn_s,
        "straight_duration_s": straight_s,
        "turn_pct": (turn_s / active_s * 100.0) if active_s else 0.0,
        "straight_pct": (straight_s / active_s * 100.0) if active_s else 0.0,
    }


def write_summary_csv(path, row):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)


def plot_donut(output_png, summary):
    turn = summary["turn_duration_s"]
    straight = summary["straight_duration_s"]
    labels = [
        f"Turn\n{summary['turn_pct']:.1f}%\n{format_duration(turn)}",
        f"Straight\n{summary['straight_pct']:.1f}%\n{format_duration(straight)}",
    ]
    colors = ["#2563eb", "#14b8a6"]

    fig, ax = plt.subplots(figsize=(9.5, 7.0), facecolor="#f8fafc")
    ax.set_facecolor("#f8fafc")
    wedges, _ = ax.pie(
        [turn, straight],
        startangle=90,
        counterclock=False,
        colors=colors,
        wedgeprops={"width": 0.36, "edgecolor": "#f8fafc", "linewidth": 4},
    )

    ax.text(
        0,
        0.11,
        "Active motion",
        ha="center",
        va="center",
        fontsize=13,
        color="#475569",
        weight="semibold",
    )
    ax.text(
        0,
        -0.05,
        format_duration(summary["active_duration_s"]),
        ha="center",
        va="center",
        fontsize=25,
        color="#0f172a",
        weight="bold",
    )
    ax.text(
        0,
        -0.23,
        f"|delta odom angular.z over {summary['turn_change_window_s']:.1f}s| >= {summary['turn_change_threshold_rad_s']:.2f} rad/s = turn",
        ha="center",
        va="center",
        fontsize=10,
        color="#64748b",
    )

    for wedge, label in zip(wedges, labels):
        angle = (wedge.theta1 + wedge.theta2) / 2.0
        x = 1.22 * math.cos(math.radians(angle))
        y = 1.22 * math.sin(math.radians(angle))
        ax.text(x, y, label, ha="center", va="center", fontsize=12, color="#0f172a", weight="bold")

    fig.text(0.5, 0.94, "LAICA Rosbag Dataset Summary", ha="center", fontsize=22, weight="bold", color="#0f172a")
    fig.text(
        0.5,
        0.895,
        f"{summary['bag_count']} rosbags | {format_bytes(summary['total_size_bytes'])} | total recording {format_duration(summary['total_duration_s'])}",
        ha="center",
        fontsize=13,
        color="#334155",
    )
    fig.text(
        0.5,
        0.055,
        f"Movement split uses {summary['active_bag_count']} active bags from laica_active_odom_dataset.csv.",
        ha="center",
        fontsize=10,
        color="#64748b",
    )
    ax.set(aspect="equal")
    ax.axis("off")
    fig.savefig(output_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--active-dataset", default=DEFAULT_ACTIVE_DATASET)
    parser.add_argument("--rosbags-root", default=DEFAULT_ROSBAGS_ROOT)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--turn-change-threshold-rad-s", type=float, default=0.30)
    parser.add_argument("--turn-change-window-s", type=float, default=0.5)
    parser.add_argument("--max-dt-s", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    bag_summary = summarize_rosbags(args.rosbags_root)
    split = movement_split(
        args.active_dataset,
        args.turn_change_threshold_rad_s,
        args.turn_change_window_s,
        args.max_dt_s,
    )
    summary = {
        **bag_summary,
        **split,
        "turn_change_threshold_rad_s": args.turn_change_threshold_rad_s,
        "turn_change_window_s": args.turn_change_window_s,
    }

    output_png = output_dir / "laica_rosbag_dataset_donut_summary.png"
    output_csv = output_dir / "laica_rosbag_dataset_donut_summary.csv"
    output_txt = output_dir / "laica_rosbag_dataset_donut_summary.txt"

    plot_donut(output_png, summary)
    write_summary_csv(output_csv, summary)
    output_txt.write_text(
        "\n".join(
            [
                "LAICA ROSBAG DATASET SUMMARY",
                f"rosbags_root={summary['rosbags_root']}",
                f"bag_count={summary['bag_count']}",
                f"total_size={format_bytes(summary['total_size_bytes'])}",
                f"total_duration={format_duration(summary['total_duration_s'])}",
                f"active_bag_count={summary['active_bag_count']}",
                f"active_duration={format_duration(summary['active_duration_s'])}",
                f"turn_metric=abs_delta_odom_twist_twist_angular_z",
                f"turn_change_window_s={summary['turn_change_window_s']:.3f}",
                f"turn_change_threshold_rad_s={summary['turn_change_threshold_rad_s']:.3f}",
                f"turn_duration={format_duration(summary['turn_duration_s'])}",
                f"turn_pct={summary['turn_pct']:.2f}",
                f"straight_duration={format_duration(summary['straight_duration_s'])}",
                f"straight_pct={summary['straight_pct']:.2f}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"Wrote {output_png}")
    print(f"Wrote {output_csv}")
    print(f"Wrote {output_txt}")


if __name__ == "__main__":
    main()
