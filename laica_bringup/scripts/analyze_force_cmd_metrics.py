#!/usr/bin/env python3
import argparse
import math
import os
import re
import sqlite3
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


FORCE_TOPIC_CANDIDATES = (
    "/laica/debug/zeroed_force",
    "/laica/teleop_debug/zeroed_force",
    "/laica/debug/filtered_force",
    "/laica/teleop_debug/filtered_force",
    "/load_cell/data",
)


def find_bags(root, pattern):
    root = Path(root).expanduser()
    if (root / "metadata.yaml").exists():
        return [root]
    return sorted(
        path
        for path in root.glob(pattern)
        if path.is_dir() and (path / "metadata.yaml").exists()
    )


def parse_bag_name(name):
    match = re.match(r"^(ad|key)_([a-zA-Z0-9]+)_(\d+)$", name)
    if not match:
        return {
            "mode": "unknown",
            "person": "unknown",
            "trial": math.nan,
            "speed": "unknown",
        }

    prefix, person, trial_text = match.groups()
    trial = int(trial_text)
    return {
        "mode": "admittance" if prefix == "ad" else "keyboard",
        "person": person.lower(),
        "trial": trial,
        "speed": "slow" if trial % 2 == 1 else "fast",
    }


def db_files_for_bag(bag_dir):
    return sorted(Path(bag_dir).glob("*.db3"))


def read_topics_from_bag(bag_dir, wanted_topics):
    rows = {topic: [] for topic in wanted_topics}

    for db_file in db_files_for_bag(bag_dir):
        conn = sqlite3.connect(str(db_file))
        try:
            topic_meta = {
                topic_id: (name, type_name)
                for topic_id, name, type_name in conn.execute(
                    "SELECT id, name, type FROM topics"
                )
            }
            message_types = {}
            topic_ids = []
            for topic_id, (name, type_name) in topic_meta.items():
                if name in wanted_topics:
                    try:
                        message_types[topic_id] = get_message(type_name)
                        topic_ids.append(topic_id)
                    except (AttributeError, ModuleNotFoundError, ValueError):
                        continue

            if not topic_ids:
                continue

            placeholders = ",".join("?" for _ in topic_ids)
            query = (
                "SELECT topic_id, timestamp, data FROM messages "
                f"WHERE topic_id IN ({placeholders}) ORDER BY timestamp"
            )
            for topic_id, stamp_ns, blob in conn.execute(query, topic_ids):
                topic_name, _ = topic_meta[topic_id]
                msg = deserialize_message(blob, message_types[topic_id])
                value = value_from_message(topic_name, msg)
                if value is not None and math.isfinite(value):
                    rows[topic_name].append((stamp_ns * 1e-9, float(value)))
        finally:
            conn.close()

    return {
        topic: np.asarray(values, dtype=float)
        for topic, values in rows.items()
        if values
    }


def value_from_message(topic_name, msg):
    if topic_name == "/cmd_vel":
        return msg.linear.x
    if hasattr(msg, "data"):
        return msg.data
    if hasattr(msg, "force_n"):
        return msg.force_n
    return None


def pick_force_series(topic_data):
    for topic in FORCE_TOPIC_CANDIDATES:
        series = topic_data.get(topic)
        if series is not None and len(series) >= 3:
            return topic, series
    return None, None


def pearson(x, y):
    mask = np.isfinite(x) & np.isfinite(y)
    if int(np.count_nonzero(mask)) < 3:
        return math.nan
    x = x[mask]
    y = y[mask]
    if float(np.std(x)) <= 1e-12 or float(np.std(y)) <= 1e-12:
        return math.nan
    return float(np.corrcoef(x, y)[0, 1])


def analyze_bag(bag_dir, max_lag_sec, lag_step_sec, active_cmd_threshold):
    wanted_topics = set(FORCE_TOPIC_CANDIDATES) | {"/cmd_vel"}
    topic_data = read_topics_from_bag(bag_dir, wanted_topics)
    cmd = topic_data.get("/cmd_vel")
    force_topic, force = pick_force_series(topic_data)

    if cmd is None or force is None:
        return {
            "bag": Path(bag_dir).name,
            "force_topic": force_topic or "",
            "cmd_topic": "/cmd_vel" if cmd is not None else "",
            "sample_count": 0,
            "zero_lag_corr": math.nan,
            "best_lag_sec": math.nan,
            "best_lag_corr": math.nan,
            "best_abs_lag_corr": math.nan,
            "force_variance": math.nan,
            "force_std": math.nan,
            "force_mean_abs": math.nan,
        }

    t0 = min(force[0, 0], cmd[0, 0])
    force_t = force[:, 0] - t0
    force_y = force[:, 1]
    cmd_t = cmd[:, 0] - t0
    cmd_y = cmd[:, 1]

    cmd_interp_zero = np.interp(force_t, cmd_t, cmd_y, left=np.nan, right=np.nan)
    active_mask = np.abs(cmd_interp_zero) > active_cmd_threshold
    if int(np.count_nonzero(active_mask)) < 10:
        active_mask = np.isfinite(cmd_interp_zero)

    zero_lag_corr = pearson(force_y[active_mask], cmd_interp_zero[active_mask])

    best_lag = math.nan
    best_corr = math.nan
    best_abs = -1.0
    lag_values = np.arange(-max_lag_sec, max_lag_sec + lag_step_sec * 0.5, lag_step_sec)
    for lag in lag_values:
        # Positive lag means cmd_vel follows force input by `lag` seconds.
        cmd_interp = np.interp(force_t + lag, cmd_t, cmd_y, left=np.nan, right=np.nan)
        mask = active_mask & np.isfinite(cmd_interp)
        corr = pearson(force_y[mask], cmd_interp[mask])
        if math.isfinite(corr) and abs(corr) > best_abs:
            best_abs = abs(corr)
            best_corr = corr
            best_lag = float(lag)

    force_active = force_y[active_mask & np.isfinite(force_y)]
    return {
        "bag": Path(bag_dir).name,
        "force_topic": force_topic,
        "cmd_topic": "/cmd_vel",
        "sample_count": int(len(force_active)),
        "zero_lag_corr": zero_lag_corr,
        "best_lag_sec": best_lag,
        "best_lag_corr": best_corr,
        "best_abs_lag_corr": abs(best_corr) if math.isfinite(best_corr) else math.nan,
        "force_variance": float(np.var(force_active)) if len(force_active) else math.nan,
        "force_std": float(np.std(force_active)) if len(force_active) else math.nan,
        "force_mean_abs": float(np.mean(np.abs(force_active))) if len(force_active) else math.nan,
    }


def grouped_summary(df):
    metric_cols = [
        "zero_lag_corr",
        "best_lag_sec",
        "best_lag_corr",
        "best_abs_lag_corr",
        "force_variance",
        "force_std",
        "force_mean_abs",
    ]
    return (
        df.groupby(["person", "mode", "speed"], dropna=False)[metric_cols]
        .agg(["mean", "std", "count"])
        .reset_index()
    )


def plot_group_metric(group_df, metric, ylabel, title, output_path):
    mean_col = (metric, "mean")
    std_col = (metric, "std")
    if mean_col not in group_df.columns:
        return

    labels = [
        f"{row[('person', '')]} {row[('mode', '')]} {row[('speed', '')]}"
        for _, row in group_df.iterrows()
    ]
    means = group_df[mean_col].astype(float).to_numpy()
    stds = group_df[std_col].astype(float).fillna(0.0).to_numpy()
    colors = [
        "#2f6f9f" if mode == "admittance" else "#b65c32"
        for mode in group_df[("mode", "")]
    ]

    fig, ax = plt.subplots(figsize=(max(8, len(labels) * 0.75), 4.8))
    x = np.arange(len(labels))
    ax.bar(x, means, yerr=stds, capsize=3, color=colors, alpha=0.86)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def flatten_columns(df):
    flat = []
    for col in df.columns:
        if isinstance(col, tuple):
            flat.append("_".join(str(part) for part in col if part))
        else:
            flat.append(str(col))
    df = df.copy()
    df.columns = flat
    return df


def main():
    parser = argparse.ArgumentParser(
        description="Compute force/cmd_vel lagged correlation and force variance from ROS2 bags."
    )
    parser.add_argument("bag_root", help="Bag directory or directory containing bag directories.")
    parser.add_argument("--pattern", default="*", help="Glob pattern for bag directories.")
    parser.add_argument(
        "--output-dir",
        default="rosbag_csv_exports/force_cmd_metrics",
        help="Directory for CSV and plot outputs.",
    )
    parser.add_argument("--max-lag-sec", type=float, default=2.0)
    parser.add_argument("--lag-step-sec", type=float, default=0.02)
    parser.add_argument("--active-cmd-threshold", type=float, default=0.03)
    args = parser.parse_args()

    bags = find_bags(args.bag_root, args.pattern)
    if not bags:
        raise SystemExit(f"No ROS2 bag directories found under {args.bag_root}")

    rows = []
    for bag in bags:
        row = analyze_bag(
            bag,
            max_lag_sec=args.max_lag_sec,
            lag_step_sec=args.lag_step_sec,
            active_cmd_threshold=args.active_cmd_threshold,
        )
        row.update(parse_bag_name(Path(bag).name))
        rows.append(row)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    trial_df = pd.DataFrame(rows)
    group_df = grouped_summary(trial_df)
    flat_group_df = flatten_columns(group_df)

    corr_cols = [
        "bag",
        "person",
        "mode",
        "speed",
        "trial",
        "force_topic",
        "cmd_topic",
        "sample_count",
        "zero_lag_corr",
        "best_lag_sec",
        "best_lag_corr",
        "best_abs_lag_corr",
    ]
    var_cols = [
        "bag",
        "person",
        "mode",
        "speed",
        "trial",
        "force_topic",
        "sample_count",
        "force_variance",
        "force_std",
        "force_mean_abs",
    ]

    trial_df[corr_cols].to_csv(
        output_dir / "force_cmd_lagged_correlation.csv", index=False
    )
    trial_df[var_cols].to_csv(output_dir / "force_variance.csv", index=False)
    flat_group_df.to_csv(output_dir / "force_metrics_grouped_summary.csv", index=False)

    plot_group_metric(
        group_df,
        "best_abs_lag_corr",
        "|correlation|",
        "Force vs lagged cmd_vel correlation",
        output_dir / "force_cmd_lagged_correlation.png",
    )
    plot_group_metric(
        group_df,
        "force_variance",
        "force variance (N^2)",
        "Force variance",
        output_dir / "force_variance.png",
    )

    print(f"Analyzed {len(trial_df)} bags")
    print(f"Wrote outputs to {output_dir}")


if __name__ == "__main__":
    main()
