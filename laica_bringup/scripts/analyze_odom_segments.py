#!/usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
import pandas as pd


DEFAULT_DATASET = "/home/artemis/Documents/LAICA_ws/plots/odomDataset/laica_aligned_odom_dataset.csv"
DEFAULT_OUTPUT_DIR = "/home/artemis/Documents/LAICA_ws/plots/odomDataset"


def intervals_from_mask(times, mask):
    indices = np.flatnonzero(mask)
    if len(indices) == 0:
        return []
    splits = np.where(np.diff(indices) > 1)[0] + 1
    groups = np.split(indices, splits)
    return [(float(times[group[0]]), float(times[group[-1]])) for group in groups if len(group)]


def merge_intervals(intervals, max_gap):
    if not intervals:
        return []

    merged = [list(intervals[0])]
    for start, end in intervals[1:]:
        if start - merged[-1][1] <= max_gap:
            merged[-1][1] = max(merged[-1][1], end)
        else:
            merged.append([start, end])
    return [(start, end) for start, end in merged]


def add_margin(intervals, margin, min_time, max_time):
    expanded = [
        (max(min_time, start - margin), min(max_time, end + margin))
        for start, end in intervals
    ]
    return merge_intervals(expanded, 0.0)


def mask_from_intervals(times, intervals):
    mask = np.zeros(len(times), dtype=bool)
    for start, end in intervals:
        mask |= (times >= start) & (times <= end)
    return mask


def interval_duration(intervals):
    return float(sum(max(0.0, end - start) for start, end in intervals))


def corr_at_lag(times, x_values, y_values, lag):
    mask = np.isfinite(times) & np.isfinite(x_values) & np.isfinite(y_values)
    times = times[mask]
    x_values = x_values[mask]
    y_values = y_values[mask]
    if len(times) < 100 or np.nanstd(x_values) < 1e-9 or np.nanstd(y_values) < 1e-9:
        return np.nan

    order = np.argsort(times)
    times = times[order]
    x_values = x_values[order]
    y_values = y_values[order]
    target_times = times + lag
    valid = (target_times >= times[0]) & (target_times <= times[-1])
    if valid.sum() < 100:
        return np.nan

    y_lagged = np.interp(target_times[valid], times, y_values)
    x_valid = x_values[valid]
    if np.nanstd(x_valid) < 1e-9 or np.nanstd(y_lagged) < 1e-9:
        return np.nan
    return float(np.corrcoef(x_valid, y_lagged)[0, 1])


def label_phase(times, start, end):
    phase = np.full(len(times), "post", dtype=object)
    phase[times < start] = "pre"
    phase[(times >= start) & (times <= end)] = "event"
    return phase


def build_active_and_events(df, args):
    active_chunks = []
    event_window_chunks = []
    quality_rows = []
    event_summary_rows = []

    for bag, bag_df in df.groupby("bag_relative", sort=True):
        bag_df = bag_df.sort_values("time").reset_index(drop=True)
        times = bag_df["time"].to_numpy(dtype=float)
        min_time = float(times[0])
        max_time = float(times[-1])
        aligned_duration = max(0.0, max_time - min_time)

        moving = bag_df["odom.speed_xy"].to_numpy(dtype=float) > args.active_speed_threshold
        moving_intervals = intervals_from_mask(times, moving)
        active_intervals = merge_intervals(moving_intervals, args.merge_gap_sec)
        active_intervals = [
            (start, end)
            for start, end in active_intervals
            if end - start >= args.min_active_duration_sec
        ]
        active_intervals = add_margin(active_intervals, args.active_margin_sec, min_time, max_time)
        active_mask = mask_from_intervals(times, active_intervals)
        active_df = bag_df.loc[active_mask].copy()
        active_df["active_segment"] = True

        force_norm = bag_df["force.norm_mad"].to_numpy(dtype=float)
        event_base_mask = active_mask & np.isfinite(force_norm)
        pull_mask = event_base_mask & (force_norm <= -args.event_threshold_mad)
        push_mask = event_base_mask & (force_norm >= args.event_threshold_mad)

        event_counts = {"strong_pull": 0, "strong_push": 0}
        for event_type, event_mask in (("strong_pull", pull_mask), ("strong_push", push_mask)):
            raw_events = intervals_from_mask(times, event_mask)
            events = merge_intervals(raw_events, args.event_merge_gap_sec)
            events = [
                (start, end)
                for start, end in events
                if end - start >= args.min_event_duration_sec
            ]
            event_counts[event_type] = len(events)

            for event_index, (start, end) in enumerate(events):
                window_start = max(min_time, start - args.event_window_margin_sec)
                window_end = min(max_time, end + args.event_window_margin_sec)
                event_window_mask = (
                    active_mask & (times >= window_start) & (times <= window_end)
                )
                window_df = bag_df.loc[event_window_mask].copy()
                if window_df.empty:
                    continue

                event_id = f"{bag}:{event_type}:{event_index}"
                window_times = window_df["time"].to_numpy(dtype=float)
                window_df["event_id"] = event_id
                window_df["event_type"] = event_type
                window_df["event_start_s"] = start
                window_df["event_end_s"] = end
                window_df["event_phase"] = label_phase(window_times, start, end)
                event_window_chunks.append(window_df)

                pre_mask = active_mask & (times >= start - args.event_window_margin_sec) & (times < start)
                during_mask = active_mask & (times >= start) & (times <= end)
                post_mask = active_mask & (times > end) & (times <= end + args.event_window_margin_sec)
                if pre_mask.sum() < 10 or during_mask.sum() < 10:
                    continue

                pre = bag_df.loc[pre_mask]
                during = bag_df.loc[during_mask]
                post = bag_df.loc[post_mask]
                event_summary_rows.append(
                    {
                        "event_id": event_id,
                        "bag_relative": bag,
                        "bag_name": bag_df["bag_name"].iloc[0],
                        "person_group": bag_df["person_group"].iloc[0],
                        "event_type": event_type,
                        "start_s": start,
                        "end_s": end,
                        "duration_s": end - start,
                        "force_norm_min": during["force.norm_mad"].min(),
                        "force_norm_max": during["force.norm_mad"].max(),
                        "pre_vx_mean": pre["odom.twist.twist.linear.x"].mean(),
                        "event_vx_mean": during["odom.twist.twist.linear.x"].mean(),
                        "post_vx_mean": post["odom.twist.twist.linear.x"].mean()
                        if len(post) >= 10
                        else np.nan,
                        "delta_event_minus_pre_vx": during[
                            "odom.twist.twist.linear.x"
                        ].mean()
                        - pre["odom.twist.twist.linear.x"].mean(),
                        "pre_speed_mean": pre["odom.speed_xy"].mean(),
                        "event_speed_mean": during["odom.speed_xy"].mean(),
                        "post_speed_mean": post["odom.speed_xy"].mean()
                        if len(post) >= 10
                        else np.nan,
                        "delta_event_minus_pre_speed": during["odom.speed_xy"].mean()
                        - pre["odom.speed_xy"].mean(),
                    }
                )

        if not active_df.empty:
            active_chunks.append(active_df)

        active_duration = interval_duration(active_intervals)
        quality_rows.append(
            {
                "bag_relative": bag,
                "bag_name": bag_df["bag_name"].iloc[0],
                "person_group": bag_df["person_group"].iloc[0],
                "aligned_rows": len(bag_df),
                "active_rows": len(active_df),
                "aligned_duration_s": aligned_duration,
                "active_segment_duration_s": active_duration,
                "active_coverage_pct": (active_duration / aligned_duration * 100.0)
                if aligned_duration > 0
                else 0.0,
                "active_intervals": len(active_intervals),
                "moving_row_pct": float(moving.mean() * 100.0),
                "active_speed_mean": active_df["odom.speed_xy"].mean()
                if len(active_df)
                else np.nan,
                "active_vx_mean": active_df["odom.twist.twist.linear.x"].mean()
                if len(active_df)
                else np.nan,
                "active_wz_abs_mean": active_df["odom.twist.twist.angular.z"].abs().mean()
                if len(active_df)
                else np.nan,
                "strong_pull_events": event_counts["strong_pull"],
                "strong_push_events": event_counts["strong_push"],
            }
        )

    active = pd.concat(active_chunks, ignore_index=True) if active_chunks else df.iloc[0:0]
    event_windows = (
        pd.concat(event_window_chunks, ignore_index=True)
        if event_window_chunks
        else df.iloc[0:0]
    )
    quality = pd.DataFrame(quality_rows)
    events = pd.DataFrame(event_summary_rows)
    return active, event_windows, quality, events


def analyze_active(active, events, args):
    bag_rows = []
    for bag, bag_df in active.groupby("bag_relative", sort=True):
        speed = bag_df["odom.speed_xy"]
        vx = bag_df["odom.twist.twist.linear.x"]
        vy = bag_df["odom.twist.twist.linear.y"]
        wz = bag_df["odom.twist.twist.angular.z"]
        bag_rows.append(
            {
                "bag_relative": bag,
                "bag_name": bag_df["bag_name"].iloc[0],
                "person_group": bag_df["person_group"].iloc[0],
                "active_rows": len(bag_df),
                "active_duration_s": bag_df["time"].iloc[-1] - bag_df["time"].iloc[0],
                "speed_mean": speed.mean(),
                "speed_std": speed.std(ddof=0),
                "speed_p50": speed.median(),
                "speed_p90": speed.quantile(0.90),
                "speed_p95": speed.quantile(0.95),
                "vx_mean": vx.mean(),
                "vx_std": vx.std(ddof=0),
                "vx_min": vx.min(),
                "vx_max": vx.max(),
                "vy_abs_mean": vy.abs().mean(),
                "wz_abs_mean": wz.abs().mean(),
                "wz_abs_p95": wz.abs().quantile(0.95),
                "force_norm_abs_mean": active.loc[
                    active["bag_relative"] == bag, "force.norm_mad"
                ]
                .abs()
                .clip(upper=5.0)
                .mean(),
            }
        )
    bag_metrics = pd.DataFrame(bag_rows)

    lags = np.round(np.arange(args.lag_min_sec, args.lag_max_sec + 1e-9, args.lag_step_sec), 3)
    lag_rows = []
    pairs = [
        ("force_signed_vs_vx", "force_norm_clip", "odom.twist.twist.linear.x", "signed"),
        ("abs_force_vs_speed", "abs_force_norm", "odom.speed_xy", "absolute"),
        ("pull_vs_vx", "pull_strength", "odom.twist.twist.linear.x", "pull"),
        ("push_vs_vx", "push_strength", "odom.twist.twist.linear.x", "push"),
        ("abs_force_vs_abs_wz", "abs_force_norm", "odom.abs_wz", "turning"),
    ]

    active = active.copy()
    active["force_norm_clip"] = active["force.norm_mad"].clip(-5.0, 5.0)
    active["pull_strength"] = (-active["force.norm_mad"]).clip(lower=0.0, upper=5.0)
    active["push_strength"] = active["force.norm_mad"].clip(lower=0.0, upper=5.0)
    active["abs_force_norm"] = active["force.norm_mad"].abs().clip(upper=5.0)
    active["odom.abs_wz"] = active["odom.twist.twist.angular.z"].abs()

    for bag, bag_df in active.groupby("bag_relative", sort=True):
        times = bag_df["time"].to_numpy(dtype=float)
        for metric, x_column, y_column, kind in pairs:
            x_values = bag_df[x_column].to_numpy(dtype=float)
            y_values = bag_df[y_column].to_numpy(dtype=float)
            values = np.array(
                [corr_at_lag(times, x_values, y_values, lag) for lag in lags],
                dtype=float,
            )
            if np.all(~np.isfinite(values)):
                best_lag = np.nan
                best_corr = np.nan
                best_abs_corr = np.nan
            else:
                best_index = int(np.nanargmax(np.abs(values)))
                best_lag = float(lags[best_index])
                best_corr = float(values[best_index])
                best_abs_corr = abs(best_corr)
            zero_index = int(np.argmin(np.abs(lags)))
            lag_rows.append(
                {
                    "bag_relative": bag,
                    "bag_name": bag_df["bag_name"].iloc[0],
                    "person_group": bag_df["person_group"].iloc[0],
                    "metric": metric,
                    "kind": kind,
                    "best_lag_s": best_lag,
                    "best_corr": best_corr,
                    "best_abs_corr": best_abs_corr,
                    "zero_lag_corr": float(values[zero_index])
                    if np.isfinite(values[zero_index])
                    else np.nan,
                }
            )

    lag_corr = pd.DataFrame(lag_rows)
    return bag_metrics, lag_corr, events


def write_summary(output_dir, active, quality, bag_metrics, lag_corr, events):
    lines = []
    lines.append("ACTIVE ODOM ANALYSIS SUMMARY")
    lines.append(f"active_rows={len(active)}")
    lines.append(f"active_bags={active['bag_relative'].nunique() if len(active) else 0}")
    lines.append("")

    lines.append("ACTIVE SEGMENT QUALITY")
    if "excluded_from_active_analysis" in quality:
        excluded = quality[quality["excluded_from_active_analysis"]]
        lines.append(f"excluded_low_active_coverage_bags={len(excluded)}")
        for row in excluded.sort_values("active_coverage_pct").itertuples(index=False):
            lines.append(
                f"  excluded {row.bag_relative}: active={row.active_coverage_pct:.1f}%, "
                f"reason={row.exclusion_reason}"
            )
    lines.append(
        f"median_active_coverage_pct={quality['active_coverage_pct'].median():.1f}"
    )
    low = quality[quality["active_coverage_pct"] < 50.0]
    lines.append(f"low_active_coverage_bags_lt_50pct={len(low)}")
    for row in low.sort_values("active_coverage_pct").itertuples(index=False):
        lines.append(
            f"  {row.bag_relative}: active={row.active_coverage_pct:.1f}%, "
            f"intervals={row.active_intervals}, strong_pull={row.strong_pull_events}, "
            f"strong_push={row.strong_push_events}"
        )
    lines.append("")

    lines.append("ACTIVE WALKING PACE BY GROUP")
    for group, group_df in bag_metrics.groupby("person_group", sort=True):
        lines.append(
            f"{group}: bags={len(group_df)}, speed_mean={group_df['speed_mean'].mean():.3f}, "
            f"speed_p90={group_df['speed_p90'].mean():.3f}, vx_mean={group_df['vx_mean'].mean():.3f}, "
            f"vx_std={group_df['vx_std'].mean():.3f}, wz_abs={group_df['wz_abs_mean'].mean():.3f}"
        )
    lines.append("")

    lines.append("CORE GROUPS")
    for group in ("ANDY", "JH", "MH"):
        group_df = bag_metrics[bag_metrics["person_group"] == group]
        if group_df.empty:
            continue
        lines.append(
            f"{group}: speed_mean={group_df['speed_mean'].mean():.3f}, "
            f"speed_p90={group_df['speed_p90'].mean():.3f}, "
            f"vx_std={group_df['vx_std'].mean():.3f}, "
            f"wz_abs={group_df['wz_abs_mean'].mean():.3f}"
        )
    lines.append("")

    lines.append("ACTIVE LAGGED CORRELATION BY GROUP, median over bags")
    for metric, sub in lag_corr.groupby("metric", sort=True):
        lines.append(metric)
        for group, group_df in sub.groupby("person_group", sort=True):
            lines.append(
                f"  {group}: best_abs_corr_median={group_df['best_abs_corr'].median():.3f}, "
                f"best_corr_median={group_df['best_corr'].median():.3f}, "
                f"best_lag_median={group_df['best_lag_s'].median():.2f}s, "
                f"zero_lag_median={group_df['zero_lag_corr'].median():.3f}"
            )
    lines.append("")

    lines.append("ACTIVE EVENT RESPONSE BY GROUP, median event minus pre")
    if events.empty:
        lines.append("no_valid_events")
    else:
        for event_type, sub in events.groupby("event_type", sort=True):
            lines.append(event_type)
            for group, group_df in sub.groupby("person_group", sort=True):
                lines.append(
                    f"  {group}: events={len(group_df)}, "
                    f"delta_vx={group_df['delta_event_minus_pre_vx'].median():.3f}, "
                    f"delta_speed={group_df['delta_event_minus_pre_speed'].median():.3f}, "
                    f"duration={group_df['duration_s'].median():.2f}s"
                )

    summary_path = output_dir / "odom_active_analysis_summary.txt"
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze active walking and strong push/pull odom segments."
    )
    parser.add_argument("--dataset", default=DEFAULT_DATASET)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--active-speed-threshold", type=float, default=0.05)
    parser.add_argument("--min-active-duration-sec", type=float, default=2.0)
    parser.add_argument("--merge-gap-sec", type=float, default=1.0)
    parser.add_argument("--active-margin-sec", type=float, default=0.5)
    parser.add_argument("--event-threshold-mad", type=float, default=2.0)
    parser.add_argument("--event-merge-gap-sec", type=float, default=0.2)
    parser.add_argument("--min-event-duration-sec", type=float, default=0.30)
    parser.add_argument("--event-window-margin-sec", type=float, default=1.0)
    parser.add_argument("--min-active-coverage-pct", type=float, default=50.0)
    parser.add_argument("--lag-min-sec", type=float, default=-2.0)
    parser.add_argument("--lag-max-sec", type=float, default=2.0)
    parser.add_argument("--lag-step-sec", type=float, default=0.1)
    return parser.parse_args()


def main():
    args = parse_args()
    dataset_path = Path(args.dataset).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading aligned dataset: {dataset_path}")
    df = pd.read_csv(dataset_path)
    numeric_columns = [
        "time",
        "force.norm_mad",
        "odom.twist.twist.linear.x",
        "odom.twist.twist.linear.y",
        "odom.twist.twist.angular.z",
        "odom.speed_xy",
    ]
    for column in numeric_columns:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    df = df.dropna(subset=numeric_columns).sort_values(["bag_relative", "time"])

    active, event_windows, quality, events = build_active_and_events(df, args)
    excluded_bags = set(
        quality.loc[
            quality["active_coverage_pct"] < args.min_active_coverage_pct,
            "bag_relative",
        ]
    )
    quality["excluded_from_active_analysis"] = quality["bag_relative"].isin(excluded_bags)
    quality["exclusion_reason"] = ""
    quality.loc[
        quality["excluded_from_active_analysis"],
        "exclusion_reason",
    ] = (
        "active_coverage_pct < "
        + str(args.min_active_coverage_pct)
    )

    if excluded_bags:
        active = active[~active["bag_relative"].isin(excluded_bags)].copy()
        event_windows = event_windows[
            ~event_windows["bag_relative"].isin(excluded_bags)
        ].copy()
        if not events.empty:
            events = events[~events["bag_relative"].isin(excluded_bags)].copy()

    bag_metrics, lag_corr, event_response = analyze_active(active, events, args)

    active_path = output_dir / "laica_active_odom_dataset.csv"
    event_window_path = output_dir / "laica_event_windows_dataset.csv"
    quality_path = output_dir / "laica_bag_quality_summary.csv"
    bag_metrics_path = output_dir / "odom_active_bag_metrics.csv"
    lag_corr_path = output_dir / "odom_active_lag_correlation.csv"
    event_response_path = output_dir / "odom_active_event_response.csv"

    active.to_csv(active_path, index=False)
    event_windows.to_csv(event_window_path, index=False)
    quality.to_csv(quality_path, index=False)
    bag_metrics.to_csv(bag_metrics_path, index=False)
    lag_corr.to_csv(lag_corr_path, index=False)
    event_response.to_csv(event_response_path, index=False)
    summary_path = write_summary(output_dir, active, quality, bag_metrics, lag_corr, event_response)

    print(f"Wrote active dataset:       {active_path}")
    print(f"Wrote event-window dataset: {event_window_path}")
    print(f"Wrote bag quality summary:  {quality_path}")
    print(f"Wrote active bag metrics:   {bag_metrics_path}")
    print(f"Wrote active lag corr:      {lag_corr_path}")
    print(f"Wrote active event response:{event_response_path}")
    print(f"Wrote summary:              {summary_path}")
    print(f"Active rows:                {len(active)}")
    print(f"Event-window rows:          {len(event_windows)}")
    print(f"Event responses:            {len(event_response)}")


if __name__ == "__main__":
    main()
