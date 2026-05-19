#!/usr/bin/env python3
"""Replay rosbag-derived force data through the 1D admittance controller."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def apply_deadband(force_n: float, deadband_n: float) -> float:
    abs_force = abs(force_n)
    if abs_force <= deadband_n:
        return 0.0
    return math.copysign(abs_force - deadband_n, force_n)


def simulate_admittance(
    data: pd.DataFrame,
    *,
    publish_rate_hz: float,
    zero_force_duration_sec: float,
    force_filter_tau_sec: float,
    force_deadband_n: float,
    force_velocity_sign: float,
    admittance_mass: float,
    admittance_damping: float,
    base_velocity_mps: float,
    min_velocity_mps: float,
    max_velocity_mps: float,
    max_accel_mps2: float,
) -> pd.DataFrame:
    """Mirror the C++ force-only admittance controller logic offline."""
    sim = data.sort_values("rel_time_s").copy()
    sim = sim[np.isfinite(sim["force.force_n"])].copy()
    if sim.empty:
        return sim

    nominal_dt = 1.0 / max(publish_rate_hz, 1.0)
    times = sim["rel_time_s"].to_numpy(dtype=float)
    forces = sim["force.force_n"].to_numpy(dtype=float)

    zero_sum = 0.0
    zero_samples = 0
    zero_offset = 0.0
    zero_complete = zero_force_duration_sec <= 0.0

    filtered_force = 0.0
    has_filtered = False
    admittance_velocity = 0.0
    previous_cmd = 0.0
    has_previous_cmd = True
    last_time = times[0]

    rows = []
    for t, force_input in zip(times, forces):
        dt = t - last_time
        last_time = t
        if not np.isfinite(dt) or dt <= 0.0:
            dt = nominal_dt
        dt = min(dt, 0.2)

        if not zero_complete:
            zero_sum += force_input
            zero_samples += 1
            if t - times[0] >= zero_force_duration_sec:
                zero_offset = zero_sum / max(zero_samples, 1)
                zero_complete = True
            else:
                rows.append(
                    {
                        "rel_time_s": t,
                        "force_zero_offset_n": np.nan,
                        "force_zeroed_n": 0.0,
                        "force_filtered_n": 0.0,
                        "force_effective_n": 0.0,
                        "admittance_velocity_mps": 0.0,
                        "cmd_vx_mps": 0.0,
                        "zeroing": True,
                    }
                )
                continue

        force_zeroed = force_input - zero_offset
        if not has_filtered or force_filter_tau_sec <= 0.0:
            filtered_force = force_zeroed
        else:
            alpha = dt / (force_filter_tau_sec + dt)
            filtered_force = filtered_force + alpha * (force_zeroed - filtered_force)
        has_filtered = True

        force_effective = force_velocity_sign * apply_deadband(
            filtered_force, force_deadband_n
        )
        accel = (
            force_effective - admittance_damping * admittance_velocity
        ) / max(admittance_mass, 1.0e-6)
        admittance_velocity += accel * dt

        target = base_velocity_mps + admittance_velocity
        clamped_target = max(min_velocity_mps, min(max_velocity_mps, target))
        if max_accel_mps2 > 0.0 and has_previous_cmd:
            max_delta = max_accel_mps2 * dt
            delta = clamped_target - previous_cmd
            previous_cmd += max(-max_delta, min(max_delta, delta))
        else:
            previous_cmd = clamped_target
        has_previous_cmd = True

        rows.append(
            {
                "rel_time_s": t,
                "force_zero_offset_n": zero_offset,
                "force_zeroed_n": force_zeroed,
                "force_filtered_n": filtered_force,
                "force_effective_n": force_effective,
                "admittance_velocity_mps": admittance_velocity,
                "cmd_vx_mps": previous_cmd,
                "zeroing": False,
            }
        )

    output = pd.DataFrame(rows)
    return sim.merge(output, on="rel_time_s", how="left")


def resample_bag(bag_df: pd.DataFrame, sample_dt: float) -> pd.DataFrame:
    bag_df = bag_df.sort_values("time").copy()
    t0 = float(bag_df["time"].iloc[0])
    bag_df["tbin"] = np.floor((bag_df["time"] - t0) / sample_dt).astype(int)
    resampled = bag_df.groupby("tbin", sort=True).mean(numeric_only=True)
    full_index = np.arange(int(resampled.index.min()), int(resampled.index.max()) + 1)
    resampled = resampled.reindex(full_index).interpolate(limit=6, limit_direction="both")
    resampled["rel_time_s"] = (resampled.index - resampled.index.min()) * sample_dt
    return resampled


def summarize(sim: pd.DataFrame, metadata: dict[str, object]) -> dict[str, object]:
    active = sim[~sim["zeroing"].fillna(False)].copy()
    if active.empty:
        active = sim.copy()
    cmd = active["cmd_vx_mps"].dropna()
    force_eff = active["force_effective_n"].dropna()
    return {
        **metadata,
        "duration_s": float(sim["rel_time_s"].max() - sim["rel_time_s"].min())
        if len(sim)
        else 0.0,
        "post_zero_duration_s": float(len(active) * np.nanmedian(np.diff(sim["rel_time_s"])))
        if len(active) > 1 and len(sim) > 1
        else 0.0,
        "cmd_vx_mean_mps": float(cmd.mean()) if len(cmd) else np.nan,
        "cmd_vx_median_mps": float(cmd.median()) if len(cmd) else np.nan,
        "cmd_vx_std_mps": float(cmd.std()) if len(cmd) else np.nan,
        "cmd_vx_min_mps": float(cmd.min()) if len(cmd) else np.nan,
        "cmd_vx_max_mps": float(cmd.max()) if len(cmd) else np.nan,
        "cmd_vx_range_mps": float(cmd.max() - cmd.min()) if len(cmd) else np.nan,
        "cmd_vx_p95_mps": float(cmd.quantile(0.95)) if len(cmd) else np.nan,
        "force_effective_abs_median_n": float(force_eff.abs().median())
        if len(force_eff)
        else np.nan,
        "force_effective_abs_p95_n": float(force_eff.abs().quantile(0.95))
        if len(force_eff)
        else np.nan,
    }


def plot_simulation(sim: pd.DataFrame, out_png: Path, title: str) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=13)
    t = sim["rel_time_s"]

    axes[0].plot(t, sim["force.force_n"], color="0.70", linewidth=1.0, label="force raw N")
    axes[0].plot(t, sim["force_filtered_n"], color="#1f77b4", linewidth=2.0, label="filtered zeroed force N")
    axes[0].axhline(0.0, color="k", linewidth=0.7, alpha=0.4)
    axes[0].set_ylabel("Force (N)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(t, sim["force_effective_n"], color="#ff7f0e", linewidth=2.0, label="effective force after deadband")
    axes[1].axhline(0.0, color="k", linewidth=0.7, alpha=0.4)
    axes[1].set_ylabel("F_eff (N)")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(t, sim["cmd_vx_mps"], color="#2ca02c", linewidth=2.0, label="admittance cmd_vx")
    if "odom.speed_xy" in sim:
        axes[2].plot(t, sim["odom.speed_xy"], color="0.55", linewidth=1.0, label="recorded odom speed")
    axes[2].set_ylabel("m/s")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.25)

    if "odom.twist.twist.linear.x" in sim:
        axes[3].plot(t, sim["odom.twist.twist.linear.x"], color="#d62728", linewidth=1.4, label="recorded odom vx")
    if "encoder.angle_unwrapped_deg" in sim:
        ax2 = axes[3].twinx()
        ax2.plot(t, sim["encoder.angle_unwrapped_deg"], color="#8c564b", linewidth=1.0, alpha=0.6, label="encoder angle")
        ax2.set_ylabel("deg")
    axes[3].set_ylabel("odom vx")
    axes[3].set_xlabel("time in simulated segment (s)")
    axes[3].legend(loc="upper left", fontsize=8)
    axes[3].grid(True, alpha=0.25)

    zeroing = sim["zeroing"].fillna(False)
    if zeroing.any():
        end = sim.loc[zeroing, "rel_time_s"].max()
        for ax in axes:
            ax.axvspan(t.min(), end, color="#cccccc", alpha=0.25)

    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input-csv",
        default="plots/odomDataset/laica_active_odom_dataset.csv",
        help="Rosbag-derived aligned dataset CSV.",
    )
    parser.add_argument(
        "--stable-samples-csv",
        default="plots/odomDataset/stable_force_30N_best_samples_mh_jh_andy.csv",
    )
    parser.add_argument(
        "--output-dir",
        default="plots/odomDataset/admittance_simulation",
    )
    parser.add_argument("--bags", nargs="*", default=None)
    parser.add_argument("--use-stable-samples", action="store_true")
    parser.add_argument("--publish-rate", type=float, default=50.0)
    parser.add_argument("--zero-force-duration-sec", type=float, default=3.0)
    parser.add_argument("--force-filter-tau-sec", type=float, default=0.25)
    parser.add_argument("--force-deadband-n", type=float, default=10.0)
    parser.add_argument("--force-velocity-sign", type=float, default=1.0)
    parser.add_argument("--admittance-mass", type=float, default=40.0)
    parser.add_argument("--admittance-damping", type=float, default=160.0)
    parser.add_argument("--base-velocity-mps", type=float, default=0.0)
    parser.add_argument("--min-velocity-mps", type=float, default=0.0)
    parser.add_argument("--max-velocity-mps", type=float, default=0.40)
    parser.add_argument("--max-accel-mps2", type=float, default=0.50)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    columns = [
        "bag_relative",
        "person_group",
        "time",
        "force.force_n",
        "force.norm_mad",
        "encoder.angle_unwrapped_deg",
        "odom.speed_xy",
        "odom.twist.twist.linear.x",
    ]
    df = pd.read_csv(args.input_csv, usecols=columns)
    sample_dt = 1.0 / args.publish_rate

    selections: list[dict[str, object]] = []
    if args.use_stable_samples:
        stable = pd.read_csv(args.stable_samples_csv)
        for row in stable.itertuples(index=False):
            selections.append(
                {
                    "bag_relative": row.bag_relative,
                    "person_group": row.person_group,
                    "start_s": float(row.start_s),
                    "end_s": float(row.end_s),
                    "label": row.window_label,
                }
            )
    else:
        bags = args.bags or sorted(df["bag_relative"].unique())
        for bag in bags:
            person = df.loc[df["bag_relative"].eq(bag), "person_group"].iloc[0]
            selections.append(
                {
                    "bag_relative": bag,
                    "person_group": person,
                    "start_s": None,
                    "end_s": None,
                    "label": "full_active_bag",
                }
            )

    summary_rows = []
    report_lines = [
        "# Force Admittance Offline Simulation",
        "",
        "Input force comes from the rosbag-derived aligned dataset.",
        "",
        "Controller parameters:",
        f"- zero force duration: `{args.zero_force_duration_sec}` s",
        f"- force filter tau: `{args.force_filter_tau_sec}` s",
        f"- force deadband: `{args.force_deadband_n}` N",
        f"- M: `{args.admittance_mass}`",
        f"- B: `{args.admittance_damping}`",
        f"- velocity clamp: `{args.min_velocity_mps}` to `{args.max_velocity_mps}` m/s",
        f"- acceleration limit: `{args.max_accel_mps2}` m/s^2",
        "",
        "|bag|group|label|cmd mean|cmd std|cmd range|cmd max|force eff p95|plot|",
        "|---|---|---|---:|---:|---:|---:|---:|---|",
    ]

    for selection in selections:
        bag = str(selection["bag_relative"])
        bag_df = df[df["bag_relative"].eq(bag)].copy()
        if bag_df.empty:
            continue
        resampled = resample_bag(bag_df, sample_dt)
        start_s = selection["start_s"]
        end_s = selection["end_s"]
        if start_s is not None and end_s is not None:
            segment = resampled[
                (resampled["rel_time_s"] >= float(start_s))
                & (resampled["rel_time_s"] <= float(end_s))
            ].copy()
            segment["rel_time_s"] = segment["rel_time_s"] - segment["rel_time_s"].iloc[0]
        else:
            segment = resampled.copy()
        if len(segment) < 5:
            continue

        sim = simulate_admittance(
            segment,
            publish_rate_hz=args.publish_rate,
            zero_force_duration_sec=args.zero_force_duration_sec,
            force_filter_tau_sec=args.force_filter_tau_sec,
            force_deadband_n=args.force_deadband_n,
            force_velocity_sign=args.force_velocity_sign,
            admittance_mass=args.admittance_mass,
            admittance_damping=args.admittance_damping,
            base_velocity_mps=args.base_velocity_mps,
            min_velocity_mps=args.min_velocity_mps,
            max_velocity_mps=args.max_velocity_mps,
            max_accel_mps2=args.max_accel_mps2,
        )

        safe_bag = bag.replace("/", "_")
        label = str(selection["label"])
        stem = f"{safe_bag}_{label}"
        sim_csv = output_dir / f"{stem}_admittance_timeseries.csv"
        plot_png = output_dir / f"{stem}_admittance_plot.png"
        sim.to_csv(sim_csv, index=False)
        plot_simulation(
            sim,
            plot_png,
            f"{bag} | {selection['person_group']} | {label}",
        )

        summary = summarize(
            sim,
            {
                "bag_relative": bag,
                "person_group": selection["person_group"],
                "label": label,
                "start_s": start_s,
                "end_s": end_s,
                "timeseries_csv": str(sim_csv),
                "plot_png": str(plot_png),
            },
        )
        summary_rows.append(summary)
        report_lines.append(
            f"|{bag}|{selection['person_group']}|{label}|"
            f"{summary['cmd_vx_mean_mps']:.3f}|{summary['cmd_vx_std_mps']:.3f}|"
            f"{summary['cmd_vx_range_mps']:.3f}|{summary['cmd_vx_max_mps']:.3f}|"
            f"{summary['force_effective_abs_p95_n']:.2f}|"
            f"[plot]({plot_png.name})|"
        )

    summary_df = pd.DataFrame(summary_rows)
    summary_csv = output_dir / "admittance_simulation_summary.csv"
    summary_df.to_csv(summary_csv, index=False)
    report_md = output_dir / "admittance_simulation_report.md"
    report_md.write_text("\n".join(report_lines) + "\n")

    print(summary_df.to_string(index=False))
    print(f"\nwrote {summary_csv}")
    print(f"wrote {report_md}")
    print(f"plots dir: {output_dir}")


if __name__ == "__main__":
    main()
