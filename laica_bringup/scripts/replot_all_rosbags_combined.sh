#!/usr/bin/env bash
set -euo pipefail

ROSBAGS_ROOT="${ROSBAGS_ROOT:-/home/artemis/Documents/rosbags}"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/artemis/Documents/LAICA_ws}"
PLOT_SCRIPT="${PLOT_SCRIPT:-$WORKSPACE_ROOT/src/laica_bringup/scripts/plot_rosbag_sensors.py}"
OUTPUT_DIR="${OUTPUT_DIR:-$WORKSPACE_ROOT/plots}"
PNG_DIR="${PNG_DIR:-$OUTPUT_DIR/png}"
CSV_DIR="${CSV_DIR:-$OUTPUT_DIR/csv}"

if [[ ! -f "$PLOT_SCRIPT" ]]; then
  echo "Plot script not found: $PLOT_SCRIPT" >&2
  exit 1
fi

mkdir -p "$PNG_DIR"

mapfile -t BAG_PATHS < <(find "$ROSBAGS_ROOT" -type f -name metadata.yaml -printf '%h\n' | sort)

if [[ ${#BAG_PATHS[@]} -eq 0 ]]; then
  echo "No ROS bag metadata.yaml files found under: $ROSBAGS_ROOT" >&2
  exit 1
fi

cleanup_config=""
cleanup() {
  if [[ -n "$cleanup_config" && -f "$cleanup_config" ]]; then
    rm -f "$cleanup_config"
  fi
}
trap cleanup EXIT

echo "Found ${#BAG_PATHS[@]} ROS bag(s)."
echo "Writing combined plots to: $PNG_DIR"

for bag_path in "${BAG_PATHS[@]}"; do
  relative_path="${bag_path#"$ROSBAGS_ROOT"/}"
  bag_name="$(basename "$bag_path")"
  parent_path="$(dirname "$relative_path")"
  if [[ "$parent_path" == "." ]]; then
    run_png_dir="$PNG_DIR"
  else
    run_png_dir="$PNG_DIR/$parent_path"
  fi

  prefix="$bag_name"
  prefix="${prefix// /_}"
  output_png_xy="$run_png_dir/${prefix}_time_encoder_force_odom_xy_cmd_vel.png"
  output_png_z="$run_png_dir/${prefix}_time_encoder_force_odom_z_cmd_vel.png"

  mkdir -p "$run_png_dir"

  cleanup_config="$(mktemp /tmp/laica_plot_bag.XXXXXX.yaml)"
  cat > "$cleanup_config" <<EOF
plot_rosbag_sensors:
  ros__parameters:
    bag_paths:
      - "$bag_path"

    output_dir: "$OUTPUT_DIR"
    png_dir: "$run_png_dir"
    csv_dir: "$CSV_DIR"
    output_prefix: "$prefix"

    encoder_topic: "/encoder/data"
    force_topic: "/load_cell/data"
    imu_topic: "/imu"
    odom_topic: "/odom"
    cmd_vel_topic: "/laica/predicted_cmd_vel"
    switch_topic: "/switch/data"

    encoder_fields:
      - "angle_deg"
      - "rev"
      - "rpm"
    force_fields:
      - "force_n"
    imu_fields:
      - "linear_acceleration.x"
      - "linear_acceleration.z"
    odom_fields:
      - "twist.twist.linear.x"
      - "twist.twist.linear.y"
      - "twist.twist.angular.z"
    cmd_vel_fields:
      - "linear.x"
      - "linear.y"
      - "angular.z"
    switch_fields:
      - "switch_1"
      - "switch_2"

    use_header_time: true
    relative_time: true
    save_csv: false
    save_png: true
    save_individual_pngs: false
    save_combined_png: true
    split_combined_odom_pngs: true
    combined_motion_stream: "odom"
    show_plots: false
    highlight_switch_intervals: true
    switch_highlight_fields:
      - "switch_2"
    switch_highlight_color: "tab:orange"
    switch_highlight_alpha: 0.18
    switch_press_debounce_sec: 0.25
    switch_min_interval_sec: 0.5

    wrap_encoder_angle_deg: true
    filter_plot_outliers: true
    outlier_mad_threshold: 8.0
    outlier_min_samples: 12
EOF

  echo "Plotting $relative_path -> ${output_png_xy#"$PNG_DIR"/}, ${output_png_z#"$PNG_DIR"/}"
  python3 "$PLOT_SCRIPT" --config "$cleanup_config"
  rm -f "$cleanup_config"
  cleanup_config=""

  if [[ ! -f "$output_png_xy" ]]; then
    echo "Expected plot was not created: $output_png_xy" >&2
    exit 1
  fi
  if [[ ! -f "$output_png_z" ]]; then
    echo "Expected plot was not created: $output_png_z" >&2
    exit 1
  fi
done

find "$PNG_DIR" -type f \
  ! -name '*_time_encoder_force_odom_xy_cmd_vel.png' \
  ! -name '*_time_encoder_force_odom_z_cmd_vel.png' \
  -delete
if [[ -d "$CSV_DIR" ]]; then
  case "$CSV_DIR" in
    "$OUTPUT_DIR"/*)
      rm -rf "$CSV_DIR"
      ;;
    *)
      echo "Skipping CSV cleanup outside OUTPUT_DIR: $CSV_DIR" >&2
      ;;
  esac
fi

echo "Done. Kept only combined odom XY/Z PNG files in: $PNG_DIR"
