#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

#include <enc_lc/msg/encoder_data.hpp>
#include <enc_lc/msg/load_cell_data.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

namespace {

class CdrReader {
public:
  explicit CdrReader(const rclcpp::SerializedMessage& msg)
  {
    const auto& serialized = msg.get_rcl_serialized_message();
    data_ = serialized.buffer;
    size_ = serialized.buffer_length;
    reset();
  }

  void reset()
  {
    offset_ = 4;
    little_endian_ = true;
    if (size_ >= 2)
    {
      little_endian_ = data_[1] == 1;
    }
  }

  bool readInt32(int32_t& value)
  {
    uint32_t raw = 0;
    if (!readUInt32(raw))
    {
      return false;
    }
    value = static_cast<int32_t>(raw);
    return true;
  }

  bool readUInt32(uint32_t& value)
  {
    align(4);
    if (offset_ + 4 > size_)
    {
      return false;
    }

    if (little_endian_)
    {
      value = static_cast<uint32_t>(data_[offset_]) |
              (static_cast<uint32_t>(data_[offset_ + 1]) << 8) |
              (static_cast<uint32_t>(data_[offset_ + 2]) << 16) |
              (static_cast<uint32_t>(data_[offset_ + 3]) << 24);
    }
    else
    {
      value = (static_cast<uint32_t>(data_[offset_]) << 24) |
              (static_cast<uint32_t>(data_[offset_ + 1]) << 16) |
              (static_cast<uint32_t>(data_[offset_ + 2]) << 8) |
              static_cast<uint32_t>(data_[offset_ + 3]);
    }

    offset_ += 4;
    return true;
  }

  bool readDouble(double& value)
  {
    align(8);
    if (offset_ + 8 > size_)
    {
      return false;
    }

    uint64_t raw = 0;
    if (little_endian_)
    {
      for (std::size_t i = 0; i < 8; ++i)
      {
        raw |= static_cast<uint64_t>(data_[offset_ + i]) << (8 * i);
      }
    }
    else
    {
      for (std::size_t i = 0; i < 8; ++i)
      {
        raw = (raw << 8) | static_cast<uint64_t>(data_[offset_ + i]);
      }
    }

    std::memcpy(&value, &raw, sizeof(value));
    offset_ += 8;
    return true;
  }

  bool skipString()
  {
    uint32_t length = 0;
    if (!readUInt32(length))
    {
      return false;
    }

    if (length > remaining())
    {
      return false;
    }

    offset_ += length;
    return true;
  }

private:
  void align(std::size_t alignment)
  {
    const std::size_t remainder = offset_ % alignment;
    if (remainder != 0)
    {
      offset_ += alignment - remainder;
    }
  }

  std::size_t remaining() const
  {
    return offset_ < size_ ? size_ - offset_ : 0;
  }

  const uint8_t* data_ = nullptr;
  std::size_t size_ = 0;
  std::size_t offset_ = 0;
  bool little_endian_ = true;
};

}  // namespace

class LaicaVelocityPredictor : public rclcpp::Node {
public:
  LaicaVelocityPredictor()
    : Node("laica_velocity_predictor")
  {
    readParameters();

    rclcpp::QoS sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    if (sensor_qos_reliability_ == "reliable")
    {
      sensor_qos.reliable();
    }
    else
    {
      sensor_qos.best_effort();
    }

    if (load_cell_subscription_mode_ == "serialized_auto")
    {
      load_cell_generic_sub_ = create_generic_subscription(
          load_cell_topic_name_,
          "enc_lc/msg/LoadCellData",
          sensor_qos,
          std::bind(&LaicaVelocityPredictor::serializedLoadCellCallback,
                    this, std::placeholders::_1));
    }
    else
    {
      load_cell_sub_ = create_subscription<enc_lc::msg::LoadCellData>(
          load_cell_topic_name_,
          sensor_qos,
          std::bind(&LaicaVelocityPredictor::loadCellCallback,
                    this, std::placeholders::_1));
    }

    if (require_encoder_)
    {
      encoder_sub_ = create_subscription<enc_lc::msg::EncoderData>(
          encoder_topic_name_,
          sensor_qos,
          std::bind(&LaicaVelocityPredictor::encoderCallback, this, std::placeholders::_1));
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        predicted_cmd_vel_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(publish_rate_, 1.0));
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LaicaVelocityPredictor::publishPredictedVelocity, this));

    RCLCPP_INFO(get_logger(), "Subscribing to %s with %s QoS (%s mode)",
                load_cell_topic_name_.c_str(), sensor_qos_reliability_.c_str(),
                load_cell_subscription_mode_.c_str());
    if (require_encoder_)
    {
      RCLCPP_INFO(get_logger(), "Encoder required; subscribing to %s",
                  encoder_topic_name_.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Encoder not required for force-only admittance; not subscribing to %s",
                  encoder_topic_name_.c_str());
    }
    RCLCPP_INFO(get_logger(), "Using LoadCellData.%s as load-cell input",
                load_cell_input_field_.c_str());
    RCLCPP_INFO(get_logger(),
                "Publishing 1D admittance Twist to %s "
                "(enabled=%s, base_vx=%.3f m/s, deadband=%.3f N)",
                predicted_cmd_vel_topic_name_.c_str(),
                admittance_enabled_ ? "true" : "false",
                base_velocity_mps_, force_deadband_n_);
  }

private:
  void readParameters()
  {
    declare_parameter<std::string>("load_cell_topic_name", load_cell_topic_name_);
    declare_parameter<std::string>("encoder_topic_name", encoder_topic_name_);
    declare_parameter<std::string>("predicted_cmd_vel_topic_name",
                                   predicted_cmd_vel_topic_name_);
    declare_parameter<double>("publish_rate", publish_rate_);
    declare_parameter<std::string>("sensor_qos_reliability", sensor_qos_reliability_);
    declare_parameter<std::string>("load_cell_subscription_mode",
                                   load_cell_subscription_mode_);
    declare_parameter<std::string>("load_cell_input_field", load_cell_input_field_);
    declare_parameter<bool>("admittance_enabled", admittance_enabled_);
    declare_parameter<bool>("require_encoder", require_encoder_);
    declare_parameter<bool>("auto_zero_force", auto_zero_force_);
    declare_parameter<double>("zero_force_duration_sec", zero_force_duration_sec_);
    declare_parameter<double>("force_filter_tau_sec", force_filter_tau_sec_);
    declare_parameter<double>("force_deadband_n", force_deadband_n_);
    declare_parameter<double>("force_velocity_sign", force_velocity_sign_);
    declare_parameter<double>("admittance_mass", admittance_mass_);
    declare_parameter<double>("admittance_damping", admittance_damping_);
    declare_parameter<double>("base_velocity_mps", base_velocity_mps_);
    declare_parameter<double>("min_velocity_mps", min_velocity_mps_);
    declare_parameter<double>("max_velocity_mps", max_velocity_mps_);
    declare_parameter<double>("max_accel_mps2", max_accel_mps2_);
    declare_parameter<double>("sensor_timeout_sec", sensor_timeout_sec_);

    get_parameter("load_cell_topic_name", load_cell_topic_name_);
    get_parameter("encoder_topic_name", encoder_topic_name_);
    get_parameter("predicted_cmd_vel_topic_name", predicted_cmd_vel_topic_name_);
    get_parameter("publish_rate", publish_rate_);
    get_parameter("sensor_qos_reliability", sensor_qos_reliability_);
    get_parameter("load_cell_subscription_mode", load_cell_subscription_mode_);
    get_parameter("load_cell_input_field", load_cell_input_field_);
    get_parameter("admittance_enabled", admittance_enabled_);
    get_parameter("require_encoder", require_encoder_);
    get_parameter("auto_zero_force", auto_zero_force_);
    get_parameter("zero_force_duration_sec", zero_force_duration_sec_);
    get_parameter("force_filter_tau_sec", force_filter_tau_sec_);
    get_parameter("force_deadband_n", force_deadband_n_);
    get_parameter("force_velocity_sign", force_velocity_sign_);
    get_parameter("admittance_mass", admittance_mass_);
    get_parameter("admittance_damping", admittance_damping_);
    get_parameter("base_velocity_mps", base_velocity_mps_);
    get_parameter("min_velocity_mps", min_velocity_mps_);
    get_parameter("max_velocity_mps", max_velocity_mps_);
    get_parameter("max_accel_mps2", max_accel_mps2_);
    get_parameter("sensor_timeout_sec", sensor_timeout_sec_);

    if (load_cell_input_field_ != "raw_count" &&
        load_cell_input_field_ != "voltage_mv" &&
        load_cell_input_field_ != "force_n")
    {
      RCLCPP_WARN(get_logger(),
                  "Unsupported load_cell_input_field '%s'. Falling back to raw_count.",
                  load_cell_input_field_.c_str());
      load_cell_input_field_ = "raw_count";
    }

    if (sensor_qos_reliability_ != "reliable" &&
        sensor_qos_reliability_ != "best_effort")
    {
      RCLCPP_WARN(get_logger(),
                  "Unsupported sensor_qos_reliability '%s'. Falling back to best_effort.",
                  sensor_qos_reliability_.c_str());
      sensor_qos_reliability_ = "best_effort";
    }

    if (load_cell_subscription_mode_ != "typed" &&
        load_cell_subscription_mode_ != "serialized_auto")
    {
      RCLCPP_WARN(get_logger(),
                  "Unsupported load_cell_subscription_mode '%s'. Falling back to serialized_auto.",
                  load_cell_subscription_mode_.c_str());
      load_cell_subscription_mode_ = "serialized_auto";
    }

    publish_rate_ = std::max(publish_rate_, 1.0);
    force_filter_tau_sec_ = std::max(force_filter_tau_sec_, 0.0);
    force_deadband_n_ = std::max(force_deadband_n_, 0.0);
    zero_force_duration_sec_ = std::max(zero_force_duration_sec_, 0.0);
    admittance_mass_ = std::max(admittance_mass_, 1.0e-6);
    admittance_damping_ = std::max(admittance_damping_, 0.0);
    max_accel_mps2_ = std::max(max_accel_mps2_, 0.0);
    sensor_timeout_sec_ = std::max(sensor_timeout_sec_, 0.0);

    if (min_velocity_mps_ > max_velocity_mps_)
    {
      RCLCPP_WARN(get_logger(),
                  "min_velocity_mps is greater than max_velocity_mps. Swapping.");
      std::swap(min_velocity_mps_, max_velocity_mps_);
    }
  }

  void loadCellCallback(const enc_lc::msg::LoadCellData::SharedPtr msg)
  {
    handleLoadCellValues(static_cast<double>(msg->raw_count),
                         msg->voltage_mv,
                         msg->force_n);
  }

  void serializedLoadCellCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    double raw_count = 0.0;
    double voltage_mv = 0.0;
    double force_n = 0.0;
    std::string layout;

    if (!decodeCurrentLoadCell(*msg, raw_count, voltage_mv, force_n))
    {
      if (!decodeLegacyLoadCell(*msg, raw_count, voltage_mv, force_n))
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Could not decode serialized LoadCellData. The bag message definition may be incompatible.");
        return;
      }
      layout = "legacy_no_header";
    }
    else
    {
      layout = "current_header";
    }

    if (layout != serialized_load_cell_layout_)
    {
      serialized_load_cell_layout_ = layout;
      RCLCPP_INFO(get_logger(), "Decoded serialized LoadCellData as %s",
                  serialized_load_cell_layout_.c_str());
    }

    handleLoadCellValues(raw_count, voltage_mv, force_n);
  }

  bool decodeCurrentLoadCell(const rclcpp::SerializedMessage& msg,
                             double& raw_count,
                             double& voltage_mv,
                             double& force_n) const
  {
    CdrReader reader(msg);

    int32_t stamp_sec = 0;
    uint32_t stamp_nanosec = 0;
    int32_t raw_count_int = 0;
    if (!reader.readInt32(stamp_sec) ||
        !reader.readUInt32(stamp_nanosec) ||
        !reader.skipString() ||
        !reader.readInt32(raw_count_int) ||
        !reader.readDouble(voltage_mv) ||
        !reader.readDouble(force_n))
    {
      return false;
    }

    (void)stamp_sec;
    (void)stamp_nanosec;
    raw_count = static_cast<double>(raw_count_int);
    return std::isfinite(voltage_mv) && std::isfinite(force_n);
  }

  bool decodeLegacyLoadCell(const rclcpp::SerializedMessage& msg,
                            double& raw_count,
                            double& voltage_mv,
                            double& force_n) const
  {
    CdrReader reader(msg);

    int32_t raw_count_int = 0;
    if (!reader.readInt32(raw_count_int) ||
        !reader.readDouble(voltage_mv) ||
        !reader.readDouble(force_n))
    {
      return false;
    }

    raw_count = static_cast<double>(raw_count_int);
    return std::isfinite(voltage_mv) && std::isfinite(force_n);
  }

  void handleLoadCellValues(double raw_count, double voltage_mv, double force_n)
  {
    if (load_cell_input_field_ == "force_n")
    {
      latest_load_cell_input_ = force_n;
    }
    else if (load_cell_input_field_ == "voltage_mv")
    {
      latest_load_cell_input_ = voltage_mv;
    }
    else
    {
      latest_load_cell_input_ = raw_count;
    }

    has_load_cell_ = true;
    latest_load_cell_time_ = now();

    updateForceZero(latest_load_cell_input_);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Loadcell %s input: %.6f",
                         load_cell_input_field_.c_str(),
                         latest_load_cell_input_);
  }

  void encoderCallback(const enc_lc::msg::EncoderData::SharedPtr msg)
  {
    (void)msg;
    has_encoder_ = true;
  }

  void publishPredictedVelocity()
  {
    const auto now_time = now();
    const bool load_cell_stale =
        !has_load_cell_ ||
        (sensor_timeout_sec_ > 0.0 &&
         (now_time - latest_load_cell_time_).seconds() > sensor_timeout_sec_);
    const bool waiting_for_encoder = require_encoder_ && !has_encoder_;

    if (load_cell_stale || waiting_for_encoder)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for required sensor data before prediction");
      publishStopAndReset();
      return;
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = computeAdmittanceVelocity(now_time);
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    cmd_vel_pub_->publish(cmd_vel);
  }

  void updateForceZero(double force_input)
  {
    if (!auto_zero_force_ || zero_complete_)
    {
      return;
    }

    if (!zero_started_)
    {
      zero_started_ = true;
      zero_start_time_ = now();
      zero_sum_ = 0.0;
      zero_samples_ = 0;
      RCLCPP_INFO(get_logger(),
                  "Starting %.2f s predictor force zeroing. Keep leash/loadcell relaxed.",
                  zero_force_duration_sec_);
    }

    zero_sum_ += force_input;
    ++zero_samples_;

    if ((now() - zero_start_time_).seconds() >= zero_force_duration_sec_)
    {
      force_zero_offset_ =
          zero_samples_ > 0 ? zero_sum_ / static_cast<double>(zero_samples_) : 0.0;
      zero_complete_ = true;
      RCLCPP_INFO(get_logger(),
                  "Predictor force zeroing complete: offset=%.6f from %zu samples",
                  force_zero_offset_, zero_samples_);
    }
  }

  double computeAdmittanceVelocity(const rclcpp::Time& now_time)
  {
    const double dt = computeDt(now_time);

    if (!admittance_enabled_)
    {
      admittance_velocity_mps_ = 0.0;
      filtered_force_n_ = latest_load_cell_input_ - force_zero_offset_;
      return rateLimitAndClamp(base_velocity_mps_, dt);
    }

    if (auto_zero_force_ && !zero_complete_)
    {
      admittance_velocity_mps_ = 0.0;
      filtered_force_n_ = 0.0;
      return rateLimitAndClamp(0.0, dt);
    }

    const double force_n = latest_load_cell_input_ - force_zero_offset_;
    filtered_force_n_ = lowPassFilter(force_n, filtered_force_n_, dt,
                                      force_filter_tau_sec_, has_filtered_force_);
    has_filtered_force_ = true;

    const double signed_force_n = force_velocity_sign_ * applyDeadband(filtered_force_n_);
    const double accel_mps2 =
        (signed_force_n - admittance_damping_ * admittance_velocity_mps_) /
        admittance_mass_;
    admittance_velocity_mps_ += accel_mps2 * dt;

    const double target_vx = base_velocity_mps_ + admittance_velocity_mps_;
    const double cmd_vx = rateLimitAndClamp(target_vx, dt);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Admittance force raw=%.3f zeroed=%.3f filtered=%.3f eff=%.3f adm_vx=%.3f cmd_vx=%.3f",
        latest_load_cell_input_, force_n, filtered_force_n_, signed_force_n,
        admittance_velocity_mps_, cmd_vx);

    return cmd_vx;
  }

  double computeDt(const rclcpp::Time& now_time)
  {
    if (!has_last_publish_time_)
    {
      last_publish_time_ = now_time;
      has_last_publish_time_ = true;
      return 1.0 / publish_rate_;
    }

    const double dt = (now_time - last_publish_time_).seconds();
    last_publish_time_ = now_time;

    if (!std::isfinite(dt) || dt <= 0.0)
    {
      return 1.0 / publish_rate_;
    }

    return std::min(dt, 0.2);
  }

  static double lowPassFilter(double input, double previous, double dt,
                              double tau, bool has_previous)
  {
    if (!has_previous || tau <= 0.0)
    {
      return input;
    }

    const double alpha = dt / (tau + dt);
    return previous + alpha * (input - previous);
  }

  double applyDeadband(double force_n) const
  {
    const double abs_force = std::abs(force_n);
    if (abs_force <= force_deadband_n_)
    {
      return 0.0;
    }

    return std::copysign(abs_force - force_deadband_n_, force_n);
  }

  double rateLimitAndClamp(double target_vx, double dt)
  {
    const double clamped_target =
        std::max(min_velocity_mps_, std::min(max_velocity_mps_, target_vx));

    if (max_accel_mps2_ > 0.0 && has_previous_cmd_)
    {
      const double max_delta = max_accel_mps2_ * dt;
      const double delta = clamped_target - previous_cmd_vx_;
      previous_cmd_vx_ += std::max(-max_delta, std::min(max_delta, delta));
    }
    else
    {
      previous_cmd_vx_ = clamped_target;
    }

    has_previous_cmd_ = true;
    return previous_cmd_vx_;
  }

  void publishStopAndReset()
  {
    admittance_velocity_mps_ = 0.0;
    has_filtered_force_ = false;
    previous_cmd_vx_ = 0.0;
    has_previous_cmd_ = true;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel_pub_->publish(cmd_vel);
  }

  rclcpp::Subscription<enc_lc::msg::LoadCellData>::SharedPtr load_cell_sub_;
  rclcpp::GenericSubscription::SharedPtr load_cell_generic_sub_;
  rclcpp::Subscription<enc_lc::msg::EncoderData>::SharedPtr encoder_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string load_cell_topic_name_;
  std::string encoder_topic_name_;
  std::string predicted_cmd_vel_topic_name_;
  std::string sensor_qos_reliability_ = "best_effort";
  std::string load_cell_subscription_mode_ = "serialized_auto";
  std::string load_cell_input_field_ = "force_n";
  std::string serialized_load_cell_layout_;
  double publish_rate_ = 50.0;

  double latest_load_cell_input_ = 0.0;
  rclcpp::Time latest_load_cell_time_;
  bool has_load_cell_ = false;
  bool has_encoder_ = false;

  bool admittance_enabled_ = true;
  bool require_encoder_ = false;
  bool auto_zero_force_ = true;
  bool zero_started_ = false;
  bool zero_complete_ = false;
  rclcpp::Time zero_start_time_;
  double zero_force_duration_sec_ = 3.0;
  double zero_sum_ = 0.0;
  std::size_t zero_samples_ = 0;
  double force_zero_offset_ = 0.0;

  double force_filter_tau_sec_ = 0.25;
  double force_deadband_n_ = 10.0;
  double force_velocity_sign_ = 1.0;
  double admittance_mass_ = 40.0;
  double admittance_damping_ = 160.0;
  double base_velocity_mps_ = 0.0;
  double min_velocity_mps_ = 0.0;
  double max_velocity_mps_ = 0.40;
  double max_accel_mps2_ = 0.50;
  double sensor_timeout_sec_ = 0.25;

  double filtered_force_n_ = 0.0;
  bool has_filtered_force_ = false;
  double admittance_velocity_mps_ = 0.0;
  rclcpp::Time last_publish_time_;
  bool has_last_publish_time_ = false;
  double previous_cmd_vx_ = 0.0;
  bool has_previous_cmd_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaicaVelocityPredictor>());
  rclcpp::shutdown();
  return 0;
}
