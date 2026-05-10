#include <rclcpp/rclcpp.hpp>

#include <enc_lc/msg/encoder_data.hpp>
#include <enc_lc/msg/load_cell_data.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

class LaicaVelocityPredictor : public rclcpp::Node {
public:
  LaicaVelocityPredictor()
    : Node("laica_velocity_predictor")
  {
    readParameters();

    const rclcpp::QoS sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    load_cell_sub_ = create_subscription<enc_lc::msg::LoadCellData>(
        load_cell_topic_name_,
        sensor_qos,
        std::bind(&LaicaVelocityPredictor::loadCellCallback, this, std::placeholders::_1));
    encoder_sub_ = create_subscription<enc_lc::msg::EncoderData>(
        encoder_topic_name_,
        sensor_qos,
        std::bind(&LaicaVelocityPredictor::encoderCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        predicted_cmd_vel_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(publish_rate_, 1.0));
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LaicaVelocityPredictor::publishPredictedVelocity, this));

    RCLCPP_INFO(get_logger(), "Subscribing to %s and %s",
                load_cell_topic_name_.c_str(), encoder_topic_name_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing zero Twist to %s",
                predicted_cmd_vel_topic_name_.c_str());
  }

private:
  void readParameters()
  {
    declare_parameter<std::string>("load_cell_topic_name", load_cell_topic_name_);
    declare_parameter<std::string>("encoder_topic_name", encoder_topic_name_);
    declare_parameter<std::string>("predicted_cmd_vel_topic_name",
                                   predicted_cmd_vel_topic_name_);
    declare_parameter<double>("publish_rate", publish_rate_);

    get_parameter("load_cell_topic_name", load_cell_topic_name_);
    get_parameter("encoder_topic_name", encoder_topic_name_);
    get_parameter("predicted_cmd_vel_topic_name", predicted_cmd_vel_topic_name_);
    get_parameter("publish_rate", publish_rate_);
  }

  void loadCellCallback(const enc_lc::msg::LoadCellData::SharedPtr msg)
  {
    latest_load_cell_raw_count_ = msg->raw_count;
    has_load_cell_ = true;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Loadcell raw_count input: %d",
                         latest_load_cell_raw_count_);
  }

  void encoderCallback(const enc_lc::msg::EncoderData::SharedPtr msg)
  {
    (void)msg;
    has_encoder_ = true;
  }

  void publishPredictedVelocity()
  {
    if (!has_load_cell_ || !has_encoder_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for Arduino sensor data before prediction");
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    cmd_vel_pub_->publish(cmd_vel);
  }

  rclcpp::Subscription<enc_lc::msg::LoadCellData>::SharedPtr load_cell_sub_;
  rclcpp::Subscription<enc_lc::msg::EncoderData>::SharedPtr encoder_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string load_cell_topic_name_;
  std::string encoder_topic_name_;
  std::string predicted_cmd_vel_topic_name_;
  double publish_rate_ = 50.0;

  int32_t latest_load_cell_raw_count_ = 0;
  bool has_load_cell_ = false;
  bool has_encoder_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaicaVelocityPredictor>());
  rclcpp::shutdown();
  return 0;
}
