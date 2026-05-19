#include <rclcpp/rclcpp.hpp>

#include <enc_lc/msg/encoder_data.hpp>
#include <enc_lc/msg/load_cell_data.hpp>
#include <enc_lc/msg/switch_data.hpp>
#include <std_msgs/msg/bool.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct SensorFrame {
  int32_t lc_raw = 0;
  double lc_mv = 0.0;
  double enc_deg = 0.0;
  double enc_rev = 0.0;
  int32_t enc_rpm = 0;
  bool sw1 = false;
  bool sw2 = false;
  uint64_t sequence = 0;
};

speed_t baudToTermios(const int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud));
  }
}

std::vector<std::string> splitComma(const std::string& text) {
  std::vector<std::string> parts;
  std::string current;

  for (const char c : text) {
    if (c == ',') {
      parts.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }

  parts.push_back(current);
  return parts;
}

bool parseSensorLine(const std::string& line, SensorFrame& frame) {
  static const std::string header = "$SENSORS,";

  if (line.rfind(header, 0) != 0) {
    return false;
  }

  const std::vector<std::string> parts = splitComma(line.substr(header.size()));
  if (parts.size() != 5 && parts.size() != 7) {
    return false;
  }

  try {
    frame.lc_raw = std::stoi(parts[0]);
    frame.lc_mv = std::stod(parts[1]);
    frame.enc_deg = std::stod(parts[2]);
    frame.enc_rev = std::stod(parts[3]);
    frame.enc_rpm = std::stoi(parts[4]);
    if (parts.size() == 7) {
      frame.sw1 = std::stoi(parts[5]) != 0;
      frame.sw2 = std::stoi(parts[6]) != 0;
    }
  } catch (const std::exception&) {
    return false;
  }

  return true;
}

class ArduinoBridge {
public:
  ArduinoBridge(const std::string& port, const int baud) {
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open " + port + ": " +
                               std::strerror(errno));
    }

    configureSerial(baud);
  }

  ~ArduinoBridge() { stop(); }

  void start() {
    running_ = true;
    read_thread_ = std::thread(&ArduinoBridge::readLoop, this);
  }

  void stop() {
    running_ = false;

    if (read_thread_.joinable()) {
      read_thread_.join();
    }

    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  SensorFrame getFrame() const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return frame_;
  }

  uint64_t parseErrors() const { return parse_errors_; }

private:
  void configureSerial(const int baud) {
    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
      throw std::runtime_error("tcgetattr failed: " +
                               std::string(std::strerror(errno)));
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baudToTermios(baud));
    cfsetospeed(&tty, baudToTermios(baud));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 20;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("tcsetattr failed: " +
                               std::string(std::strerror(errno)));
    }
  }

  void readLoop() {
    std::string line;
    char buffer[256];

    while (running_) {
      const ssize_t count = read(fd_, buffer, sizeof(buffer));

      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }

      if (count == 0) {
        continue;
      }

      for (ssize_t i = 0; i < count; ++i) {
        const char c = buffer[i];
        if (c == '\n') {
          handleLine(line);
          line.clear();
        } else if (c != '\r') {
          line.push_back(c);
        }
      }
    }
  }

  void handleLine(const std::string& line) {
    if (line.empty()) {
      return;
    }

    SensorFrame parsed;
    if (parseSensorLine(line, parsed)) {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      parsed.sequence = frame_.sequence + 1;
      frame_ = parsed;
    } else if (line.front() != '#') {
      ++parse_errors_;
    }
  }

  int fd_ = -1;
  std::atomic<bool> running_ {false};
  std::thread read_thread_;
  mutable std::mutex frame_mutex_;
  SensorFrame frame_;
  std::atomic<uint64_t> parse_errors_ {0};
};

}  // namespace

class ArduinoSensorPub : public rclcpp::Node {
public:
  ArduinoSensorPub()
      : Node("arduino_sensor_pub") {
    readParameters();

    load_cell_pub_ = create_publisher<enc_lc::msg::LoadCellData>(
        load_cell_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    encoder_pub_ = create_publisher<enc_lc::msg::EncoderData>(
        encoder_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    switch_pub_ = create_publisher<enc_lc::msg::SwitchData>(
        switch_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    load_cell_calibration_done_pub_ = create_publisher<std_msgs::msg::Bool>(
        load_cell_calibration_done_topic_name_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    RCLCPP_INFO(get_logger(), "Connecting to Arduino on %s at %d baud",
                port_.c_str(), baud_);

    bridge_ = std::make_unique<ArduinoBridge>(port_, baud_);
    bridge_->start();

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(publish_rate_, 1.0));
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ArduinoSensorPub::publishSensorData, this));

    RCLCPP_INFO(get_logger(), "arduino_sensor_pub running");
  }

  ~ArduinoSensorPub() override {
    if (bridge_) {
      bridge_->stop();
    }
  }

private:
  void readParameters() {
    declare_parameter<std::string>("port", port_);
    declare_parameter<int>("baud", baud_);
    declare_parameter<double>("publish_rate", publish_rate_);
    declare_parameter<std::string>("frame_id_lc", frame_id_lc_);
    declare_parameter<std::string>("frame_id_enc", frame_id_enc_);
    declare_parameter<std::string>("frame_id_switch", frame_id_switch_);
    declare_parameter<std::string>("load_cell_topic_name", load_cell_topic_name_);
    declare_parameter<std::string>("encoder_topic_name", encoder_topic_name_);
    declare_parameter<std::string>("switch_topic_name", switch_topic_name_);
    declare_parameter<std::string>(
        "load_cell_calibration_done_topic_name",
        load_cell_calibration_done_topic_name_);
    declare_parameter<double>("force_gradient_n_per_mv", force_gradient_n_per_mv_);
    declare_parameter<double>("force_bias_n", force_bias_n_);
    declare_parameter<double>("load_cell_startup_calibration_sec",
                              load_cell_startup_calibration_sec_);

    get_parameter("port", port_);
    get_parameter("baud", baud_);
    get_parameter("publish_rate", publish_rate_);
    get_parameter("frame_id_lc", frame_id_lc_);
    get_parameter("frame_id_enc", frame_id_enc_);
    get_parameter("frame_id_switch", frame_id_switch_);
    get_parameter("load_cell_topic_name", load_cell_topic_name_);
    get_parameter("encoder_topic_name", encoder_topic_name_);
    get_parameter("switch_topic_name", switch_topic_name_);
    get_parameter("load_cell_calibration_done_topic_name",
                  load_cell_calibration_done_topic_name_);
    get_parameter("force_gradient_n_per_mv", force_gradient_n_per_mv_);
    get_parameter("force_bias_n", force_bias_n_);
    get_parameter("load_cell_startup_calibration_sec",
                  load_cell_startup_calibration_sec_);
  }

  void publishSensorData() {
    const SensorFrame frame = bridge_->getFrame();
    if (frame.sequence == 0) {
      return;
    }

    updateLoadCellStartupCalibration(frame);
    const rclcpp::Time now = get_clock()->now();

    enc_lc::msg::LoadCellData load_cell_msg;
    load_cell_msg.header.stamp = now;
    load_cell_msg.header.frame_id = frame_id_lc_;
    load_cell_msg.raw_count = calibratedRawCount(frame.lc_raw);
    load_cell_msg.voltage_mv = frame.lc_mv - load_cell_voltage_offset_mv_;
    load_cell_msg.force_n =
        (frame.lc_mv * force_gradient_n_per_mv_ + force_bias_n_) -
        load_cell_force_offset_n_;
    load_cell_pub_->publish(load_cell_msg);

    enc_lc::msg::EncoderData encoder_msg;
    encoder_msg.header.stamp = now;
    encoder_msg.header.frame_id = frame_id_enc_;
    encoder_msg.angle_deg = frame.enc_deg;
    encoder_msg.rev = frame.enc_rev;
    encoder_msg.rpm = frame.enc_rpm;
    encoder_pub_->publish(encoder_msg);

    enc_lc::msg::SwitchData switch_msg;
    switch_msg.header.stamp = now;
    switch_msg.header.frame_id = frame_id_switch_;
    switch_msg.switch_1 = frame.sw1;
    switch_msg.switch_2 = frame.sw2;
    switch_pub_->publish(switch_msg);

    std_msgs::msg::Bool calibration_done_msg;
    calibration_done_msg.data = isLoadCellStartupCalibrationDone();
    load_cell_calibration_done_pub_->publish(calibration_done_msg);

    if (bridge_->parseErrors() > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Arduino parse errors: %llu",
                           static_cast<unsigned long long>(bridge_->parseErrors()));
    }
  }

  void updateLoadCellStartupCalibration(const SensorFrame& frame) {
    if (load_cell_startup_calibration_sec_ <= 0.0 ||
        load_cell_startup_calibration_done_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!load_cell_startup_calibration_started_) {
      load_cell_startup_calibration_started_ = true;
      load_cell_startup_calibration_start_ = now;
      RCLCPP_INFO(get_logger(),
                  "Starting %.2f s loadcell startup calibration. Keep the "
                  "loadcell unloaded.",
                  load_cell_startup_calibration_sec_);
    }

    load_cell_raw_sum_ += static_cast<double>(frame.lc_raw);
    load_cell_voltage_sum_mv_ += frame.lc_mv;
    ++load_cell_calibration_sample_count_;

    load_cell_raw_offset_ =
        load_cell_raw_sum_ /
        static_cast<double>(load_cell_calibration_sample_count_);
    load_cell_voltage_offset_mv_ =
        load_cell_voltage_sum_mv_ /
        static_cast<double>(load_cell_calibration_sample_count_);
    load_cell_force_offset_n_ =
        load_cell_voltage_offset_mv_ * force_gradient_n_per_mv_ + force_bias_n_;

    const std::chrono::duration<double> elapsed =
        now - load_cell_startup_calibration_start_;
    if (elapsed.count() >= load_cell_startup_calibration_sec_) {
      load_cell_startup_calibration_done_ = true;
      RCLCPP_INFO(get_logger(),
                  "Loadcell startup calibration complete: raw_offset=%.2f, "
                  "voltage_offset=%.6f mV, force_offset=%.6f N (%llu samples)",
                  load_cell_raw_offset_, load_cell_voltage_offset_mv_,
                  load_cell_force_offset_n_,
                  static_cast<unsigned long long>(
                      load_cell_calibration_sample_count_));
    }
  }

  bool isLoadCellStartupCalibrationDone() const {
    return load_cell_startup_calibration_sec_ <= 0.0 ||
           load_cell_startup_calibration_done_;
  }

  int32_t calibratedRawCount(const int32_t raw_count) const {
    const double calibrated =
        std::round(static_cast<double>(raw_count) - load_cell_raw_offset_);
    return static_cast<int32_t>(std::clamp(
        calibrated, static_cast<double>(std::numeric_limits<int32_t>::min()),
        static_cast<double>(std::numeric_limits<int32_t>::max())));
  }

  std::string port_ = "/dev/ttyUSB0";
  int baud_ = 115200;
  double publish_rate_ = 100.0;
  std::string frame_id_lc_ = "load_cell";
  std::string frame_id_enc_ = "encoder";
  std::string frame_id_switch_ = "switch";
  std::string load_cell_topic_name_ = "/load_cell/data";
  std::string encoder_topic_name_ = "/encoder/data";
  std::string switch_topic_name_ = "/switch/data";
  std::string load_cell_calibration_done_topic_name_ =
      "/load_cell/calibration_done";
  double force_gradient_n_per_mv_ = 1.0;
  double force_bias_n_ = 0.0;
  double load_cell_startup_calibration_sec_ = 5.0;
  bool load_cell_startup_calibration_started_ = false;
  bool load_cell_startup_calibration_done_ = false;
  std::chrono::steady_clock::time_point load_cell_startup_calibration_start_;
  uint64_t load_cell_calibration_sample_count_ = 0;
  double load_cell_raw_sum_ = 0.0;
  double load_cell_voltage_sum_mv_ = 0.0;
  double load_cell_raw_offset_ = 0.0;
  double load_cell_voltage_offset_mv_ = 0.0;
  double load_cell_force_offset_n_ = 0.0;

  std::unique_ptr<ArduinoBridge> bridge_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<enc_lc::msg::LoadCellData>::SharedPtr load_cell_pub_;
  rclcpp::Publisher<enc_lc::msg::EncoderData>::SharedPtr encoder_pub_;
  rclcpp::Publisher<enc_lc::msg::SwitchData>::SharedPtr switch_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      load_cell_calibration_done_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<ArduinoSensorPub>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("arduino_sensor_pub"), "%s", error.what());
  }

  rclcpp::shutdown();
  return 0;
}
