#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class LatestFrameRelayNode : public rclcpp::Node {
public:
  LatestFrameRelayNode() : Node("latest_frame_relay_node") {
    input_color_topic_ = declare_parameter<std::string>(
      "input_color_topic", "/camera/camera/color/image_raw");
    input_depth_topic_ = declare_parameter<std::string>(
      "input_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    input_camera_info_topic_ = declare_parameter<std::string>(
      "input_camera_info_topic", "/camera/camera/color/camera_info");
    output_color_topic_ = declare_parameter<std::string>(
      "output_color_topic", "/camera/relay/color/image_raw");
    output_depth_topic_ = declare_parameter<std::string>(
      "output_depth_topic", "/camera/relay/aligned_depth_to_color/image_raw");
    output_camera_info_topic_ = declare_parameter<std::string>(
      "output_camera_info_topic", "/camera/relay/color/camera_info");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 15.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 2.0);
    require_depth_stream_ = declare_parameter<bool>("require_depth_stream", false);
    log_interval_sec_ = declare_parameter<double>("log_interval_sec", 15.0);

    if (publish_rate_hz_ < 1.0) {
      publish_rate_hz_ = 1.0;
    }
    if (stale_timeout_sec_ < 0.2) {
      stale_timeout_sec_ = 0.2;
    }
    if (log_interval_sec_ < 1.0) {
      log_interval_sec_ = 1.0;
    }

    auto qos_sensor = rclcpp::SensorDataQoS();
    qos_sensor.keep_last(1);

    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_color_topic_, qos_sensor,
      std::bind(&LatestFrameRelayNode::on_color, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_depth_topic_, qos_sensor,
      std::bind(&LatestFrameRelayNode::on_depth, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      input_camera_info_topic_, qos_sensor,
      std::bind(&LatestFrameRelayNode::on_info, this, std::placeholders::_1));

    color_pub_ = create_publisher<sensor_msgs::msg::Image>(output_color_topic_, qos_sensor);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(output_depth_topic_, qos_sensor);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_camera_info_topic_, qos_sensor);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LatestFrameRelayNode::publish_latest, this));
    log_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(log_interval_sec_)),
      std::bind(&LatestFrameRelayNode::log_stats, this));

    RCLCPP_INFO(
      get_logger(),
      "latest_frame_relay_node started: in(color=%s, depth=%s, info=%s), out(color=%s, depth=%s, info=%s), rate=%.2f",
      input_color_topic_.c_str(),
      input_depth_topic_.c_str(),
      input_camera_info_topic_.c_str(),
      output_color_topic_.c_str(),
      output_depth_topic_.c_str(),
      output_camera_info_topic_.c_str(),
      publish_rate_hz_);
  }

private:
  void on_color(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_color_ = msg;
    last_color_time_ = now();
    in_color_count_++;
  }

  void on_depth(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_ = msg;
    last_depth_time_ = now();
    in_depth_count_++;
  }

  void on_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_info_ = msg;
    last_info_time_ = now();
    in_info_count_++;
  }

  bool is_fresh(const rclcpp::Time &stamp) const {
    if (stamp.nanoseconds() <= 0) {
      return false;
    }
    return (now() - stamp).seconds() <= stale_timeout_sec_;
  }

  void publish_latest() {
    sensor_msgs::msg::Image::SharedPtr color;
    sensor_msgs::msg::Image::SharedPtr depth;
    sensor_msgs::msg::CameraInfo::SharedPtr info;
    rclcpp::Time color_ts(0, 0, get_clock()->get_clock_type());
    rclcpp::Time depth_ts(0, 0, get_clock()->get_clock_type());
    rclcpp::Time info_ts(0, 0, get_clock()->get_clock_type());

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      color = latest_color_;
      depth = latest_depth_;
      info = latest_info_;
      color_ts = last_color_time_;
      depth_ts = last_depth_time_;
      info_ts = last_info_time_;
    }

    const bool color_ready = color != nullptr && is_fresh(color_ts);
    const bool depth_ready = depth != nullptr && is_fresh(depth_ts);
    const bool info_ready = info != nullptr && is_fresh(info_ts);

    if (!color_ready || !info_ready) {
      return;
    }
    if (require_depth_stream_ && !depth_ready) {
      return;
    }

    color_pub_->publish(*color);
    out_color_count_++;
    info_pub_->publish(*info);
    out_info_count_++;
    if (depth_ready) {
      depth_pub_->publish(*depth);
      out_depth_count_++;
    }
  }

  void log_stats() {
    RCLCPP_INFO(
      get_logger(),
      "relay stats in(color=%llu depth=%llu info=%llu) out(color=%llu depth=%llu info=%llu)",
      static_cast<unsigned long long>(in_color_count_),
      static_cast<unsigned long long>(in_depth_count_),
      static_cast<unsigned long long>(in_info_count_),
      static_cast<unsigned long long>(out_color_count_),
      static_cast<unsigned long long>(out_depth_count_),
      static_cast<unsigned long long>(out_info_count_));
  }

  std::string input_color_topic_;
  std::string input_depth_topic_;
  std::string input_camera_info_topic_;
  std::string output_color_topic_;
  std::string output_depth_topic_;
  std::string output_camera_info_topic_;
  double publish_rate_hz_{15.0};
  double stale_timeout_sec_{2.0};
  bool require_depth_stream_{false};
  double log_interval_sec_{15.0};

  std::mutex data_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_color_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_info_;
  rclcpp::Time last_color_time_{0, 0};
  rclcpp::Time last_depth_time_{0, 0};
  rclcpp::Time last_info_time_{0, 0};

  uint64_t in_color_count_{0};
  uint64_t in_depth_count_{0};
  uint64_t in_info_count_{0};
  uint64_t out_color_count_{0};
  uint64_t out_depth_count_{0};
  uint64_t out_info_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LatestFrameRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
