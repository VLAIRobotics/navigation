#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace fast_lio {

class CloudWindowFilter : public rclcpp::Node {
 public:
  CloudWindowFilter() : rclcpp::Node("cloud_window_filter") {
    window_radius_ = declare_parameter<double>("window_radius", 5.0);
    robot_frame_ = declare_parameter<std::string>("robot_frame", "body");

    const auto world_topic =
        declare_parameter<std::string>("world_cloud_topic", "/cloud_registered");
    const auto body_topic = declare_parameter<std::string>(
        "body_cloud_topic", "/cloud_registered_body_downsampled");
    const auto world_window_topic = declare_parameter<std::string>(
        "world_cloud_window_topic", "/cloud_registered_window");
    const auto body_window_topic = declare_parameter<std::string>(
        "body_cloud_window_topic", "/cloud_registered_body_downsampled_window");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::SensorDataQoS();
    world_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        world_topic, qos,
        std::bind(&CloudWindowFilter::WorldCallback, this, std::placeholders::_1));
    body_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        body_topic, qos,
        std::bind(&CloudWindowFilter::BodyCallback, this, std::placeholders::_1));
    world_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(world_window_topic, qos);
    body_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(body_window_topic, qos);
    world_window_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CloudWindowFilter::PublishWorldWindow, this));

    RCLCPP_INFO(get_logger(), "Cloud window filter initialized (radius %.2f m)",
                window_radius_);
  }

 private:
  void WorldCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (msg->header.frame_id.empty()) return;
    {
      std::lock_guard<std::mutex> lock(world_mutex_);
      latest_world_cloud_ = msg;
    }
    PublishWorldWindow();
  }

  void PublishWorldWindow() {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(world_mutex_);
      msg = latest_world_cloud_;
    }
    if (!msg) return;

    double center_x = 0.0;
    double center_y = 0.0;
    try {
      const auto transform = tf_buffer_->lookupTransform(
          msg->header.frame_id, robot_frame_, tf2::TimePointZero);
      center_x = transform.transform.translation.x;
      center_y = transform.transform.translation.y;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Dropping cloud: TF %s -> %s unavailable: %s",
                           msg->header.frame_id.c_str(), robot_frame_.c_str(), ex.what());
      return;
    }
    world_pub_->publish(Crop(*msg, center_x, center_y));
  }

  void BodyCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    body_pub_->publish(Crop(*msg, 0.0, 0.0));
  }

  sensor_msgs::msg::PointCloud2 Crop(const sensor_msgs::msg::PointCloud2 &msg,
                                     double center_x, double center_y) {
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg.header;
    out.height = 1;
    out.fields = msg.fields;
    out.point_step = msg.point_step;
    out.is_bigendian = msg.is_bigendian;
    out.is_dense = false;

    const float radius_squared = static_cast<float>(window_radius_ * window_radius_);
    const std::size_t point_count = static_cast<std::size_t>(msg.width) * msg.height;
    out.data.reserve(msg.data.size());

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
      for (std::size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y) {
        const float x = *iter_x;
        const float y = *iter_y;
        if (!std::isfinite(x) || !std::isfinite(y)) {
          continue;
        }
        const float dx = x - static_cast<float>(center_x);
        const float dy = y - static_cast<float>(center_y);
        if (dx * dx + dy * dy > radius_squared) {
          continue;
        }
        const auto *src = msg.data.data() + i * msg.point_step;
        out.data.insert(out.data.end(), src, src + msg.point_step);
      }
    } catch (const std::runtime_error &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Dropping cloud without x/y fields: %s", ex.what());
      return out;
    }

    out.width = static_cast<std::uint32_t>(out.data.size() / msg.point_step);
    out.row_step = out.width * msg.point_step;
    return out;
  }

  double window_radius_{5.0};
  std::string robot_frame_{"body"};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr world_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr body_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_pub_;
  rclcpp::TimerBase::SharedPtr world_window_timer_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_world_cloud_;
  std::mutex world_mutex_;
};

}  // namespace fast_lio

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fast_lio::CloudWindowFilter>());
  rclcpp::shutdown();
  return 0;
}
