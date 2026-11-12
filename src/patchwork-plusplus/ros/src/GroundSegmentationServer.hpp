// Patchwork++ and Patchwork classic
#include "patchwork/patchwork.h"
#include "patchwork/patchworkpp.h"

// Standard library
#include <string>
#include <cstddef>
#include <cstdint>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace patchworkpp_ros {

class GroundSegmentationServer : public rclcpp::Node {
 public:
  /// GroundSegmentationServer constructor
  GroundSegmentationServer() = delete;
  explicit GroundSegmentationServer(const rclcpp::NodeOptions &options);

 private:
  /// Register new frame
  void EstimateGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

  /// Stream the point clouds for visualization
  void PublishClouds(const Eigen::MatrixX3f &est_ground,
                     const Eigen::MatrixX3f &est_nonground,
                     const std_msgs::msg::Header header_msg,
                     const std::string &input_frame,
                     std::size_t input_near_range_count);

  /// Parameter loaders — only the selected algorithm's loader is called
  patchwork::Params loadPlusplusParamsFromROS();
  patchwork::PatchworkParams loadClassicParamsFromROS();

 private:
  /// Data subscribers.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  /// Data publishers.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_window_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;

  /// Algorithm implementation (patchworkpp or patchwork classic)
  using ImplVariant =
      std::variant<std::unique_ptr<patchwork::PatchWorkpp>, std::unique_ptr<patchwork::PatchWork>>;
  ImplVariant impl_;

   std::string base_frame_{"base_link"};
   double sensor_height_{0.20};
   double ground_visualization_range_{5.0};
   int ground_visualization_publish_stride_{2};
   double ground_visualization_voxel_size_{0.08};
   double ground_residual_xy_tolerance_{0.10};
   double ground_residual_z_tolerance_{0.03};
   double ground_min_range_{0.50};
   std::uint64_t frame_count_{0};
   tf2_ros::Buffer tf_buffer_;
   tf2_ros::TransformListener tf_listener_;
};

}  // namespace patchworkpp_ros
