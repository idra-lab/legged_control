// Republishes the simulated RoboSense Airy cloud as rslidar_sdk does (legged_unitree_description/urdf/common/robosense_airy.xacro):
// /rslidar/points_optical (Gazebo gpu_lidar, frame rslidar_optical, +-inf for no return) ->
// /rslidar_points (XYZIRT, frame rslidar, NaN for no return, the Airy frame gap every 10th frame).
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "legged_gazebo/AiryPointsConversion.h"

namespace legged {

class AiryPointsRelay : public rclcpp::Node {
 public:
  AiryPointsRelay() : Node("airy_points_relay") {
    target_frame_ = declare_parameter<std::string>("target_frame", "rslidar");
    gap_.width_deg = declare_parameter<double>("gap_deg", 32.0);
    gap_.start_deg = declare_parameter<double>("gap_start_deg", 180.0);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Reliable on both sides, like ros_gz_bridge and rslidar_sdk (plain queue length): a 900 x 96 cloud is over 2 MB
    // and best effort loses most of them to fragment drops with the default 212 KB socket buffers
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/rslidar_points", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar/points_optical", 10,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(*msg); });
  }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2& in) {
    // The optical center is a fixed joint in the URDF: look the transform up once, then stop listening, the
    // dynamic /tf of the robot joints comes at the joint state rate
    if (!transform_) {
      try {
        transform_ =
            tf2::transformToEigen(tf_buffer_->lookupTransform(target_frame_, in.header.frame_id, tf2::TimePointZero))
                .cast<float>();
        tf_listener_.reset();
      } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No TF %s -> %s yet, cloud dropped: %s",
                             target_frame_.c_str(), in.header.frame_id.c_str(), e.what());
        return;
      }
    }

    const auto start = std::chrono::steady_clock::now();
    AiryGap gap = gap_;
    gap.active = frame_count_++ % 10 == 0;
    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    if (!toRslidarCloud(in, *transform_, gap, target_frame_, *out)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Cloud without FLOAT32 x y z fields or with short data, dropped");
      return;
    }
    publisher_->publish(std::move(out));
    RCLCPP_DEBUG(get_logger(), "Relayed %u x %u points in %.2f ms", in.height, in.width,
                 std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  }

  std::string target_frame_;
  AiryGap gap_;
  uint64_t frame_count_ = 0;
  std::optional<Eigen::Isometry3f> transform_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace legged

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<legged::AiryPointsRelay>());
  rclcpp::shutdown();
  return 0;
}
