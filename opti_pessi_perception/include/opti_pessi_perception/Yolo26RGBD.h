#pragma once

// ROS 2 node: YOLO26 detection on RGB + per-object distance/3D point from depth, run with YOLOs-CPP (ONNX Runtime).
//
// C++ port of yolo/yolo26_rgbd_detect.py: same node name, parameters and topics, but `model` is an ONNX export:
//   .venv-yolo/bin/yolo export model=yolo/yolo26n.pt format=onnx
// A relative `model` or `labels` path is looked up in this package's weights/ folder (installed to
// share/opti_pessi_perception/weights, rebuild after adding a file), then from the current directory.
//
// Depth must be registered to the color frame (RealSense: align_depth.enable:=true).
//
// Publishes:
//   ~/detections           vision_msgs/Detection3DArray      (camera optical frame, meters)
//   ~/detections_image     sensor_msgs/Image                 (annotated RGB, persons blurred if blur_persons:=true)
//   /opti_pessi/obstacles  legged_controllers/ObstacleArray  (persons and forklifts in odom, read by the Opti-Pessi solver)

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <legged_controllers/msg/obstacle_array.hpp>

// Only a pointer is held here, so the ONNX Runtime headers stay out of this one.
namespace yolos::det {
class YOLODetector;
}  // namespace yolos::det

namespace opti_pessi_perception {

class Yolo26RGBD : public rclcpp::Node {
 public:
  Yolo26RGBD();
  ~Yolo26RGBD() override;

  bool show() const { return show_; }
  const cv::Mat& view() const { return view_; }  // latest side-by-side frame for the viewer window

 private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using ObstacleArray = legged_controllers::msg::ObstacleArray;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void loadModel(const std::string& model, const std::string& labels, bool useGpu);
  void onInfo(const CameraInfo& msg) { K_ = msg.k; }
  void onImages(const Image::ConstSharedPtr& rgbMsg, const Image::ConstSharedPtr& depthMsg);

  /// Publish the obstacles in obstacles_frame, the whole set at once: the controller replaces its set with each message.
  ///
  /// An empty set is published too, so obstacles that left the view are cleared. Without the TF nothing is
  /// published and the controller keeps the last set.
  /// @param obstacles (Obstacle type, xyz in the camera optical frame)
  void publishObstacles(const std_msgs::msg::Header& header, const std::vector<std::pair<uint8_t, cv::Point3d>>& obstacles);

  std::unique_ptr<yolos::det::YOLODetector> detector_;
  std::vector<std::string> names_;  // class names in class-id order
  std::set<int> classIds_;
  double conf_;
  bool show_;
  double maxDepth_;
  bool blurPersons_;
  std::string obstaclesFrame_;
  double centerOffset_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
  std::optional<std::array<double, 9>> K_;
  cv::Mat view_;
  std::optional<double> lastStamp_;
  double rate_ = 0.0;

  rclcpp::Subscription<CameraInfo>::SharedPtr infoSub_;
  message_filters::Subscriber<Image> rgbSub_, depthSub_;
  std::unique_ptr<Synchronizer> sync_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detPub_;
  rclcpp::Publisher<Image>::SharedPtr visPub_;
  rclcpp::Publisher<ObstacleArray>::SharedPtr obstaclePub_;
};

}  // namespace opti_pessi_perception
