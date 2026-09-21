#include "opti_pessi_perception/Yolo26RGBD.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yolos/tasks/detection.hpp>

namespace opti_pessi_perception {

namespace {

using legged_controllers::msg::Obstacle;

// YOLO class name -> obstacle type the Opti-Pessi controller keeps out of (legged_controllers/msg/Obstacle.msg).
const std::map<std::string, uint8_t> kObstacleTypes = {{"person", Obstacle::HUMAN}, {"forklift", Obstacle::CAR}};

struct Box {
  float x1, y1, x2, y2;
};

struct DrawnDetection {
  int classId;
  std::string label;
  float conf;
  Box box;
  double z;                        // NaN when the box has no valid depth
  std::optional<cv::Point3d> xyz;  // camera optical frame, only once the camera info arrived
};

/// Absolute paths as they are; relative ones from this package's installed weights folder when the file is there,
/// otherwise from the current directory.
std::string resolveWeightsPath(const std::string& path) {
  if (path.empty() || std::filesystem::path(path).is_absolute()) {
    return path;
  }
  const std::filesystem::path installed =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("opti_pessi_perception")) / "weights" / path;
  return std::filesystem::exists(installed) ? installed.string() : path;
}

/// Median of valid depth in the box center region (avoids background at edges).
double boxDepth(const cv::Mat& depth, const Box& b, double shrink = 0.25) {
  const double w = b.x2 - b.x1, h = b.y2 - b.y1;
  const int r0 = std::max(0, static_cast<int>(b.y1 + h * shrink)), r1 = std::min(depth.rows, static_cast<int>(b.y2 - h * shrink));
  const int c0 = std::max(0, static_cast<int>(b.x1 + w * shrink)), c1 = std::min(depth.cols, static_cast<int>(b.x2 - w * shrink));
  std::vector<float> valid;
  for (int r = r0; r < r1; ++r) {
    const float* row = depth.ptr<float>(r);
    for (int c = c0; c < c1; ++c) {
      if (std::isfinite(row[c]) && row[c] > 0) {
        valid.push_back(row[c]);
      }
    }
  }
  if (valid.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // As np.median: the mean of the two middle values for an even count.
  const size_t mid = valid.size() / 2;
  std::nth_element(valid.begin(), valid.begin() + mid, valid.end());
  double median = valid[mid];
  if (valid.size() % 2 == 0) {
    median = 0.5 * (median + *std::max_element(valid.begin(), valid.begin() + mid));
  }
  return median;
}

/// Stable, distinct BGR color per class id.
cv::Scalar classColor(int classId) {
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar((classId * 47) % 180, 220, 255)), bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(c[0], c[1], c[2]);
}

cv::Mat colorizeDepth(const cv::Mat& depth, double maxDepth) {
  cv::Mat norm(depth.size(), CV_8UC1, cv::Scalar(0)), invalid(depth.size(), CV_8UC1, cv::Scalar(255));
  for (int r = 0; r < depth.rows; ++r) {
    const float* d = depth.ptr<float>(r);
    for (int c = 0; c < depth.cols; ++c) {
      if (std::isfinite(d[c]) && d[c] > 0) {
        norm.at<uint8_t>(r, c) = static_cast<uint8_t>(255 * (1.0 - std::clamp(d[c] / maxDepth, 0.0, 1.0)));  // near = warm
        invalid.at<uint8_t>(r, c) = 0;
      }
    }
  }
  cv::Mat vis;
  cv::applyColorMap(norm, vis, cv::COLORMAP_TURBO);
  vis.setTo(cv::Scalar::all(0), invalid);
  return vis;
}

void drawLabel(cv::Mat& img, const std::string& text, double x, double y, const cv::Scalar& color) {
  const int font = cv::FONT_HERSHEY_SIMPLEX, thick = 1;
  const double scale = 0.5;
  int base = 0;
  const cv::Size t = cv::getTextSize(text, font, scale, thick, &base);
  // Keep the label fully inside the image so it is never cut at the edges.
  const int xi = static_cast<int>(std::min(std::max(x, 0.0), static_cast<double>(std::max(img.cols - t.width - 4, 0))));
  const int yi = static_cast<int>(std::min(std::max(y, static_cast<double>(t.height + base + 2)), static_cast<double>(img.rows)));
  cv::rectangle(img, cv::Point(xi, yi - t.height - base - 2), cv::Point(xi + t.width + 4, yi), color, cv::FILLED);
  cv::putText(img, text, cv::Point(xi + 2, yi - base), font, scale, cv::Scalar(0, 0, 0), thick, cv::LINE_AA);
}

/// Blur a box region in place, kernel scaled to box size so faces are unrecognizable.
void blurBox(cv::Mat& img, const Box& b) {
  const int x1 = std::clamp(static_cast<int>(b.x1), 0, img.cols), y1 = std::clamp(static_cast<int>(b.y1), 0, img.rows);
  const int x2 = std::clamp(static_cast<int>(b.x2), 0, img.cols), y2 = std::clamp(static_cast<int>(b.y2), 0, img.rows);
  if (x2 <= x1 || y2 <= y1) {
    return;
  }
  const int k = (std::max(x2 - x1, y2 - y1) / 3) | 1;  // odd kernel
  cv::Mat roi = img(cv::Rect(x1, y1, x2 - x1, y2 - y1));
  cv::GaussianBlur(roi, roi, cv::Size(k, k), 0);
}

/// Returns (annotated RGB, side-by-side RGB | depth view).
std::pair<cv::Mat, cv::Mat> render(const cv::Mat& rgb, const cv::Mat& depth, const std::vector<DrawnDetection>& detections,
                                   const std::string& hud, double maxDepth, bool blurPersons) {
  cv::Mat rgbVis = rgb.clone();
  cv::Mat depthVis = colorizeDepth(depth, maxDepth);
  if (blurPersons) {
    // Blur before drawing so boxes and labels stay sharp on top.
    for (const auto& d : detections) {
      if (d.label == "person") {
        blurBox(rgbVis, d.box);
      }
    }
  }
  for (const auto& d : detections) {
    const cv::Scalar color = classColor(d.classId);
    const cv::Point p1(static_cast<int>(d.box.x1), static_cast<int>(d.box.y1));
    const cv::Point p2(static_cast<int>(d.box.x2), static_cast<int>(d.box.y2));
    const cv::Point center(static_cast<int>((d.box.x1 + d.box.x2) / 2), static_cast<int>((d.box.y1 + d.box.y2) / 2));
    const std::string dist = std::isfinite(d.z) ? cv::format("%.2f m", d.z) : std::string("-- m");
    // for (cv::Mat* img : {&rgbVis, &depthVis}) {
    for (cv::Mat* img : {&rgbVis}) {
      cv::rectangle(*img, p1, p2, color, 2);
      cv::drawMarker(*img, center, color, cv::MARKER_CROSS, 12, 2);
      drawLabel(*img, cv::format("%s %.0f%% | %s", d.label.c_str(), d.conf * 100.0, dist.c_str()), p1.x, p1.y, color);
    }
    if (d.xyz) {
      drawLabel(rgbVis, cv::format("x%+.2f y%+.2f z%.2f", d.xyz->x, d.xyz->y, d.xyz->z), p1.x, p2.y + 18, color);
    }
  }

  // cv::Mat view;
  // cv::hconcat(rgbVis, depthVis, view);
  cv::Mat view = rgbVis;
  cv::rectangle(view, cv::Point(0, 0), cv::Point(view.cols, 22), cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(view, hud, cv::Point(6, 16), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  cv::putText(view, cv::format("depth 0-%.0f m", maxDepth), cv::Point(rgb.cols + 6, rgb.rows - 8), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  return {rgbVis, view};
}

std::string join(const std::vector<std::string>& items) {
  std::string out;
  for (const auto& item : items) {
    out += (out.empty() ? "" : ", ") + item;
  }
  return out;
}

}  // namespace

Yolo26RGBD::Yolo26RGBD() : Node("yolo26_rgbd_detect") {
  declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
  declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
  declare_parameter<std::string>("info_topic", "/camera/color/camera_info");
  declare_parameter<std::string>("model", "yolo26n.onnx");  // file in weights/, or a path
  // declare_parameter<std::string>("model", "yolo/runs/detect/train-3/best.onnx");
  declare_parameter<std::string>("labels", "");  // class names file, one per line; empty: the names stored in the ONNX export
  declare_parameter<bool>("use_gpu", true);      // CUDA through ONNX Runtime; CPU when the CUDA provider cannot load
  declare_parameter<double>("conf", 0.4);
  declare_parameter<std::vector<std::string>>("classes", {"person"});  // class names to detect, others are ignored
  declare_parameter<bool>("show", true);
  declare_parameter<double>("max_depth", 10.0);
  declare_parameter<bool>("blur_persons", true);
  declare_parameter<std::string>("obstacles_topic", "/opti_pessi/obstacles");
  declare_parameter<std::string>("obstacles_frame", "odom");  // the controller ignores any other frame
  // The depth is the median over the visible front surface; the obstacle centre lies this far behind it along the ray.
  declare_parameter<double>("obstacle_center_offset", 0.15);

  const std::string model = resolveWeightsPath(get_parameter("model").as_string());
  loadModel(model, resolveWeightsPath(get_parameter("labels").as_string()), get_parameter("use_gpu").as_bool());
  const std::vector<std::string> classes = get_parameter("classes").as_string_array();
  std::vector<std::string> missing;
  for (const auto& name : classes) {
    const auto it = std::find(names_.begin(), names_.end(), name);
    if (it == names_.end()) {
      missing.push_back(name);
    } else {
      classIds_.insert(static_cast<int>(it - names_.begin()));
    }
  }
  if (!missing.empty()) {
    RCLCPP_WARN(get_logger(), "classes not in model %s: [%s]", model.c_str(), join(missing).c_str());
  }
  conf_ = get_parameter("conf").as_double();
  show_ = get_parameter("show").as_bool();
  maxDepth_ = get_parameter("max_depth").as_double();
  blurPersons_ = get_parameter("blur_persons").as_bool();
  obstaclesFrame_ = get_parameter("obstacles_frame").as_string();
  centerOffset_ = get_parameter("obstacle_center_offset").as_double();
  // Own spin thread: TF keeps arriving while onImages waits for the transform at the image stamp.
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_, this, true);

  const std::string rgbTopic = get_parameter("rgb_topic").as_string();
  const std::string depthTopic = get_parameter("depth_topic").as_string();
  infoSub_ = create_subscription<CameraInfo>(get_parameter("info_topic").as_string(), rclcpp::SensorDataQoS(),
                                             [this](const CameraInfo::ConstSharedPtr& msg) { onInfo(*msg); });
  rgbSub_.subscribe(this, rgbTopic, rmw_qos_profile_sensor_data);
  depthSub_.subscribe(this, depthTopic, rmw_qos_profile_sensor_data);
  sync_ = std::make_unique<Synchronizer>(SyncPolicy(10), rgbSub_, depthSub_);
  sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
  sync_->registerCallback(std::bind(&Yolo26RGBD::onImages, this, std::placeholders::_1, std::placeholders::_2));

  detPub_ = create_publisher<vision_msgs::msg::Detection3DArray>("~/detections", 10);
  visPub_ = create_publisher<Image>("~/detections_image", 1);
  obstaclePub_ = create_publisher<ObstacleArray>(get_parameter("obstacles_topic").as_string(), 1);
  RCLCPP_INFO(get_logger(), "waiting for %s + %s", rgbTopic.c_str(), depthTopic.c_str());
}

// Here, where YOLODetector is a complete type.
Yolo26RGBD::~Yolo26RGBD() = default;

void Yolo26RGBD::loadModel(const std::string& model, const std::string& labels, bool useGpu) {
  if (!std::filesystem::exists(model)) {
    throw std::runtime_error("model " + model + " not found in share/opti_pessi_perception/weights or the current directory: "
                             "export it with `yolo export model=<weights>.pt format=onnx`");
  }
  try {
    detector_ = std::make_unique<yolos::det::YOLODetector>(model, labels, useGpu);
  } catch (const Ort::Exception& e) {
    if (!useGpu) {
      throw;
    }
    // The CUDA provider needs the CUDA 12 runtime, cuBLAS and cuDNN 9 on the loader path.
    RCLCPP_WARN(get_logger(), "GPU session failed, falling back to CPU: %s", e.what());
    detector_ = std::make_unique<yolos::det::YOLODetector>(model, labels, false);
  }
  names_ = labels.empty() ? detector_->getExportedClassNamesFromMetadata() : detector_->getClassNames();
  if (names_.empty()) {
    throw std::runtime_error("no class names for " + model + ": the export has no `names` metadata, pass labels:=<file>");
  }
  RCLCPP_INFO(get_logger(), "model %s on %s, %zu classes", model.c_str(), detector_->getDevice().c_str(), names_.size());
}

void Yolo26RGBD::onImages(const Image::ConstSharedPtr& rgbMsg, const Image::ConstSharedPtr& depthMsg) {
  const cv_bridge::CvImageConstPtr rgbCv = cv_bridge::toCvShare(rgbMsg, "bgr8");
  const cv::Mat& rgb = rgbCv->image;
  cv::Mat depth;
  const bool millimeters = depthMsg->encoding == "16UC1" || depthMsg->encoding == "mono16";
  cv_bridge::toCvShare(depthMsg)->image.convertTo(depth, CV_32F, millimeters ? 0.001 : 1.0);  // millimeters -> meters
  if (depth.size() != rgb.size()) {
    // Different size means depth is not registered to color; resizing would give wrong ranges.
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "depth %dx%d != rgb %dx%d: subscribe to depth aligned to color", depth.cols,
                          depth.rows, rgb.cols, rgb.rows);
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();
  std::vector<yolos::det::Detection> result;
  try {
    result = detector_->detect(rgb, static_cast<float>(conf_));
  } catch (const Ort::Exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "inference failed: %s", e.what());
    return;
  }
  const double inferMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  vision_msgs::msg::Detection3DArray detArray;
  detArray.header = rgbMsg->header;
  std::vector<DrawnDetection> drawn;
  std::vector<std::pair<uint8_t, cv::Point3d>> obstacles;  // (Obstacle type, xyz in the camera optical frame)
  for (const auto& box : result) {
    if (classIds_.count(box.classId) == 0) {
      continue;
    }
    const Box b{static_cast<float>(box.box.x), static_cast<float>(box.box.y), static_cast<float>(box.box.x + box.box.width),
                static_cast<float>(box.box.y + box.box.height)};
    const std::string& label = names_[box.classId];
    const double z = boxDepth(depth, b);
    std::optional<cv::Point3d> xyz;

    std::string line = cv::format("%s %.2f z=%.2fm", label.c_str(), box.conf, z);
    if (K_ && std::isfinite(z)) {
      const double fx = (*K_)[0], fy = (*K_)[4], cx = (*K_)[2], cy = (*K_)[5];
      const double u = (b.x1 + b.x2) / 2.0, v = (b.y1 + b.y2) / 2.0;
      const double x = (u - cx) * z / fx, y = (v - cy) * z / fy;
      xyz = cv::Point3d(x, y, z);
      line += cv::format(" xyz=(%.2f, %.2f, %.2f)", x, y, z);

      vision_msgs::msg::Detection3D det;
      det.header = rgbMsg->header;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = label;
      hyp.hypothesis.score = box.conf;
      hyp.pose.pose.position.x = x;
      hyp.pose.pose.position.y = y;
      hyp.pose.pose.position.z = z;
      hyp.pose.pose.orientation.w = 1.0;
      det.results.push_back(hyp);
      det.bbox.center = hyp.pose.pose;
      // Metric width/height from the 2D box at depth z; thickness unknown from one view.
      det.bbox.size.x = (b.x2 - b.x1) * z / fx;
      det.bbox.size.y = (b.y2 - b.y1) * z / fy;
      det.bbox.size.z = 0.0;
      detArray.detections.push_back(det);
      const auto type = kObstacleTypes.find(label);
      if (type != kObstacleTypes.end()) {
        obstacles.emplace_back(type->second, *xyz);
      }
    }
    RCLCPP_INFO(get_logger(), "%s", line.c_str());
    drawn.push_back({box.classId, label, box.conf, b, z, xyz});
  }

  detPub_->publish(detArray);
  publishObstacles(rgbMsg->header, obstacles);

  // Input rate from message stamps, so it reflects sim time when use_sim_time is on.
  const double stamp = rclcpp::Time(rgbMsg->header.stamp).seconds();
  if (lastStamp_ && stamp > *lastStamp_) {
    rate_ = 0.8 * rate_ + 0.2 / (stamp - *lastStamp_);
  }
  lastStamp_ = stamp;

  const std::string hud = cv::format("%zu detections | inference %.0f ms | input %.1f Hz", drawn.size(), inferMs, rate_);
  cv::Mat rgbVis;
  std::tie(rgbVis, view_) = render(rgb, depth, drawn, hud, maxDepth_, blurPersons_);

  visPub_->publish(*cv_bridge::CvImage(rgbMsg->header, "bgr8", rgbVis).toImageMsg());
}

void Yolo26RGBD::publishObstacles(const std_msgs::msg::Header& header, const std::vector<std::pair<uint8_t, cv::Point3d>>& obstacles) {
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tfBuffer_->lookupTransform(obstaclesFrame_, header.frame_id, rclcpp::Time(header.stamp), rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "obstacles not published, no %s <- '%s' TF: %s", obstaclesFrame_.c_str(),
                         header.frame_id.c_str(), e.what());
    return;
  }
  ObstacleArray msg;
  msg.header.stamp = header.stamp;
  msg.header.frame_id = obstaclesFrame_;
  for (const auto& [obstacleType, xyz] : obstacles) {
    const cv::Point3d centre = xyz * (1.0 + centerOffset_ / cv::norm(xyz));
    geometry_msgs::msg::PointStamped p, pOdom;
    p.point.x = centre.x;
    p.point.y = centre.y;
    p.point.z = centre.z;
    tf2::doTransform(p, pOdom, tf);
    Obstacle obstacle;
    obstacle.type = obstacleType;
    obstacle.position = pOdom.point;
    msg.obstacles.push_back(obstacle);
  }
  obstaclePub_->publish(msg);
}

}  // namespace opti_pessi_perception
