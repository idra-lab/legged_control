// Runs opti_pessi_perception::Yolo26RGBD (see include/opti_pessi_perception/Yolo26RGBD.h) with an optional viewer.
//
// Run:
//   ros2 run opti_pessi_perception yolo26_rgbd_detect
//   ros2 run opti_pessi_perception yolo26_rgbd_detect --ros-args -p use_sim_time:=true
//
// Viewer window (show:=true): annotated RGB | colorized depth with the same boxes.
//   q / Esc: quit    s: save snapshot PNG in the current directory

#include <chrono>
#include <ctime>
#include <exception>
#include <memory>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>

#include "opti_pessi_perception/Yolo26RGBD.h"

namespace {

using opti_pessi_perception::Yolo26RGBD;

constexpr char kWindow[] = "YOLO26 RGB-D";

/// Show the latest frame and handle keys. Returns false when the user quits.
bool updateViewer(const Yolo26RGBD& node) {
  if (!node.view().empty()) {
    cv::imshow(kWindow, node.view());
  }
  const int key = cv::waitKey(1) & 0xFF;
  if (key == 'q' || key == 27) {
    RCLCPP_INFO(node.get_logger(), "quit key pressed, exiting");
    return false;
  }
  if (cv::getWindowProperty(kWindow, cv::WND_PROP_VISIBLE) < 1) {
    RCLCPP_INFO(node.get_logger(), "viewer window closed, exiting");
    return false;
  }
  if (key == 's' && !node.view().empty()) {
    char path[64];
    const std::time_t now = std::time(nullptr);
    std::strftime(path, sizeof(path), "yolo26_rgbd_%Y%m%d_%H%M%S.png", std::localtime(&now));
    cv::imwrite(path, node.view());
    RCLCPP_INFO(node.get_logger(), "saved %s", path);
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Yolo26RGBD> node;
  try {
    node = std::make_shared<Yolo26RGBD>();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("yolo26_rgbd_detect"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  if (node->show()) {
    cv::namedWindow(kWindow, cv::WINDOW_NORMAL);
    // GUI calls must stay on this thread, so spin in small steps instead of executor.spin().
    while (rclcpp::ok()) {
      executor.spin_once(std::chrono::milliseconds(20));
      executor.spin_some();  // drain the rest, spin_once runs a single callback
      if (!updateViewer(*node)) {
        break;
      }
    }
    cv::destroyAllWindows();
  } else {
    executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}
