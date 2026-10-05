#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace legged {

// rslidar_sdk XYZIRT point: x y z intensity FLOAT32, ring UINT16, timestamp FLOAT64, packed (offsets 0/4/8/12/16/18).
constexpr uint32_t kRslidarPointStep = 26;

// The Airy has no points for ~9 ms per second: one frame in ten misses a ~32 deg azimuth wedge.
struct AiryGap {
  bool active = false;       // true on the frames that miss the wedge
  double start_deg = 180.0;  // wedge start, azimuth atan2(y, x) of the input frame
  double width_deg = 32.0;   // 0 disables the gap
};

// Converts a Gazebo gpu_lidar cloud (x y z FLOAT32, optional intensity FLOAT32 and ring UINT16, +-inf for no return)
// into what rslidar_sdk publishes for the Airy: the XYZIRT layout above, organized like the input, is_dense false.
// Points are moved by transform (input frame -> frame_id); non-finite points and, when gap.active, points in the gap
// wedge become NaN; every point's timestamp is the header stamp in seconds (the Gazebo scan is instantaneous).
// A missing intensity or ring field is written as 0. Returns false, leaving out unspecified, when the input has no
// FLOAT32 x y z fields or its data is shorter than its height, width and steps say.
bool toRslidarCloud(const sensor_msgs::msg::PointCloud2& in, const Eigen::Isometry3f& transform, const AiryGap& gap,
                    const std::string& frame_id, sensor_msgs::msg::PointCloud2& out);

}  // namespace legged
