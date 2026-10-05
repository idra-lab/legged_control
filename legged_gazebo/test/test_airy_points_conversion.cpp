#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "legged_gazebo/AiryPointsConversion.h"

using legged::AiryGap;
using legged::kRslidarPointStep;
using legged::toRslidarCloud;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace {

void addField(PointCloud2& cloud, const std::string& name, uint32_t offset, uint8_t datatype) {
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  cloud.fields.push_back(field);
}

// Gazebo gpu_lidar-like cloud: x y z intensity FLOAT32, ring UINT16 at 16, padded to point_step 32.
PointCloud2 makeGzCloud(const std::vector<std::array<float, 4>>& xyzi, const std::vector<uint16_t>& rings,
                        uint32_t height, uint32_t width) {
  PointCloud2 cloud;
  cloud.header.stamp.sec = 12;
  cloud.header.stamp.nanosec = 500000000;
  cloud.header.frame_id = "rslidar_optical";
  cloud.height = height;
  cloud.width = width;
  addField(cloud, "x", 0, PointField::FLOAT32);
  addField(cloud, "y", 4, PointField::FLOAT32);
  addField(cloud, "z", 8, PointField::FLOAT32);
  addField(cloud, "intensity", 12, PointField::FLOAT32);
  addField(cloud, "ring", 16, PointField::UINT16);
  cloud.point_step = 32;
  cloud.row_step = width * cloud.point_step;
  cloud.data.assign(static_cast<size_t>(height) * cloud.row_step, 0);
  for (size_t i = 0; i < xyzi.size(); ++i) {
    uint8_t* point = cloud.data.data() + i * cloud.point_step;
    std::memcpy(point, xyzi[i].data(), 4 * sizeof(float));
    std::memcpy(point + 16, &rings[i], sizeof(uint16_t));
  }
  return cloud;
}

template <typename T>
T readAt(const PointCloud2& cloud, size_t index, uint32_t offset) {
  T value;
  std::memcpy(&value, cloud.data.data() + index * cloud.point_step + offset, sizeof(T));
  return value;
}

constexpr float kInf = std::numeric_limits<float>::infinity();

}  // namespace

TEST(AiryPointsConversion, OutputHasRslidarXyzirtLayout) {
  const PointCloud2 in = makeGzCloud({{1, 0, 0, 0}, {2, 0, 0, 0}, {3, 0, 0, 0}, {4, 0, 0, 0}, {5, 0, 0, 0}, {6, 0, 0, 0}},
                                     {0, 0, 0, 1, 1, 1}, 2, 3);
  PointCloud2 out;
  ASSERT_TRUE(toRslidarCloud(in, Eigen::Isometry3f::Identity(), AiryGap{}, "rslidar", out));

  const std::vector<std::string> names{"x", "y", "z", "intensity", "ring", "timestamp"};
  const std::vector<uint32_t> offsets{0, 4, 8, 12, 16, 18};
  const std::vector<uint8_t> types{PointField::FLOAT32, PointField::FLOAT32, PointField::FLOAT32,
                                   PointField::FLOAT32, PointField::UINT16,  PointField::FLOAT64};
  ASSERT_EQ(out.fields.size(), names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    EXPECT_EQ(out.fields[i].name, names[i]);
    EXPECT_EQ(out.fields[i].offset, offsets[i]);
    EXPECT_EQ(out.fields[i].datatype, types[i]);
    EXPECT_EQ(out.fields[i].count, 1u);
  }
  EXPECT_EQ(out.point_step, kRslidarPointStep);
  EXPECT_EQ(out.point_step, 26u);
  EXPECT_EQ(out.height, 2u);
  EXPECT_EQ(out.width, 3u);
  EXPECT_EQ(out.row_step, 3u * 26u);
  EXPECT_EQ(out.data.size(), 6u * 26u);
  EXPECT_FALSE(out.is_dense);
  EXPECT_FALSE(out.is_bigendian);
  EXPECT_EQ(out.header.frame_id, "rslidar");
  EXPECT_EQ(out.header.stamp.sec, 12);
  EXPECT_EQ(out.header.stamp.nanosec, 500000000u);
}

TEST(AiryPointsConversion, TransformsPointsAndCopiesRingIntensityStamp) {
  const PointCloud2 in = makeGzCloud({{1, 0, 0, 5}, {0, 2, 1, 9}}, {7, 95}, 1, 2);
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.translation() = Eigen::Vector3f(0, 0, 0.04f);
  PointCloud2 out;
  ASSERT_TRUE(toRslidarCloud(in, transform, AiryGap{}, "rslidar", out));

  EXPECT_FLOAT_EQ(readAt<float>(out, 0, 0), 1.0f);
  EXPECT_FLOAT_EQ(readAt<float>(out, 0, 4), 0.0f);
  EXPECT_FLOAT_EQ(readAt<float>(out, 0, 8), 0.04f);
  EXPECT_FLOAT_EQ(readAt<float>(out, 0, 12), 5.0f);
  EXPECT_EQ(readAt<uint16_t>(out, 0, 16), 7);
  EXPECT_DOUBLE_EQ(readAt<double>(out, 0, 18), 12.5);

  EXPECT_FLOAT_EQ(readAt<float>(out, 1, 0), 0.0f);
  EXPECT_FLOAT_EQ(readAt<float>(out, 1, 4), 2.0f);
  EXPECT_FLOAT_EQ(readAt<float>(out, 1, 8), 1.04f);
  EXPECT_FLOAT_EQ(readAt<float>(out, 1, 12), 9.0f);
  EXPECT_EQ(readAt<uint16_t>(out, 1, 16), 95);
  EXPECT_DOUBLE_EQ(readAt<double>(out, 1, 18), 12.5);
}

TEST(AiryPointsConversion, NonFiniteBecomesNan) {
  const PointCloud2 in = makeGzCloud({{kInf, kInf, kInf, 0}, {-kInf, 1, 1, 0}}, {3, 4}, 1, 2);
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.translation() = Eigen::Vector3f(0, 0, 0.04f);
  PointCloud2 out;
  ASSERT_TRUE(toRslidarCloud(in, transform, AiryGap{}, "rslidar", out));

  for (size_t i = 0; i < 2; ++i) {
    EXPECT_TRUE(std::isnan(readAt<float>(out, i, 0)));
    EXPECT_TRUE(std::isnan(readAt<float>(out, i, 4)));
    EXPECT_TRUE(std::isnan(readAt<float>(out, i, 8)));
  }
  EXPECT_EQ(readAt<uint16_t>(out, 0, 16), 3);
  EXPECT_EQ(readAt<uint16_t>(out, 1, 16), 4);
}

TEST(AiryPointsConversion, GapBlanksWedgeOnlyWhenActive) {
  const float c10 = std::cos(10.0f * static_cast<float>(M_PI) / 180.0f);
  const float s10 = std::sin(10.0f * static_cast<float>(M_PI) / 180.0f);
  // azimuth 190 deg (in the 180-212 wedge) and 170 deg (outside)
  const PointCloud2 in = makeGzCloud({{-c10, -s10, 0, 0}, {-c10, s10, 0, 0}}, {0, 0}, 1, 2);
  AiryGap gap;
  gap.start_deg = 180.0;
  gap.width_deg = 32.0;

  PointCloud2 out;
  gap.active = true;
  ASSERT_TRUE(toRslidarCloud(in, Eigen::Isometry3f::Identity(), gap, "rslidar", out));
  EXPECT_TRUE(std::isnan(readAt<float>(out, 0, 0)));
  EXPECT_FALSE(std::isnan(readAt<float>(out, 1, 0)));

  gap.active = false;
  ASSERT_TRUE(toRslidarCloud(in, Eigen::Isometry3f::Identity(), gap, "rslidar", out));
  EXPECT_FALSE(std::isnan(readAt<float>(out, 0, 0)));
  EXPECT_FALSE(std::isnan(readAt<float>(out, 1, 0)));

  gap.active = true;
  gap.width_deg = 0.0;
  ASSERT_TRUE(toRslidarCloud(in, Eigen::Isometry3f::Identity(), gap, "rslidar", out));
  EXPECT_FALSE(std::isnan(readAt<float>(out, 0, 0)));
}

TEST(AiryPointsConversion, GapWrapsAroundZeroAzimuth) {
  const auto at = [](float deg) {
    const float rad = deg * static_cast<float>(M_PI) / 180.0f;
    return std::array<float, 4>{std::cos(rad), std::sin(rad), 0, 0};
  };
  // wedge 350-22 deg: 10 deg is inside, 30 deg outside
  const PointCloud2 in = makeGzCloud({at(10.0f), at(30.0f)}, {0, 0}, 1, 2);
  AiryGap gap;
  gap.active = true;
  gap.start_deg = 350.0;
  gap.width_deg = 32.0;
  PointCloud2 out;
  ASSERT_TRUE(toRslidarCloud(in, Eigen::Isometry3f::Identity(), gap, "rslidar", out));
  EXPECT_TRUE(std::isnan(readAt<float>(out, 0, 0)));
  EXPECT_FALSE(std::isnan(readAt<float>(out, 1, 0)));
}

TEST(AiryPointsConversion, RejectsCloudWithoutXyz) {
  PointCloud2 in = makeGzCloud({{1, 0, 0, 0}}, {0}, 1, 1);
  in.fields.erase(in.fields.begin() + 2);  // drop z
  PointCloud2 out;
  EXPECT_FALSE(toRslidarCloud(in, Eigen::Isometry3f::Identity(), AiryGap{}, "rslidar", out));
}

TEST(AiryPointsConversion, RejectsTruncatedData) {
  PointCloud2 in = makeGzCloud({{1, 0, 0, 0}, {2, 0, 0, 0}}, {0, 0}, 1, 2);
  in.data.resize(40);  // second point cut short
  PointCloud2 out;
  EXPECT_FALSE(toRslidarCloud(in, Eigen::Isometry3f::Identity(), AiryGap{}, "rslidar", out));
}

TEST(AiryPointsConversion, MissingIntensityAndRingWriteZero) {
  PointCloud2 in = makeGzCloud({{1, 0, 0, 5}}, {7}, 1, 1);
  in.fields.resize(3);  // keep x y z only
  PointCloud2 out;
  ASSERT_TRUE(toRslidarCloud(in, Eigen::Isometry3f::Identity(), AiryGap{}, "rslidar", out));
  EXPECT_FLOAT_EQ(readAt<float>(out, 0, 0), 1.0f);
  EXPECT_FLOAT_EQ(readAt<float>(out, 0, 12), 0.0f);
  EXPECT_EQ(readAt<uint16_t>(out, 0, 16), 0);
}
