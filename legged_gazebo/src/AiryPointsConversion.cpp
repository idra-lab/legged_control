#include "legged_gazebo/AiryPointsConversion.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace legged {
namespace {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

int fieldOffset(const PointCloud2& cloud, const std::string& name, uint8_t datatype) {
  for (const auto& field : cloud.fields) {
    if (field.name == name && field.datatype == datatype && field.count == 1) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

void addField(PointCloud2& cloud, const std::string& name, uint32_t offset, uint8_t datatype) {
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  cloud.fields.push_back(field);
}

template <typename T>
T readAt(const uint8_t* point, int offset) {
  T value;
  std::memcpy(&value, point + offset, sizeof(T));
  return value;
}

template <typename T>
void writeAt(uint8_t* point, uint32_t offset, T value) {
  std::memcpy(point + offset, &value, sizeof(T));
}

bool inGap(float x, float y, const AiryGap& gap) {
  if (!gap.active || gap.width_deg <= 0.0) {
    return false;
  }
  const double azimuth = std::atan2(y, x) * 180.0 / M_PI;
  return std::fmod(azimuth - gap.start_deg + 720.0, 360.0) < gap.width_deg;
}

}  // namespace

bool toRslidarCloud(const PointCloud2& in, const Eigen::Isometry3f& transform, const AiryGap& gap,
                    const std::string& frame_id, PointCloud2& out) {
  const int x_offset = fieldOffset(in, "x", PointField::FLOAT32);
  const int y_offset = fieldOffset(in, "y", PointField::FLOAT32);
  const int z_offset = fieldOffset(in, "z", PointField::FLOAT32);
  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    return false;
  }
  if (in.height > 0 && in.width > 0 &&
      in.data.size() < static_cast<size_t>(in.height - 1) * in.row_step + static_cast<size_t>(in.width) * in.point_step) {
    return false;
  }
  const int intensity_offset = fieldOffset(in, "intensity", PointField::FLOAT32);
  const int ring_offset = fieldOffset(in, "ring", PointField::UINT16);

  out.header.stamp = in.header.stamp;
  out.header.frame_id = frame_id;
  out.height = in.height;
  out.width = in.width;
  out.is_bigendian = false;
  out.is_dense = false;
  out.fields.clear();
  addField(out, "x", 0, PointField::FLOAT32);
  addField(out, "y", 4, PointField::FLOAT32);
  addField(out, "z", 8, PointField::FLOAT32);
  addField(out, "intensity", 12, PointField::FLOAT32);
  addField(out, "ring", 16, PointField::UINT16);
  addField(out, "timestamp", 18, PointField::FLOAT64);
  out.point_step = kRslidarPointStep;
  out.row_step = out.width * out.point_step;
  out.data.resize(static_cast<size_t>(out.height) * out.row_step);

  const double stamp = in.header.stamp.sec + in.header.stamp.nanosec * 1e-9;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (size_t row = 0; row < in.height; ++row) {
    for (size_t col = 0; col < in.width; ++col) {
      const uint8_t* src = in.data.data() + row * in.row_step + col * in.point_step;
      uint8_t* dst = out.data.data() + row * out.row_step + col * out.point_step;
      const Eigen::Vector3f p(readAt<float>(src, x_offset), readAt<float>(src, y_offset), readAt<float>(src, z_offset));
      Eigen::Vector3f q(nan, nan, nan);
      if (p.allFinite() && !inGap(p.x(), p.y(), gap)) {
        q = transform * p;
      }
      writeAt(dst, 0, q.x());
      writeAt(dst, 4, q.y());
      writeAt(dst, 8, q.z());
      writeAt(dst, 12, intensity_offset >= 0 ? readAt<float>(src, intensity_offset) : 0.0f);
      writeAt(dst, 16, ring_offset >= 0 ? readAt<uint16_t>(src, ring_offset) : uint16_t{0});
      writeAt(dst, 18, stamp);
    }
  }
  return true;
}

}  // namespace legged
