#pragma once
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>



namespace legged {

// Analytic IK of one leg. p: foot in the base frame, sx: +1 front / -1 hind, sy: +1 left / -1 right.
// Returns [HAA, HFE, KFE]
inline Eigen::Vector3d legIk(const Eigen::Vector3d& p, double sx, double sy, double hip_x, double hip_y, double thigh_offset, double L) {
    const double x = p.x() - sx * hip_x;
    const double y = p.y() - sy * hip_y;
    const double z = p.z();
    const double d = sy * thigh_offset;

    const double l2 = std::max(y * y + z * z - d * d, 1e-6);
    const double lz = -std::sqrt(l2);
    const double c = std::clamp((x * x + l2) / (2.0 * L * L) - 1.0, -1.0, 1.0);

    const double kfe = -std::acos(c);
    const double a = std::atan2(z, y) - std::atan2(lz, d);
    const double haa = std::atan2(std::sin(a), std::cos(a));   // wrap to [-pi, pi]
    const double hfe = std::atan2(-x, -lz) - kfe / 2.0;
    return {haa, hfe, kfe};
}


} // namespace legged