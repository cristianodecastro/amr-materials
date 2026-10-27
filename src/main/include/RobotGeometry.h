#pragma once

#include <cmath>
#include "LocalizationConstants.h"

struct RobotPose { double x_MM; double y_MM; double headingRad; };
struct ArenaPoint { double x; double y; };

namespace robot_geometry {
inline double Wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double Radians(double degrees) { return degrees * localization_config::PI / 180.0; }
inline double Distance(ArenaPoint a, ArenaPoint b) { return std::hypot(a.x-b.x, a.y-b.y); }
inline bool Finite(const RobotPose& p) {
  return std::isfinite(p.x_MM) && std::isfinite(p.y_MM) && std::isfinite(p.headingRad);
}
}
