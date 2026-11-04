#pragma once

#include <cmath>

// Shared units, geometry and sensor/action data.
struct RobotPose { double x_MM; double y_MM; double headingRad; };
struct ArenaPoint { double x; double y; };

namespace robot_geometry {
static constexpr double PI = 3.14159265358979323846;
inline double Wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double Radians(double degrees) { return degrees * PI / 180.0; }
inline double Distance(ArenaPoint a, ArenaPoint b) { return std::hypot(a.x-b.x, a.y-b.y); }
inline bool Finite(const RobotPose& p) {
  return std::isfinite(p.x_MM) && std::isfinite(p.y_MM) && std::isfinite(p.headingRad);
}
}

struct LineReading {
  bool valid = false;
  double leftErrorMM = 0.0;
  double strength = 0.0;
  double volts[4] = {-1,-1,-1,-1};
};

enum class ObjectActionStatus { Unavailable, Busy, Complete, Failed };

// Future autonomous object handling must report physically confirmed success.
