#pragma once

#include <cmath>

// Shared units, geometry and sensor readings.
struct RobotPose { double x_MM; double y_MM; double headingRad; };
struct ArenaPoint { double x; double y; };
struct ForbiddenRegion { double xmin,ymin,xmax,ymax; };

namespace robot_geometry {
static constexpr double PI = 3.14159265358979323846;
inline double Wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double Radians(double degrees) { return degrees * PI / 180.0; }
// Arena yaw zero is NORTH; Cartesian trigonometry zero is east.
// Body +X remains robot-forward. Positive yaw remains CCW in both frames.
inline double ArenaAngle(double headingRad) { return Wrap(headingRad+PI/2); }
inline bool Finite(const RobotPose& p) {
  return std::isfinite(p.x_MM) && std::isfinite(p.y_MM) && std::isfinite(p.headingRad);
}
}

struct IrReading {
  double voltage=-1,distanceMM=-1;
  bool valid=false; // plausible in-range measurement, not proof of a connected sensor
};

struct LineReading {
  double volts[4] = {-1,-1,-1,-1};
  bool communicationValid = false; // a fresh I2C read completed successfully
};
