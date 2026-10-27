#pragma once

#include <cmath>

#include "LocalizationConstants.h"

namespace drive_kinematics {

inline constexpr double LayoutVx(double forward, double left) {
  const double a=localization_config::DRIVE_FORWARD_DIRECTION_DEG*localization_config::PI/180.0;
  return std::cos(a)*forward-std::sin(a)*left;
}
inline constexpr double LayoutVy(double forward, double left) {
  const double a=localization_config::DRIVE_FORWARD_DIRECTION_DEG*localization_config::PI/180.0;
  return std::sin(a)*forward+std::cos(a)*left;
}
inline constexpr double BodyForward(double layoutVx, double layoutVy) {
  const double a=localization_config::DRIVE_FORWARD_DIRECTION_DEG*localization_config::PI/180.0;
  return std::cos(a)*layoutVx+std::sin(a)*layoutVy;
}
inline constexpr double BodyLeft(double layoutVx, double layoutVy) {
  const double a=localization_config::DRIVE_FORWARD_DIRECTION_DEG*localization_config::PI/180.0;
  return -std::sin(a)*layoutVx+std::cos(a)*layoutVy;
}

inline constexpr double WheelSpeedMMps(int wheel, double vx_MMps, double vy_MMps,
                             double yawRate_Radps) {
  const double alpha = localization_config::WHEEL_ANGLE_DEG[wheel] *
                       localization_config::PI / 180.0;
  return -std::sin(alpha) * vx_MMps + std::cos(alpha) * vy_MMps +
         yawRate_Radps * localization_config::WHEEL_CENTER_RADIUS_MM[wheel];
}

inline constexpr double TravelCoefficient(int wheel, double directionDeg) {
  const double direction = directionDeg * localization_config::PI / 180.0;
  const double coefficient = WheelSpeedMMps(
      wheel, std::cos(direction), std::sin(direction), 0.0);
  // A wheel perpendicular to the travel vector must be exactly idle.
  return std::fabs(coefficient) < 1.0e-10 ? 0.0 : coefficient;
}

// All wheel requests share one denominator, preserving the combined
// translation/rotation vector when any motor saturates. Zero rejects input.
inline constexpr double OutputDenominator(const double (&wheelSpeed)[3],
                                           double maxOutput) {
  if (!std::isfinite(maxOutput) || maxOutput <= 0.0 ||
      !std::isfinite(localization_config::MAX_WHEEL_SPEED_MMPS) ||
      localization_config::MAX_WHEEL_SPEED_MMPS <= 0.0) return 0.0;
  if (maxOutput > 1.0) maxOutput = 1.0;
  double largest = localization_config::MAX_WHEEL_SPEED_MMPS;
  for (int wheel = 0; wheel < 3; ++wheel) {
    if (!std::isfinite(wheelSpeed[wheel])) return 0.0;
    const double required = std::fabs(wheelSpeed[wheel]) / maxOutput;
    if (required > largest) largest = required;
  }
  return largest;
}

}  // namespace drive_kinematics
