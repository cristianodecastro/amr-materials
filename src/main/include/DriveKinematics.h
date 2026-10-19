#pragma once

#include <cmath>

#include "LocalizationConstants.h"

namespace drive_kinematics {

inline double WheelSpeedMMps(int wheel, double vx_MMps, double vy_MMps,
                             double yawRate_Radps) {
  const double alpha = localization_config::WHEEL_ANGLE_DEG[wheel] *
                       localization_config::PI / 180.0;
  return -std::sin(alpha) * vx_MMps + std::cos(alpha) * vy_MMps +
         yawRate_Radps * localization_config::WHEEL_CENTER_RADIUS_MM[wheel];
}

inline double TravelCoefficient(int wheel, double directionDeg) {
  const double direction = directionDeg * localization_config::PI / 180.0;
  const double coefficient = WheelSpeedMMps(
      wheel, std::cos(direction), std::sin(direction), 0.0);
  // A wheel perpendicular to the travel vector must be exactly idle.
  return std::fabs(coefficient) < 1.0e-10 ? 0.0 : coefficient;
}

}  // namespace drive_kinematics
