#pragma once

#include <cmath>

#include "LocalizationConstants.h"

// Pure manual-drive logic, shared by the robot and compile-time regression
// checks. Inputs are Driver Station values; outputs are speed requests.
class TeleopDriveController {
 public:
  constexpr void Update(bool motionAllowed, bool connected, double rawX,
                        double rawY, bool l2, bool r2, double dt) {
    if (!connected) {
      Stop("Controller inputs missing - check connection/axes/buttons", true);
      return;
    }
    if (!Finite(rawX) || !Finite(rawY) || Abs(rawX) > 1.01 ||
        Abs(rawY) > 1.01 || !Finite(dt) || dt < 0.0) {
      Stop("Invalid controller input or timing", true);
      return;
    }
    if (!motionAllowed) {
      Stop("Disabled - select and enable Teleoperated mode", true);
      return;
    }
    const double deadband = localization_config::DRIVER_STICK_DEADBAND;
    if (!(deadband >= 0.0 && deadband < 1.0) ||
        !Finite(localization_config::TELEOP_FORWARD_DIRECTION_DEG) ||
        !Positive(localization_config::TELEOP_SPEED_MMPS) ||
        !Positive(localization_config::TELEOP_ACCELERATION_MMPS2) ||
        !Positive(localization_config::TELEOP_MAX_MOTOR_OUTPUT) ||
        localization_config::TELEOP_MAX_MOTOR_OUTPUT > 1.0 ||
        !Positive(localization_config::TELEOP_MAX_YAW_RATE_DPS) ||
        !Positive(localization_config::TELEOP_YAW_ACCELERATION_DPS2) ||
        Abs(localization_config::TELEOP_YAW_SIGN) != 1.0 ||
        Abs(localization_config::DRIVER_LEFT_X_SIGN) != 1.0 ||
        Abs(localization_config::DRIVER_LEFT_Y_SIGN) != 1.0) {
      Stop("Invalid teleop configuration", true);
      return;
    }
    const double magnitude = std::hypot(rawX, rawY);
    if (!armed_) {
      if (magnitude > deadband || l2 || r2) {
        Stop("Center left stick and release L2/R2 to arm", true);
        return;
      }
      armed_ = true;
    }
    const double seconds = Clamp(dt, 0.0, 0.050);
    if (magnitude <= deadband) {
      // Neutral translation stops immediately even if a trigger remains held.
      forward_MMps_ = 0.0;
      left_MMps_ = 0.0;
    } else {
      const double speed = (Min(1.0, magnitude) - deadband) /
          (1.0 - deadband) * localization_config::TELEOP_SPEED_MMPS;
      const double targetForward = rawY *
          localization_config::DRIVER_LEFT_Y_SIGN / magnitude * speed;
      const double targetLeft = -rawX *
          localization_config::DRIVER_LEFT_X_SIGN / magnitude * speed;
      const double deltaForward = targetForward - forward_MMps_;
      const double deltaLeft = targetLeft - left_MMps_;
      const double delta = std::hypot(deltaForward, deltaLeft);
      const double step = localization_config::TELEOP_ACCELERATION_MMPS2 * seconds;
      const double fraction = delta > step && delta > 0.0 ? step / delta : 1.0;
      forward_MMps_ += deltaForward * fraction;
      left_MMps_ += deltaLeft * fraction;
    }
    // Independent digital triggers: L2 = CW, R2 = CCW; both cancel. Releasing
    // rotation commands zero immediately, independently of translation.
    const int rotation = (r2 ? 1 : 0) - (l2 ? 1 : 0);
    if (rotation == 0) {
      yawRate_Radps_ = 0.0;
    } else {
      const double targetYaw = rotation * localization_config::TELEOP_YAW_SIGN *
          localization_config::TELEOP_MAX_YAW_RATE_DPS *
          localization_config::PI / 180.0;
      const double step = localization_config::TELEOP_YAW_ACCELERATION_DPS2 *
          localization_config::PI / 180.0 * seconds;
      yawRate_Radps_ += Clamp(targetYaw - yawRate_Radps_, -step, step);
    }
    const bool translating = magnitude > deadband;
    status_ = rotation != 0
        ? (translating ? "Driving and rotating" : "Rotating - L2 CW / R2 CCW")
        : (translating ? "Driving - left stick translation"
                       : (l2 && r2 ? "L2 + R2 - rotation cancelled"
                                   : "Ready - left stick / L2 / R2"));
  }

  constexpr bool IsArmed() const { return armed_; }
  constexpr double GetForwardMMps() const { return forward_MMps_; }
  constexpr double GetLeftMMps() const { return left_MMps_; }
  constexpr double GetYawRateRadps() const { return yawRate_Radps_; }
  constexpr double GetLayoutVxMMps() const {
    const double direction = localization_config::TELEOP_FORWARD_DIRECTION_DEG *
        localization_config::PI / 180.0;
    return std::cos(direction) * forward_MMps_ -
           std::sin(direction) * left_MMps_;
  }
  constexpr double GetLayoutVyMMps() const {
    const double direction = localization_config::TELEOP_FORWARD_DIRECTION_DEG *
        localization_config::PI / 180.0;
    return std::sin(direction) * forward_MMps_ +
           std::cos(direction) * left_MMps_;
  }
  constexpr const char* GetStatus() const { return status_; }

 private:
  static constexpr double Abs(double x) { return x < 0.0 ? -x : x; }
  static constexpr double Min(double a, double b) { return a < b ? a : b; }
  static constexpr double Max(double a, double b) { return a > b ? a : b; }
  static constexpr double Clamp(double x, double low, double high) {
    return Min(high, Max(low, x));
  }
  static constexpr bool Finite(double x) { return x >= -1.0e15 && x <= 1.0e15; }
  static constexpr bool Positive(double x) { return Finite(x) && x > 0.0; }
  constexpr void Stop(const char* status, bool disarm) {
    forward_MMps_ = left_MMps_ = yawRate_Radps_ = 0.0;
    if (disarm) armed_ = false;
    status_ = status;
  }

  bool armed_ = false;
  double forward_MMps_ = 0.0;
  double left_MMps_ = 0.0;
  double yawRate_Radps_ = 0.0;
  const char* status_ = "Waiting for Control Station";
};
