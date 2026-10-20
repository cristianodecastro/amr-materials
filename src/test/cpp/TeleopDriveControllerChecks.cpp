#include "TeleopDriveController.h"
#include "DriveKinematics.h"

#include <limits>

// Evaluate the production controller and motor mixer with the project's ARM
// GCC compiler; no hardware or executable is needed for these static checks.
namespace {
constexpr double Abs(double x) { return x < 0.0 ? -x : x; }
constexpr bool Near(double a, double b) { return Abs(a - b) < 1.0e-7; }
constexpr bool Stopped(const TeleopDriveController& controller) {
  return controller.GetForwardMMps() == 0.0 &&
      controller.GetLeftMMps() == 0.0 && controller.GetYawRateRadps() == 0.0;
}
constexpr TeleopDriveController Armed() {
  TeleopDriveController controller;
  controller.Update(true, true, 0.0, 0.0, false, false, 0.02);
  return controller;
}
constexpr TeleopDriveController Settled(double x, double y, bool l2, bool r2) {
  auto controller = Armed();
  for (int tick = 0; tick < 100; ++tick)
    controller.Update(true, true, x, y, l2, r2, 0.02);
  return controller;
}

static_assert(localization_config::MAX_WHEEL_SPEED_MMPS > 598.15 &&
              localization_config::MAX_WHEEL_SPEED_MMPS < 598.17,
              "112 gearbox-output RPM and 102 mm diameter must give 598.16 mm/s");
static_assert(localization_config::TELEOP_MAX_MOTOR_OUTPUT == 1.0,
              "Manual drive must no longer be limited to 25 percent");

constexpr bool HeldControlsCannotArm() {
  TeleopDriveController controller;
  controller.Update(true, true, 0.0, 0.0, true, false, 0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true, true, 0.0, 0.0, false, true, 0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true, true, 0.0, 0.0, true, true, 0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true, true, 1.0, 0.0, false, false, 0.02);
  return !controller.IsArmed() && Stopped(controller);
}
static_assert(HeldControlsCannotArm(), "Neutral stick and released triggers are required to arm");

constexpr bool DeadbandDoesNotCreep() {
  auto controller = Armed();
  for (int tick = 0; tick < 100; ++tick)
    controller.Update(true, true, 0.05, -0.05, false, false, 0.02);
  return controller.IsArmed() && Stopped(controller);
}
static_assert(DeadbandDoesNotCreep(), "Neutral stick noise must not accumulate full-output motion");

constexpr bool TriggerDirections() {
  const auto ccw = Settled(0.0, 0.0, true, false);
  const auto cw = Settled(0.0, 0.0, false, true);
  const double limit = localization_config::TELEOP_MAX_YAW_RATE_DPS *
      localization_config::PI / 180.0;
  return ccw.GetForwardMMps() == 0.0 && cw.GetLeftMMps() == 0.0 &&
      Near(ccw.GetYawRateRadps(), limit) && Near(cw.GetYawRateRadps(), -limit);
}
static_assert(TriggerDirections(), "L2 requests CCW and R2 requests CW with a centered stick");

constexpr bool BothTriggersCancelImmediately() {
  auto controller = Settled(0.0, 0.0, true, false);
  controller.Update(true, true, 0.0, 0.0, true, true, 0.02);
  return controller.IsArmed() && Stopped(controller);
}
static_assert(BothTriggersCancelImmediately(), "Both triggers must cancel rotation immediately");

constexpr bool CardinalAndDiagonalTranslation() {
  const double limit = localization_config::TELEOP_SPEED_MMPS;
  const auto forward = Settled(0.0, -1.0, false, false);
  const auto backward = Settled(0.0, 1.0, false, false);
  const auto left = Settled(-1.0, 0.0, false, false);
  const auto right = Settled(1.0, 0.0, false, false);
  const auto diagonal = Settled(1.0, -1.0, false, false);
  return Near(forward.GetForwardMMps(), limit) &&
      Near(backward.GetForwardMMps(), -limit) &&
      Near(left.GetLeftMMps(), limit) && Near(right.GetLeftMMps(), -limit) &&
      diagonal.GetForwardMMps() > 0.0 && diagonal.GetLeftMMps() < 0.0 &&
      Near(std::hypot(diagonal.GetForwardMMps(), diagonal.GetLeftMMps()), limit);
}
static_assert(CardinalAndDiagonalTranslation(), "Cardinal signs and diagonal speed limit must hold");

constexpr bool SimultaneousTranslationAndRotation() {
  const auto controller = Settled(-1.0, -1.0, true, false);
  return controller.GetForwardMMps() > 0.0 && controller.GetLeftMMps() > 0.0 &&
      controller.GetYawRateRadps() > 0.0;
}
static_assert(SimultaneousTranslationAndRotation(), "Translation and trigger rotation must combine");

constexpr bool ReleaseTranslationPreservesRotation() {
  auto controller = Settled(0.0, -1.0, true, false);
  controller.Update(true, true, 0.0, 0.0, true, false, 0.02);
  return controller.GetForwardMMps() == 0.0 && controller.GetLeftMMps() == 0.0 &&
      controller.GetYawRateRadps() > 0.0;
}
static_assert(ReleaseTranslationPreservesRotation(), "Centering the stick must allow pure rotation");

constexpr bool ReleaseRotationPreservesTranslation() {
  auto controller = Settled(0.0, -1.0, true, false);
  controller.Update(true, true, 0.0, -1.0, false, false, 0.02);
  return controller.GetForwardMMps() > 0.0 && controller.GetYawRateRadps() == 0.0;
}
static_assert(ReleaseRotationPreservesTranslation(), "Trigger release stops yaw independently of drive");

constexpr bool DisableAndDisconnectStop() {
  auto controller = Settled(0.0, -1.0, true, false);
  controller.Update(false, true, 0.0, -1.0, true, false, 0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true, true, 0.0, -1.0, true, false, 0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true, true, 0.0, 0.0, false, false, 0.02);
  controller.Update(true, true, 0.0, -1.0, true, false, 0.02);
  controller.Update(true, false, 0.0, -1.0, true, false, 0.02);
  return !controller.IsArmed() && Stopped(controller);
}
static_assert(DisableAndDisconnectStop(), "Disable/disconnect must stop every component and disarm");

constexpr bool InvalidInputsStop() {
  auto controller = Settled(0.0, -1.0, true, false);
  controller.Update(true, true, std::numeric_limits<double>::infinity(),
                    -1.0, true, false, 0.02);
  return !controller.IsArmed() && Stopped(controller);
}
static_assert(InvalidInputsStop(), "Invalid input must not reach motor output");

constexpr bool LongLoopCannotSkipRamp() {
  auto controller = Armed();
  controller.Update(true, true, 0.0, -1.0, true, false, 10.0);
  return Near(controller.GetForwardMMps(),
              localization_config::TELEOP_ACCELERATION_MMPS2 * 0.050) &&
      Near(controller.GetYawRateRadps(),
           localization_config::TELEOP_YAW_ACCELERATION_DPS2 *
           localization_config::PI / 180.0 * 0.050);
}
static_assert(LongLoopCannotSkipRamp(), "Ramp integration must cap a delayed loop to 50 ms");

constexpr bool DriveAndYawUseSeparateFrames() {
  const auto forward = Settled(0.0, -1.0, false, false);
  const auto ccw = Settled(0.0, 0.0, true, false);
  const auto cw = Settled(0.0, 0.0, false, true);
  if (!Near(forward.GetLayoutVxMMps(), 0.0) ||
      forward.GetLayoutVyMMps() >= 0.0) return false;
  for (int wheel = 0; wheel < 3; ++wheel) {
    if (drive_kinematics::WheelSpeedMMps(wheel, 0.0, 0.0,
            ccw.GetYawRateRadps()) <= 0.0) return false;
    if (drive_kinematics::WheelSpeedMMps(wheel, 0.0, 0.0,
            cw.GetYawRateRadps()) >= 0.0) return false;
  }
  return true;
}
static_assert(DriveAndYawUseSeparateFrames(), "Translation offset must not reverse CCW yaw mixing");

constexpr bool MixedOutputsRespectLimit() {
  const auto controller = Settled(-1.0, -1.0, true, false);
  double wheelSpeed[3]{};
  for (int wheel = 0; wheel < 3; ++wheel)
    wheelSpeed[wheel] = drive_kinematics::WheelSpeedMMps(wheel,
        controller.GetLayoutVxMMps(), controller.GetLayoutVyMMps(),
        controller.GetYawRateRadps());
  const double denominator = drive_kinematics::OutputDenominator(wheelSpeed, 1.0);
  if (denominator <= 0.0) return false;
  for (int wheel = 0; wheel < 3; ++wheel)
    if (Abs(wheelSpeed[wheel] / denominator) > 1.0000001) return false;
  const double lowerLimit = drive_kinematics::OutputDenominator(wheelSpeed, 0.25);
  for (int wheel = 0; wheel < 3; ++wheel)
    if (Abs(wheelSpeed[wheel] / lowerLimit) > 0.2500001) return false;
  return lowerLimit > denominator;
}
static_assert(MixedOutputsRespectLimit(), "Mixed translation/yaw must preserve a common output limit");

constexpr bool FullStickHasMoreOutput() {
  const auto forward = Settled(0.0, -1.0, false, false);
  double speeds[3]{};
  for (int wheel = 0; wheel < 3; ++wheel)
    speeds[wheel] = drive_kinematics::WheelSpeedMMps(wheel,
        forward.GetLayoutVxMMps(), forward.GetLayoutVyMMps(), 0.0);
  const double denominator = drive_kinematics::OutputDenominator(speeds, 1.0);
  return speeds[0] / denominator > 0.86 && speeds[1] / denominator < -0.86 &&
      Near(speeds[2] / denominator, 0.0);
}
static_assert(FullStickHasMoreOutput(), "Full forward should reach about 86.6 percent on M0/M1");

constexpr double kBadSpeeds[3]{1.0, std::numeric_limits<double>::infinity(), 1.0};
static_assert(drive_kinematics::OutputDenominator(kBadSpeeds, 1.0) == 0.0,
              "Invalid wheel speed must reject the whole output vector");
}  // namespace
