#include "TeleopDriveController.h"
#include "DriveKinematics.h"
#include "LiftTeleopController.h"
#include "ServoTeleopController.h"

#include <limits>

// Evaluate the production controller and motor mixer with the project's ARM
// GCC compiler; no hardware or executable is needed for these static checks.
namespace {
constexpr double kForwardRawY=localization_config::DRIVER_LEFT_Y_SIGN;
constexpr double kLeftRawX=-localization_config::DRIVER_LEFT_X_SIGN;
static_assert(localization_config::WHEEL_TITAN_PORT[0]==0 &&
              manipulator_config::TITAN_MOTOR_PORT==1 &&
              localization_config::WHEEL_TITAN_PORT[1]==2 &&
              localization_config::WHEEL_TITAN_PORT[2]==3,
              "Titan labels: left rear M0, lift M1, right rear M2, front M3");

constexpr bool LiftCannotRunDuringDriveOrResumeFromHeldDpad() {
  LiftTeleopController lift;
  lift.Update(true,true,false,false,false,false,true); // arm at rest
  lift.Update(true,true,true,false,false,false,true);
  if (lift.Output()!=manipulator_config::LIFT_OUTPUT*manipulator_config::LIFT_UP_SIGN) return false;
  lift.Update(true,true,true,false,false,false,false); // driving interrupts lift
  if (lift.Output()!=0) return false;
  lift.Update(true,true,true,false,false,false,true); // drive stops, D-pad still held
  if (lift.Output()!=0) return false;
  lift.Update(true,true,false,false,false,false,true); // release, then press down
  lift.Update(true,true,false,true,false,false,true);
  if (lift.Output()!=-manipulator_config::LIFT_OUTPUT*manipulator_config::LIFT_UP_SIGN) return false;
  lift.Update(true,true,false,false,false,false,true);
  if (lift.Output()!=0) return false;
  lift.Update(false,true,true,false,false,false,true); // disabled
  if (lift.Output()!=0) return false;
  lift.Update(true,true,true,false,false,false,true); // held across enable
  return lift.Output()==0;
}
static_assert(LiftCannotRunDuringDriveOrResumeFromHeldDpad(),
              "Driving must stop lift; release D-pad before restarting after drive or disable");

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

constexpr bool TranslationReversedWithoutReversingYaw() {
  const auto up=Settled(0,-1,false,false);
  const auto down=Settled(0,1,false,false);
  const auto left=Settled(-1,0,false,false);
  const auto right=Settled(1,0,false,false);
  return up.GetForwardMMps()<0 && down.GetForwardMMps()>0 &&
      left.GetLeftMMps()<0 && right.GetLeftMMps()>0 &&
      Settled(0,0,true,false).GetYawRateRadps()<0 &&
      Settled(0,0,false,true).GetYawRateRadps()>0;
}
static_assert(TranslationReversedWithoutReversingYaw(),
              "Installed-robot correction reverses both stick axes and preserves L2/R2");

constexpr bool ServoToggleEdgesAndModeChanges() {
  ServoTeleopController servos;
  servos.Update(false,true,true,true);
  if (servos.OutputEnabled(0) || servos.OutputEnabled(1)) return false;
  servos.Update(true,true,true,true); // held while enabling must not actuate
  if (servos.OutputEnabled(0) || servos.TargetDegrees(0)!=0 || servos.TargetDegrees(1)!=0) return false;
  servos.Update(true,true,false,false); // release to arm both
  if (!servos.OutputEnabled(0) || !servos.OutputEnabled(1)) return false;
  servos.Update(true,true,true,false); // X changes only FlexDIO 8
  if (servos.TargetDegrees(0)!=100 || servos.TargetDegrees(1)!=0) return false;
  for (int i=0;i<20;++i) servos.Update(true,true,true,false);
  if (servos.TargetDegrees(0)!=100) return false;
  servos.Update(true,true,true,true); // triangle changes only FlexDIO 9
  if (servos.TargetDegrees(0)!=100 || servos.TargetDegrees(1)!=180) return false;
  servos.Update(true,true,false,false);
  servos.Update(true,true,true,false); // second X press returns to 0
  if (servos.TargetDegrees(0)!=0 || servos.TargetDegrees(1)!=180) return false;
  servos.Update(true,false,false,false); // missing controller inputs
  if (servos.OutputEnabled(0) || servos.OutputEnabled(1)) return false;
  servos.Update(true,true,false,true); // triangle held during reconnect: no toggle
  if (servos.OutputEnabled(1) || servos.TargetDegrees(1)!=180) return false;
  servos.Update(true,true,false,false);
  servos.Update(true,true,false,true); // fresh second triangle press returns to 0
  if (servos.TargetDegrees(1)!=0) return false;
  servos.Disarm();
  return !servos.OutputEnabled(0) && !servos.OutputEnabled(1);
}
static_assert(ServoToggleEdgesAndModeChanges(),
              "Servo buttons toggle independently once per press, with no queued press across enable/disconnect");
static_assert(manipulator_config::SERVO_PWM[0]==18 && manipulator_config::SERVO_PWM[1]==19,
              "Physical FlexDIO 8 and 9 use VMX WPI PWM 18 and 19");

static_assert(localization_config::MAX_WHEEL_SPEED_MMPS > 598.15 &&
              localization_config::MAX_WHEEL_SPEED_MMPS < 598.17,
              "112 gearbox-output RPM and 102 mm diameter must give 598.16 mm/s");

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
  const auto ccw = Settled(0.0, 0.0, false, true);
  const auto cw = Settled(0.0, 0.0, true, false);
  const double limit = localization_config::TELEOP_MAX_YAW_RATE_DPS *
      localization_config::PI / 180.0;
  return ccw.GetForwardMMps() == 0.0 && cw.GetLeftMMps() == 0.0 &&
      Near(ccw.GetYawRateRadps(), limit) && Near(cw.GetYawRateRadps(), -limit);
}
static_assert(TriggerDirections(), "L2 requests CW and R2 requests CCW with a centered stick");

constexpr bool BothTriggersCancelImmediately() {
  auto controller = Settled(0.0, 0.0, true, false);
  controller.Update(true, true, 0.0, 0.0, true, true, 0.02);
  return controller.IsArmed() && Stopped(controller);
}
static_assert(BothTriggersCancelImmediately(), "Both triggers must cancel rotation immediately");

constexpr bool CardinalAndDiagonalTranslation() {
  const double limit = localization_config::TELEOP_SPEED_MMPS;
  const auto forward = Settled(0.0, kForwardRawY, false, false);
  const auto backward = Settled(0.0, -kForwardRawY, false, false);
  const auto left = Settled(kLeftRawX, 0.0, false, false);
  const auto right = Settled(-kLeftRawX, 0.0, false, false);
  const auto diagonal = Settled(-kLeftRawX, kForwardRawY, false, false);
  return Near(forward.GetForwardMMps(), limit) &&
      Near(backward.GetForwardMMps(), -limit) &&
      Near(left.GetLeftMMps(), limit) && Near(right.GetLeftMMps(), -limit) &&
      diagonal.GetForwardMMps() > 0.0 && diagonal.GetLeftMMps() < 0.0 &&
      Near(std::hypot(diagonal.GetForwardMMps(), diagonal.GetLeftMMps()), limit);
}
static_assert(CardinalAndDiagonalTranslation(), "Cardinal signs and diagonal speed limit must hold");

constexpr bool SimultaneousTranslationAndRotation() {
  const auto cw = Settled(kLeftRawX, kForwardRawY, true, false);
  const auto ccw = Settled(kLeftRawX, kForwardRawY, false, true);
  return cw.GetForwardMMps() > 0.0 && cw.GetLeftMMps() > 0.0 &&
      cw.GetYawRateRadps() < 0.0 && ccw.GetForwardMMps() > 0.0 &&
      ccw.GetLeftMMps() > 0.0 && ccw.GetYawRateRadps() > 0.0;
}
static_assert(SimultaneousTranslationAndRotation(), "Translation and trigger rotation must combine");

constexpr bool ReleaseTranslationPreservesRotation() {
  auto controller = Settled(0.0, kForwardRawY, true, false);
  controller.Update(true, true, 0.0, 0.0, true, false, 0.02);
  return controller.GetForwardMMps() == 0.0 && controller.GetLeftMMps() == 0.0 &&
      controller.GetYawRateRadps() < 0.0;
}
static_assert(ReleaseTranslationPreservesRotation(), "Centering the stick must allow pure rotation");

constexpr bool ReleaseRotationPreservesTranslation() {
  auto controller = Settled(0.0, kForwardRawY, true, false);
  controller.Update(true, true, 0.0, kForwardRawY, false, false, 0.02);
  return controller.GetForwardMMps() > 0.0 && controller.GetYawRateRadps() == 0.0;
}
static_assert(ReleaseRotationPreservesTranslation(), "Trigger release stops yaw independently of drive");

constexpr bool DisableAndDisconnectStop() {
  auto controller = Settled(0.0, kForwardRawY, true, false);
  controller.Update(false, true, 0.0, kForwardRawY, true, false, 0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true, true, 0.0, kForwardRawY, true, false, 0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true, true, 0.0, 0.0, false, false, 0.02);
  controller.Update(true, true, 0.0, kForwardRawY, true, false, 0.02);
  controller.Update(true, false, 0.0, kForwardRawY, true, false, 0.02);
  return !controller.IsArmed() && Stopped(controller);
}
static_assert(DisableAndDisconnectStop(), "Disable/disconnect must stop every component and disarm");

constexpr bool InvalidInputsStop() {
  auto controller = Settled(0.0, kForwardRawY, true, false);
  controller.Update(true, true, std::numeric_limits<double>::infinity(),
                    -1.0, true, false, 0.02);
  return !controller.IsArmed() && Stopped(controller);
}
static_assert(InvalidInputsStop(), "Invalid input must not reach motor output");

constexpr bool LongLoopCannotSkipRamp() {
  auto controller = Armed();
  controller.Update(true, true, 0.0, kForwardRawY, true, false, 10.0);
  return Near(controller.GetForwardMMps(),
              localization_config::TELEOP_ACCELERATION_MMPS2 * 0.050) &&
      Near(controller.GetYawRateRadps(),
           -localization_config::TELEOP_YAW_ACCELERATION_DPS2 *
           localization_config::PI / 180.0 * 0.050);
}
static_assert(LongLoopCannotSkipRamp(), "Ramp integration must cap a delayed loop to 50 ms");

constexpr bool DriveAndYawUseSeparateFrames() {
  const auto forward = Settled(0.0, kForwardRawY, false, false);
  const auto ccw = Settled(0.0, 0.0, false, true);
  const auto cw = Settled(0.0, 0.0, true, false);
  if (!Near(forward.GetLayoutVxMMps(), localization_config::TELEOP_SPEED_MMPS) ||
      !Near(forward.GetLayoutVyMMps(), 0.0)) return false;
  for (int wheel = 0; wheel < 3; ++wheel) {
    if (drive_kinematics::WheelSpeedMMps(wheel, 0.0, 0.0,
            ccw.GetYawRateRadps()) <= 0.0) return false;
    if (drive_kinematics::WheelSpeedMMps(wheel, 0.0, 0.0,
            cw.GetYawRateRadps()) >= 0.0) return false;
  }
  return true;
}
static_assert(DriveAndYawUseSeparateFrames(), "Body-aligned translation must preserve CCW yaw mixing");

constexpr bool MixedOutputsRespectLimit() {
  const auto controller = Settled(kLeftRawX, kForwardRawY, true, false);
  double wheelSpeed[3]{};
  for (int wheel = 0; wheel < 3; ++wheel)
    wheelSpeed[wheel] = drive_kinematics::WheelSpeedMMps(wheel,
        controller.GetLayoutVxMMps(), controller.GetLayoutVyMMps(),
        controller.GetYawRateRadps());
  const double denominator = drive_kinematics::OutputDenominator(
      wheelSpeed, localization_config::TELEOP_MAX_MOTOR_OUTPUT);
  if (denominator <= 0.0) return false;
  for (int wheel = 0; wheel < 3; ++wheel)
    if (Abs(wheelSpeed[wheel] / denominator) >
        localization_config::TELEOP_MAX_MOTOR_OUTPUT + 1.0e-7) return false;
  const double lowerLimit = drive_kinematics::OutputDenominator(wheelSpeed, 0.25);
  for (int wheel = 0; wheel < 3; ++wheel)
    if (Abs(wheelSpeed[wheel] / lowerLimit) > 0.2500001) return false;
  return lowerLimit > denominator;
}
static_assert(MixedOutputsRespectLimit(), "Mixed translation/yaw must preserve a common output limit");

constexpr bool FullStickHasMoreOutput() {
  const auto forward = Settled(0.0, kForwardRawY, false, false);
  double speeds[3]{};
  for (int wheel = 0; wheel < 3; ++wheel)
    speeds[wheel] = drive_kinematics::WheelSpeedMMps(wheel,
        forward.GetLayoutVxMMps(), forward.GetLayoutVyMMps(), 0.0);
  const double denominator = drive_kinematics::OutputDenominator(
      speeds, localization_config::TELEOP_MAX_MOTOR_OUTPUT);
  return speeds[0] / denominator < -0.86 && speeds[1] / denominator > 0.86 &&
      Near(speeds[2] / denominator, 0.0);
}
static_assert(FullStickHasMoreOutput(), "Full forward: left rear negative, right rear positive, front idle");

constexpr double kBadSpeeds[3]{1.0, std::numeric_limits<double>::infinity(), 1.0};
static_assert(drive_kinematics::OutputDenominator(kBadSpeeds, 1.0) == 0.0,
              "Invalid wheel speed must reject the whole output vector");
}  // namespace
