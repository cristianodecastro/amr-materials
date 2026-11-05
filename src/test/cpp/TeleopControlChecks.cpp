#include "TeleopControl.h"
#include <limits>

// Evaluate the production controller and motor mixer with the project's ARM
// GCC compiler; no hardware or executable is needed for these static checks.
namespace teleop_checks {
constexpr double kForwardRawY=localization_config::DRIVER_LEFT_Y_SIGN;
constexpr double kLeftRawX=-localization_config::DRIVER_LEFT_X_SIGN;
constexpr double kCwRawX=localization_config::DRIVER_RIGHT_X_SIGN;
static_assert(localization_config::WHEEL_TITAN_PORT[0]==0 &&
              manipulator_config::TITAN_MOTOR_PORT==1 &&
              localization_config::WHEEL_TITAN_PORT[1]==2 &&
              localization_config::WHEEL_TITAN_PORT[2]==3,
              "Titan labels: left rear M0, lift M1, right rear M2, front M3");

constexpr bool LiftRunsIndependentlyAndDisarmsOnDisable() {
  LiftTeleopController lift;
  TeleopDriveController drive;
  drive.Update(true,true,0,0,0,0.02);
  lift.Update(true,true,false,false,false,false); // arm at rest
  lift.Update(true,true,true,false,false,false);
  if (lift.Output()!=manipulator_config::LIFT_OUTPUT*manipulator_config::LIFT_UP_SIGN) return false;
  drive.Update(true,true,0,kForwardRawY,kCwRawX,0.02,false);
  lift.Update(true,true,true,false,false,false); // driving and rotating do not interrupt lift
  if (drive.GetForwardMMps()==0 || drive.GetYawRateRadps()==0 ||
      lift.Output()!=manipulator_config::LIFT_OUTPUT*manipulator_config::LIFT_UP_SIGN) return false;
  lift.Update(true,true,false,false,false,false); // release, then press down
  lift.Update(true,true,false,true,false,false);
  if (lift.Output()!=-manipulator_config::LIFT_OUTPUT*manipulator_config::LIFT_UP_SIGN) return false;
  lift.Update(true,true,false,false,false,false);
  if (lift.Output()!=0) return false;
  lift.Update(false,true,true,false,false,false); // disabled
  if (lift.Output()!=0) return false;
  lift.Update(true,true,true,false,false,false); // held across enable
  return lift.Output()==0;
}
static_assert(LiftRunsIndependentlyAndDisarmsOnDisable(),
              "Lift and drive may run together, but disabling disarms the lift");

constexpr double Abs(double x) { return x < 0.0 ? -x : x; }
constexpr bool Near(double a, double b) { return Abs(a - b) < 1.0e-7; }
constexpr bool Stopped(const TeleopDriveController& controller) {
  return controller.GetForwardMMps() == 0.0 &&
      controller.GetLeftMMps() == 0.0 && controller.GetYawRateRadps() == 0.0;
}
constexpr TeleopDriveController Armed() {
  TeleopDriveController controller;
  controller.Update(true,true,0.0,0.0,0,0.02);
  return controller;
}
constexpr TeleopDriveController Settled(double x, double y, double rightX=0) {
  auto controller = Armed();
  for (int tick = 0; tick < 100; ++tick)
    controller.Update(true,true,x,y,rightX,0.02);
  return controller;
}

constexpr bool CanonicalTranslationWithoutReversingYaw() {
  const auto up=Settled(0,-1,0);
  const auto down=Settled(0,1,0);
  const auto left=Settled(-1,0,0);
  const auto right=Settled(1,0,0);
  return up.GetForwardMMps()>0 && down.GetForwardMMps()<0 &&
      left.GetLeftMMps()>0 && right.GetLeftMMps()<0 &&
      Settled(0,0,kCwRawX).GetYawRateRadps()<0 &&
      Settled(0,0,-kCwRawX).GetYawRateRadps()>0;
}
static_assert(CanonicalTranslationWithoutReversingYaw(),
              "Stick requests use canonical forward/left; wheel calibration is shared with odometry");

constexpr bool ExistingTeleopWheelOutputsPreserved() {
  for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int right=-1;right<=1;++right) {
    const auto c=Settled(x,y,right);
    const double magnitude=std::hypot(x,y);
    const double speed=magnitude==0?0:localization_config::TELEOP_SPEED_MMPS/magnitude;
    // Independent legacy model: layout vx=raw Y, vy=raw X; raw right X
    // contributes positive wheel yaw. Check both rotations and mixed moves.
    const double oldYaw=right*localization_config::TELEOP_MAX_YAW_RATE_DPS*robot_geometry::PI/180;
    for (int wheel=0;wheel<3;++wheel) {
      const double a=localization_config::WHEEL_ANGLE_DEG[wheel]*robot_geometry::PI/180;
      const double old=-std::sin(a)*y*speed+std::cos(a)*x*speed+
          oldYaw*localization_config::WHEEL_CENTER_RADIUS_MM[wheel];
      const double now=drive_kinematics::WheelSpeedMMps(wheel,
          drive_kinematics::LayoutVx(c.GetForwardMMps(),c.GetLeftMMps()),
          drive_kinematics::LayoutVy(c.GetForwardMMps(),c.GetLeftMMps()),c.GetYawRateRadps());
      if (!Near(old,now)) return false;
    }
  }
  return true;
}
static_assert(ExistingTeleopWheelOutputsPreserved(),"Shared translation/yaw calibration must preserve all teleop wheel outputs");

constexpr bool TriangleToggleIsIndependentOfManualServo() {
  ServoTeleopController servos;
  servos.Update(false,true,true);
  servos.Update(true,true,true);
  if (servos.OutputEnabled(1) || servos.TargetDegrees(1)!=0) return false;
  servos.Update(true,true,false);
  servos.Update(true,true,true);
  if (servos.TargetDegrees(0)!=0 || servos.TargetDegrees(1)!=180) return false;
  for (int tick=0;tick<20;++tick) servos.Update(true,true,true);
  if (servos.TargetDegrees(1)!=180) return false;
  servos.Update(true,true,false);
  servos.Update(true,true,true);
  if (servos.TargetDegrees(1)!=0) return false;
  servos.Update(true,false,true);
  if (servos.OutputEnabled(0) || servos.OutputEnabled(1)) return false;
  servos.Update(true,true,true);
  if (servos.OutputEnabled(1) || servos.TargetDegrees(1)!=0) return false;
  servos.Update(true,true,false,true,false);
  if (servos.TargetDegrees(0)!=15 || servos.TargetDegrees(1)!=0) return false;
  servos.Update(true,true,true,false,false);
  return servos.TargetDegrees(0)==15 && servos.TargetDegrees(1)==180;
}
static_assert(TriangleToggleIsIndependentOfManualServo(),
              "Triangle toggles only servo 9; servo 8 moves only with L1/R1");

constexpr bool ManualServoButtonsWorkIndependently() {
  ServoTeleopController servos;
  servos.Update(true,true,false,false,false);
  servos.Update(true,true,false,true,false);
  servos.Update(true,true,false,false,false);
  servos.Update(true,true,false,true,false);
  if (servos.TargetDegrees(0)!=30) return false;
  for (int i=0;i<20;++i) servos.Update(true,true,false,true,false);
  if (servos.TargetDegrees(0)!=30) return false;
  servos.Update(true,true,false,true,true);
  if (servos.TargetDegrees(0)!=15) return false;
  servos.Update(true,true,false,true,false);
  servos.Update(true,true,false,true,true);
  if (servos.TargetDegrees(0)!=0) return false;
  servos.Update(true,true,false,false,true);
  servos.Update(true,true,false,true,true);
  if (servos.TargetDegrees(0)!=15) return false;
  servos.Update(true,true,false,false,false);
  servos.Update(true,true,false,true,true);
  return servos.TargetDegrees(0)==15 && servos.TargetDegrees(1)==0;
}
static_assert(ManualServoButtonsWorkIndependently(),
              "L1/R1 each step once, in any order, including while the other remains held");

constexpr bool ManualServoLimitsAndRearming() {
  ServoTeleopController servos;
  servos.Update(true,true,false,true,false);
  if (servos.OutputEnabled(0) || servos.TargetDegrees(0)!=0) return false;
  servos.Update(true,true,false,false,false);
  for (int i=0;i<25;++i) {
    servos.Update(true,true,false,true,false);
    servos.Update(true,true,false,false,false);
  }
  if (servos.TargetDegrees(0)!=300) return false;
  for (int i=0;i<25;++i) {
    servos.Update(true,true,false,false,true);
    servos.Update(true,true,false,false,false);
  }
  if (servos.TargetDegrees(0)!=0) return false;
  servos.Update(false,true,false,true,false);
  servos.Update(true,true,false,true,false);
  if (servos.OutputEnabled(0) || servos.TargetDegrees(0)!=0) return false;
  servos.Update(true,true,false,false,false);
  servos.Update(true,true,false,true,false);
  if (servos.TargetDegrees(0)!=15) return false;
  servos.Update(true,false,false);
  servos.Update(true,true,false,true,false);
  return !servos.OutputEnabled(0) && servos.TargetDegrees(0)==15;
}
static_assert(ManualServoLimitsAndRearming(),
              "Servo 8 clamps at 0/300 and requires released bumpers after enable/reconnect");
static_assert(manipulator_config::SERVO_TRIANGLE_BUTTON!=2 &&
              manipulator_config::SERVO_STEP_BUTTON[0]!=2 && manipulator_config::SERVO_STEP_BUTTON[1]!=2,
              "X/button 2 has no servo assignment");

static_assert(manipulator_config::SERVO_PWM[0]==18 && manipulator_config::SERVO_PWM[1]==19,
              "Physical FlexDIO 8 and 9 use VMX WPI PWM 18 and 19");

static_assert(localization_config::MAX_WHEEL_SPEED_MMPS > 598.15 &&
              localization_config::MAX_WHEEL_SPEED_MMPS < 598.17,
              "112 gearbox-output RPM and 102 mm diameter must give 598.16 mm/s");

constexpr bool HeldControlsCannotArm() {
  TeleopDriveController controller;
  controller.Update(true,true,0,0,kCwRawX,0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true,true,0,0,-kCwRawX,0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true,true,1,0,0,0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true,true,0,0,0,0.02,false);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true,true,0,0,0,0.02);
  return controller.IsArmed() && Stopped(controller);
}
static_assert(HeldControlsCannotArm(), "Both sticks and D-pad must be neutral after enable/reconnect");

constexpr bool DeadbandDoesNotCreep() {
  auto controller = Armed();
  for (int tick = 0; tick < 100; ++tick)
    controller.Update(true,true,0.05,-0.05,0.05,0.02);
  return controller.IsArmed() && Stopped(controller);
}
static_assert(DeadbandDoesNotCreep(), "Neutral stick noise must not accumulate full-output motion");

constexpr bool RightStickDirections() {
  const auto ccw = Settled(0.0,0.0,-kCwRawX);
  const auto cw = Settled(0.0,0.0,kCwRawX);
  const double limit = localization_config::TELEOP_MAX_YAW_RATE_DPS *
      robot_geometry::PI / 180.0;
  return ccw.GetForwardMMps() == 0.0 && cw.GetLeftMMps() == 0.0 &&
      Near(ccw.GetYawRateRadps(), limit) && Near(cw.GetYawRateRadps(), -limit);
}
static_assert(RightStickDirections(), "Right stick left requests CCW and right requests CW");

constexpr bool RightStickNeutralStopsRotation() {
  auto controller = Settled(0.0,0.0,kCwRawX);
  controller.Update(true,true,0.0,0.0,0,0.02);
  return controller.IsArmed() && Stopped(controller);
}
static_assert(RightStickNeutralStopsRotation(), "Centering right stick stops rotation immediately");

constexpr bool CardinalAndDiagonalTranslation() {
  const double limit=localization_config::TELEOP_SPEED_MMPS;
  const auto forward=Settled(0,kForwardRawY);
  const auto backward=Settled(0,-kForwardRawY);
  const auto left=Settled(kLeftRawX,0);
  const auto right=Settled(-kLeftRawX,0);
  if (!Near(forward.GetForwardMMps(),limit) ||
      !Near(backward.GetForwardMMps(),-limit) ||
      !Near(left.GetLeftMMps(),limit) || !Near(right.GetLeftMMps(),-limit)) return false;
  for (int x=-1;x<=1;x+=2) for (int y=-1;y<=1;y+=2) {
    const auto diagonal=Settled(kLeftRawX*x,kForwardRawY*y);
    if (diagonal.GetForwardMMps()*y<=0 || diagonal.GetLeftMMps()*x<=0 ||
        !Near(Abs(diagonal.GetForwardMMps()),Abs(diagonal.GetLeftMMps())) ||
        !Near(std::hypot(diagonal.GetForwardMMps(),diagonal.GetLeftMMps()),limit)) return false;
  }
  const auto partial=Settled(kLeftRawX*0.4,kForwardRawY*0.8);
  return Near(partial.GetForwardMMps(),partial.GetLeftMMps()*2) &&
      std::hypot(partial.GetForwardMMps(),partial.GetLeftMMps())<limit;
}
static_assert(CardinalAndDiagonalTranslation(), "All four diagonals follow stick direction without exceeding the translation speed cap");

constexpr bool SidewaysTargetsStayEqualWithStickOffset() {
  const auto sideways=Settled(kLeftRawX*0.8,kForwardRawY*0.05,0);
  if (sideways.GetForwardMMps()!=0 || sideways.GetLeftMMps()<=0) return false;
  const double rear0=drive_kinematics::WheelSpeedMMps(0,drive_kinematics::LayoutVx(sideways.GetForwardMMps(),sideways.GetLeftMMps()),drive_kinematics::LayoutVy(sideways.GetForwardMMps(),sideways.GetLeftMMps()),0);
  const double rear1=drive_kinematics::WheelSpeedMMps(1,drive_kinematics::LayoutVx(sideways.GetForwardMMps(),sideways.GetLeftMMps()),drive_kinematics::LayoutVy(sideways.GetForwardMMps(),sideways.GetLeftMMps()),0);
  const auto forward=Settled(kLeftRawX*0.05,kForwardRawY*0.8,0);
  return Near(rear0,rear1) && forward.GetLeftMMps()==0 && forward.GetForwardMMps()>0;
}
static_assert(SidewaysTargetsStayEqualWithStickOffset(),
              "Within-deadband offsets preserve straight mixer targets; M0 output trim is applied later");

constexpr bool DirectionChangesUseVectorAccelerationLimit() {
  auto controller=Settled(kLeftRawX,0);
  const double beforeLeft=controller.GetLeftMMps();
  controller.Update(true,true,0,kForwardRawY,0,0.02);
  if (controller.GetForwardMMps()<=0 || controller.GetLeftMMps()<=0 ||
      !Near(std::hypot(controller.GetForwardMMps(),controller.GetLeftMMps()-beforeLeft),
            localization_config::TELEOP_ACCELERATION_MMPS2*0.02)) return false;
  for (int i=0;i<100;++i)
    controller.Update(true,true,0,kForwardRawY,0,0.02);
  return Near(controller.GetForwardMMps(),localization_config::TELEOP_SPEED_MMPS) &&
      Near(controller.GetLeftMMps(),0);
}
static_assert(DirectionChangesUseVectorAccelerationLimit(), "Direction changes smoothly blend translation axes within the vector acceleration limit");

constexpr bool RotationStartsLowAndTakesTwoSeconds() {
  auto controller=Armed();
  controller.Update(true,true,0,0,-kCwRawX,0.02);
  const double first=controller.GetYawRateRadps()*180/robot_geometry::PI;
  if (!Near(first,0.9)) return false;
  for (int i=1;i<50;++i) controller.Update(true,true,0,0,-kCwRawX,0.02);
  if (!Near(controller.GetYawRateRadps()*180/robot_geometry::PI,45)) return false;
  for (int i=50;i<100;++i) controller.Update(true,true,0,0,-kCwRawX,0.02);
  return Near(controller.GetYawRateRadps()*180/robot_geometry::PI,90);
}
static_assert(RotationStartsLowAndTakesTwoSeconds(), "Right-stick full rotation ramps from zero over two seconds");

constexpr bool SimultaneousTranslationAndRotation() {
  const auto cw=Settled(kLeftRawX,kForwardRawY,kCwRawX);
  const auto ccw=Settled(-kLeftRawX,-kForwardRawY,-kCwRawX);
  return cw.GetForwardMMps()>0 && cw.GetLeftMMps()>0 &&
      cw.GetYawRateRadps()<0 && ccw.GetForwardMMps()<0 &&
      ccw.GetLeftMMps()<0 && ccw.GetYawRateRadps()>0;
}
static_assert(SimultaneousTranslationAndRotation(), "Diagonal translation and right-stick rotation combine independently");

constexpr bool ReleaseTranslationPreservesRotation() {
  auto controller = Settled(0.0,kForwardRawY,kCwRawX);
  controller.Update(true,true,0.0,0.0,kCwRawX,0.02);
  return controller.GetForwardMMps() == 0.0 && controller.GetLeftMMps() == 0.0 &&
      controller.GetYawRateRadps() < 0.0;
}
static_assert(ReleaseTranslationPreservesRotation(), "Centering the stick must allow pure rotation");

constexpr bool ReleaseRotationPreservesTranslation() {
  auto controller = Settled(0.0,kForwardRawY,kCwRawX);
  controller.Update(true,true,0.0,kForwardRawY,0,0.02);
  return controller.GetForwardMMps() > 0.0 && controller.GetYawRateRadps() == 0.0;
}
static_assert(ReleaseRotationPreservesTranslation(), "Centering right stick stops yaw independently of translation");

constexpr bool DisableAndDisconnectStop() {
  auto controller = Settled(0.0,kForwardRawY,kCwRawX);
  controller.Update(false,true,0.0,kForwardRawY,kCwRawX,0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true,true,0.0,kForwardRawY,kCwRawX,0.02);
  if (controller.IsArmed() || !Stopped(controller)) return false;
  controller.Update(true,true,0.0,0.0,0,0.02);
  controller.Update(true,true,0.0,kForwardRawY,kCwRawX,0.02);
  controller.Update(true,false,0.0,kForwardRawY,kCwRawX,0.02);
  return !controller.IsArmed() && Stopped(controller);
}
static_assert(DisableAndDisconnectStop(), "Disable/disconnect must stop every component and disarm");

constexpr bool InvalidInputsStop() {
  const double bad[3]={std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN(),1.02};
  for (int i=0;i<3;++i) {
    auto controller=Settled(0,kForwardRawY,kCwRawX);
    controller.Update(true,true,bad[i],0,0,0.02);
    if (controller.IsArmed() || !Stopped(controller)) return false;
    controller=Settled(0,kForwardRawY,kCwRawX);
    controller.Update(true,true,0,bad[i],0,0.02);
    if (controller.IsArmed() || !Stopped(controller)) return false;
    controller=Settled(0,kForwardRawY,kCwRawX);
    controller.Update(true,true,0,0,bad[i],0.02);
    if (controller.IsArmed() || !Stopped(controller)) return false;
  }
  return true;
}
static_assert(InvalidInputsStop(), "Invalid values on any drive axis must stop and disarm");

constexpr bool LongLoopCannotSkipRamp() {
  auto controller = Armed();
  controller.Update(true,true,0.0,kForwardRawY,kCwRawX,10.0);
  return Near(controller.GetForwardMMps(),
              localization_config::TELEOP_ACCELERATION_MMPS2 * 0.050) &&
      Near(controller.GetYawRateRadps(),
           -localization_config::TELEOP_YAW_ACCELERATION_DPS2 *
           robot_geometry::PI / 180.0 * 0.050);
}
static_assert(LongLoopCannotSkipRamp(), "Ramp integration must cap a delayed loop to 50 ms");

constexpr bool DriveAndYawUseSeparateFrames() {
  const auto forward = Settled(0.0,kForwardRawY,0);
  const auto ccw = Settled(0.0,0.0,-kCwRawX);
  const auto cw = Settled(0.0,0.0,kCwRawX);
  if (!Near(drive_kinematics::LayoutVx(forward.GetForwardMMps(),forward.GetLeftMMps()), -localization_config::TELEOP_SPEED_MMPS) ||
      !Near(drive_kinematics::LayoutVy(forward.GetForwardMMps(),forward.GetLeftMMps()), 0.0)) return false;
  for (int wheel = 0; wheel < 3; ++wheel) {
    if (drive_kinematics::WheelSpeedMMps(wheel, 0.0, 0.0,
            ccw.GetYawRateRadps()) >= 0.0) return false;
    if (drive_kinematics::WheelSpeedMMps(wheel, 0.0, 0.0,
            cw.GetYawRateRadps()) <= 0.0) return false;
  }
  return true;
}
static_assert(DriveAndYawUseSeparateFrames(), "Positive wheel yaw is installed CW; body CCW requests need negative wheels");

constexpr bool MixedOutputsRespectLimit() {
  const auto controller = Settled(kLeftRawX,kForwardRawY,kCwRawX);
  double wheelSpeed[3]{};
  for (int wheel = 0; wheel < 3; ++wheel)
    wheelSpeed[wheel] = drive_kinematics::WheelSpeedMMps(wheel,
        drive_kinematics::LayoutVx(controller.GetForwardMMps(),controller.GetLeftMMps()), drive_kinematics::LayoutVy(controller.GetForwardMMps(),controller.GetLeftMMps()),
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

constexpr bool FullStickRespectsTranslationSpeedScale() {
  const auto forward = Settled(0.0,kForwardRawY,0);
  double speeds[3]{};
  for (int wheel = 0; wheel < 3; ++wheel)
    speeds[wheel] = drive_kinematics::WheelSpeedMMps(wheel,
        drive_kinematics::LayoutVx(forward.GetForwardMMps(),forward.GetLeftMMps()), drive_kinematics::LayoutVy(forward.GetForwardMMps(),forward.GetLeftMMps()), 0.0);
  const double denominator = drive_kinematics::OutputDenominator(
      speeds, localization_config::TELEOP_MAX_MOTOR_OUTPUT);
  return speeds[0] / denominator > 0.43 && speeds[0] / denominator < 0.44 &&
      speeds[1] / denominator < -0.43 && speeds[1] / denominator > -0.44 &&
      Near(speeds[2] / denominator, 0.0);
}
static_assert(FullStickRespectsTranslationSpeedScale(),
              "Half-speed forward: rear wheel outputs about +/-43.3 percent, front idle");


constexpr bool RightStickDeflectionControlsSpeed() {
  const double raw=(localization_config::DRIVER_STICK_DEADBAND+
      (1-localization_config::DRIVER_STICK_DEADBAND)*0.5)*kCwRawX;
  const auto half=Settled(0,0,raw);
  const auto full=Settled(0,0,kCwRawX);
  return Near(half.GetYawRateRadps(),full.GetYawRateRadps()*0.5);
}
static_assert(RightStickDeflectionControlsSpeed(), "Right-stick yaw speed scales after deadband");

constexpr bool TranslationVectorStaysWithinLimits() {
  auto controller=Armed();
  const double maxStep=localization_config::TELEOP_ACCELERATION_MMPS2*0.02;
  for (int x=-2;x<=2;++x) for (int y=-2;y<=2;++y) {
    for (int tick=0;tick<60;++tick) {
      const double oldForward=controller.GetForwardMMps();
      const double oldLeft=controller.GetLeftMMps();
      controller.Update(true,true,x*0.5,y*0.5,0,0.02);
      if (std::hypot(controller.GetForwardMMps(),controller.GetLeftMMps())>
          localization_config::TELEOP_SPEED_MMPS+1.0e-7) return false;
      // Returning to neutral intentionally stops immediately.
      if ((x!=0 || y!=0) &&
          std::hypot(controller.GetForwardMMps()-oldForward,
                     controller.GetLeftMMps()-oldLeft)>maxStep+1.0e-7) return false;
    }
  }
  return controller.IsArmed();
}
static_assert(TranslationVectorStaysWithinLimits(), "Translation stays within the speed and acceleration caps across stick positions");

constexpr double kBadSpeeds[3]{1.0, std::numeric_limits<double>::infinity(), 1.0};
static_assert(drive_kinematics::OutputDenominator(kBadSpeeds, 1.0) == 0.0,
              "Invalid wheel speed must reject the whole output vector");
}  // namespace

// These evaluate production code at compile time with the project's ARM GCC.
// They verify electrical commands, not real motor response on the arena floor.
namespace wheel_checks {
constexpr double Abs(double x) { return x<0?-x:x; }
constexpr bool Near(double a,double b) { return Abs(a-b)<1.0e-7; }
constexpr double mmPerCount=2*robot_geometry::PI*localization_config::WHEEL_RADIUS_MM/
    (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV*localization_config::MOTOR_TO_WHEEL_RATIO);

constexpr bool StartupPreservesSidewaysRatios() {
  WheelDriveController control;
  const double desired[3]={-150,-150,300};
  double counts[3]={};
  for (int step=0;step<45;++step) {
    for (int i=0;i<3;++i) counts[i]+=desired[i]*0.02/mmPerCount;
    control.Update(desired,counts,0.02,0.9);
    if (control.HasFault() || !Near(control.Output(0),control.Output(1)) ||
        !Near(control.Output(2),-2*control.Output(0))) return false;
    if (step==0 && (!Near(control.Output(0),-0.0075) || !Near(control.Output(2),0.015))) return false;
  }
  return Near(control.Output(0),desired[0]/localization_config::MAX_WHEEL_SPEED_MMPS) &&
      Near(control.Output(2),desired[2]/localization_config::MAX_WHEEL_SPEED_MMPS);
}
static_assert(StartupPreservesSidewaysRatios(),
              "A 2:1 front/rear request must remain 2:1 from the FIRST tick; neither rear gets a trim or boost");

constexpr bool BothSidewaysDirectionsAndForwardAreSymmetric() {
  const double zero[3]={};
  double left[3]={},right[3]={},forward[3]={};
  for (int i=0;i<3;++i) {
    left[i]=drive_kinematics::WheelSpeedMMps(i,0,100,0);
    right[i]=drive_kinematics::WheelSpeedMMps(i,0,-100,0);
    forward[i]=drive_kinematics::WheelSpeedMMps(i,100,0,0);
  }
  WheelDriveController a,b,c;
  for (int tick=0;tick<15;++tick) {
    a.Update(left,zero,0.02,0.9);
    b.Update(right,zero,0.02,0.9);
    c.Update(forward,zero,0.02,0.9);
    for (int i=0;i<3;++i) if (!Near(a.Output(i),-b.Output(i))) return false;
    if (!Near(a.Output(0),a.Output(1)) || !Near(c.Output(0),-c.Output(1)) || c.Output(2)!=0) return false;
  }
  return true;
}
static_assert(BothSidewaysDirectionsAndForwardAreSymmetric(),
              "Left/right must be mirrored; forward rear outputs must have equal magnitudes and opposite signs");

constexpr bool EncoderSpeedDoesNotCompensatePower() {
  WheelDriveController a,b;
  const double desired[3]={-120,-120,240};
  double countsA[3]={},countsB[3]={};
  for (int tick=0;tick<35;++tick) {
    for (int i=0;i<3;++i) {
      countsA[i]+=desired[i]*0.02/mmPerCount;
      countsB[i]+=desired[i]*0.02*1.5/mmPerCount;
    }
    a.Update(desired,countsA,0.02,0.9);
    b.Update(desired,countsB,0.02,0.9);
    if (a.HasFault() || b.HasFault()) return false;
    for (int i=0;i<3;++i) if (!Near(a.Output(i),b.Output(i))) return false;
  }
  return Abs(b.MeasuredMMps(0))>Abs(a.MeasuredMMps(0));
}
static_assert(EncoderSpeedDoesNotCompensatePower(),
              "Different encoder speeds may change telemetry but must not apply hidden PI/friction power adjustments");

constexpr bool CommonRampBlendsTheWholeVector() {
  WheelDriveController control;
  double desired[3]={-120,120,0},counts[3]={};
  for (int tick=0;tick<15;++tick) control.Update(desired,counts,0.02,0.9);
  double previous[3]={control.Output(0),control.Output(1),control.Output(2)};
  desired[1]=-120; desired[2]=240;
  control.Update(desired,counts,0.02,0.9);
  const double delta1=desired[1]/localization_config::MAX_WHEEL_SPEED_MMPS-previous[1];
  const double delta2=desired[2]/localization_config::MAX_WHEEL_SPEED_MMPS-previous[2];
  const double moved1=control.Output(1)-previous[1],moved2=control.Output(2)-previous[2];
  return Near(moved1*delta2,moved2*delta1) && moved1<0 && moved2>0 &&
      Abs(moved1)<=wheel_drive_config::PWM_FALL_PER_S*0.02+1.0e-8 &&
      Abs(moved2)<=wheel_drive_config::PWM_RISE_PER_S*0.02+1.0e-8;
}
static_assert(CommonRampBlendsTheWholeVector(), "Every changing wheel must use the same interpolation fraction");

constexpr bool SaturationAndNeutralAreSafe() {
  WheelDriveController control;
  double desired[3]={2000,-1000,500},counts[3]={};
  for (int tick=0;tick<20;++tick) {
    control.Update(desired,counts,0.02,0.25);
    for (int i=0;i<3;++i) if (Abs(control.Output(i))>0.2500001) return false;
    if (!Near(control.Output(0),-2*control.Output(1)) ||
        !Near(control.Output(0),4*control.Output(2))) return false;
  }
  desired[0]=desired[1]=desired[2]=0;
  control.Update(desired,counts,0.02,0.25);
  return control.Output(0)==0 && control.Output(1)==0 && control.Output(2)==0;
}
static_assert(SaturationAndNeutralAreSafe(), "Shared saturation preserves ratios; a neutral command stops immediately");

constexpr bool EncoderFaultsStillStopAllWheels() {
  WheelDriveController control;
  const double desired[3]={-120,-120,240};
  double counts[3]={};
  for (int tick=0;tick<75;++tick) control.Update(desired,counts,0.02,0.9);
  if (!control.HasFault() || control.Output(0)!=0 || control.Output(1)!=0 || control.Output(2)!=0) return false;
  control.Reset();
  counts[0]=std::numeric_limits<double>::quiet_NaN();
  control.Update(desired,counts,0.02,0.9);
  return control.HasFault() && control.Output(0)==0 && control.Output(2)==0;
}
static_assert(EncoderFaultsStillStopAllWheels(), "Removing compensation must retain all-wheel fault stopping");

constexpr bool FrontWaitsForBothRearsThenReleasesSmoothly() {
  WheelDriveController control;
  const double desired[3]={-60,-60,120};
  double counts[3]={};
  for (int i=0;i<20;++i) control.Update(desired,counts,0.02,0.9);
  if (!control.IsStarting()) return false;
  counts[2]=2; // Front moves before either rear.
  for (int i=0;i<10;++i) control.Update(desired,counts,0.02,0.9);
  if (!control.IsStarting() || control.Output(2)!=0 || control.WaitingWheel()!=0) return false;
  if (!Near(control.Output(0),desired[0]/localization_config::MAX_WHEEL_SPEED_MMPS) ||
      !Near(control.Output(1),desired[1]/localization_config::MAX_WHEEL_SPEED_MMPS)) return false;
  counts[0]=-2;
  for (int i=0;i<5;++i) control.Update(desired,counts,0.02,0.9);
  if (!control.IsStarting() || control.WaitingWheel()!=1 || control.Output(0)!=0 || control.Output(2)!=0) return false;
  counts[1]=-2; // Last rear responds; no full-power jump on release.
  control.Update(desired,counts,0.02,0.9);
  if (control.IsStarting() || control.HasFault() || control.Output(2)<=0 || control.Output(2)>0.0150001) return false;
  for (int i=0;i<20;++i) control.Update(desired,counts,0.02,0.9);
  return !control.HasFault() && Near(control.Output(0),control.Output(1)) &&
      Near(control.Output(2),-2*control.Output(0));
}
static_assert(FrontWaitsForBothRearsThenReleasesSmoothly(),
              "Early front movement must be held while rears start; group release must be gradual");

constexpr bool EarlyLeftRearIsHeldWithoutBoostingOtherWheels() {
  WheelDriveController control;
  const double desired[3]={-80,-80,160};
  double counts[3]={};
  for (int i=0;i<15;++i) control.Update(desired,counts,0.02,0.9);
  counts[0]=-2;
  for (int i=0;i<10;++i) {
    control.Update(desired,counts,0.02,0.9);
    for (int j=0;j<3;++j)
      if (Abs(control.Output(j))>Abs(desired[j])/localization_config::MAX_WHEEL_SPEED_MMPS+1.0e-8) return false;
  }
  return control.IsStarting() && control.Output(0)==0 && control.Output(1)<0 && control.Output(2)>0 &&
      control.WaitingWheel()==1;
}
static_assert(EarlyLeftRearIsHeldWithoutBoostingOtherWheels(),
              "Startup gating must treat the left rear like the front, with no motor-specific trim or power boost");

constexpr bool IdleFrontDoesNotBlockForwardAndNeutralResetsGate() {
  WheelDriveController control;
  const double desired[3]={-100,100,0};
  double counts[3]={};
  control.Update(desired,counts,0.02,0.9);
  counts[0]=-2; counts[1]=2;
  control.Update(desired,counts,0.02,0.9);
  if (control.IsStarting() || control.Output(2)!=0) return false;
  control.Reset();
  control.Update(desired,counts,0.02,0.9);
  if (!control.IsStarting() || control.WaitingWheel()!=0) return false;
  const double zero[3]={};
  control.Update(zero,counts,0.02,0.9);
  return !control.IsStarting() && control.Output(0)==0 && control.Output(1)==0;
}
static_assert(IdleFrontDoesNotBlockForwardAndNeutralResetsGate(),
              "Only wheels needed for the requested direction participate; neutral cancels startup immediately");

constexpr bool ChangingDirectionCancelsOldStartupHold() {
  WheelDriveController control;
  double desired[3]={-60,-60,120},counts[3]={};
  control.Update(desired,counts,0.02,0.9);
  counts[2]=2;
  for (int i=0;i<10;++i) control.Update(desired,counts,0.02,0.9);
  if (!control.IsStarting() || control.Output(2)!=0) return false;
  desired[0]=-100; desired[1]=100; desired[2]=0;
  for (int i=0;i<15;++i) control.Update(desired,counts,0.02,0.9);
  return !control.IsStarting() && !control.HasFault() && control.Output(0)<0 &&
      control.Output(1)>0 && control.Output(2)==0;
}
static_assert(ChangingDirectionCancelsOldStartupHold(),
              "A new requested direction must not inherit wheel holds from the old startup");

constexpr bool InsufficientLowPowerGetsAStartupTimeout() {
  WheelDriveController control;
  const double desired[3]={-20,-20,40};
  double counts[3]={};
  control.Update(desired,counts,0.02,0.9);
  counts[2]=2;
  for (int i=0;i<110;++i) control.Update(desired,counts,0.02,0.9);
  return control.HasFault() && control.FaultWheel()==0 && control.Output(0)==0 &&
      control.Output(1)==0 && control.Output(2)==0;
}
static_assert(InsufficientLowPowerGetsAStartupTimeout(),
              "If a low command cannot start all required wheels, stop instead of driving indefinitely on one wheel");

constexpr bool DifferentFrictionMotorSimulationStartsTheGroup() {
  WheelDriveController control;
  const double desired[3]={-140,-140,280};
  double position[3]={},counts[3]={},velocity[3]={};
  bool frontHeld=false,groupReleased=false;
  double previousFront=0;
  for (int tick=0;tick<250;++tick) {
    control.Update(desired,counts,0.02,0.9);
    if (control.HasFault()) return false;
    if (tick>0 && control.IsStarting() && counts[2]>=2 &&
        control.Output(2)<previousFront-1.0e-8) frontHeld=true;
    previousFront=control.Output(2);
    if (tick>0 && !control.IsStarting()) groupReleased=true;
    for (int i=0;i<3;++i) {
      const double power=control.Output(i);
      const double friction=i==0?0.17:(i==1?0.19:0.08);
      const double steady=Abs(power)>friction?
          (power>0?1:-1)*(Abs(power)-friction)/(1-friction)*localization_config::MAX_WHEEL_SPEED_MMPS:0;
      velocity[i]+=(steady-velocity[i])*(0.02/(0.10+0.02));
      position[i]+=velocity[i]*0.02;
      counts[i]=static_cast<int>(position[i]/mmPerCount);
    }
  }
  return frontHeld && groupReleased && velocity[0]<0 && velocity[1]<0 && velocity[2]>0;
}
static_assert(DifferentFrictionMotorSimulationStartsTheGroup(),
              "Startup must hold an early low-friction wheel and release the group when the slower wheels respond");
}
