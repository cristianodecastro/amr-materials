#include "RobotControl.h"
#include "TeleopControl.h"
#include <frc/RobotState.h>
#include <frc/DriverStation.h>

#include <algorithm>
#include <cmath>

namespace {
constexpr bool PortsValid() {
  const int lift=manipulator_config::TITAN_MOTOR_PORT;
  if (lift<0 || lift>3) return false;
  for (int i=0;i<3;++i) {
    const int port=localization_config::WHEEL_TITAN_PORT[i];
    if (port<0 || port>3 || port==lift) return false;
    for (int j=0;j<i;++j) if (port==localization_config::WHEEL_TITAN_PORT[j]) return false;
  }
  return true;
}
static_assert(PortsValid(),"Three distinct drive ports and a separate lift port must cover Titan 0..3");
static_assert(localization_config::TELEOP_LEFT_REAR_OUTPUT_SCALE > 0.0 &&
              localization_config::TELEOP_LEFT_REAR_OUTPUT_SCALE <= 1.0,
              "Left rear teleop output scale must be in (0, 1]");
}

bool Hardware::TeleopMotorMotionAllowed() {
  return frc::DriverStation::GetInstance().IsDSAttached() &&
      frc::RobotState::IsEnabled() && frc::RobotState::IsOperatorControl() &&
      !frc::RobotState::IsTest();
}

bool Hardware::DriveMotorMotionAllowed() const {
  if (TeleopMotorMotionAllowed()) return true;
  return autonomousDriveEnabled_ &&
      frc::DriverStation::GetInstance().IsDSAttached() && frc::RobotState::IsEnabled() &&
      frc::RobotState::IsAutonomous() && !frc::RobotState::IsTest();
}

Hardware::Hardware() {
  StopServos();
  // Manual, staggered pings keep the four sonars from interfering. Ping()
  // resets the echo counter and sends WPILib's 10 us active-high pulse.
  // Do not also enable the global ultrasonic automatic-mode thread.
  frc::Ultrasonic::SetAutomaticMode(false);
  if (autonomous_config::COBRA_INSTALLED)
    cobra_.reset(new studica::Cobra(static_cast<float>(autonomous_config::COBRA_REFERENCE_V)));
  if (manipulator_config::LIMIT_DIO[0]>=0)
    liftUpperLimit_.reset(new frc::DigitalInput(manipulator_config::LIMIT_DIO[0]));
  if (manipulator_config::LIMIT_DIO[1]>=0)
    liftLowerLimit_.reset(new frc::DigitalInput(manipulator_config::LIMIT_DIO[1]));
  ResetDriveEncoders();
  StopDrive();
  SetLiftPwm(0);
}

void Hardware::ResetDriveEncoders() {
  leftRearEncoder_.Reset();
  rightRearEncoder_.Reset();
  frontEncoder_.Reset();
}

double Hardware::GetWheelEncoderRaw(int wheel) {
  switch (wheel) {
    case 0: return leftRearEncoder_.GetRaw();
    case 1: return rightRearEncoder_.GetRaw();
    case 2: return frontEncoder_.GetRaw();
    default: return 0.0;
  }
}

void Hardware::SetWheelPwm(int wheel, double output) {
  if (wheel < 0 || wheel >= localization_config::WHEEL_COUNT) return;
  if (!std::isfinite(output)) output = 0.0;
  if (!DriveMotorMotionAllowed()) output=0;
  // Apply the left-rear trim only in Teleop, after smoothing. Autonomous uses
  // the full mixed request for every wheel, with the same shared speed limits.
  const double scale=wheel==0 && TeleopMotorMotionAllowed()?
      localization_config::TELEOP_LEFT_REAR_OUTPUT_SCALE:1.0;
  const double limited = std::max(-1.0, std::min(1.0, output)) *
                         scale * localization_config::WHEEL_MOTOR_SIGN[wheel];
  wheelCommands_[wheel] = limited;
  switch (wheel) {
    case 0: leftRearMotor_.Set(limited); break;
    case 1: rightRearMotor_.Set(limited); break;
    case 2: frontMotor_.Set(limited); break;
    default: break;
  }
}

double Hardware::GetWheelCommand(int wheel) const {
  return wheel >= 0 && wheel < localization_config::WHEEL_COUNT
      ? wheelCommands_[wheel] : 0.0;
}

void Hardware::StopDrive() {
  wheelDrive_.Reset();
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel)
    SetWheelPwm(wheel, 0.0);
}

bool Hardware::GetLiftUpperLimit() const {
  return liftUpperLimit_ && (manipulator_config::LIMIT_ACTIVE_LOW?!liftUpperLimit_->Get():liftUpperLimit_->Get());
}
bool Hardware::GetLiftLowerLimit() const {
  return liftLowerLimit_ && (manipulator_config::LIMIT_ACTIVE_LOW?!liftLowerLimit_->Get():liftLowerLimit_->Get());
}
double Hardware::GetLiftEncoderRaw() { return liftEncoder_.GetRaw(); }
void Hardware::SetLiftPwm(double output) {
  const double cap=manipulator_config::LIFT_OUTPUT;
  if (!TeleopMotorMotionAllowed() || !std::isfinite(output) || !std::isfinite(cap) ||
      cap<=0 || cap>1 || !std::isfinite(manipulator_config::LIFT_UP_SIGN) ||
      std::fabs(manipulator_config::LIFT_UP_SIGN)!=1) output=0;
  else {
    output=std::max(-cap,std::min(cap,output));
    const double direction=output*manipulator_config::LIFT_UP_SIGN;
    if ((direction>0 && GetLiftUpperLimit()) || (direction<0 && GetLiftLowerLimit())) output=0;
  }
  liftCommand_=output;
  liftMotor_.Set(output);
}

void Hardware::DriveRobotRelative(double forward,double left,double yawRadps,
                                           double maxOutput,double dt) {
  if (!DriveMotorMotionAllowed()) { StopDrive(); return; }
  // Neutral is an immediate stop and the deliberate reset for a latched fault.
  if (std::isfinite(forward) && std::isfinite(left) && std::isfinite(yawRadps) &&
      std::fabs(forward)<0.01 && std::fabs(left)<0.01 && std::fabs(yawRadps)<0.0001) {
    StopDrive(); return;
  }
  double desired[3],counts[3];
  for (int i=0;i<3;++i) {
    desired[i]=drive_kinematics::WheelSpeedMMps(i,drive_kinematics::LayoutVx(forward,left),
        drive_kinematics::LayoutVy(forward,left),yawRadps);
    counts[i]=GetWheelEncoderRaw(i);
  }
  wheelDrive_.Update(desired,counts,dt,maxOutput);
  for (int i=0;i<3;++i) SetWheelPwm(i,wheelDrive_.Output(i));
}

std::string Hardware::GetDriveStatus() const {
  const int wheel=wheelDrive_.HasFault()?wheelDrive_.FaultWheel():wheelDrive_.WaitingWheel();
  const std::string prefix=wheel>=0 && wheel<3?
      std::string(localization_config::WHEEL_NAME[wheel])+" M"+
      std::to_string(localization_config::WHEEL_TITAN_PORT[wheel])+": ":"";
  return prefix+wheelDrive_.Status();
}

void Hardware::SetServoAngle(int servo, double degrees, bool enabled) {
  if (servo<0 || servo>=2) return;
  auto& output=servo==0?manualServo_:triangleServo_;
  if (!enabled || !TeleopMotorMotionAllowed() || !std::isfinite(degrees) ||
      degrees<manipulator_config::SERVO_MIN_DEG || degrees>manipulator_config::SERVO_MAX_DEG) {
    output.SetOffline();
    return;
  }
  // Studica's positional servo maps 0..300 degrees to 500..2500 us.
  output.SetAngle(degrees);
}

void Hardware::StopServos() {
  manualServo_.SetOffline();
  triangleServo_.SetOffline();
}

// The navX reports clockwise-positive yaw and a board frame with +Y forward,
// +X right. Localization uses CCW-positive yaw and +X forward, +Y left.
double Hardware::GetYawDegrees() { return -imu_.GetYaw(); }
double Hardware::GetYawRateDegreesPerSecond() { return -imu_.GetRate(); }
double Hardware::GetWorldAccelX_G() { return imu_.GetWorldLinearAccelY(); }
double Hardware::GetWorldAccelY_G() { return -imu_.GetWorldLinearAccelX(); }
double Hardware::GetRawAccelX_G() { return imu_.GetRawAccelX(); }
double Hardware::GetRawAccelY_G() { return imu_.GetRawAccelY(); }
double Hardware::GetRawAccelZ_G() { return imu_.GetRawAccelZ(); }
double Hardware::GetRawGyroX_Dps() { return imu_.GetRawGyroX(); }
double Hardware::GetRawGyroY_Dps() { return imu_.GetRawGyroY(); }
double Hardware::GetRawGyroZ_Dps() { return imu_.GetRawGyroZ(); }
bool Hardware::IsImuConnected() { return imu_.IsConnected(); }
bool Hardware::IsImuCalibrating() { return imu_.IsCalibrating(); }

void Hardware::PingSonar(int index) {
  switch (index) {
    case 0: leftSonar_.Ping(); break;
    case 1: rearSonar_.Ping(); break;
    case 2: rightSonar_.Ping(); break;
    case 3: frontSonar_.Ping(); break;
    default: break;
  }
}

double Hardware::GetSonarRangeMM(int index) const {
  switch (index) {
    case 0: return leftSonar_.GetRangeMM();
    case 1: return rearSonar_.GetRangeMM();
    case 2: return rightSonar_.GetRangeMM();
    case 3: return frontSonar_.GetRangeMM();
    default: return -1.0;
  }
}

bool Hardware::IsSonarRangeValid(int index) const {
  switch (index) {
    case 0: return leftSonar_.IsRangeValid();
    case 1: return rearSonar_.IsRangeValid();
    case 2: return rightSonar_.IsRangeValid();
    case 3: return frontSonar_.IsRangeValid();
    default: return false;
  }
}

LineReading Hardware::ReadLine() {
  if (!cobra_) return {};
  double volts[4];
  for (int i=0;i<4;++i) volts[i]=cobra_->GetVoltage(i);
  return TrackLine(volts);
}

void Hardware::StopManipulator() {
  SetLiftPwm(0);
  StopServos();
}
