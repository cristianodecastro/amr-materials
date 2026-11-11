#include "RobotControl.h"
#include "TeleopControl.h"
#include <frc/RobotState.h>
#include <frc/DriverStation.h>
#include <hal/I2C.h>

#include <algorithm>
#include <cmath>

namespace {
constexpr bool PortsValid() {
  const int lift=manipulator_config::TITAN_MOTOR_PORT;
  if (lift<0 || lift>3) return false;
  for (int i=0;i<3;++i) {
    if ((localization_config::WHEEL_MOTOR_SIGN[i]!=1 && localization_config::WHEEL_MOTOR_SIGN[i]!=-1) ||
        (localization_config::WHEEL_ENCODER_SIGN[i]!=1 && localization_config::WHEEL_ENCODER_SIGN[i]!=-1)) return false;
    const int port=localization_config::WHEEL_TITAN_PORT[i];
    if (port<0 || port>3 || port==lift) return false;
    for (int j=0;j<i;++j) if (port==localization_config::WHEEL_TITAN_PORT[j]) return false;
  }
  return true;
}
static_assert(PortsValid(),"Drive/lift ports must be distinct; motor and encoder signs must be +1 or -1");
}

bool Hardware::TeleopMotorMotionAllowed() {
  return frc::DriverStation::GetInstance().IsDSAttached() &&
      frc::RobotState::IsEnabled() && frc::RobotState::IsOperatorControl() &&
      !frc::RobotState::IsTest();
}

bool Hardware::AutonomousMotorMotionAllowed() const {
  return autonomousDriveEnabled_ &&
      frc::DriverStation::GetInstance().IsDSAttached() && frc::RobotState::IsEnabled() &&
      frc::RobotState::IsAutonomous() && !frc::RobotState::IsTest();
}

bool Hardware::DriveMotorMotionAllowed() const {
  return teleopWheelActive_?TeleopMotorMotionAllowed():AutonomousMotorMotionAllowed();
}

Hardware::Hardware() {
  StopServos();
  // Manual, staggered pings keep the two sonars from interfering. Ping()
  // resets the echo counter and sends WPILib's 10 us active-high pulse.
  // Do not also enable the global ultrasonic automatic-mode thread.
  frc::Ultrasonic::SetAutomaticMode(false);
  if (autonomous_config::COBRA_INSTALLED) {
    cobra_.reset(new studica::Cobra(static_cast<float>(autonomous_config::COBRA_REFERENCE_V)));
  }
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

void Hardware::SetWheelPwm(int wheel, double output,double maxOutput) {
  if (wheel < 0 || wheel >= localization_config::WHEEL_COUNT) return;
  if (!std::isfinite(output)) output = 0.0;
  if (!DriveMotorMotionAllowed()) output=0;
  const int percent=motor_output::TitanPercent(output*localization_config::WHEEL_MOTOR_SIGN[wheel],
      std::min(maxOutput,localization_config::TELEOP_MAX_MOTOR_OUTPUT));
  const double limited=motor_output::TitanSetArgument(percent);
  wheelCommands_[wheel] = percent/100.0;
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
  teleopWheelDrive_.Reset();
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
    output=motor_output::EnforceFloor(output,cap);
    const double direction=output*manipulator_config::LIFT_UP_SIGN;
    if ((direction>0 && GetLiftUpperLimit()) || (direction<0 && GetLiftLowerLimit())) output=0;
  }
  const int percent=motor_output::TitanPercent(output,cap);
  liftCommand_=percent/100.0;
  liftMotor_.Set(motor_output::TitanSetArgument(percent));
}

void Hardware::DriveTeleopRobotRelative(double forward,double left,double yawRadps,
                                      double maxOutput,double dt) {
  DriveWithContext(forward,left,yawRadps,maxOutput,dt,WheelDriveContext::Teleop);
}

void Hardware::DriveAutonomousRobotRelative(double forward,double left,double yawRadps,
                                          double maxOutput,double dt) {
  DriveWithContext(forward,left,yawRadps,maxOutput,dt,WheelDriveContext::Autonomous);
}

void Hardware::DriveWithContext(double forward,double left,double yawRadps,
                               double maxOutput,double dt,WheelDriveContext context) {
  const bool teleop=context==WheelDriveContext::Teleop;
  if (!(teleop?TeleopMotorMotionAllowed():AutonomousMotorMotionAllowed())) {
    StopDrive(); return;
  }
  teleopWheelActive_=teleop;
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
  auto& controller=teleopWheelActive_?teleopWheelDrive_:wheelDrive_;
  controller.Update(desired,counts,dt,maxOutput,context);
  for (int i=0;i<3;++i) SetWheelPwm(i,controller.Output(i),maxOutput);
}

std::string Hardware::GetDriveStatus() const {
  const auto& controller=ActiveWheelDrive();
  const int wheel=controller.HasFault()?controller.FaultWheel():controller.WaitingWheel();
  const std::string prefix=wheel>=0 && wheel<3?
      std::string(localization_config::WHEEL_NAME[wheel])+" M"+
      std::to_string(localization_config::WHEEL_TITAN_PORT[wheel])+": ":"";
  return prefix+controller.Status();
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
double Hardware::GetYawRateDegreesPerSecond() {
  const auto timestamp=static_cast<std::uint32_t>(imu_.GetLastSensorTimestamp());
  const double yaw=GetYawDegrees(); // same CCW sign and frame as localization
  // An AHRS update may arrive between the timestamp and angle reads. Keep the
  // last consistent rate rather than combine two different sensor samples.
  if (timestamp!=static_cast<std::uint32_t>(imu_.GetLastSensorTimestamp())) return yawRate_.Rate();
  return yawRate_.Update(yaw,timestamp);
}
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
    case 0: frontSonar_.Ping(); break;
    case 1: rearSonar_.Ping(); break;
    default: break;
  }
}

double Hardware::GetSonarRangeMM(int index) const {
  switch (index) {
    case 0: return frontSonar_.GetRangeMM();
    case 1: return rearSonar_.GetRangeMM();
    default: return -1.0;
  }
}

bool Hardware::IsSonarRangeValid(int index) const {
  switch (index) {
    case 0: return frontSonar_.IsRangeValid();
    case 1: return rearSonar_.IsRangeValid();
    default: return false;
  }
}

IrReading Hardware::ReadIr(int index) {
  if (index<0 || index>=localization_config::IR_COUNT) return {};
  auto& input=index==0?leftIr_:rightIr_;
  const double voltage=input.GetAverageVoltage();
  if (input.StatusIsFatal()) return {};
  return SharpIrFromVoltage(voltage);
}

LineReading Hardware::ReadLine() {
  if (!cobra_) return {};
  // A real read verifies that the ADC responds. All-zero vendor readings alone
  // cannot distinguish a disconnected module from a dark/low-voltage surface.
  // Use the bus already opened by studica::Cobra. VMX HAL has one I2C resource;
  // constructing another frc::I2C would attempt to allocate it a second time.
  // A read does not write or change ADC configuration. Negative means aborted.
  uint8_t response[2]={};
  if (HAL_ReadI2C(HAL_I2C_kOnboard,autonomous_config::COBRA_I2C_ADDRESS,response,2)<0) return {};
  double volts[4];
  for (int i=0;i<4;++i) volts[i]=cobra_->GetVoltage(i);
  auto reading=TrackLine(volts);
  reading.communicationValid=true;
  return reading;
}

void Hardware::StopManipulator() {
  SetLiftPwm(0);
  StopServos();
}
