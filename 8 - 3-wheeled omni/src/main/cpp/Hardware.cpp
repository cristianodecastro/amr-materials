#include "RobotControl.h"
#include "TeleopControl.h"
#include <frc/RobotState.h>
#include <frc/DriverStation.h>
#include <hal/I2C.h>

#include <algorithm>
#include <cmath>

namespace {
constexpr bool PortsValid() {
  for (int i=0;i<3;++i) {
    if ((localization_config::WHEEL_MOTOR_SIGN[i]!=1 && localization_config::WHEEL_MOTOR_SIGN[i]!=-1) ||
        (localization_config::WHEEL_ENCODER_SIGN[i]!=1 && localization_config::WHEEL_ENCODER_SIGN[i]!=-1)) return false;
    const int port=localization_config::WHEEL_TITAN_PORT[i];
    if (port<0 || port>3) return false;
    for (int j=0;j<i;++j) if (port==localization_config::WHEEL_TITAN_PORT[j]) return false;
  }
  return true;
}
static_assert(PortsValid(),"Drive ports must be distinct; motor and encoder signs must be +1 or -1");
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
  // Manual, staggered pings keep the two sonars from interfering. Ping()
  // resets the echo counter and sends WPILib's 10 us active-high pulse.
  // Do not also enable the global ultrasonic automatic-mode thread.
  frc::Ultrasonic::SetAutomaticMode(false);
  cobra_.reset(new studica::Cobra(static_cast<float>(cobra_config::COBRA_REFERENCE_V)));
  ResetDriveEncoders();
  StopDrive();
}

void Hardware::ResetDriveEncoders() {
  frontLeftEncoder_.Reset();
  rearEncoder_.Reset();
  frontRightEncoder_.Reset();
}

double Hardware::GetWheelEncoderRaw(int wheel) {
  switch (wheel) {
    case 0: return frontLeftEncoder_.GetRaw();
    case 1: return rearEncoder_.GetRaw();
    case 2: return frontRightEncoder_.GetRaw();
    default: return 0.0;
  }
}

void Hardware::SetWheelPwm(int wheel, double output,double maxOutput) {
  if (wheel < 0 || wheel >= localization_config::WHEEL_COUNT) return;
  if (!std::isfinite(output)) output = 0.0;
  if (!DriveMotorMotionAllowed()) output=0;
  const int percent=motor_output::WheelTitanPercent(wheel,output,
      std::min(maxOutput,localization_config::TELEOP_MAX_MOTOR_OUTPUT));
  const double limited=motor_output::TitanSetArgument(percent);
  wheelCommands_[wheel] = percent/100.0;
  switch (wheel) {
    case 0: frontLeftMotor_.Set(limited); break;
    case 1: rearMotor_.Set(limited); break;
    case 2: frontRightMotor_.Set(limited); break;
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
    desired[i]=drive_kinematics::WheelSpeedMMps(i,forward,left,yawRadps);
    counts[i]=GetWheelEncoderRaw(i);
  }
  auto& controller=wheelDrive_;
  controller.Update(desired,counts,dt,maxOutput);
  double output[3]; for (int i=0;i<3;++i) output[i]=controller.Output(i);
  if (!teleop) motor_output::PrepareAutonomousVector(output,maxOutput);
  for (int i=0;i<3;++i) SetWheelPwm(i,output[i],maxOutput);
}

std::string Hardware::GetDriveStatus() const {
  const auto& controller=wheelDrive_;
  const int wheel=controller.FaultWheel();
  const std::string prefix=wheel>=0 && wheel<3?
      std::string(localization_config::WHEEL_NAME[wheel])+" M"+
      std::to_string(localization_config::WHEEL_TITAN_PORT[wheel])+": ":"";
  return prefix+controller.Status();
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
  if (HAL_ReadI2C(HAL_I2C_kOnboard,cobra_config::COBRA_I2C_ADDRESS,response,2)<0) return {};
  LineReading reading;
  for (int i=0;i<4;++i) reading.volts[i]=cobra_->GetVoltage(i);

  reading.communicationValid=true;
  return reading;
}
