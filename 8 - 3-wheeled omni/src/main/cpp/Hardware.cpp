#include "RobotControl.h"
#include "TeleopControl.h"
#include <frc/RobotState.h>
#include <frc/DriverStation.h>
#include <hal/I2C.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

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
      std::fabs(forward)<wheel_drive_config::COMMAND_STOP_MMPS && std::fabs(left)<wheel_drive_config::COMMAND_STOP_MMPS &&
      std::fabs(yawRadps)<wheel_drive_config::COMMAND_STOP_RADPS) {
    StopDrive(); return;
  }
  double desired[3],counts[3];
  for (int i=0;i<3;++i) {
    desired[i]=drive_kinematics::WheelSpeedMMps(i,forward,left,yawRadps);
    counts[i]=GetWheelEncoderRaw(i);
  }
  auto& controller=wheelDrive_;
  if (teleop) controller.Update(desired,counts,dt,maxOutput);
  else controller.UpdateAutonomous(desired,counts,dt,maxOutput);
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
void Hardware::SampleImu(double now) {
  imuConnected_=imu_.IsConnected(); imuCalibrating_=imu_.IsCalibrating();
  if (!imuConnected_) {
    imuReady_=imuDataFresh_=false; imuCalibrationDoneSince_=-1; imuHaveSample_=false;
    yawRate_.Reset(); imuYawRateDps_=0; return;
  }
  const auto stamp=static_cast<std::uint32_t>(imu_.GetLastSensorTimestamp());
  const double updates=imu_.GetUpdateCount();
  const double yaw=-imu_.GetYaw();
  if (stamp==static_cast<std::uint32_t>(imu_.GetLastSensorTimestamp()) && std::isfinite(yaw)) {
    if (!imuHaveSample_ || stamp!=imuTimestamp_ || (std::isfinite(updates) && updates!=imuUpdateCount_)) {
      imuSampleTime_=now; imuTimestamp_=stamp; imuHaveSample_=true;
      imuUpdateCount_=updates;
      imuYawDeg_=yaw;
      imuYawRateDps_=imuCalibrating_?0:yawRate_.Update(yaw,stamp);
      imuTemperatureC_=imu_.GetTempC();
    }
  }
  imuDataFresh_=imuHaveSample_ && now>=imuSampleTime_ && now-imuSampleTime_<=localization_config::IMU_SAMPLE_STALE_S;
  if (imuCalibrating_ || !imuDataFresh_) {
    imuReady_=false; imuCalibrationDoneSince_=-1; yawRate_.Reset(); imuYawRateDps_=0; return;
  }
  // Startup calibration is followed by a separate initial yaw-offset phase.
  // Fixed nonblocking grace period after calibration. Motion flags are not
  // readiness gates: bias/noise can assert them indefinitely while stationary.
  if (imuCalibrationDoneSince_<0) imuCalibrationDoneSince_=now;
  imuReady_=now-imuCalibrationDoneSince_>=localization_config::IMU_INITIAL_YAW_SETTLE_S;
}
double Hardware::GetYawDegrees() { return imuYawDeg_; }
double Hardware::GetYawRateDegreesPerSecond() { return imuYawRateDps_; }
std::string Hardware::GetImuStatus() const {
  if (!imuConnected_) return "IMU disconnected";
  if (imuCalibrating_) return "Gyro calibrating: keep robot still";
  if (!imuDataFresh_) return "IMU stream stale: waiting for new yaw packets";
  if (!imuReady_) return "Initial yaw settling (2.1 s after calibration): keep robot still";
  std::ostringstream text;
  text<<std::fixed<<std::setprecision(3)<<"Ready | "<<imuYawRateDps_<<" deg/s | "
      <<std::setprecision(1)<<imuTemperatureC_<<" C";
  return text.str();
}
double Hardware::GetWorldAccelX_G() { return imu_.GetWorldLinearAccelY(); }
double Hardware::GetWorldAccelY_G() { return -imu_.GetWorldLinearAccelX(); }
bool Hardware::IsImuConnected() { return imuConnected_; }
bool Hardware::IsImuCalibrating() { return imuCalibrating_; }

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
