#include "Hardware.h"
#include "DriveKinematics.h"
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
}

bool Hardware::TeleopMotorMotionAllowed() {
  return frc::DriverStation::GetInstance().IsDSAttached() &&
      frc::RobotState::IsEnabled() && frc::RobotState::IsOperatorControl() &&
      !frc::RobotState::IsTest();
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
  if (!TeleopMotorMotionAllowed()) output=0;
  const double limited = std::max(-1.0, std::min(1.0, output)) *
                         localization_config::WHEEL_MOTOR_SIGN[wheel];
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

void Hardware::DriveRobotRelative(double vx_MMps, double vy_MMps,
                                  double yawRate_Radps, double maxOutput) {
  if (!std::isfinite(vx_MMps) || !std::isfinite(vy_MMps) ||
      !std::isfinite(yawRate_Radps)) {
    StopDrive();
    return;
  }
  double wheelSpeed[3];
  for (int wheel = 0; wheel < 3; ++wheel)
    wheelSpeed[wheel] = drive_kinematics::WheelSpeedMMps(
        wheel, drive_kinematics::LayoutVx(vx_MMps,vy_MMps),
        drive_kinematics::LayoutVy(vx_MMps,vy_MMps), yawRate_Radps);
  DriveWheelsMMps(wheelSpeed, maxOutput);
}

void Hardware::DriveWheelsMMps(const double (&wheelSpeed)[3], double maxOutput) {
  const double denominator =
      drive_kinematics::OutputDenominator(wheelSpeed, maxOutput);
  if (denominator <= 0.0) {
    StopDrive();
    return;
  }
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    SetWheelPwm(wheel, wheelSpeed[wheel] / denominator);
  }
}

void Hardware::StopDrive() {
  speedBaselineValid_=false;
  for (auto& integral:speedIntegral_) integral=0;
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

void Hardware::DriveRobotRelativeFeedback(double forward,double left,double yawRadps,
                                           double maxOutput,double dt) {
  if (!std::isfinite(forward) || !std::isfinite(left) || !std::isfinite(yawRadps) ||
      !std::isfinite(dt) || dt<=0 || dt>autonomous_config::MAX_CONTROL_GAP_S ||
      !std::isfinite(maxOutput) || maxOutput<=0 || maxOutput>1) { StopDrive(); return; }
  double desired[3],counts[3],measured[3];
  const double mmPerCount=2*localization_config::PI*localization_config::WHEEL_RADIUS_MM/
      (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV*localization_config::MOTOR_TO_WHEEL_RATIO);
  for (int i=0;i<3;++i) {
    desired[i]=drive_kinematics::WheelSpeedMMps(i,drive_kinematics::LayoutVx(forward,left),
        drive_kinematics::LayoutVy(forward,left),yawRadps);
    counts[i]=GetWheelEncoderRaw(i);
    if (!std::isfinite(counts[i])) { StopDrive(); return; }
    measured[i]=speedBaselineValid_?(counts[i]-speedPreviousCounts_[i])*
        localization_config::WHEEL_ENCODER_SIGN[i]*mmPerCount/dt:0;
  }
  const double denominator=drive_kinematics::OutputDenominator(desired,maxOutput);
  if (denominator<=0) { StopDrive(); return; }
  const double scale=localization_config::MAX_WHEEL_SPEED_MMPS/denominator;
  for (int i=0;i<3;++i) {
    speedPreviousCounts_[i]=counts[i];
    const double request=desired[i]*scale;
    if (std::fabs(request)<1) { speedIntegral_[i]=0; SetWheelPwm(i,0); continue; }
    const double error=request-measured[i];
    const double candidate=std::max(-autonomous_config::WHEEL_INTEGRAL_LIMIT,
        std::min(autonomous_config::WHEEL_INTEGRAL_LIMIT,
          speedIntegral_[i]+autonomous_config::WHEEL_KI*error*dt));
    const double feedforward=request/localization_config::MAX_WHEEL_SPEED_MMPS+
        std::copysign(autonomous_config::WHEEL_KS,request);
    const double output=feedforward+autonomous_config::WHEEL_KP*error+candidate;
    // Conditional integration prevents accumulating error into saturation.
    if (std::fabs(output)<=maxOutput || output*error<0) speedIntegral_[i]=candidate;
    SetWheelPwm(i,std::max(-maxOutput,std::min(maxOutput,
        feedforward+autonomous_config::WHEEL_KP*error+speedIntegral_[i])));
  }
  speedBaselineValid_=true;
}

bool Hardware::GetStartPressed() const {
  return false; // Physical start button removed; never read servo FlexDIO 8 as an input.
}

void Hardware::SetServoAngle(int servo, double degrees, bool enabled) {
  if (servo<0 || servo>=2) return;
  auto& output=servo==0?crossServo_:triangleServo_;
  if (!enabled || !TeleopMotorMotionAllowed() || !std::isfinite(degrees) ||
      degrees<0 || degrees>300) {
    output.SetOffline();
    return;
  }
  // Studica's positional servo maps 0..300 degrees to 500..2500 us.
  output.SetAngle(degrees);
}

void Hardware::StopServos() {
  crossServo_.SetOffline();
  triangleServo_.SetOffline();
}

void Hardware::ZeroYaw() { imu_.ZeroYaw(); }
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
    case 0: backLeftSonar_.Ping(); break;
    case 1: backRightSonar_.Ping(); break;
    case 2: leftSonar_.Ping(); break;
    case 3: rightSonar_.Ping(); break;
    default: break;
  }
}

double Hardware::GetSonarRangeMM(int index) const {
  switch (index) {
    case 0: return backLeftSonar_.GetRangeMM();
    case 1: return backRightSonar_.GetRangeMM();
    case 2: return leftSonar_.GetRangeMM();
    case 3: return rightSonar_.GetRangeMM();
    default: return -1.0;
  }
}

bool Hardware::IsSonarRangeValid(int index) const {
  switch (index) {
    case 0: return backLeftSonar_.IsRangeValid();
    case 1: return backRightSonar_.IsRangeValid();
    case 2: return leftSonar_.IsRangeValid();
    case 3: return rightSonar_.IsRangeValid();
    default: return false;
  }
}

LineReading Hardware::ReadLine() {
  if (!cobra_) return {};
  double volts[4];
  for (int i=0;i<4;++i) volts[i]=cobra_->GetVoltage(i);
  return TrackLine(volts);
}

void Hardware::BeginObjectAction(ObjectAction action) {
  (void)action; // No invented motor ports or simulated completion.
}
ObjectActionStatus Hardware::GetObjectActionStatus() const {
  return ObjectActionStatus::Unavailable;
}
void Hardware::StopObjectHandler() {
  SetLiftPwm(0);
  StopServos();
}
