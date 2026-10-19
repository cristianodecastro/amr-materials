#include "Hardware.h"
#include "DriveKinematics.h"

#include <algorithm>
#include <cmath>

Hardware::Hardware() {
  // Manual, staggered pings keep the four sonars from interfering. Ping()
  // resets the echo counter and sends WPILib's 10 us active-high pulse.
  // Do not also enable the global ultrasonic automatic-mode thread.
  frc::Ultrasonic::SetAutomaticMode(false);
  ResetDriveEncoders();
  StopDrive();
}

void Hardware::ResetDriveEncoders() {
  encoder0_.Reset();
  encoder1_.Reset();
  encoder2_.Reset();
}

double Hardware::GetWheelEncoderRaw(int wheel) {
  switch (wheel) {
    case 0: return encoder0_.GetRaw();
    case 1: return encoder1_.GetRaw();
    case 2: return encoder2_.GetRaw();
    default: return 0.0;
  }
}

void Hardware::SetWheelPwm(int wheel, double output) {
  if (wheel < 0 || wheel >= localization_config::WHEEL_COUNT) return;
  if (!std::isfinite(output)) output = 0.0;
  const double limited = std::max(-1.0, std::min(1.0, output)) *
                         localization_config::WHEEL_MOTOR_SIGN[wheel];
  wheelCommands_[wheel] = limited;
  switch (wheel) {
    case 0: motor0_.Set(limited); break;
    case 1: motor1_.Set(limited); break;
    case 2: motor2_.Set(limited); break;
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
        wheel, vx_MMps, vy_MMps, yawRate_Radps);
  DriveWheelsMMps(wheelSpeed, maxOutput);
}

void Hardware::DriveWheelsMMps(const double (&wheelSpeed)[3], double maxOutput) {
  if (!std::isfinite(maxOutput) || maxOutput <= 0.0 ||
      !std::isfinite(localization_config::MAX_WHEEL_SPEED_MMPS) ||
      localization_config::MAX_WHEEL_SPEED_MMPS <= 0.0) {
    StopDrive();
    return;
  }
  maxOutput = std::min(1.0, maxOutput);
  double largest = localization_config::MAX_WHEEL_SPEED_MMPS;
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    if (!std::isfinite(wheelSpeed[wheel])) {
      StopDrive();
      return;
    }
    // Scale the whole vector together to preserve translation direction.
    largest = std::max(largest, std::fabs(wheelSpeed[wheel]) / maxOutput);
  }
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    SetWheelPwm(wheel, wheelSpeed[wheel] / largest);
  }
}

void Hardware::StopDrive() {
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel)
    SetWheelPwm(wheel, 0.0);
}

bool Hardware::GetStartPressed() const {
  const bool raw = startButton_.Get();
  return localization_config::START_BUTTON_ACTIVE_LOW ? !raw : raw;
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
