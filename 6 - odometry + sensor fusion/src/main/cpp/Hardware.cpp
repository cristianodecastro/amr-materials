#include "Hardware.h"

#include <algorithm>
#include <cmath>

Hardware::Hardware() {
  // Manual pings are staggered by Robot so the four sonars do not interfere.
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
  if (wheel < 0 || wheel >= config::WHEEL_COUNT) return;
  const double limited = std::max(-1.0, std::min(1.0, output)) *
                         config::WHEEL_MOTOR_SIGN[wheel];
  switch (wheel) {
    case 0: motor0_.Set(limited); break;
    case 1: motor1_.Set(limited); break;
    case 2: motor2_.Set(limited); break;
    default: break;
  }
}

void Hardware::DriveRobotRelative(double vx_Mps, double vy_Mps,
                                  double yawRate_Radps) {
  double wheelSpeed[config::WHEEL_COUNT];
  double largest = config::MAX_WHEEL_SPEED_MPS;
  for (int wheel = 0; wheel < config::WHEEL_COUNT; ++wheel) {
    const double alpha = config::WHEEL_ANGLE_DEG[wheel] * config::PI / 180.0;
    wheelSpeed[wheel] = -std::sin(alpha) * vx_Mps +
                        std::cos(alpha) * vy_Mps +
                        yawRate_Radps * config::WHEEL_CENTER_RADIUS_M[wheel];
    largest = std::max(largest, std::fabs(wheelSpeed[wheel]));
  }
  for (int wheel = 0; wheel < config::WHEEL_COUNT; ++wheel) {
    SetWheelPwm(wheel, wheelSpeed[wheel] / largest);
  }
}

void Hardware::StopDrive() {
  motor0_.Set(0.0);
  motor1_.Set(0.0);
  motor2_.Set(0.0);
}

bool Hardware::GetStartPressed() const {
  const bool raw = startButton_.Get();
  return config::START_BUTTON_ACTIVE_LOW ? !raw : raw;
}

void Hardware::ZeroYaw() { imu_.ZeroYaw(); }
double Hardware::GetYawDegrees() { return imu_.GetYaw(); }
double Hardware::GetYawRateDegreesPerSecond() { return imu_.GetRate(); }
double Hardware::GetWorldAccelX_G() { return imu_.GetWorldLinearAccelX(); }
double Hardware::GetWorldAccelY_G() { return imu_.GetWorldLinearAccelY(); }
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

double Hardware::GetSonarRangeMetres(int index) const {
  double millimetres = 0.0;
  switch (index) {
    case 0: millimetres = backLeftSonar_.GetRangeMM(); break;
    case 1: millimetres = backRightSonar_.GetRangeMM(); break;
    case 2: millimetres = leftSonar_.GetRangeMM(); break;
    case 3: millimetres = rightSonar_.GetRangeMM(); break;
    default: return -1.0;
  }
  return millimetres / 1000.0;
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
