#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <string>

#include "Constants.h"

void Robot::RobotInit() {
  mockDriverStation_.Enable();
  hardware_.StopDrive();
  lastLoopTime_ = frc::Timer::GetFPGATimestamp();
  frc::Ultrasonic::SetAutomaticMode(false);
  frc::SmartDashboard::PutString("localization_status",
                                  "Waiting for start button");
  frc::SmartDashboard::PutBoolean("imu_connected", hardware_.IsImuConnected());
  frc::SmartDashboard::PutBoolean("imu_calibrating", hardware_.IsImuCalibrating());
}

void Robot::RobotPeriodic() {
  const double now = frc::Timer::GetFPGATimestamp();
  const double dt = now - lastLoopTime_;
  lastLoopTime_ = now;
  hardware_.StopDrive();  // No motion command was requested; keep all wheels stopped.

  if (!started_) {
    if (hardware_.GetStartPressed() && !hardware_.IsImuCalibrating()) {
      ++startButtonDebounceCount_;
    } else {
      startButtonDebounceCount_ = 0;
    }

    if (startButtonDebounceCount_ >= 3) BeginLocalization();
    frc::SmartDashboard::PutBoolean("start_button_pressed",
                                     hardware_.GetStartPressed());
    frc::SmartDashboard::PutBoolean("imu_connected", hardware_.IsImuConnected());
    frc::SmartDashboard::PutBoolean("imu_calibrating", hardware_.IsImuCalibrating());
    return;
  }

  localization_.UpdateOdometry(dt);
  UpdateSonarPoll(now);
  localization_.PublishTelemetry(lastSonarRange_M_, lastSonarIndex_);
  for (int sensor = 0; sensor < 4; ++sensor) {
    const std::string key = config::SONARS[sensor].name;
    frc::SmartDashboard::PutNumber(key + "_m", sonarRanges_M_[sensor]);
    frc::SmartDashboard::PutBoolean(key + "_valid", sonarRangeValid_[sensor]);
    frc::SmartDashboard::PutBoolean(key + "_used_for_correction",
                                    sonarCorrectionAccepted_[sensor]);
  }
  frc::SmartDashboard::PutString("localization_status", "Running");
  frc::SmartDashboard::PutBoolean("imu_connected", hardware_.IsImuConnected());
  frc::SmartDashboard::PutBoolean("imu_calibrating", hardware_.IsImuCalibrating());
}

void Robot::BeginLocalization() {
  hardware_.ZeroYaw();
  hardware_.ResetDriveEncoders();
  localization_.Reset(config::START_X_M, config::START_Y_M,
                      config::START_HEADING_DEG);
  started_ = true;
  lastSonarPingTime_ = frc::Timer::GetFPGATimestamp();
  frc::SmartDashboard::PutString("localization_status", "Started");
}

void Robot::UpdateSonarPoll(double nowSeconds) {
  if (sonarAwaitingEcho_) {
    if (nowSeconds - sonarPingTime_ < config::SONAR_ECHO_SETTLE_S) return;

    sonarRangeValid_[nextSonarIndex_] =
        hardware_.IsSonarRangeValid(nextSonarIndex_);
    sonarCorrectionAccepted_[nextSonarIndex_] = false;
    if (sonarRangeValid_[nextSonarIndex_]) {
      const double range = hardware_.GetSonarRangeMetres(nextSonarIndex_);
      sonarRanges_M_[nextSonarIndex_] = range;
      sonarCorrectionAccepted_[nextSonarIndex_] =
          localization_.CorrectFromSonar(nextSonarIndex_, range);
      if (sonarCorrectionAccepted_[nextSonarIndex_]) {
        lastSonarRange_M_ = range;
        lastSonarIndex_ = nextSonarIndex_;
      }
    } else {
      sonarRanges_M_[nextSonarIndex_] = -1.0;
    }

    nextSonarIndex_ = (nextSonarIndex_ + 1) % 4;
    sonarAwaitingEcho_ = false;
    lastSonarPingTime_ = nowSeconds;
    return;
  }

  if (nowSeconds - lastSonarPingTime_ >= config::SONAR_PING_SPACING_S) {
    hardware_.PingSonar(nextSonarIndex_);
    sonarPingTime_ = nowSeconds;
    sonarAwaitingEcho_ = true;
  }
}

int main() { return frc::StartRobot<Robot>(); }
