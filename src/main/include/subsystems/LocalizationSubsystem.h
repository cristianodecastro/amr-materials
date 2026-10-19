#pragma once

#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>

#include <string>

#include "Hardware.h"
#include "ForwardDriveController.h"
#include "Localization.h"

class LocalizationSubsystem : public frc2::SubsystemBase {
 public:
  LocalizationSubsystem();
  void Periodic() override;

 private:
  void BeginLocalization();
  void InitializeDashboard();
  void InitializeForwardDashboard();
  void InitializeTeleopDashboard();
  void UpdateTeleopDrive(double dt);
  void UpdateSonarPoll(double nowSeconds, bool robotEnabled);
  void PublishDiagnostics();

  Hardware hardware_;
  Localization localization_{&hardware_};
  ForwardDriveController forwardDrive_;
  frc::Joystick driver_{localization_config::DRIVER_JOYSTICK_PORT};
  bool driverConnected_ = false;
  bool teleopArmed_ = false;
  double driverRawX_ = 0.0;
  double driverRawY_ = 0.0;
  double teleopForward_MMps_ = 0.0;
  double teleopLeft_MMps_ = 0.0;
  std::string teleopStatus_ = "Waiting for Control Station";
  bool started_ = false;
  int startButtonDebounceCount_ = 0;
  int nextSonarIndex_ = 0;
  bool sonarAwaitingEcho_ = false;
  bool sonarPollingEnabled_ = false;
  double sonarPingTime_ = 0.0;
  double lastSonarPingTime_ = 0.0;
  double lastLoopTime_ = 0.0;
  double lastSonarRange_MM_ = -1.0;
  int lastSonarIndex_ = -1;
  double sonarRanges_MM_[4] = {-1.0, -1.0, -1.0, -1.0};
  bool sonarRangeValid_[4] = {false, false, false, false};
  bool sonarEchoReceived_[4] = {false, false, false, false};
  bool sonarCorrectionAccepted_[4] = {false, false, false, false};
  unsigned int sonarPingRequests_[4] = {0, 0, 0, 0};
  unsigned int sonarEchoResponses_[4] = {0, 0, 0, 0};
  unsigned int sonarTimeouts_[4] = {0, 0, 0, 0};
  double sonarLastReadTime_S_[4] = {-1.0, -1.0, -1.0, -1.0};
  const char* sonarStatus_[4] = {"Waiting for robot enable",
                                "Waiting for robot enable",
                                "Waiting for robot enable",
                                "Waiting for robot enable"};
};
