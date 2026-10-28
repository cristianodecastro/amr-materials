#pragma once

#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>

#include "Hardware.h"
#include "Localization.h"
#include "AutonomousController.h"
#include "TeleopDriveController.h"
#include "CobraCheck.h"
#include "LiftTeleopController.h"
#include "ServoTeleopController.h"

class LocalizationSubsystem : public frc2::SubsystemBase {
 public:
  LocalizationSubsystem();
  void Periodic() override;
  // Explicit lifecycle entry/exit: no autonomous state survives a mode change.
  void EnterAutonomous();
  void StopAll();

 private:
  std::string PreflightError() const;
  void InitializeDashboard();
  void InitializeTeleopDashboard();
  void InitializeCobraDashboard();
  void UpdateTeleopDrive(double dt);
  void UpdateSonarPoll(double now, bool enabled);
  void UpdateAutonomous(double now, double dt);
  void PublishDiagnostics();
  bool SonarMotionSafe(double now) const;

  Hardware hardware_;
  Localization localization_{&hardware_};
  AutonomousController autonomous_;
  frc::Joystick driver_{localization_config::DRIVER_JOYSTICK_PORT};
  TeleopDriveController teleopDrive_;
  bool driverConnected_=false;
  double driverRawX_=0,driverRawY_=0;
  bool driverL2_=false,driverR2_=false;
  bool driverPovPresent_=false;
  int driverPov_=-1;
  LiftTeleopController liftTeleop_;
  ServoTeleopController servoTeleop_;
  bool driverCross_=false,driverTriangle_=false;
  bool autoSession_=false;
  bool poseAnchored_=false;
  ObjectAction objectRequest_=ObjectAction::None;
  LineReading line_;
  CobraCheck cobraCheck_;
  nt::NetworkTableEntry captureCobraWhite_,captureCobraBlack_;
  double lastLineCorrection_=0;
  double lastLoopTime_=0;
  int nextSonarIndex_=0;
  bool sonarAwaitingEcho_=false;
  bool sonarSlowAtPing_=false;
  double sonarPingTime_=0,lastSonarPingTime_=0;
  double sonarRanges_MM_[4]={-1,-1,-1,-1};
  double sonarReadTime_[4]={-1,-1,-1,-1};
  bool sonarRangeValid_[4]={false,false,false,false};
  bool sonarCorrectionAccepted_[4]={false,false,false,false};
};
