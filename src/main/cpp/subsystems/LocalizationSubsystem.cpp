#include "subsystems/LocalizationSubsystem.h"

#include <frc/RobotState.h>
#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>
#include <string>

LocalizationSubsystem::LocalizationSubsystem() {
  lastLoopTime_=lastSonarPingTime_=frc::Timer::GetFPGATimestamp();
  hardware_.StopDrive();
  InitializeDashboard();
  InitializeTeleopDashboard();
  InitializeCobraDashboard();
}

std::string LocalizationSubsystem::PreflightError() const {
  using namespace autonomous_config;
  if (!AUTONOMOUS_ENABLED) return "Autonomous disabled by configuration; motors held at zero";
  std::string missing;
  auto require=[&missing](bool condition,const char* name) {
    if (!condition) { if (!missing.empty()) missing+=", "; missing+=name; }
  };
  require(ARENA_AND_FOOTPRINT_CONFIRMED,"arena/footprint");
  require(DRIVE_AND_IMU_FRAME_CONFIRMED,"drive/IMU frame");
  require(WHEEL_SPEED_CONTROL_CONFIRMED,"wheel-speed tuning");
  require(SONAR_MOUNTS_CONFIRMED,"sonar mounts");
  require(START_POSE_CONFIRMED,"start pose");
  require(STATION_ASSIGNMENTS_CONFIRMED,"station assignment");
  require(COBRA_INSTALLED && COBRA_CALIBRATED,"Cobra installation/calibration");
  require(hardware_.IsObjectHandlerReady(),"pickup/release hardware");
  return missing.empty()?missing:"Blocked: configure "+missing;
}

void LocalizationSubsystem::EnterAutonomous() {
  StopAll();
  if (!autonomous_config::AUTONOMOUS_ENABLED) {
    autonomous_.Enter(frc::Timer::GetFPGATimestamp(),PreflightError());
    return;
  }
  autoSession_=true;
  poseAnchored_=false;
  // Snapshot encoder baselines and align existing AHRS yaw to the known arena
  // start heading. Do not issue asynchronous ZeroYaw/encoder resets here.
  localization_.Reset(localization_config::START_X_MM,localization_config::START_Y_MM,
                      localization_config::START_HEADING_DEG);
  const double now=frc::Timer::GetFPGATimestamp();
  lastLoopTime_=now;
  sonarAwaitingEcho_=false;
  nextSonarIndex_=0; lastSonarPingTime_=now;
  for (int i=0;i<4;++i) { sonarRanges_MM_[i]=-1; sonarRangeValid_[i]=false; sonarCorrectionAccepted_[i]=false; }
  autonomous_.Enter(now,PreflightError());
}

void LocalizationSubsystem::StopAll() {
  hardware_.StopDrive();
  hardware_.StopObjectHandler();
  objectRequest_=ObjectAction::None;
  teleopDrive_.Update(false,false,0,0,false,false,localization_config::CONTROL_PERIOD_S);
  liftTeleop_.Update(false,false,false,false,false,false);
  servoTeleop_.Disarm();
  autonomous_.Abort();
  autoSession_=false;
}

void LocalizationSubsystem::Periodic() {
  const double now=frc::Timer::GetFPGATimestamp(),dt=now-lastLoopTime_;
  lastLoopTime_=now;
  const bool enabled=frc::RobotState::IsEnabled();
  const bool autoMode=frc::RobotState::IsAutonomous() && !frc::RobotState::IsTest();

  // Pose remains observable in manual mode; only autonomous start anchors it
  // to an arena placement. Never use manual-mode pose as a global guarantee.
  if (hardware_.IsImuConnected() && !hardware_.IsImuCalibrating()) {
    if (autoSession_ && !poseAnchored_) {
      localization_.Reset(localization_config::START_X_MM,localization_config::START_Y_MM,
          localization_config::START_HEADING_DEG);
      poseAnchored_=true;
    } else localization_.UpdateOdometry(dt);
  }
  line_=hardware_.ReadLine();
  // Consume each capture request once; no dashboard entry commands movement.
  const bool captureWhite=captureCobraWhite_.GetBoolean(false);
  const bool captureBlack=captureCobraBlack_.GetBoolean(false);
  if (captureWhite) captureCobraWhite_.SetBoolean(false);
  if (captureBlack) captureCobraBlack_.SetBoolean(false);
  cobraCheck_.Update(line_,now,!enabled,captureWhite,captureBlack);
  UpdateSonarPoll(now,enabled);

  if (enabled && autoMode) {
    if (!autonomous_config::AUTONOMOUS_ENABLED || !autoSession_) {
      hardware_.StopDrive(); // require explicit AutonomousInit, never resume
      hardware_.StopObjectHandler();
    } else UpdateAutonomous(now,dt);
  } else {
    if (autoSession_) StopAll();
    UpdateTeleopDrive(dt);
  }
  PublishDiagnostics();
}

void LocalizationSubsystem::UpdateAutonomous(double now,double dt) {
  if (!autonomous_config::AUTONOMOUS_ENABLED) { StopAll(); return; }
  bool sonarReady=true;
  if (autonomous_.GetState()==AutonomousController::State::WaitingStart) {
    for (int i=0;i<4;++i)
      sonarReady=sonarReady && sonarRangeValid_[i] && now-sonarReadTime_[i]<=autonomous_config::SONAR_STALE_S;
  }
  const AutonomousInput input={localization_.GetPose(),line_,now,dt,
      hardware_.GetStartPressed(),hardware_.IsImuConnected() &&
      !hardware_.IsImuCalibrating() && localization_.IsHealthy() && sonarReady,
      hardware_.GetObjectActionStatus()};
  autonomous_.Update(input);
  const auto action=autonomous_.GetObjectRequest();
  if (action!=objectRequest_) {
    hardware_.StopObjectHandler();
    objectRequest_=action;
    if (action!=ObjectAction::None) hardware_.BeginObjectAction(action);
  }
  if (autonomous_.IsRunning() && !SonarMotionSafe(now)) {
    autonomous_.SafetyStop("Stopped: close obstacle or stale sonar in commanded direction");
  }
  if (autonomous_.IsRunning()) {
    hardware_.DriveRobotRelativeFeedback(autonomous_.GetForwardMMps(),autonomous_.GetLeftMMps(),
        autonomous_.GetYawRadps(),autonomous_config::MAX_OUTPUT,dt);
  } else {
    hardware_.StopDrive();
    hardware_.StopObjectHandler();
  }
  // Avoid treating highly correlated 50 Hz tape samples as independent pose
  // measurements. The controller still follows the tape on every fresh read.
  if (autonomous_.CanCorrectFromLine() && line_.valid && now-lastLineCorrection_>=0.20) {
    localization_.CorrectFromLine(autonomous_.GetStation(),line_.leftErrorMM);
    lastLineCorrection_=now;
  }
}

bool LocalizationSubsystem::SonarMotionSafe(double now) const {
  const double vx=autonomous_.GetForwardMMps(),vy=autonomous_.GetLeftMMps(),w=autonomous_.GetYawRadps();
  for (int i=0;i<4;++i) {
    const auto& sensor=localization_config::SONARS[i];
    const double a=robot_geometry::Radians(sensor.beamAngleDeg),c=std::cos(a),s=std::sin(a);
    const double closing=vx*c+vy*s+w*(-sensor.mountY_MM*c+sensor.mountX_MM*s);
    if (closing<=5) continue;
    if (!sonarRangeValid_[i] || now-sonarReadTime_[i]>autonomous_config::SONAR_STALE_S) return false;
    const double age=now-sonarReadTime_[i]+localization_config::SONAR_ECHO_SETTLE_S;
    const double envelope=autonomous_config::ROBOT_ENVELOPE_RADIUS_MM+autonomous_config::CLEARANCE_MM-
        (sensor.mountX_MM*c+sensor.mountY_MM*s);
    const double stopDistance=closing*closing/(2*autonomous_config::ACCEL_MMPS2)+closing*age;
    if (sonarRanges_MM_[i]<envelope+stopDistance) return false;
  }
  return true;
}

void LocalizationSubsystem::UpdateTeleopDrive(double dt) {
  auto& ds=frc::DriverStation::GetInstance();
  const bool attached=ds.IsDSAttached();
  const int xAxis=localization_config::DRIVER_LEFT_X_AXIS;
  const int yAxis=localization_config::DRIVER_LEFT_Y_AXIS;
  const int l2=localization_config::DRIVER_L2_BUTTON,r2=localization_config::DRIVER_R2_BUTTON;
  const int axes=attached?driver_.GetAxisCount():0;
  const int buttons=attached?driver_.GetButtonCount():0;
  const bool axesPresent=xAxis>=0 && yAxis>=0 && xAxis!=yAxis && axes>std::max(xAxis,yAxis);
  const bool buttonsPresent=l2>0 && r2>0 && l2!=r2 && buttons>=std::max(l2,r2);
  driverConnected_=attached && axesPresent && buttonsPresent;
  driverRawX_=axesPresent?driver_.GetRawAxis(xAxis):0;
  driverRawY_=axesPresent?driver_.GetRawAxis(yAxis):0;
  driverL2_=buttonsPresent && driver_.GetRawButton(l2);
  driverR2_=buttonsPresent && driver_.GetRawButton(r2);
  const int povIndex=manipulator_config::DRIVER_POV_INDEX;
  driverPovPresent_=attached && povIndex>=0 && driver_.GetPOVCount()>povIndex;
  driverPov_=driverPovPresent_?driver_.GetPOV(povIndex):-1;
  const bool liftUp=driverPov_==manipulator_config::DRIVER_POV_UP_DEG;
  const bool liftDown=driverPov_==manipulator_config::DRIVER_POV_DOWN_DEG;
  const bool allowed=localization_config::TELEOP_DRIVE_ENABLED &&
      frc::RobotState::IsEnabled() && frc::RobotState::IsOperatorControl() && !frc::RobotState::IsTest();
  teleopDrive_.Update(allowed,driverConnected_,driverRawX_,driverRawY_,driverL2_,driverR2_,dt,
      driverPov_==-1);
  if (allowed && teleopDrive_.IsArmed())
    hardware_.DriveRobotRelative(teleopDrive_.GetForwardMMps(),teleopDrive_.GetLeftMMps(),
        teleopDrive_.GetYawRateRadps(),localization_config::TELEOP_MAX_MOTOR_OUTPUT);
  else hardware_.StopDrive();
  liftTeleop_.Update(allowed && teleopDrive_.IsArmed(),driverConnected_ && driverPovPresent_,
      liftUp,liftDown,hardware_.GetLiftUpperLimit(),hardware_.GetLiftLowerLimit());
  hardware_.SetLiftPwm(liftTeleop_.Output());
  const int cross=manipulator_config::SERVO_BUTTON[0];
  const int triangle=manipulator_config::SERVO_BUTTON[1];
  const bool servoButtonsPresent=cross>0 && triangle>0 && cross!=triangle &&
      buttons>=std::max(cross,triangle);
  driverCross_=servoButtonsPresent && driver_.GetRawButton(cross);
  driverTriangle_=servoButtonsPresent && driver_.GetRawButton(triangle);
  servoTeleop_.Update(allowed && teleopDrive_.IsArmed(),driverConnected_ && servoButtonsPresent,
      driverCross_,driverTriangle_);
  for (int i=0;i<2;++i)
    hardware_.SetServoAngle(i,servoTeleop_.TargetDegrees(i),servoTeleop_.OutputEnabled(i));
}

void LocalizationSubsystem::UpdateSonarPoll(double now,bool enabled) {
  if (!enabled) {
    sonarAwaitingEcho_=false;
    nextSonarIndex_=0; lastSonarPingTime_=now;
    for (int i=0;i<4;++i) { sonarRanges_MM_[i]=-1; sonarRangeValid_[i]=sonarCorrectionAccepted_[i]=false; }
    return;
  }
  const bool slow=std::hypot(localization_.GetBodyVelocityX_MMps(),localization_.GetBodyVelocityY_MMps())
          <=autonomous_config::SONAR_FUSION_MAX_SPEED_MMPS &&
      std::fabs(hardware_.GetYawRateDegreesPerSecond())<=autonomous_config::SONAR_FUSION_MAX_YAW_DPS;
  if (sonarAwaitingEcho_) {
    const double age=now-sonarPingTime_;
    if (age<localization_config::SONAR_ECHO_SETTLE_S) return;
    const int i=nextSonarIndex_;
    sonarReadTime_[i]=now;
    const double range=hardware_.GetSonarRangeMM(i);
    sonarRangeValid_[i]=hardware_.IsSonarRangeValid(i) && std::isfinite(range) &&
        range>=localization_config::SONAR_MIN_RANGE_MM && range<=localization_config::SONAR_MAX_RANGE_MM;
    sonarRanges_MM_[i]=sonarRangeValid_[i]?range:-1;
    sonarCorrectionAccepted_[i]=false;
    // Global wall correction is valid only after explicit arena initialization.
    if (autoSession_ && autonomous_.IsRunning() && sonarRangeValid_[i] && slow && sonarSlowAtPing_ &&
        age<=autonomous_config::SONAR_FUSION_MAX_AGE_S && autonomous_config::SONAR_MOUNTS_CONFIRMED)
      sonarCorrectionAccepted_[i]=localization_.CorrectFromSonar(i,range);
    nextSonarIndex_=(i+1)%4; sonarAwaitingEcho_=false; lastSonarPingTime_=now;
  } else if (now-lastSonarPingTime_>=localization_config::SONAR_PING_SPACING_S) {
    hardware_.PingSonar(nextSonarIndex_);
    sonarPingTime_=now; sonarSlowAtPing_=slow; sonarAwaitingEcho_=true;
  }
}

void LocalizationSubsystem::InitializeDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Autonomous");
  tab.AddString("Mission status",[this] { return autonomous_.GetStatus(); }).WithPosition(0,0).WithSize(8,1);
  tab.AddBoolean("Enabled",[] { return frc::RobotState::IsEnabled(); }).WithPosition(8,0).WithSize(2,1);
  tab.AddNumber("X (mm)",[this] { return localization_.GetPose().x_MM; }).WithPosition(0,1).WithSize(3,1);
  tab.AddNumber("Y (mm)",[this] { return localization_.GetPose().y_MM; }).WithPosition(3,1).WithSize(3,1);
  tab.AddNumber("Heading (deg)",[this] { return localization_.GetPose().headingRad*180/localization_config::PI; }).WithPosition(6,1).WithSize(3,1);
  tab.AddNumber("Confirmed deliveries",[this] { return autonomous_.GetCompletedJobs(); }).WithPosition(9,1).WithSize(3,1);
  const char* names[4]={"Rear left (mm)","Rear right (mm)","Left (mm)","Right (mm)"};
  for (int i=0;i<4;++i)
    tab.AddNumber(names[i],[this,i] { return sonarRanges_MM_[i]; }).WithPosition(i*3,2).WithSize(3,1);
  tab.AddBoolean("Tape detected",[this] { return line_.valid; }).WithPosition(0,3).WithSize(2,1);
  tab.AddNumber("Tape offset (mm)",[this] { return line_.valid?line_.leftErrorMM:0; }).WithPosition(2,3).WithSize(2,1);
  tab.AddBoolean("IMU ready",[this] { return hardware_.IsImuConnected() && !hardware_.IsImuCalibrating(); }).WithPosition(4,3).WithSize(2,1);
  tab.AddNumber("Position sigma (mm)",[this] { return localization_.GetPositionSigmaMM(); }).WithPosition(6,3).WithSize(2,1);
  tab.AddString("Station",[this] { return std::string(autonomous_.GetStation().name); }).WithPosition(8,3).WithSize(2,1);
  tab.AddNumber("Job",[this] { return autonomous_.GetJobNumber(); }).WithPosition(10,3).WithSize(2,1);

  // A separate calibration tab keeps the operating screen uncluttered.
  auto& calibration=frc::Shuffleboard::GetTab("Calibration");
  calibration.AddString("Preflight",[this] { return PreflightError(); }).WithPosition(0,0).WithSize(12,1);
  for (int i=0;i<4;++i)
    calibration.AddNumber("Cobra "+std::to_string(i)+" (V)",[this,i] { return line_.volts[i]; }).WithPosition(i*3,1).WithSize(3,1);
  calibration.AddNumber("IMU world X (g)",[this] { return hardware_.GetWorldAccelX_G(); }).WithPosition(0,2).WithSize(3,1);
  calibration.AddNumber("IMU world Y (g)",[this] { return hardware_.GetWorldAccelY_G(); }).WithPosition(3,2).WithSize(3,1);
  calibration.AddNumber("IMU yaw (deg)",[this] { return hardware_.GetYawDegrees(); }).WithPosition(6,2).WithSize(3,1);
  calibration.AddNumber("Yaw rate (deg/s)",[this] { return hardware_.GetYawRateDegreesPerSecond(); }).WithPosition(9,2).WithSize(3,1);
}

void LocalizationSubsystem::InitializeTeleopDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Teleop");
  tab.AddString("Drive status",[this] { return std::string(teleopDrive_.GetStatus()); }).WithPosition(0,0).WithSize(6,1);
  tab.AddBoolean("Enabled",[] { return frc::RobotState::IsEnabled(); }).WithPosition(6,0).WithSize(2,1);
  tab.AddBoolean("Controller",[this] { return driverConnected_; }).WithPosition(8,0).WithSize(2,1);
  tab.AddBoolean("Stick armed",[this] { return teleopDrive_.IsArmed(); }).WithPosition(10,0).WithSize(2,1);
  tab.AddNumber("Left stick X",[this] { return driverRawX_; }).WithPosition(0,1).WithSize(3,1);
  tab.AddNumber("Left stick Y",[this] { return driverRawY_; }).WithPosition(3,1).WithSize(3,1);
  tab.AddNumber("Forward request (mm/s)",[this] { return teleopDrive_.GetForwardMMps(); }).WithPosition(6,1).WithSize(3,1);
  tab.AddNumber("Left request (mm/s)",[this] { return teleopDrive_.GetLeftMMps(); }).WithPosition(9,1).WithSize(3,1);
  for (int i=0;i<3;++i) {
    const std::string port=std::string(localization_config::WHEEL_NAME[i])+" M"+
        std::to_string(localization_config::WHEEL_TITAN_PORT[i]);
    tab.AddNumber(port+" encoder (counts)",[this,i] { return hardware_.GetWheelEncoderRaw(i); }).WithPosition(i*2,2).WithSize(2,1);
    tab.AddNumber(port+" output (%)",[this,i] { return hardware_.GetWheelCommand(i)*100; }).WithPosition(6+i*2,2).WithSize(2,1);
  }
  tab.AddBoolean("L2 - CW",[this] { return driverL2_; }).WithPosition(0,3).WithSize(3,1);
  tab.AddBoolean("R2 - CCW",[this] { return driverR2_; }).WithPosition(3,3).WithSize(3,1);
  tab.AddNumber("Rotation request (deg/s)",[this] { return teleopDrive_.GetYawRateRadps()*180/localization_config::PI; }).WithPosition(6,3).WithSize(3,1);
  tab.AddNumber("Output limit (%)",[] { return localization_config::TELEOP_MAX_MOTOR_OUTPUT*100; }).WithPosition(9,3).WithSize(3,1);
  tab.AddString("Lift status",[this] { return std::string(liftTeleop_.Status()); })
      .WithPosition(0,4).WithSize(4,1);
  tab.AddNumber("D-pad POV (deg)",[this] { return driverPov_; }).WithPosition(4,4).WithSize(2,1);
  tab.AddBoolean("D-pad present",[this] { return driverPovPresent_; }).WithPosition(6,4).WithSize(2,1);
  tab.AddNumber("Lift M1 output (%)",[this] { return hardware_.GetLiftCommand()*100; }).WithPosition(8,4).WithSize(2,1);
  tab.AddNumber("Lift encoder (counts)",[this] { return hardware_.GetLiftEncoderRaw(); }).WithPosition(10,4).WithSize(2,1);
  tab.AddBoolean("X button",[this] { return driverCross_; }).WithPosition(0,5).WithSize(2,1);
  tab.AddNumber("FlexDIO 8 target (deg)",[this] { return servoTeleop_.TargetDegrees(0); }).WithPosition(2,5).WithSize(2,1);
  tab.AddBoolean("Servo 8 output enabled",[this] { return servoTeleop_.OutputEnabled(0); }).WithPosition(4,5).WithSize(2,1);
  tab.AddBoolean("Triangle button",[this] { return driverTriangle_; }).WithPosition(6,5).WithSize(2,1);
  tab.AddNumber("FlexDIO 9 target (deg)",[this] { return servoTeleop_.TargetDegrees(1); }).WithPosition(8,5).WithSize(2,1);
  tab.AddBoolean("Servo 9 output enabled",[this] { return servoTeleop_.OutputEnabled(1); }).WithPosition(10,5).WithSize(2,1);
}

void LocalizationSubsystem::InitializeCobraDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Cobra check");
  tab.AddString("Instructions",[this] { return cobraCheck_.Status(); })
      .WithPosition(0,0).WithSize(6,1);
  captureCobraWhite_=tab.Add("Capture white",false)
      .WithWidget(frc::BuiltInWidgets::kToggleButton).WithPosition(6,0).WithSize(3,1).GetEntry();
  captureCobraBlack_=tab.Add("Capture black",false)
      .WithWidget(frc::BuiltInWidgets::kToggleButton).WithPosition(9,0).WithSize(3,1).GetEntry();
  // Discard old dashboard requests on program restart.
  captureCobraWhite_.SetBoolean(false); captureCobraBlack_.SetBoolean(false);
  for (int i=0;i<4;++i) {
    const std::string channel="CH"+std::to_string(i);
    tab.AddNumber(channel+" live (V)",[this,i] { return line_.volts[i]; })
        .WithPosition(i*3,1).WithSize(3,1);
    tab.AddBoolean(channel+" BLACK",[this,i] { return cobraCheck_.IsBlack(i); })
        .WithPosition(i*3,2).WithSize(3,1);
    tab.AddNumber(channel+" white reference (V)",[this,i] { return cobraCheck_.White(i); })
        .WithPosition(i*3,3).WithSize(3,1);
    tab.AddNumber(channel+" black reference (V)",[this,i] { return cobraCheck_.Black(i); })
        .WithPosition(i*3,4).WithSize(3,1);
  }
}

void LocalizationSubsystem::PublishDiagnostics() {
  if (!localization_config::ENABLE_DETAILED_TELEMETRY) return;
  for (int i=0;i<4;++i)
    frc::SmartDashboard::PutBoolean("Diagnostics/Sonar/"+std::to_string(i)+"/fused",sonarCorrectionAccepted_[i]);
  frc::SmartDashboard::PutNumber("Diagnostics/Auto/target_x_mm",autonomous_.GetTarget().x);
  frc::SmartDashboard::PutNumber("Diagnostics/Auto/target_y_mm",autonomous_.GetTarget().y);
  localization_.PublishTelemetry(-1,-1);
}
