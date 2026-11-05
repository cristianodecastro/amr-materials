#include "RobotControl.h"

#include <frc/RobotState.h>
#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>
#include <string>

RobotSubsystem::RobotSubsystem() {
  lastLoopTime_=lastSonarPingTime_=frc::Timer::GetFPGATimestamp();
  hardware_.StopDrive();
  InitializeDashboard();
  InitializeTeleopDashboard();
  InitializeCobraDashboard();
}

std::string RobotSubsystem::PreflightError() const {
  if (arena_test_config::ENABLED) {
    const auto error=arenaTest_.ValidateGeometry();
    return error.empty()?"SP -> MW -> RDW test ready; select Autonomous, enable, press Start routine":error;
  }
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
  require(MANIPULATOR_READY,"pickup/release hardware");
  return missing.empty()?missing:"Blocked: configure "+missing;
}

void RobotSubsystem::EnterAutonomous() {
  StopAll();
  if (arena_test_config::ENABLED) {
    autoSession_=true; poseAnchored_=false;
    const double now=frc::Timer::GetFPGATimestamp();
    lastLoopTime_=now;
    startRoutine_.SetBoolean(false); stopRoutine_.SetBoolean(false);
    for (int i=0;i<4;++i) sonarCorrectionTime_[i]=-1;
    arenaTest_.Enter(now);
    return;
  }
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

void RobotSubsystem::StopAll() {
  hardware_.SetAutonomousDriveEnabled(false);
  hardware_.StopDrive();
  hardware_.StopManipulator();
  teleopDrive_.Update(false,false,0,0,0,localization_config::CONTROL_PERIOD_S);
  liftTeleop_.Update(false,false,false,false,false,false);
  servoTeleop_.Disarm();
  autonomous_.Abort();
  arenaTest_.Abort();
  startRoutine_.SetBoolean(false); stopRoutine_.SetBoolean(false);
  autoSession_=false;
}

void RobotSubsystem::Periodic() {
  const double now=frc::Timer::GetFPGATimestamp(),dt=now-lastLoopTime_;
  lastLoopTime_=now;
  const bool enabled=frc::RobotState::IsEnabled();
  const bool autoMode=frc::RobotState::IsAutonomous() && !frc::RobotState::IsTest();

  // Pose remains observable in manual mode; only autonomous start anchors it
  // to an arena placement. Never use manual-mode pose as a global guarantee.
  if (hardware_.IsImuConnected() && !hardware_.IsImuCalibrating()) {
    if (autoSession_ && !poseAnchored_ && !arena_test_config::ENABLED) {
      localization_.Reset(localization_config::START_X_MM,localization_config::START_Y_MM,
          localization_config::START_HEADING_DEG);
      poseAnchored_=true;
    } else localization_.UpdateOdometry(dt);
  }
  line_=hardware_.ReadLine();
  // Consume dashboard button pulses in every mode. Disabled/Teleop presses
  // cannot remain queued and start a routine when Autonomous is enabled later.
  startRoutineRequested_=startRoutine_.GetBoolean(false);
  stopRoutineRequested_=stopRoutine_.GetBoolean(false);
  if (startRoutineRequested_) startRoutine_.SetBoolean(false);
  if (stopRoutineRequested_) stopRoutine_.SetBoolean(false);
  // Consume each capture request once; no dashboard entry commands movement.
  const bool captureWhite=captureCobraWhite_.GetBoolean(false);
  const bool captureBlack=captureCobraBlack_.GetBoolean(false);
  if (captureWhite) captureCobraWhite_.SetBoolean(false);
  if (captureBlack) captureCobraBlack_.SetBoolean(false);
  cobraCheck_.Update(line_,now,!enabled,captureWhite,captureBlack);
  UpdateSonarPoll(now,enabled);

  if (enabled && autoMode) {
    if ((!autonomous_config::AUTONOMOUS_ENABLED && !arena_test_config::ENABLED) || !autoSession_) {
      hardware_.StopDrive(); // require explicit AutonomousInit, never resume
      hardware_.StopManipulator();
    } else UpdateAutonomous(now,dt);
  } else {
    if (autoSession_) StopAll();
    UpdateTeleopDrive(dt);
  }
  PublishDiagnostics();
}

void RobotSubsystem::UpdateAutonomous(double now,double dt) {
  if (arena_test_config::ENABLED) { UpdateArenaTest(now,dt); return; }
  if (!autonomous_config::AUTONOMOUS_ENABLED) { StopAll(); return; }
  bool sonarReady=true;
  if (autonomous_.GetState()==AutonomousController::State::WaitingStart) {
    for (int i=0;i<4;++i)
      sonarReady=sonarReady && sonarRangeValid_[i] && now-sonarReadTime_[i]<=autonomous_config::SONAR_STALE_S;
  }
  const AutonomousInput input={localization_.GetPose(),line_,now,dt,
      false, // No physical start input is installed.
      hardware_.IsImuConnected() &&
      !hardware_.IsImuCalibrating() && localization_.IsHealthy() && sonarReady,
      ObjectActionStatus::Unavailable};
  autonomous_.Update(input);
  // Automatic object handling is blocked until a real mechanism is implemented.
  if (autonomous_.IsRunning() && !SonarMotionSafe(now)) {
    autonomous_.SafetyStop("Stopped: close obstacle or stale sonar in commanded direction");
  }
  if (autonomous_.IsRunning()) {
    hardware_.DriveRobotRelative(autonomous_.GetForwardMMps(),autonomous_.GetLeftMMps(),
        autonomous_.GetYawRadps(),autonomous_config::MAX_OUTPUT,dt);
  } else {
    hardware_.StopDrive();
    hardware_.StopManipulator();
  }
  // Avoid treating highly correlated 50 Hz tape samples as independent pose
  // measurements. The controller still follows the tape on every fresh read.
  if (autonomous_.CanCorrectFromLine() && line_.valid && now-lastLineCorrection_>=0.20) {
    localization_.CorrectFromLine(autonomous_.GetStation(),line_.leftErrorMM);
    lastLineCorrection_=now;
  }
}

void RobotSubsystem::UpdateArenaTest(double now,double dt) {
  const bool imuReady=hardware_.IsImuConnected() && !hardware_.IsImuCalibrating();
  if (startRoutineRequested_ && !stopRoutineRequested_ && imuReady &&
      arenaTest_.GetState()==ArenaTestController::State::WaitingStart) {
    // The explicit start click anchors the manually placed robot to SP.
    // Encoder baselines and IMU offset are sampled, without resetting hardware.
    const auto& sp=arena_test_config::SP;
    localization_.Reset(sp.x_MM,sp.y_MM,sp.headingDeg);
    poseAnchored_=true;
    for (int i=0;i<4;++i) sonarCorrectionTime_[i]=-1;
  }
  const bool stationary=std::hypot(localization_.GetBodyVelocityX_MMps(),
      localization_.GetBodyVelocityY_MMps())<=arena_test_config::STATIONARY_SPEED_MMPS &&
      std::fabs(hardware_.GetYawRateDegreesPerSecond())<=arena_test_config::STATIONARY_YAW_DPS;
  ArenaTestInput in={localization_.GetPose(),now,dt,
      imuReady && frc::DriverStation::GetInstance().IsDSAttached() &&
          localization_.IsHealthy(arena_test_config::MAX_POSITION_SIGMA_MM),
      stationary,startRoutineRequested_,stopRoutineRequested_,{}};
  for (int i=0;i<4;++i) in.correctionTime[i]=sonarCorrectionTime_[i];
  if (hardware_.HasDriveFault() && arenaTest_.IsRunning()) arenaTest_.Fail(hardware_.GetDriveStatus());
  arenaTest_.Update(in);
  hardware_.SetAutonomousDriveEnabled(arenaTest_.IsRunning());
  if (arenaTest_.IsRunning()) {
    hardware_.DriveRobotRelative(arenaTest_.GetForwardMMps(),arenaTest_.GetLeftMMps(),
        arenaTest_.GetYawRadps(),arena_test_config::MAX_OUTPUT,dt);
    if (hardware_.HasDriveFault()) {
      arenaTest_.Fail(hardware_.GetDriveStatus());
      hardware_.SetAutonomousDriveEnabled(false);
      hardware_.StopDrive();
    }
  } else hardware_.StopDrive();
  // This first test never actuates the lift or either servo.
  hardware_.StopManipulator();
}

bool RobotSubsystem::SonarMotionSafe(double now) const {
  const double vx=autonomous_.GetForwardMMps(),vy=autonomous_.GetLeftMMps(),w=autonomous_.GetYawRadps();
  for (int i=0;i<4;++i) {
    const auto& sensor=localization_config::SONARS[i];
    const double a=robot_geometry::Radians(sensor.beamAngleDeg),c=std::cos(a),s=std::sin(a);
    const double closing=vx*c+vy*s+w*(-sensor.mountY_MM*c+sensor.mountX_MM*s);
    if (closing<=5) continue;
    if (!sonarRangeValid_[i] || now-sonarReadTime_[i]>autonomous_config::SONAR_STALE_S) return false;
    const double age=now-sonarReadTime_[i]+localization_config::SONAR_ECHO_SETTLE_S;
    const double envelope=autonomous_config::CLEARANCE_MM;
    const double stopDistance=closing*closing/(2*autonomous_config::ACCEL_MMPS2)+closing*age;
    if (sonarRanges_MM_[i]<envelope+stopDistance) return false;
  }
  return true;
}

void RobotSubsystem::UpdateTeleopDrive(double dt) {
  auto& ds=frc::DriverStation::GetInstance();
  const bool attached=ds.IsDSAttached();
  const int xAxis=localization_config::DRIVER_LEFT_X_AXIS;
  const int yAxis=localization_config::DRIVER_LEFT_Y_AXIS;
  const int rightXAxis=localization_config::DRIVER_RIGHT_X_AXIS;
  const int axes=attached?driver_.GetAxisCount():0;
  const int buttons=attached?driver_.GetButtonCount():0;
  const bool axesPresent=xAxis>=0 && yAxis>=0 && rightXAxis>=0 &&
      xAxis!=yAxis && rightXAxis!=xAxis && rightXAxis!=yAxis &&
      axes>std::max(rightXAxis,std::max(xAxis,yAxis));
  driverConnected_=attached && axesPresent;
  driverRawX_=axesPresent?driver_.GetRawAxis(xAxis):0;
  driverRawY_=axesPresent?driver_.GetRawAxis(yAxis):0;
  driverRightX_=axesPresent?driver_.GetRawAxis(rightXAxis):0;
  const int povIndex=manipulator_config::DRIVER_POV_INDEX;
  driverPovPresent_=attached && povIndex>=0 && driver_.GetPOVCount()>povIndex;
  driverPov_=driverPovPresent_?driver_.GetPOV(povIndex):-1;
  const bool liftUp=driverPov_==manipulator_config::DRIVER_POV_UP_DEG;
  const bool liftDown=driverPov_==manipulator_config::DRIVER_POV_DOWN_DEG;
  const bool allowed=localization_config::TELEOP_DRIVE_ENABLED &&
      frc::RobotState::IsEnabled() && frc::RobotState::IsOperatorControl() && !frc::RobotState::IsTest();
  teleopDrive_.Update(allowed,driverConnected_,driverRawX_,driverRawY_,driverRightX_,dt,
      driverPov_==-1);
  if (allowed && teleopDrive_.IsArmed()) {
    // Teleop yaw comes only from right X; the gyro is observed for odometry only.
    hardware_.DriveRobotRelative(teleopDrive_.GetForwardMMps(),teleopDrive_.GetLeftMMps(),
        teleopDrive_.GetYawRateRadps(),
        localization_config::TELEOP_MAX_MOTOR_OUTPUT,dt);
  } else {
    hardware_.StopDrive();
  }
  liftTeleop_.Update(allowed && teleopDrive_.IsArmed(),driverConnected_ && driverPovPresent_,
      liftUp,liftDown,hardware_.GetLiftUpperLimit(),hardware_.GetLiftLowerLimit());
  hardware_.SetLiftPwm(liftTeleop_.Output());
  const int triangle=manipulator_config::SERVO_TRIANGLE_BUTTON;
  const int l1=manipulator_config::SERVO_STEP_BUTTON[0];
  const int r1=manipulator_config::SERVO_STEP_BUTTON[1];
  const bool servoButtonsPresent=triangle>0 && l1>0 && r1>0 &&
      l1!=r1 && l1!=triangle && r1!=triangle && buttons>=std::max(triangle,std::max(l1,r1));
  driverTriangle_=servoButtonsPresent && driver_.GetRawButton(triangle);
  driverL1_=servoButtonsPresent && driver_.GetRawButton(l1);
  driverR1_=servoButtonsPresent && driver_.GetRawButton(r1);
  servoTeleop_.Update(allowed && teleopDrive_.IsArmed(),driverConnected_ && servoButtonsPresent,
      driverTriangle_,driverL1_,driverR1_);
  for (int i=0;i<2;++i)
    hardware_.SetServoAngle(i,servoTeleop_.TargetDegrees(i),servoTeleop_.OutputEnabled(i));
}

void RobotSubsystem::UpdateSonarPoll(double now,bool enabled) {
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
    if (autoSession_ && poseAnchored_ && sonarRangeValid_[i] && slow && sonarSlowAtPing_ &&
        age<=autonomous_config::SONAR_FUSION_MAX_AGE_S) {
      if (arena_test_config::ENABLED && (arenaTest_.ReferenceMask()&(1u<<i))) {
        // Only stationary reference states may change the pose. Ignore invalid
        // echoes, old pings, moving samples and a different wall association.
        const auto& target=arena_test_config::MW;
        const auto& reference=arenaTest_.GetState()==ArenaTestController::State::ReferenceMW?
            target:arena_test_config::RDW;
        const bool stationary=std::hypot(localization_.GetBodyVelocityX_MMps(),
            localization_.GetBodyVelocityY_MMps())<=arena_test_config::STATIONARY_SPEED_MMPS &&
            std::fabs(hardware_.GetYawRateDegreesPerSecond())<=arena_test_config::STATIONARY_YAW_DPS;
        const bool aligned=std::fabs(robot_geometry::Wrap(localization_.GetPose().headingRad-
            robot_geometry::Radians(reference.headingDeg)))<=arena_test_config::HEADING_TOLERANCE_RAD;
        const int wall=arenaTest_.ReferenceSurface(i);
        if (stationary && aligned && wall>=0)
          sonarCorrectionAccepted_[i]=localization_.CorrectFromSonar(i,range,wall);
      } else if (!arena_test_config::ENABLED && autonomous_.IsRunning() &&
          autonomous_config::SONAR_MOUNTS_CONFIRMED) {
        sonarCorrectionAccepted_[i]=localization_.CorrectFromSonar(i,range);
      }
      if (sonarCorrectionAccepted_[i]) sonarCorrectionTime_[i]=now;
    }
    nextSonarIndex_=(i+1)%4; sonarAwaitingEcho_=false; lastSonarPingTime_=now;
  } else if (now-lastSonarPingTime_>=localization_config::SONAR_PING_SPACING_S) {
    hardware_.PingSonar(nextSonarIndex_);
    sonarPingTime_=now; sonarSlowAtPing_=slow; sonarAwaitingEcho_=true;
  }
}

void RobotSubsystem::InitializeDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Autonomous");
  tab.AddString("Mission status",[this] {
    return arena_test_config::ENABLED?arenaTest_.GetStatus():autonomous_.GetStatus();
  }).WithPosition(0,0).WithSize(8,1);
  tab.AddBoolean("Enabled",[] { return frc::RobotState::IsEnabled(); }).WithPosition(8,0).WithSize(2,1);
  tab.AddNumber("Target heading (deg)",[this] { return arenaTest_.GetHeadingTargetDeg(); }).WithPosition(10,0).WithSize(2,1);
  tab.AddNumber("X (mm)",[this] { return localization_.GetPose().x_MM; }).WithPosition(0,1).WithSize(3,1);
  tab.AddNumber("Y (mm)",[this] { return localization_.GetPose().y_MM; }).WithPosition(3,1).WithSize(3,1);
  tab.AddNumber("Heading (deg)",[this] { return localization_.GetPose().headingRad*180/robot_geometry::PI; }).WithPosition(6,1).WithSize(3,1);
  tab.AddNumber("Position sigma (mm)",[this] { return localization_.GetPositionSigmaMM(); }).WithPosition(9,1).WithSize(3,1);
  tab.AddString("Ultrasonic status",[] {
    return frc::RobotState::IsEnabled()?"Polling - distances from sensor face; -1 = no valid echo":
        "Paused while Disabled - VMX FlexDIO outputs require Enable";
  }).WithPosition(0,4).WithSize(12,1);
  for (int i=0;i<4;++i) {
    const std::string name=localization_config::SONARS[i].name;
    tab.AddNumber(name+" (mm)",[this,i] { return sonarRanges_MM_[i]; }).WithPosition(i*3,2).WithSize(3,1);
    tab.AddBoolean(name+" valid",[this,i] { return sonarRangeValid_[i]; }).WithPosition(i*3,5).WithSize(3,1);
  }
  startRoutine_=tab.Add("Start routine",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(0,3).WithSize(3,1).GetEntry();
  stopRoutine_=tab.Add("Stop routine",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(3,3).WithSize(3,1).GetEntry();
  startRoutine_.SetBoolean(false); stopRoutine_.SetBoolean(false);
  tab.AddNumber("Target X (mm)",[this] { return arenaTest_.GetTarget().x; }).WithPosition(6,3).WithSize(3,1);
  tab.AddNumber("Target Y (mm)",[this] { return arenaTest_.GetTarget().y; }).WithPosition(9,3).WithSize(3,1);
  tab.AddString("Reference progress",[this] { return arenaTest_.GetReferenceReport(); }).WithPosition(0,6).WithSize(12,1);
  tab.AddString("Reference results",[this] { return arenaTest_.GetLastReferenceReport(); }).WithPosition(0,7).WithSize(12,1);
  tab.AddString("Test instructions",[] {
    return "Place at SP; select Autonomous, enable, then Start routine. Disable/Stop aborts; reselect Autonomous to restart.";
  }).WithPosition(0,8).WithSize(12,1);
  tab.AddNumber("Heading error (deg)",[this] {
    return robot_geometry::Wrap(robot_geometry::Radians(arenaTest_.GetHeadingTargetDeg())-
        localization_.GetPose().headingRad)*180/robot_geometry::PI;
  }).WithPosition(0,9).WithSize(3,1);
  tab.AddNumber("Yaw request (deg/s)",[this] { return arenaTest_.GetYawRadps()*180/robot_geometry::PI; })
      .WithPosition(3,9).WithSize(3,1);
  tab.AddString("Fusion status",[this] { return localization_.GetFusionStatus(); }).WithPosition(6,9).WithSize(6,1);

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

void RobotSubsystem::InitializeTeleopDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Teleop");
  tab.AddString("Drive status",[this] {
    return hardware_.HasDriveFault() || hardware_.IsDriveStarting()?
        hardware_.GetDriveStatus():std::string(teleopDrive_.GetStatus());
  }).WithPosition(0,0).WithSize(6,1);
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
  tab.AddNumber("Right stick X",[this] { return driverRightX_; }).WithPosition(0,3).WithSize(3,1);
  tab.AddString("Rotation control",[] { return "Right stick: left CCW / right CW"; }).WithPosition(3,3).WithSize(3,1);
  tab.AddNumber("Rotation request (deg/s)",[this] { return teleopDrive_.GetYawRateRadps()*180/robot_geometry::PI; }).WithPosition(6,3).WithSize(3,1);
  tab.AddNumber("Output limit (%)",[] { return localization_config::TELEOP_MAX_MOTOR_OUTPUT*100; }).WithPosition(9,3).WithSize(3,1);
  tab.AddString("Lift status",[this] { return std::string(liftTeleop_.Status()); })
      .WithPosition(0,4).WithSize(4,1);
  tab.AddNumber("D-pad POV (deg)",[this] { return driverPov_; }).WithPosition(4,4).WithSize(2,1);
  tab.AddBoolean("D-pad present",[this] { return driverPovPresent_; }).WithPosition(6,4).WithSize(2,1);
  tab.AddNumber("Lift M1 output (%)",[this] { return hardware_.GetLiftCommand()*100; }).WithPosition(8,4).WithSize(2,1);
  tab.AddNumber("Lift encoder (counts)",[this] { return hardware_.GetLiftEncoderRaw(); }).WithPosition(10,4).WithSize(2,1);
  tab.AddBoolean("L1 +15",[this] { return driverL1_; }).WithPosition(0,5).WithSize(1,1);
  tab.AddBoolean("R1 -15",[this] { return driverR1_; }).WithPosition(1,5).WithSize(1,1);
  tab.AddNumber("FlexDIO 8 target (deg)",[this] { return servoTeleop_.TargetDegrees(0); }).WithPosition(2,5).WithSize(2,1);
  tab.AddBoolean("Servo 8 output enabled",[this] { return servoTeleop_.OutputEnabled(0); }).WithPosition(4,5).WithSize(2,1);
  tab.AddBoolean("Triangle button",[this] { return driverTriangle_; }).WithPosition(6,5).WithSize(2,1);
  tab.AddNumber("FlexDIO 9 target (deg)",[this] { return servoTeleop_.TargetDegrees(1); }).WithPosition(8,5).WithSize(2,1);
  tab.AddBoolean("Servo 9 output enabled",[this] { return servoTeleop_.OutputEnabled(1); }).WithPosition(10,5).WithSize(2,1);

  auto& tuning=frc::Shuffleboard::GetTab("Drive tuning");
  tuning.AddString("Wheel controller",[this] { return hardware_.GetDriveStatus(); }).WithPosition(0,0).WithSize(8,1);
  tuning.AddString("Stick direction",[this] { return std::string(teleopDrive_.GetTranslationMode()); }).WithPosition(8,0).WithSize(4,1);
  for (int i=0;i<3;++i) {
    const std::string name=std::string(localization_config::WHEEL_NAME[i])+" M"+
        std::to_string(localization_config::WHEEL_TITAN_PORT[i]);
    tuning.AddNumber(name+" target (mm/s)",[this,i] { return hardware_.GetWheelTargetMMps(i); }).WithPosition(i*4,1).WithSize(4,1);
    tuning.AddNumber(name+" measured (mm/s)",[this,i] { return hardware_.GetWheelMeasuredMMps(i); }).WithPosition(i*4,2).WithSize(4,1);
    tuning.AddNumber(name+" output (%)",[this,i] { return hardware_.GetWheelCommand(i)*100; }).WithPosition(i*4,3).WithSize(4,1);
  }
}

void RobotSubsystem::InitializeCobraDashboard() {
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

void RobotSubsystem::PublishDiagnostics() {
  if (!localization_config::ENABLE_DETAILED_TELEMETRY) return;
  for (int i=0;i<4;++i)
    frc::SmartDashboard::PutBoolean(std::string("Diagnostics/Sonar/")+
        localization_config::SONARS[i].name+"/fused",sonarCorrectionAccepted_[i]);
  const auto target=arena_test_config::ENABLED?arenaTest_.GetTarget():autonomous_.GetTarget();
  frc::SmartDashboard::PutNumber("Diagnostics/Auto/target_x_mm",target.x);
  frc::SmartDashboard::PutNumber("Diagnostics/Auto/target_y_mm",target.y);
  localization_.PublishTelemetry();
}
