#include "RobotControl.h"

#include <frc/RobotState.h>
#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <algorithm>
#include <cmath>
#include <string>

RobotSubsystem::RobotSubsystem() {
  lastLoopTime_=lastSonarPingTime_=frc::Timer::GetFPGATimestamp();
  hardware_.StopDrive();
}
void RobotSubsystem::InitializeDashboard() {
  InitializeMotionDashboard();
  InitializeSensingDashboard();
}
void RobotSubsystem::EnterAutonomous() {
  StopAll();
  localization_.SetAutonomousMode(true);
  tapeRoutine_.Enter(); autonomousSession_=true; routineNotice_.clear();
}
void RobotSubsystem::StopAll() {
  hardware_.SetAutonomousDriveEnabled(false);
  hardware_.StopDrive();
  hardware_.StopManipulator();
  teleopDrive_.Update(false,false,0,0,0,localization_config::CONTROL_PERIOD_S);
  liftTeleop_.Update(false,false,false,false,false,false);
  servoTeleop_.Disarm();
  tapeRoutine_.Abort(); autonomousSession_=false;
  startTape_.SetBoolean(false); stopTape_.SetBoolean(false);
  localization_.SetAutonomousMode(false);
}
void RobotSubsystem::Periodic() {
  const double now=frc::Timer::GetFPGATimestamp(),dt=now-lastLoopTime_;
  lastLoopTime_=now;
  controlDt_=dt;
  const bool enabled=frc::RobotState::IsEnabled();
  if (hardware_.IsImuConnected() && !hardware_.IsImuCalibrating())
    localization_.UpdateOdometry(dt);
  const bool white=captureCobraWhite_.GetBoolean(false),black=captureCobraBlack_.GetBoolean(false);
  if (white) captureCobraWhite_.SetBoolean(false);
  if (black) captureCobraBlack_.SetBoolean(false);
  // Disabled-mode I2C diagnostics at 20 Hz. Calibration counts fresh samples,
  // rather than repeatedly averaging the same cached sensor value.
  if (lastSensorTime_<0 || now-lastSensorTime_>=localization_config::SENSOR_UPDATE_PERIOD_S || white || black) {
    line_=hardware_.ReadLine();
    for (int i=0;i<localization_config::IR_COUNT;++i) irReading_[i]=hardware_.ReadIr(i);
    lastSensorTime_=now;
    cobraCheck_.Update(line_,now,!enabled,white,black);
  } else if (enabled && cobraCheck_.Capturing()) {
    cobraCheck_.Update(line_,now,false,false,false);
  }
  UpdateSonarPoll(now,enabled);
  const bool start=startTape_.GetBoolean(false),stop=stopTape_.GetBoolean(false);
  if (start) startTape_.SetBoolean(false);
  if (stop) stopTape_.SetBoolean(false);
  if (enabled && frc::RobotState::IsAutonomous() && !frc::RobotState::IsTest()) {
    if (autonomousSession_) UpdateTapeRoutine(now,dt,start,stop);
    else { hardware_.StopDrive(); hardware_.StopManipulator(); }
  } else {
    if (autonomousSession_) StopAll();
    if (start) routineNotice_="Start ignored: select Autonomous, enable, then press Start tape test again";
    UpdateTeleopDrive(dt);
  }
}

void RobotSubsystem::UpdateTapeRoutine(double now,double dt,bool start,bool stop) {
  if (start) routineNotice_.clear();
  const bool stationary=std::hypot(localization_.GetBodyVelocityX_MMps(),localization_.GetBodyVelocityY_MMps())
      <=arena_test_config::STATIONARY_SPEED_MMPS &&
      std::fabs(hardware_.GetYawRateDegreesPerSecond())<=arena_test_config::STATIONARY_YAW_DPS;
  TapeRoutineInput in;
  in.motion={localization_.GetPose(),now,dt,
      hardware_.IsImuConnected() && !hardware_.IsImuCalibrating() && localization_.IsHealthy() &&
          frc::DriverStation::GetInstance().IsDSAttached(),
      stationary,start,stop,robot_geometry::Radians(hardware_.GetYawRateDegreesPerSecond())};
  in.leftRangeMM=irReading_[0].distanceMM; in.leftValid=irReading_[0].valid; in.leftTime=lastSensorTime_;
  in.rearRangeMM=sonarRanges_MM_[1]; in.rearValid=sonarRangeValid_[1]; in.rearTime=sonarReadTime_[1];
  in.line=line_; in.cobraTime=lastSensorTime_; in.cobraCalibrated=cobraCheck_.Ready();
  for (int i=0;i<4;++i) in.blackLevel[i]=cobraCheck_.BlackLevel(i);
  if (hardware_.HasDriveFault() && tapeRoutine_.IsRunning()) tapeRoutine_.Fail(hardware_.GetDriveStatus());
  tapeRoutine_.Update(in);
  if (tapeRoutine_.NeedsPoseReset()) {
    const auto& pose=tapeRoutine_.GetReferencePose();
    localization_.Reset(pose.x_MM,pose.y_MM,pose.headingRad*180/robot_geometry::PI);
    tapeRoutine_.ConfirmPoseReset(now);
  }
  hardware_.SetAutonomousDriveEnabled(tapeRoutine_.IsRunning());
  if (tapeRoutine_.IsRunning()) {
    hardware_.DriveAutonomousRobotRelative(tapeRoutine_.GetForwardMMps(),tapeRoutine_.GetLeftMMps(),
        tapeRoutine_.GetYawRadps(),localization_config::TELEOP_MAX_MOTOR_OUTPUT,dt);
    if (hardware_.HasDriveFault()) {
      tapeRoutine_.Fail(hardware_.GetDriveStatus()); hardware_.SetAutonomousDriveEnabled(false); hardware_.StopDrive();
    }
  } else hardware_.StopDrive();
  hardware_.StopManipulator(); // XY/heading test only; no lift or servo movement
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
    hardware_.DriveTeleopRobotRelative(teleopDrive_.GetForwardMMps(),teleopDrive_.GetLeftMMps(),
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
  for (int i=0;i<localization_config::SONAR_COUNT;++i) {
    if (!enabled || now-sonarReadTime_[i]>localization_config::SONAR_READING_STALE_S) {
      sonarRanges_MM_[i]=-1; sonarRangeValid_[i]=false;
    }
  }
  if (!enabled) {
    sonarAwaitingEcho_=false; nextSonarIndex_=0; lastSonarPingTime_=now;
    return;
  }
  if (sonarAwaitingEcho_) {
    if (now-sonarPingTime_<localization_config::SONAR_ECHO_SETTLE_S) return;
    const int i=nextSonarIndex_;
    const double range=hardware_.GetSonarRangeMM(i);
    sonarReadTime_[i]=now;
    sonarRangeValid_[i]=hardware_.IsSonarRangeValid(i) && std::isfinite(range) &&
        range>=localization_config::SONAR_MIN_RANGE_MM && range<=localization_config::SONAR_MAX_RANGE_MM;
    sonarRanges_MM_[i]=sonarRangeValid_[i]?range:-1;
    nextSonarIndex_=(i+1)%localization_config::SONAR_COUNT;
    sonarAwaitingEcho_=false; lastSonarPingTime_=now;
  } else if (now-lastSonarPingTime_>=localization_config::SONAR_PING_SPACING_S) {
    hardware_.PingSonar(nextSonarIndex_); sonarPingTime_=now; sonarAwaitingEcho_=true;
  }
}
void RobotSubsystem::InitializeMotionDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Motion");
  tab.AddString("Drive status",[this] {
    if (frc::RobotState::IsAutonomous()) return hardware_.GetDriveStatus();
    return hardware_.HasDriveFault() || hardware_.IsDriveStarting()?
        hardware_.GetDriveStatus():std::string(teleopDrive_.GetStatus());
  }).WithPosition(0,0).WithSize(6,1);
  tab.AddBoolean("Enabled",[] { return frc::RobotState::IsEnabled(); }).WithPosition(6,0).WithSize(2,1);
  tab.AddBoolean("Controller",[this] { return driverConnected_; }).WithPosition(8,0).WithSize(2,1);
  tab.AddBoolean("Drive armed",[this] { return teleopDrive_.IsArmed(); }).WithPosition(10,0).WithSize(2,1);
  tab.AddNumber("Heading (deg)",[this] { return localization_.GetPose().headingRad*180/robot_geometry::PI; })
      .WithPosition(0,1).WithSize(4,1);
  tab.AddBoolean("IMU ready",[this] { return hardware_.IsImuConnected() && !hardware_.IsImuCalibrating(); })
      .WithPosition(4,1).WithSize(4,1);
  tab.AddNumber("Control period (ms)",[this] { return controlDt_*1000; }).WithPosition(8,1).WithSize(4,1);
  for (int i=0;i<localization_config::WHEEL_COUNT;++i) {
    const std::string wheel=std::string(localization_config::WHEEL_NAME[i])+" M"+
        std::to_string(localization_config::WHEEL_TITAN_PORT[i]);
    tab.AddNumber(wheel+" encoder (counts)",[this,i] { return hardware_.GetWheelEncoderRaw(i); })
        .WithPosition(i*4,2).WithSize(4,1);
    tab.AddNumber(wheel+" speed (mm/s)",[this,i] { return hardware_.GetWheelMeasuredMMps(i); })
        .WithPosition(i*4,3).WithSize(4,1);
    tab.AddNumber(wheel+" output (%)",[this,i] { return hardware_.GetWheelCommand(i)*100; })
        .WithPosition(i*4,4).WithSize(4,1);
  }
  tab.AddNumber("Lift M1 encoder (counts)",[this] { return hardware_.GetLiftEncoderRaw(); })
      .WithPosition(0,5).WithSize(4,1);
  tab.AddNumber("Lift M1 output (%)",[this] { return hardware_.GetLiftCommand()*100; })
      .WithPosition(4,5).WithSize(4,1);
  tab.AddString("Lift status",[this] { return std::string(liftTeleop_.Status()); }).WithPosition(8,5).WithSize(4,1);
  tab.AddNumber("Servo 8 target (deg)",[this] { return servoTeleop_.TargetDegrees(0); })
      .WithPosition(0,6).WithSize(6,1);
  tab.AddNumber("Servo 9 target (deg)",[this] { return servoTeleop_.TargetDegrees(1); })
      .WithPosition(6,6).WithSize(6,1);
  tab.AddNumber("X (mm)",[this] { return localization_.GetPose().x_MM; }).WithPosition(0,7).WithSize(3,1);
  tab.AddNumber("Y (mm)",[this] { return localization_.GetPose().y_MM; }).WithPosition(3,7).WithSize(3,1);
  startTape_=tab.Add("Start tape test",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(6,7).WithSize(3,1).GetEntry();
  stopTape_=tab.Add("Stop tape test",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(9,7).WithSize(3,1).GetEntry();
  startTape_.SetBoolean(false); stopTape_.SetBoolean(false);
  tab.AddString("Tape routine",[this] { return routineNotice_.empty()?tapeRoutine_.GetStatus():routineNotice_; })
      .WithPosition(0,8).WithSize(12,1);
}
void RobotSubsystem::InitializeSensingDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Sensing");
  for (int i=0;i<localization_config::SONAR_COUNT;++i) {
    const std::string name=std::string(localization_config::SONARS[i].name)+" sonar";
    tab.AddNumber(name+" (mm)",[this,i] { return sonarRanges_MM_[i]; }).WithPosition(i*3,0).WithSize(3,1);
    tab.AddBoolean(name+" valid",[this,i] { return sonarRangeValid_[i]; }).WithPosition(i*3,1).WithSize(3,1);
  }
  for (int i=0;i<localization_config::IR_COUNT;++i) {
    const std::string name=i==0?"Left IR":"Right IR";
    tab.AddNumber(name+" (mm)",[this,i] {
      const bool fresh=lastSensorTime_>=0 &&
          frc::Timer::GetFPGATimestamp()-lastSensorTime_<=localization_config::SENSOR_STALE_S;
      return fresh && irReading_[i].valid?irReading_[i].distanceMM:-1;
    }).WithPosition(6+i*3,0).WithSize(3,1);
    tab.AddBoolean(name+" valid",[this,i] {
      return irReading_[i].valid && lastSensorTime_>=0 &&
          frc::Timer::GetFPGATimestamp()-lastSensorTime_<=localization_config::SENSOR_STALE_S;
    }).WithPosition(6+i*3,1).WithSize(3,1);
    tab.AddNumber(name+" voltage (V)",[this,i] {
      return lastSensorTime_>=0 &&
          frc::Timer::GetFPGATimestamp()-lastSensorTime_<=localization_config::SENSOR_STALE_S?
          irReading_[i].voltage:-1;
    }).WithPosition(6+i*3,2).WithSize(3,1);
  }
  tab.AddString("Sonar status",[] {
    return frc::RobotState::IsEnabled()?"Front/rear polling; -1 = no valid echo":
        "Paused while Disabled; Cobra and IR inputs remain live";
  }).WithPosition(0,2).WithSize(6,1);
  tab.AddString("IR status",[] {
    return "Sharp analog: left AI0 / right AI1. Range 100-800 mm; -1 = invalid. Readings remain live while Disabled.";
  }).WithPosition(0,8).WithSize(6,1);
  tab.AddString("Cobra status",[this] { return cobraCheck_.Status(); }).WithPosition(0,3).WithSize(8,1);
  tab.AddBoolean("Cobra I2C link",[this] { return line_.communicationValid; }).WithPosition(8,3).WithSize(2,1);
  tab.AddBoolean("Cobra calibrated",[this] { return cobraCheck_.Ready(); }).WithPosition(10,3).WithSize(2,1);
  for (int i=0;i<4;++i) {
    const std::string channel="Cobra CH"+std::to_string(i);
    tab.AddNumber(channel+" (V)",[this,i] { return line_.volts[i]; }).WithPosition(i*3,4).WithSize(3,1);
    tab.AddBoolean(channel+" valid",[this,i] { return cobraCheck_.ChannelReadingValid(i); })
        .WithPosition(i*3,5).WithSize(3,1);
    tab.AddBoolean(channel+" black",[this,i] { return cobraCheck_.IsBlack(i); })
        .WithPosition(i*3,6).WithSize(3,1);
  }
  captureCobraWhite_=tab.Add("Capture white",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(0,7).WithSize(3,1).GetEntry();
  captureCobraBlack_=tab.Add("Capture black",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(3,7).WithSize(3,1).GetEntry();
  captureCobraWhite_.SetBoolean(false); captureCobraBlack_.SetBoolean(false);
  tab.AddString("Cobra check",[] {
    return "While Disabled: place ALL channels over white floor and Capture white; repeat over black tape with Capture black. "
        "Hold still until saved, then sweep the tape across CH0-3. Calibrate again after restarting.";
  }).WithPosition(6,7).WithSize(6,2);
  tab.AddNumber("CH1 - CH2 (V)",[this] { return cobraCheck_.VoltagesValid()?line_.volts[1]-line_.volts[2]:-1; })
      .WithPosition(0,9).WithSize(6,1);
  tab.AddNumber("Tape scan travel (mm)",[this] { return tapeRoutine_.GetScanTravelMM(); })
      .WithPosition(6,9).WithSize(6,1);
}
