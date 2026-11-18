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
  // CobraCheck starts with the exact user-supplied RobotConfig references.
  // Deliberately do not read a saved calibration file or accept recaptures.
  lastLoopTime_=lastSonarPingTime_=frc::Timer::GetFPGATimestamp();
  hardware_.StopDrive();
}
void RobotSubsystem::InitializeDashboard() {
  InitializeMotionDashboard();
  InitializeSensingDashboard();
  InitializeNavigationDashboard();
  InitializePoseDashboard();
}
void RobotSubsystem::EnterAutonomous() {
  const bool keepReference=navigation_.HasReference();
  StopAll();
  localization_.SetAutonomousMode(true);
  tapeRoutine_.Enter(); autonomousSession_=true; routineNotice_.clear();
  if (!keepReference) navigation_.Enter();
  navigationSelected_=keepReference;
  peakControlDt_=0;
}
void RobotSubsystem::StopAll() {
  hardware_.SetAutonomousDriveEnabled(false);
  hardware_.StopDrive();

  teleopDrive_.Update(false,false,0,0,0,localization_config::CONTROL_PERIOD_S);
  tapeRoutine_.Abort(); autonomousSession_=false;
  if (navigation_.IsRunning()) navigation_.Abort();
  startTapeLeft_.SetBoolean(false); startTapeRight_.SetBoolean(false); stopTape_.SetBoolean(false);
  initializePose_.SetBoolean(false); goToGoal_.SetBoolean(false); stopNavigation_.SetBoolean(false);
  localization_.SetAutonomousMode(false);
}
void RobotSubsystem::Periodic() {
  const double now=frc::Timer::GetFPGATimestamp(),dt=now-lastLoopTime_;
  lastLoopTime_=now;
  controlDt_=dt;
  if (std::isfinite(dt)) peakControlDt_=std::max(peakControlDt_,dt);
  const bool enabled=frc::RobotState::IsEnabled();
  if (hardware_.IsImuConnected() && !hardware_.IsImuCalibrating())
    localization_.UpdateOdometry(dt);
  // Live sensing at 20 Hz; classification always uses supplied calibration.
  if (lastSensorTime_<0 || now-lastSensorTime_>=localization_config::SENSOR_UPDATE_PERIOD_S) {
    line_=hardware_.ReadLine();
    for (int i=0;i<localization_config::IR_COUNT;++i) irReading_[i]=hardware_.ReadIr(i);
    lastSensorTime_=now;
    // No calibration requests, regardless of Control Station mode.
    cobraCheck_.Update(line_,now);
  }
  UpdateSonarPoll(now);
  UpdateStartingAreaSelection();
  UpdateGoalSelection(); // editable settings for the next run; never changes an active route
  const bool startLeft=startTapeLeft_.GetBoolean(false),startRight=startTapeRight_.GetBoolean(false);
  const bool stop=stopTape_.GetBoolean(false);
  const bool reference=initializePose_.GetBoolean(false),go=goToGoal_.GetBoolean(false);
  const bool stopNav=stopNavigation_.GetBoolean(false);
  if (reference) initializePose_.SetBoolean(false);
  if (go) goToGoal_.SetBoolean(false);
  if (stopNav) stopNavigation_.SetBoolean(false);
  if (startLeft) startTapeLeft_.SetBoolean(false);
  if (startRight) startTapeRight_.SetBoolean(false);
  if (stop) stopTape_.SetBoolean(false);
  if (enabled && frc::RobotState::IsAutonomous() && !frc::RobotState::IsTest()) {
    if (!autonomousSession_) hardware_.StopDrive();
    else if (stop || stopNav) {
      tapeRoutine_.Abort(); navigation_.Abort();
      hardware_.SetAutonomousDriveEnabled(false); hardware_.StopDrive(); routineNotice_.clear();
    } else if ((reference || go) && (startLeft || startRight)) {
      routineNotice_="Choose either navigation or a tape test";
      // Reject conflicting starts without dropping an active control update.
      if (navigationSelected_) UpdateNavigation(now,dt,false,false);
      else UpdateTapeRoutine(now,dt,false,false,false);
    } else if (reference || go) {
      if (tapeRoutine_.IsRunning()) {
        routineNotice_="Tape test running; Stop before navigation";
        UpdateTapeRoutine(now,dt,false,false,false);
      } else { navigationSelected_=true; routineNotice_.clear(); UpdateNavigation(now,dt,reference,go); }
    } else if (startLeft || startRight) {
      if (navigation_.IsRunning()) {
        routineNotice_="Navigation running; Stop before a tape test";
        UpdateNavigation(now,dt,false,false);
      } else {
        navigationSelected_=false; navigation_.Enter();
        UpdateTapeRoutine(now,dt,startLeft,startRight,false);
      }
    } else if (navigationSelected_) UpdateNavigation(now,dt,false,false);
    else UpdateTapeRoutine(now,dt,false,false,false);
  } else {
    if (autonomousSession_) StopAll();
    if (stop || stopNav) { navigation_.Abort(); routineNotice_.clear(); }
    else if (!enabled && (reference || (navigationSelected_ && navigation_.GetState()==GoalNavigationRoutine::State::Reference))) {
      // Referencing only resets the measured pose; no drive state is started.
      navigationSelected_=true; routineNotice_.clear(); UpdateNavigation(now,dt,reference,false);
    } else if (reference) routineNotice_="Disable robot or select enabled Autonomous to initialize pose";
    if (startLeft || startRight || go) routineNotice_="Select and enable Autonomous, then press the requested task again";
    UpdateTeleopDrive(dt);
  }
}

NavigationInput RobotSubsystem::NavigationSample(double now,double dt,bool referenceOnly) {
  NavigationInput in;
  const bool station=frc::DriverStation::GetInstance().IsDSAttached(),imu=hardware_.IsImuConnected();
  // A sensor reset can recover old odometry faults; it still requires live IMU,
  // Control Station and valid clock/range samples. Travel requires full odometry.
  const bool calibrating=hardware_.IsImuCalibrating(),odometry=referenceOnly || localization_.IsHealthy();
  in.motion={localization_.GetPose(),now,dt,
      imu && !calibrating && odometry && station,false,false,
      robot_geometry::Radians(hardware_.GetYawRateDegreesPerSecond())};
  in.forwardMMps=localization_.GetBodyVelocityX_MMps(); in.leftMMps=localization_.GetBodyVelocityY_MMps();
  if (!station) in.healthIssue="Control Station disconnected";
  else if (!imu) in.healthIssue="IMU disconnected";
  else if (calibrating) in.healthIssue="IMU is calibrating";
  else if (!odometry) in.healthIssue=std::string("Odometry: ")+localization_.OdometryFaultReason();
  in.rangeMM[0]=irReading_[0].distanceMM; in.valid[0]=irReading_[0].valid; in.stamp[0]=lastSensorTime_;
  in.rangeMM[2]=irReading_[1].distanceMM; in.valid[2]=irReading_[1].valid; in.stamp[2]=lastSensorTime_;
  in.rangeMM[1]=sonarRanges_MM_[1]; in.valid[1]=sonarRangeValid_[1]; in.stamp[1]=sonarReadTime_[1];
  in.rangeMM[3]=sonarRanges_MM_[0]; in.valid[3]=sonarRangeValid_[0]; in.stamp[3]=sonarReadTime_[0];
  return in;
}
void RobotSubsystem::UpdateNavigation(double now,double dt,bool reference,bool go) {
  const bool referenceOnly=navigation_.GetState()==GoalNavigationRoutine::State::Reference ||
      (reference && !go && !navigation_.IsRunning());
  auto in=NavigationSample(now,dt,referenceOnly);
  if (reference && go) routineNotice_="Initialize and Go must be requested separately";
  else if (reference) {
    if (navigation_.IsRunning() && navigation_.GetState()!=GoalNavigationRoutine::State::Reference)
      routineNotice_="Navigation running; Stop before reinitializing";
    else {
      PoseInitialization settings;
      const bool named=selectedStartingArea_>=0 && selectedStartingArea_<navigation_config::GOAL_COUNT;
      settings.xMM=named?"-":initialX_.GetString("-"); settings.yMM=named?"-":initialY_.GetString("-");
      if (named) { initialX_.SetString("-"); initialY_.SetString("-"); }
      settings.headingDeg=initialHeading_.GetDouble(0);
      for (int i=0;i<4;++i) settings.wall[i]=wallReferences_[i].GetString("-");
      if (navigation_.InitializePose(settings))
        initializedSource_=selectedStartingArea_<navigation_config::GOAL_COUNT?selectedStartingArea_:-1;
    }
  } else if (go) {
    const RobotPose goal={goalX_.GetDouble(navigation_config::DEFAULT_GOAL_X_MM),
        goalY_.GetDouble(navigation_config::DEFAULT_GOAL_Y_MM),
        robot_geometry::Radians(goalHeading_.GetDouble(navigation_config::DEFAULT_GOAL_HEADING_DEG))};
    navigation_.StartGoal(goal,in,intermediatePoses_.GetString(""));
  }
  if (hardware_.HasDriveFault() && (navigation_.GetState()==GoalNavigationRoutine::State::Driving ||
      navigation_.GetState()==GoalNavigationRoutine::State::FinalHeading)) navigation_.Fail(hardware_.GetDriveStatus());
  navigation_.Update(in);
  localization_.CorrectAutonomousPosition(navigation_.PositionCorrection());
  if (navigation_.NeedsPoseReset()) {
    const auto pose=navigation_.RequestedPose();
    localization_.Reset(pose.x_MM,pose.y_MM,pose.headingRad*180/robot_geometry::PI);
    navigation_.ConfirmReference(); in=NavigationSample(now,dt);
  }
  const auto state=navigation_.GetState();
  const bool drive=state==GoalNavigationRoutine::State::Driving || state==GoalNavigationRoutine::State::FinalHeading;
  hardware_.SetAutonomousDriveEnabled(drive);
  if (drive) {
    hardware_.DriveAutonomousRobotRelative(navigation_.Forward(),navigation_.Left(),navigation_.Yaw(),
        localization_config::TELEOP_MAX_MOTOR_OUTPUT,dt);
    if (hardware_.HasDriveFault()) { navigation_.Fail(hardware_.GetDriveStatus()); hardware_.SetAutonomousDriveEnabled(false); hardware_.StopDrive(); }
  } else hardware_.StopDrive();
}

void RobotSubsystem::UpdateTapeRoutine(double now,double dt,bool startLeft,bool startRight,bool stop) {
  bool start=startLeft!=startRight;
  if (stop) { start=false; routineNotice_.clear(); }
  else if (startLeft && startRight) routineNotice_="Choose one approach: both start requests ignored";
  else if (start && tapeRoutine_.IsRunning()) {
    start=false; routineNotice_="Start ignored: tape task already running; Stop before changing approach";
  } else if (start) routineNotice_.clear();
  TapeRoutineInput in;
  // Actual body velocity and sustained pose tolerances govern completion.
  in.motion={localization_.GetPose(),now,dt,
      hardware_.IsImuConnected() && !hardware_.IsImuCalibrating() && localization_.IsHealthy() &&
          frc::DriverStation::GetInstance().IsDSAttached(),
      start,stop,robot_geometry::Radians(hardware_.GetYawRateDegreesPerSecond())};
  in.leftRangeMM=irReading_[0].distanceMM; in.leftValid=irReading_[0].valid; in.leftTime=lastSensorTime_;
  in.rearRangeMM=sonarRanges_MM_[1]; in.rearValid=sonarRangeValid_[1]; in.rearTime=sonarReadTime_[1];
  in.line=line_; in.cobraTime=lastSensorTime_; in.cobraCalibrated=cobraCheck_.Ready();
  in.bodyForwardMMps=localization_.GetBodyVelocityX_MMps();
  in.bodyLeftMMps=localization_.GetBodyVelocityY_MMps();
  in.approach=startRight?TapeApproach::FromRight:TapeApproach::FromLeft;
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
}

void RobotSubsystem::UpdateTeleopDrive(double dt) {
  auto& ds=frc::DriverStation::GetInstance();
  const bool attached=ds.IsDSAttached();
  const int xAxis=localization_config::DRIVER_LEFT_X_AXIS;
  const int yAxis=localization_config::DRIVER_LEFT_Y_AXIS;
  const int rightXAxis=localization_config::DRIVER_RIGHT_X_AXIS;
  const int axes=attached?driver_.GetAxisCount():0;
  const bool axesPresent=xAxis>=0 && yAxis>=0 && rightXAxis>=0 &&
      xAxis!=yAxis && rightXAxis!=xAxis && rightXAxis!=yAxis &&
      axes>std::max(rightXAxis,std::max(xAxis,yAxis));
  driverConnected_=attached && axesPresent;
  driverRawX_=axesPresent?driver_.GetRawAxis(xAxis):0;
  driverRawY_=axesPresent?driver_.GetRawAxis(yAxis):0;
  driverRightX_=axesPresent?driver_.GetRawAxis(rightXAxis):0;
  const bool allowed=localization_config::TELEOP_DRIVE_ENABLED &&
      frc::RobotState::IsEnabled() && frc::RobotState::IsOperatorControl() && !frc::RobotState::IsTest();
  teleopDrive_.Update(allowed,driverConnected_,driverRawX_,driverRawY_,driverRightX_,dt);
  if (allowed && teleopDrive_.IsArmed()) {
    // Teleop yaw comes only from right X; the gyro is observed for odometry only.
    hardware_.DriveTeleopRobotRelative(teleopDrive_.GetForwardMMps(),teleopDrive_.GetLeftMMps(),
        teleopDrive_.GetYawRateRadps(),
        localization_config::TELEOP_MAX_MOTOR_OUTPUT,dt);
  } else {
    hardware_.StopDrive();
  }
}

void RobotSubsystem::UpdateSonarPoll(double now) {
  for (int i=0;i<localization_config::SONAR_COUNT;++i) {
    if (now-sonarReadTime_[i]>localization_config::SONAR_READING_STALE_S) {
      sonarRanges_MM_[i]=-1; sonarRangeValid_[i]=false;
    }
  }
  // Sensor diagnostics remain live while Disabled; motor enable gates are
  // independent. Keep staggering pings to avoid acoustic crosstalk.
  if (sonarAwaitingEcho_) {
    if (now-sonarPingTime_<localization_config::SONAR_ECHO_SETTLE_S) return;
    const int i=nextSonarIndex_;
    const double range=hardware_.GetSonarRangeMM(i);
    // Range belongs to the ping pose, not the later counter-read pose.
    sonarReadTime_[i]=sonarPingTime_;
    sonarRangeValid_[i]=hardware_.IsSonarRangeValid(i) && std::isfinite(range) &&
        range>=localization_config::SONAR_MIN_RANGE_MM && range<=localization_config::SONAR_MAX_RANGE_MM;
    sonarRanges_MM_[i]=sonarRangeValid_[i]?range:-1;
    nextSonarIndex_=(i+1)%localization_config::SONAR_COUNT;
    sonarAwaitingEcho_=false; lastSonarPingTime_=now;
  } else if (now-lastSonarPingTime_>=localization_config::SONAR_PING_SPACING_S) {
    hardware_.PingSonar(nextSonarIndex_); sonarPingTime_=now; sonarAwaitingEcho_=true;
  }
}
// Compact grids: Motion (including initialization) 8x5, Sensing 8x5.
// Short labels and small numeric/boolean tiles fit the laptop without scrolling.
void RobotSubsystem::InitializeMotionDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Motion");
  tab.AddString("Motion status",[this] {
    // Visible revision distinguishes this executable from stale robot/UI data.
    const std::string revision="P8-r33 MVP | ";
    if (!routineNotice_.empty()) return revision+routineNotice_;
    if (frc::RobotState::IsAutonomous()) return revision+(navigationSelected_?navigation_.Status():tapeRoutine_.GetStatus());
    return revision+(hardware_.HasDriveFault()?
        hardware_.GetDriveStatus():std::string(teleopDrive_.GetStatus()));
  }).WithPosition(0,0).WithSize(4,1);
  tab.AddBoolean("Enabled",[] { return frc::RobotState::IsEnabled(); }).WithPosition(4,0).WithSize(1,1);
  tab.AddBoolean("Controller",[this] { return driverConnected_; }).WithPosition(5,0).WithSize(1,1);
  tab.AddBoolean("Referenced",[this] { return navigation_.HasReference(); }).WithPosition(6,0).WithSize(1,1);
  tab.AddBoolean("IMU ready",[this] { return hardware_.IsImuConnected() && !hardware_.IsImuCalibrating(); })
      .WithPosition(7,0).WithSize(1,1);
  tab.AddNumber("X (mm)",[this] { return localization_.GetPose().x_MM; }).WithPosition(0,1).WithSize(2,1);
  tab.AddNumber("Y (mm)",[this] { return localization_.GetPose().y_MM; }).WithPosition(2,1).WithSize(2,1);
  tab.AddNumber("Heading N=0 deg",[this] { return localization_.GetPose().headingRad*180/robot_geometry::PI; })
      .WithPosition(4,1).WithSize(2,1);
  for (int i=0;i<localization_config::WHEEL_COUNT;++i) {
    const int x=i==1?3:0,y=i==2?3:2;
    const std::string port="M"+std::to_string(localization_config::WHEEL_TITAN_PORT[i]);
    tab.AddNumber(port+" counts",[this,i] { return hardware_.GetWheelEncoderRaw(i); })
        .WithPosition(x,y).WithSize(1,1);
    tab.AddNumber(port+" mm/s",[this,i] { return hardware_.GetWheelMeasuredMMps(i); })
        .WithPosition(x+1,y).WithSize(1,1);
    // Signed WHOLE percentage actually encoded for Titan, not a speed setpoint.
    tab.AddNumber(port+" PWM %",[this,i] { return std::round(hardware_.GetWheelCommand(i)*100); })
        .WithPosition(x+2,y).WithSize(1,1);
  }
  startTapeLeft_=tab.Add("Tape from left",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(6,2).WithSize(1,1).GetEntry();
  startTapeRight_=tab.Add("Tape from right",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(7,2).WithSize(1,1).GetEntry();
  stopTape_=tab.Add("Stop tape",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(7,3).WithSize(1,1).GetEntry();
  startTapeLeft_.SetBoolean(false); startTapeRight_.SetBoolean(false); stopTape_.SetBoolean(false);
}
void RobotSubsystem::InitializeSensingDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Sensing");
  for (int i=0;i<localization_config::SONAR_COUNT;++i) {
    const std::string name=std::string(localization_config::SONARS[i].name);
    tab.AddNumber(name+" sonar mm",[this,i] { return sonarRanges_MM_[i]; })
        .WithPosition(i*3,0).WithSize(2,1);
    tab.AddBoolean(name+" valid",[this,i] { return sonarRangeValid_[i]; })
        .WithPosition(i*3+2,0).WithSize(1,1);
  }
  tab.AddBoolean("Cobra I2C",[this] { return line_.communicationValid; }).WithPosition(6,0).WithSize(1,1);
  tab.AddBoolean("Cal. fixed",[this] { return cobraCheck_.Ready(); }).WithPosition(7,0).WithSize(1,1);
  for (int i=0;i<localization_config::IR_COUNT;++i) {
    const std::string name=i==0?"Left IR":"Right IR";
    tab.AddNumber(name+" mm",[this,i] {
      const bool fresh=lastSensorTime_>=0 &&
          frc::Timer::GetFPGATimestamp()-lastSensorTime_<=localization_config::SENSOR_STALE_S;
      return fresh && irReading_[i].valid?irReading_[i].distanceMM:-1;
    }).WithPosition(i*4,1).WithSize(2,1);
    tab.AddBoolean(name+" valid",[this,i] {
      return irReading_[i].valid && lastSensorTime_>=0 &&
          frc::Timer::GetFPGATimestamp()-lastSensorTime_<=localization_config::SENSOR_STALE_S;
    }).WithPosition(i*4+2,1).WithSize(1,1);
    tab.AddNumber(name+" V",[this,i] {
      return lastSensorTime_>=0 &&
          frc::Timer::GetFPGATimestamp()-lastSensorTime_<=localization_config::SENSOR_STALE_S?
          irReading_[i].voltage:-1;
    }).WithPosition(i*4+3,1).WithSize(1,1);
  }
  for (int i=0;i<4;++i) {
    const std::string channel="CH"+std::to_string(i);
    tab.AddNumber("Cobra "+channel+" V",[this,i] { return line_.volts[i]; })
        .WithPosition(i*2,2).WithSize(2,1);
    tab.AddBoolean(channel+" valid",[this,i] { return cobraCheck_.ChannelReadingValid(i); })
        .WithPosition(i*2,3).WithSize(1,1);
    tab.AddBoolean(channel+" black",[this,i] { return cobraCheck_.IsBlack(i); })
        .WithPosition(i*2+1,3).WithSize(1,1);
  }
  tab.AddNumber("CH1 - CH2 (V)",[this] { return cobraCheck_.VoltagesValid()?line_.volts[1]-line_.volts[2]:-1; })
      .WithPosition(0,4).WithSize(2,1);
  tab.AddNumber("Scan travel (mm)",[this] { return tapeRoutine_.GetScanTravelMM(); })
      .WithPosition(2,4).WithSize(2,1);
  tab.AddString("Sensing status",[this] {
    return std::string("All sensors live; fixed Cobra calibration | ")+cobraCheck_.Status();
  }).WithPosition(4,4).WithSize(4,1);
}

void RobotSubsystem::InitializePoseDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Motion");
  tab.AddString("Pose reference",[this] {
    return routineNotice_.empty()?navigation_.InitializationStatus():routineNotice_;
  }).WithPosition(4,4).WithSize(4,1);
  for (int i=0;i<navigation_config::GOAL_COUNT;++i) {
    if (i==navigation_config::START) startingAreaChooser_.SetDefaultOption(navigation_config::GOALS[i].name,i);
    else startingAreaChooser_.AddOption(navigation_config::GOALS[i].name,i);
  }
  startingAreaChooser_.AddOption("Manual",navigation_config::GOAL_COUNT);
  tab.Add("Starting area",startingAreaChooser_).WithWidget(frc::BuiltInWidgets::kComboBoxChooser)
      .WithPosition(6,1).WithSize(2,1);
  initialX_=tab.Add("Start X mm or -",wpi::Twine("-"))
      .WithWidget(frc::BuiltInWidgets::kTextView).WithPosition(4,3).WithSize(1,1).GetEntry();
  initialY_=tab.Add("Start Y mm or -",wpi::Twine("-"))
      .WithWidget(frc::BuiltInWidgets::kTextView).WithPosition(5,3).WithSize(1,1).GetEntry();
  initialHeading_=tab.Add("Start heading deg",localization_config::START_HEADING_DEG).WithPosition(6,3).WithSize(1,1).GetEntry();
  initializePose_=tab.Add("Initialize pose",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(3,3).WithSize(1,1).GetEntry();
  const PoseInitialization defaults;
  const char* labels[]={"Left IR wall","Rear sonar wall","Right IR wall","Front sonar wall"};
  for (int i=0;i<4;++i)
    wallReferences_[i]=tab.Add(labels[i],wpi::Twine(defaults.wall[i]))
        .WithWidget(frc::BuiltInWidgets::kTextView).WithPosition(i,4).WithSize(1,1).GetEntry();
  initializePose_.SetBoolean(false);
  UpdateStartingAreaSelection();
}

void RobotSubsystem::UpdateStartingAreaSelection() {
  // Freeze selection during driving. A waiting reference can be replaced with
  // another area; Initialize must then be pressed for the newly loaded fields.
  if (navigation_.IsRunning() && navigation_.GetState()!=GoalNavigationRoutine::State::Reference) return;
  const int selected=startingAreaChooser_.GetSelected();
  if (selected<0 || selected>navigation_config::GOAL_COUNT || selected==selectedStartingArea_) return;
  navigation_.Enter(); initializedSource_=-1; selectedStartingArea_=selected;
  if (selected==navigation_config::GOAL_COUNT) return; // keep fields for manual editing
  initialX_.SetString("-"); initialY_.SetString("-");
  initialHeading_.SetDouble(goalSettings_[selected].headingDeg);
  for (int sensor=0;sensor<4;++sensor) {
    const auto wall=navigation_config::INITIAL_WALLS[selected][sensor];
    wallReferences_[sensor].SetString(wall.axis<0?"-":std::string(wall.axis==0?"X=":"Y=")+std::to_string(wall.coordinate));
  }
}

void RobotSubsystem::InitializeNavigationDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Navigation");
  tab.AddString("Navigation status",[this] {
    std::string status=std::string("P8-r33 MVP | ")+(routineNotice_.empty()?navigation_.Status():routineNotice_);
    if (navigation_.HasReference()) {
      status+=" | from ";
      status+=selectedSource_<navigation_config::GOAL_COUNT?navigation_config::GOALS[selectedSource_].name:"Current pose";
    }
    return status;
  }).WithPosition(0,0).WithSize(6,1);
  stopNavigation_=tab.Add("Stop",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(6,0).WithSize(2,1).GetEntry();
  goToGoal_=tab.Add("Go to goal",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(0,1).WithSize(4,1).GetEntry();
  tab.AddBoolean("Referenced",[this] { return navigation_.HasReference(); }).WithPosition(4,1).WithSize(1,1);
  tab.AddBoolean("Enabled",[] { return frc::RobotState::IsEnabled(); }).WithPosition(5,1).WithSize(1,1);
  tab.AddNumber("Remaining mm",[this] { return navigation_.RemainingMM(); }).WithPosition(6,1).WithSize(2,1);
  goalX_=tab.Add("Goal X mm",navigation_config::DEFAULT_GOAL_X_MM)
      .WithPosition(0,2).WithSize(3,1).GetEntry();
  goalY_=tab.Add("Goal Y mm",navigation_config::DEFAULT_GOAL_Y_MM)
      .WithPosition(3,2).WithSize(3,1).GetEntry();
  goalHeading_=tab.Add("Goal heading deg",navigation_config::DEFAULT_GOAL_HEADING_DEG)
      .WithPosition(6,2).WithSize(2,1).GetEntry();
  tab.AddNumber("Driving X mm",[this] { return navigation_.HasReference()?navigation_.ControlPose().x_MM:localization_.GetPose().x_MM; })
      .WithPosition(0,3).WithSize(3,1);
  tab.AddNumber("Driving Y mm",[this] { return navigation_.HasReference()?navigation_.ControlPose().y_MM:localization_.GetPose().y_MM; })
      .WithPosition(3,3).WithSize(3,1);
  tab.AddNumber("Robot heading deg",[this] { return localization_.GetPose().headingRad*180/robot_geometry::PI; })
      .WithPosition(6,3).WithSize(2,1);
  for (const auto& preset:navigation_config::GOALS) {
    const int index=static_cast<int>(goalSettings_.size());
    goalSettings_.push_back({preset.xMM,preset.yMM,preset.headingDeg});
    if (index==0) goalChooser_.SetDefaultOption(preset.name,index);
    else goalChooser_.AddOption(preset.name,index);
  }
  goalChooser_.AddOption("Custom",static_cast<int>(goalSettings_.size()));
  goalSettings_.push_back({navigation_config::DEFAULT_GOAL_X_MM,navigation_config::DEFAULT_GOAL_Y_MM,navigation_config::DEFAULT_GOAL_HEADING_DEG});
  const int count=static_cast<int>(goalSettings_.size());
  routeSettings_.assign(count*count,""); // extra row/column: Current pose / Custom
  for (int source=0;source<navigation_config::GOAL_COUNT;++source)
    for (int goal=0;goal<navigation_config::GOAL_COUNT;++goal)
      routeSettings_[source*count+goal]=GoalNavigationRoutine::DefaultIntermediates(source,goal);
  tab.Add("Saved goal",goalChooser_).WithWidget(frc::BuiltInWidgets::kComboBoxChooser)
      .WithPosition(0,4).WithSize(2,1);
  intermediatePoses_=tab.Add("Intermediate poses (x,y,deg; ...)",wpi::Twine(""))
      .WithWidget(frc::BuiltInWidgets::kTextView).WithPosition(2,4).WithSize(6,1).GetEntry();
  UpdateGoalSelection();
  goToGoal_.SetBoolean(false); stopNavigation_.SetBoolean(false);
}

void RobotSubsystem::UpdateGoalSelection() {
  if (goalSettings_.empty()) return;
  const int count=static_cast<int>(goalSettings_.size());
  // Save the displayed destination and source-specific guides before switching.
  // Editing the dashboard does not modify a route already captured by StartGoal.
  if (selectedGoal_>=0) {
    auto& old=goalSettings_[selectedGoal_];
    old={goalX_.GetDouble(old.xMM),goalY_.GetDouble(old.yMM),goalHeading_.GetDouble(old.headingDeg)};
    auto& route=routeSettings_[selectedSource_*count+selectedGoal_];
    route=intermediatePoses_.GetString(route);
  }
  const int selected=goalChooser_.GetSelected();
  if (selected<0 || selected>=count) return;
  int source=selectedSource_;
  // Freeze the source during travel. After arrival, use the current measured
  // pose to select the next transfer, including a return to Start.
  if (!navigation_.IsRunning()) {
    source=navigation_config::GOAL_COUNT;
    if (navigation_.HasReference()) {
      const auto pose=navigation_.GetState()==GoalNavigationRoutine::State::Complete?
          navigation_.ControlPose():localization_.GetPose();
      if (navigation_.GetState()==GoalNavigationRoutine::State::Ready && initializedSource_>=0)
        source=initializedSource_; // near a named area, not necessarily within 75 mm
      else {
        double nearest=navigation_config::SOURCE_MATCH_RADIUS_MM;
        for (int i=0;i<navigation_config::GOAL_COUNT;++i) {
          const double distance=std::hypot(pose.x_MM-goalSettings_[i].xMM,pose.y_MM-goalSettings_[i].yMM);
          if (distance<=nearest) { nearest=distance; source=i; }
        }
      }
    }
  }
  if (selected==selectedGoal_ && source==selectedSource_) return;
  const auto& next=goalSettings_[selected];
  goalX_.SetDouble(next.xMM); goalY_.SetDouble(next.yMM); goalHeading_.SetDouble(next.headingDeg);
  intermediatePoses_.SetString(routeSettings_[source*count+selected]);
  selectedGoal_=selected; selectedSource_=source;
}
