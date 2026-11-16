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
}
void RobotSubsystem::EnterAutonomous() {
  StopAll();
  localization_.SetAutonomousMode(true);
  tapeRoutine_.Enter(); autonomousSession_=true; routineNotice_.clear();
  navigation_.Enter(); navigationSelected_=false;
}
void RobotSubsystem::StopAll() {
  hardware_.SetAutonomousDriveEnabled(false);
  hardware_.StopDrive();

  teleopDrive_.Update(false,false,0,0,0,localization_config::CONTROL_PERIOD_S);
  tapeRoutine_.Abort(); autonomousSession_=false;
  navigation_.Abort(); ++requestGeneration_;
  startTapeLeft_.SetBoolean(false); startTapeRight_.SetBoolean(false); stopTape_.SetBoolean(false);
  referenceCorner_.SetBoolean(false); goToGoal_.SetBoolean(false); stopNavigation_.SetBoolean(false);
  localization_.SetAutonomousMode(false);
}
void RobotSubsystem::Periodic() {
  const double now=frc::Timer::GetFPGATimestamp(),dt=now-lastLoopTime_;
  lastLoopTime_=now;
  controlDt_=dt;
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
  const bool startLeft=startTapeLeft_.GetBoolean(false),startRight=startTapeRight_.GetBoolean(false);
  const bool stop=stopTape_.GetBoolean(false);
  const bool reference=referenceCorner_.GetBoolean(false),go=goToGoal_.GetBoolean(false);
  const bool stopNav=stopNavigation_.GetBoolean(false);
  if (reference) referenceCorner_.SetBoolean(false);
  if (go) goToGoal_.SetBoolean(false);
  if (stopNav) stopNavigation_.SetBoolean(false);
  if (startLeft) startTapeLeft_.SetBoolean(false);
  if (startRight) startTapeRight_.SetBoolean(false);
  if (stop) stopTape_.SetBoolean(false);
  if (enabled && frc::RobotState::IsAutonomous() && !frc::RobotState::IsTest()) {
    if (!autonomousSession_) hardware_.StopDrive();
    else if (stop || stopNav) {
      tapeRoutine_.Abort(); navigation_.Abort(); ++requestGeneration_;
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
        navigationSelected_=false; navigation_.Enter(); ++requestGeneration_;
        UpdateTapeRoutine(now,dt,startLeft,startRight,false);
      }
    } else if (navigationSelected_) UpdateNavigation(now,dt,false,false);
    else UpdateTapeRoutine(now,dt,false,false,false);
  } else {
    if (autonomousSession_) StopAll();
    if (startLeft || startRight || reference || go) routineNotice_="Select and enable Autonomous, then press the requested task again";
    UpdateTeleopDrive(dt);
  }
}

NavigationInput RobotSubsystem::NavigationSample(double now,double dt) {
  NavigationInput in;
  in.motion={localization_.GetPose(),now,dt,
      hardware_.IsImuConnected() && !hardware_.IsImuCalibrating() && localization_.IsHealthy() &&
      frc::DriverStation::GetInstance().IsDSAttached(),false,false,
      robot_geometry::Radians(hardware_.GetYawRateDegreesPerSecond())};
  in.forwardMMps=localization_.GetBodyVelocityX_MMps(); in.leftMMps=localization_.GetBodyVelocityY_MMps();
  in.rangeMM[0]=irReading_[0].distanceMM; in.rangeMM[2]=irReading_[1].distanceMM;
  in.valid[0]=irReading_[0].valid; in.valid[2]=irReading_[1].valid;
  in.stamp[0]=in.stamp[2]=lastSensorTime_;
  in.rangeMM[1]=sonarRanges_MM_[1]; in.rangeMM[3]=sonarRanges_MM_[0];
  in.valid[1]=sonarRangeValid_[1]; in.valid[3]=sonarRangeValid_[0];
  in.stamp[1]=sonarReadTime_[1]; in.stamp[3]=sonarReadTime_[0];
  return in;
}
void RobotSubsystem::UpdateNavigation(double now,double dt,bool reference,bool go) {
  auto in=NavigationSample(now,dt);
  if (reference && go) routineNotice_="Reference corner and Go must be requested separately";
  else if (reference) {
    if (navigation_.IsRunning()) routineNotice_="Navigation running; Stop before taking a new corner reference";
    else { ++requestGeneration_; navigation_.ReferenceCorner(now); }
  } else if (go) {
    const RobotPose goal={goalX_.GetDouble(navigation_config::DEFAULT_GOAL_X_MM),
        goalY_.GetDouble(navigation_config::DEFAULT_GOAL_Y_MM),
        robot_geometry::Radians(goalHeading_.GetDouble(navigation_config::DEFAULT_GOAL_HEADING_DEG))};
    if (navigation_.StartGoal(goal,in)) ++requestGeneration_;
  }
  if (hardware_.HasDriveFault() && navigation_.IsRunning()) navigation_.Fail(hardware_.GetDriveStatus());
  navigation_.Update(in);
  if (navigation_.NeedsPoseReset()) {
    const auto pose=navigation_.RequestedPose();
    localization_.Reset(pose.x_MM,pose.y_MM,pose.headingRad*180/robot_geometry::PI);
    navigation_.ConfirmReference(); in=NavigationSample(now,dt);
  }
  if (navigation_.NeedsPositionCorrection()) {
    const auto requested=navigation_.RequestedPose(),current=localization_.GetPose();
    if (!localization_.CorrectPosition(requested.x_MM-current.x_MM,requested.y_MM-current.y_MM))
      navigation_.Fail("Position reference could not be applied");
    else { in=NavigationSample(now,dt); navigation_.ConfirmPositionCorrection(in); }
  }
  // One immutable planning job at a time. Old results cannot restart a stopped
  // task or replace a newer goal. No planning work blocks the 20 ms drive loop.
  if (planFuture_.valid() && planFuture_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
    try {
      auto plan=planFuture_.get();
      if (planGeneration_==requestGeneration_) navigation_.AcceptPlan(std::move(plan),in);
    } catch (const std::exception& error) {
      if (planGeneration_==requestGeneration_) navigation_.Fail(std::string("Planning failed: ")+error.what());
    }
  }
  if (navigation_.NeedsPlan() && !planFuture_.valid()) {
    const auto map=navigation_.Map(); const auto start=navigation_.PlanStart(in),goal=navigation_.Goal();
    planGeneration_=requestGeneration_;
    try {
      planFuture_=std::async(std::launch::async,[map,start,goal] {
        return ClearancePlanner(map,DefaultNavigationFootprint()).Plan(start,goal);
      });
      navigation_.PlanningStarted();
    } catch (const std::exception& error) { navigation_.Fail(std::string("Cannot start planner: ")+error.what()); }
  }
  const auto state=navigation_.GetState();
  const bool drive=state==GoalNavigationRoutine::State::Following ||
      state==GoalNavigationRoutine::State::Observing || state==GoalNavigationRoutine::State::FinalHeading;
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
// Compact grids: Motion 8x4, Sensing 8x5.
// Short labels and small numeric/boolean tiles fit the laptop without scrolling.
void RobotSubsystem::InitializeMotionDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Motion");
  tab.AddString("Motion status",[this] {
    // Visible revision distinguishes this executable from stale robot/UI data.
    const std::string revision="P8-r13 | ";
    if (!routineNotice_.empty()) return revision+routineNotice_;
    if (frc::RobotState::IsAutonomous()) return revision+(navigationSelected_?navigation_.Status():tapeRoutine_.GetStatus());
    return revision+(hardware_.HasDriveFault()?
        hardware_.GetDriveStatus():std::string(teleopDrive_.GetStatus()));
  }).WithPosition(0,0).WithSize(4,1);
  tab.AddBoolean("Enabled",[] { return frc::RobotState::IsEnabled(); }).WithPosition(4,0).WithSize(1,1);
  tab.AddBoolean("Controller",[this] { return driverConnected_; }).WithPosition(5,0).WithSize(1,1);
  stopTape_=tab.Add("Stop tape",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(6,0).WithSize(1,1).GetEntry();
  tab.AddBoolean("IMU ready",[this] { return hardware_.IsImuConnected() && !hardware_.IsImuCalibrating(); })
      .WithPosition(7,0).WithSize(1,1);
  tab.AddNumber("X (mm)",[this] { return localization_.GetPose().x_MM; }).WithPosition(0,1).WithSize(2,1);
  tab.AddNumber("Y (mm)",[this] { return localization_.GetPose().y_MM; }).WithPosition(2,1).WithSize(2,1);
  tab.AddNumber("Heading N=0 deg",[this] { return localization_.GetPose().headingRad*180/robot_geometry::PI; })
      .WithPosition(4,1).WithSize(2,1);
  tab.AddNumber("Loop (ms)",[this] { return controlDt_*1000; }).WithPosition(6,1).WithSize(2,1);
  for (int i=0;i<localization_config::WHEEL_COUNT;++i) {
    const int x=i==1?4:0,y=i==2?3:2;
    const std::string port="M"+std::to_string(localization_config::WHEEL_TITAN_PORT[i]);
    const std::string wheel=std::string(localization_config::WHEEL_NAME[i])+" "+port;
    tab.AddNumber(wheel+" counts",[this,i] { return hardware_.GetWheelEncoderRaw(i); })
        .WithPosition(x,y).WithSize(2,1);
    tab.AddNumber(port+" mm/s",[this,i] { return hardware_.GetWheelMeasuredMMps(i); })
        .WithPosition(x+2,y).WithSize(1,1);
    // Signed WHOLE percentage actually encoded for Titan, not a speed setpoint.
    tab.AddNumber(port+" PWM %",[this,i] { return std::round(hardware_.GetWheelCommand(i)*100); })
        .WithPosition(x+3,y).WithSize(1,1);
  }
  startTapeLeft_=tab.Add("Tape from left",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(4,3).WithSize(2,1).GetEntry();
  startTapeRight_=tab.Add("Tape from right",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(6,3).WithSize(2,1).GetEntry();
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

void RobotSubsystem::InitializeNavigationDashboard() {
  auto& tab=frc::Shuffleboard::GetTab("Navigation");
  tab.AddString("Navigation status",[this] {
    return std::string("P8-r13 | ")+(routineNotice_.empty()?navigation_.Status():routineNotice_);
  }).WithPosition(0,0).WithSize(6,1);
  stopNavigation_=tab.Add("Stop",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(6,0).WithSize(2,1).GetEntry();
  referenceCorner_=tab.Add("Reference corner",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(0,1).WithSize(2,1).GetEntry();
  goToGoal_=tab.Add("Go to goal",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(2,1).WithSize(2,1).GetEntry();
  tab.AddBoolean("Referenced",[this] { return navigation_.HasReference(); }).WithPosition(4,1).WithSize(1,1);
  tab.AddNumber("Path mm",[this] { return navigation_.PathLengthMM(); }).WithPosition(5,1).WithSize(1,1);
  tab.AddString("Waypoint",[this] {
    return std::to_string(navigation_.WaypointIndex())+"/"+
        std::to_string(navigation_.WaypointCount()>0?navigation_.WaypointCount()-1:0);
  }).WithPosition(6,1).WithSize(2,1);
  goalX_=tab.Add("Goal X mm",navigation_config::DEFAULT_GOAL_X_MM)
      .WithPosition(0,2).WithSize(3,1).GetEntry();
  goalY_=tab.Add("Goal Y mm",navigation_config::DEFAULT_GOAL_Y_MM)
      .WithPosition(3,2).WithSize(3,1).GetEntry();
  goalHeading_=tab.Add("Goal heading deg",navigation_config::DEFAULT_GOAL_HEADING_DEG)
      .WithPosition(6,2).WithSize(2,1).GetEntry();
  tab.AddNumber("Robot X mm",[this] { return localization_.GetPose().x_MM; }).WithPosition(0,3).WithSize(3,1);
  tab.AddNumber("Robot Y mm",[this] { return localization_.GetPose().y_MM; }).WithPosition(3,3).WithSize(3,1);
  tab.AddNumber("Robot heading deg",[this] { return localization_.GetPose().headingRad*180/robot_geometry::PI; })
      .WithPosition(6,3).WithSize(2,1);
  tab.AddString("Guide",[] {
    return "Autonomous + Enable. Face north (0 deg), rear/left beams clear to outer walls. "
        "Reference corner, then Go. +90 west / -90 east. 40 mm clearance; 10 mm / 1 deg tolerance.";
  }).WithPosition(0,4).WithSize(8,1);
  referenceCorner_.SetBoolean(false); goToGoal_.SetBoolean(false); stopNavigation_.SetBoolean(false);
}
