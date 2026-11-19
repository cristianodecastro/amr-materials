#pragma once

#include <memory>
#include <string>
#include <vector>

#include <frc/AnalogInput.h>
#include <frc/SPI.h>
#include <frc/Ultrasonic.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>
#include <studica/Cobra.h>

#include "AHRS.h"
#include "Sensors.h"
#include "TeleopControl.h"
#include "Autonomous.h"

// Physical IO and final output gating.
class Hardware {
 public:
  Hardware();

  void ResetDriveEncoders();
  double GetWheelEncoderRaw(int wheel);
  void SetWheelPwm(int wheel, double output,
                   double maxOutput=localization_config::TELEOP_MAX_MOTOR_OUTPUT);
  double GetWheelCommand(int wheel) const;
  // Explicit callers keep autonomous output treatment out of manual driving.
  void DriveTeleopRobotRelative(double forward, double left, double yawRadps,
                               double maxOutput, double dt);
  void DriveAutonomousRobotRelative(double forward, double left, double yawRadps,
                                   double maxOutput, double dt);
  double GetWheelMeasuredMMps(int i) const { return wheelDrive_.MeasuredMMps(i); }
  bool HasDriveFault() const { return wheelDrive_.HasFault(); }
  std::string GetDriveStatus() const;
  void StopDrive();
  void SetAutonomousDriveEnabled(bool enabled) { autonomousDriveEnabled_=enabled; }

  double GetYawDegrees();
  double GetYawRateDegreesPerSecond();
  void SampleImu(double now);
  bool IsImuReady() const { return imuReady_; }
  std::string GetImuStatus() const;
  double GetWorldAccelX_G();
  double GetWorldAccelY_G();
  bool IsImuConnected();
  bool IsImuCalibrating();

  void PingSonar(int index);
  double GetSonarRangeMM(int index) const;
  bool IsSonarRangeValid(int index) const;
  IrReading ReadIr(int index);
  LineReading ReadLine();

 private:
  static bool TeleopMotorMotionAllowed();
  bool AutonomousMotorMotionAllowed() const;
  bool DriveMotorMotionAllowed() const;
  void DriveWithContext(double forward,double left,double yawRadps,
                        double maxOutput,double dt,WheelDriveContext context);
  bool autonomousDriveEnabled_=false;
  std::unique_ptr<studica::Cobra> cobra_;
  WheelDriveController wheelDrive_;
  bool teleopWheelActive_=false;
  double wheelCommands_[localization_config::WHEEL_COUNT] = {0.0, 0.0, 0.0};
  studica::TitanQuad frontLeftMotor_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              localization_config::WHEEL_TITAN_PORT[0]};
  studica::TitanQuad rearMotor_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              localization_config::WHEEL_TITAN_PORT[1]};
  studica::TitanQuad frontRightMotor_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              localization_config::WHEEL_TITAN_PORT[2]};
  studica::TitanQuadEncoder frontLeftEncoder_{
      frontLeftMotor_, localization_config::WHEEL_TITAN_PORT[0],
      localization_config::ENCODER_MM_PER_COUNT};
  studica::TitanQuadEncoder rearEncoder_{
      rearMotor_, localization_config::WHEEL_TITAN_PORT[1],
      localization_config::ENCODER_MM_PER_COUNT};
  studica::TitanQuadEncoder frontRightEncoder_{
      frontRightMotor_, localization_config::WHEEL_TITAN_PORT[2],
      localization_config::ENCODER_MM_PER_COUNT};

  frc::Ultrasonic frontSonar_{localization_config::SONARS[0].triggerDio,
                            localization_config::SONARS[0].echoDio};
  frc::Ultrasonic rearSonar_{localization_config::SONARS[1].triggerDio,
                           localization_config::SONARS[1].echoDio};
  frc::AnalogInput leftIr_{localization_config::IR_ANALOG_CHANNEL[0]};
  frc::AnalogInput rightIr_{localization_config::IR_ANALOG_CHANNEL[1]};

  AHRS imu_{frc::SPI::Port::kMXP}; // preserve the original device sampling configuration
  YawRateEstimator yawRate_;
  double imuYawDeg_=0,imuYawRateDps_=0,imuTemperatureC_=0,imuSampleTime_=-1,imuCalibrationDoneSince_=-1;
  double imuUpdateCount_=-1;
  std::uint32_t imuTimestamp_=0;
  bool imuHaveSample_=false,imuConnected_=false,imuCalibrating_=true,imuReady_=false,imuDataFresh_=false;
};

// Mode-isolated pose observers. Only Autonomous uses the simplified odometry.
class Localization {
 public:
  explicit Localization(Hardware* hardware);
  void Reset(double x_MM,double y_MM,double headingDeg);
  void ApplyReference(RobotPose pose,bool resetHeading);
  void UpdateOdometry(double dtSeconds);
  void SetAutonomousMode(bool autonomous);
  void CorrectAutonomousPosition(ArenaPoint delta) {
    if (autonomousMode_) odometry_.CorrectPosition(delta.x,delta.y);
  }
  bool IsHealthy() const { return autonomousMode_?odometry_.Healthy():teleopTelemetry_.Healthy(); }
  const char* OdometryFaultReason() const { return autonomousMode_?odometry_.FaultReason():"Teleop odometry unavailable"; }
  const RobotPose& GetPose() const { return autonomousMode_?odometry_.Pose():teleopTelemetry_.Pose(); }
  double HeadingOffsetDegrees() const {
    return (autonomousMode_?odometry_.HeadingOffsetRad():teleopTelemetry_.HeadingOffsetRad())*180/robot_geometry::PI;
  }
  double GetBodyVelocityX_MMps() const { return autonomousMode_?odometry_.ForwardSpeed():teleopTelemetry_.ForwardSpeed(); }
  double GetBodyVelocityY_MMps() const { return autonomousMode_?odometry_.LeftSpeed():teleopTelemetry_.LeftSpeed(); }
 private:
  Hardware* hardware_;
  drive_kinematics::EncoderGyroOdometry odometry_;
  drive_kinematics::TeleopPoseTelemetry teleopTelemetry_;
  bool autonomousMode_=false;
  bool imuSynchronized_=false;
};
// Project 8: manual driving and one explicit-start sensor/tape test.
class RobotSubsystem : public frc2::SubsystemBase {
 public:
  RobotSubsystem();
  void Periodic() override;
  void InitializeDashboard(); // called after stale NetworkTables are cleared
  void EnterAutonomous();
  void StopAll();

 private:
  void InitializeMotionDashboard();
  void InitializeSensingDashboard();
  void InitializeNavigationDashboard();
  void InitializeTapeDashboard();
  void UpdateStartingAreaSelection();
  void UpdateGoalSelection();
  void UpdateTeleopDrive(double dt);
  void UpdateSonarPoll(double now);
  void UpdateTapeRoutine(double now,double dt,bool startLeft,bool startRight,bool stop);
  NavigationInput NavigationSample(double now,double dt,bool referenceOnly=false);
  void UpdateNavigation(double now,double dt,bool reference,bool go);

  Hardware hardware_;
  Localization localization_{&hardware_};
  TapeAlignmentRoutine tapeRoutine_;
  GoalNavigationRoutine navigation_;
  bool navigationSelected_=false;
  nt::NetworkTableEntry initializePose_,goToGoal_,stopNavigation_,goalX_,goalY_,goalHeading_;
  nt::NetworkTableEntry intermediatePoses_,useEditedRoute_,initialX_,initialY_,initialHeading_;
  frc::SendableChooser<int> goalChooser_;
  frc::SendableChooser<int> startingAreaChooser_;
  int selectedStartingArea_=-1,initializedSource_=-1,activeGoal_=-1;
  struct GoalSettings { double xMM,yMM,headingDeg; };
  std::vector<GoalSettings> goalSettings_;
  std::vector<std::string> routeSettings_;
  int selectedGoal_=-1;
  int selectedSource_=navigation_config::GOAL_COUNT; // GOAL_COUNT = unnamed current pose
  nt::NetworkTableEntry startTapeLeft_,startTapeRight_,stopTape_;
  bool autonomousSession_=false;
  std::string routineNotice_;
  frc::Joystick driver_{localization_config::DRIVER_JOYSTICK_PORT};
  TeleopDriveController teleopDrive_;
  bool driverConnected_=false;
  double driverRawX_=0,driverRawY_=0;
  double driverRightX_=0;
  LineReading line_;
  CobraCheck cobraCheck_;
  double lastLoopTime_=0;
  double controlDt_=localization_config::CONTROL_PERIOD_S;
  double peakControlDt_=0;
  double lastSensorTime_=-1;
  IrReading irReading_[localization_config::IR_COUNT];
  int nextSonarIndex_=0;
  bool sonarAwaitingEcho_=false;
  double sonarPingTime_=0,lastSonarPingTime_=0;
  double sonarRanges_MM_[localization_config::SONAR_COUNT]={-1,-1};
  double sonarReadTime_[localization_config::SONAR_COUNT]={-1,-1};
  bool sonarRangeValid_[localization_config::SONAR_COUNT]={false,false};
};
