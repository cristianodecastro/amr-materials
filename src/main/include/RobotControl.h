#pragma once

#include <memory>
#include <string>
#include <frc/DigitalInput.h>
#include <frc/SPI.h>
#include <frc/Ultrasonic.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>
#include <studica/Cobra.h>
#include <studica/Servo.h>
#include "AHRS.h"
#include "Autonomous.h"
#include "Sensors.h"
#include "TeleopControl.h"

// Physical IO and final output gating.
class Hardware {
 public:
  Hardware();

  void ResetDriveEncoders();
  double GetWheelEncoderRaw(int wheel);
  void SetWheelPwm(int wheel, double output);
  double GetWheelCommand(int wheel) const;
  void DriveRobotRelative(double forward, double left, double yawRadps,
                                   double maxOutput, double dt);
  double GetWheelTargetMMps(int i) const { return ActiveWheelDrive().TargetMMps(i); }
  double GetWheelMeasuredMMps(int i) const { return ActiveWheelDrive().MeasuredMMps(i); }
  bool HasDriveFault() const { return ActiveWheelDrive().HasFault(); }
  bool IsDriveStarting() const { return ActiveWheelDrive().IsStarting(); }
  std::string GetDriveStatus() const;
  void StopDrive();
  void SetAutonomousDriveEnabled(bool enabled) { autonomousDriveEnabled_=enabled; }
  void SetLiftPwm(double output);
  double GetLiftCommand() const { return liftCommand_; }
  double GetLiftEncoderRaw();
  bool GetLiftUpperLimit() const;
  bool GetLiftLowerLimit() const;
  void SetServoAngle(int servo, double degrees, bool enabled);
  void StopServos();

  double GetYawDegrees();
  double GetYawRateDegreesPerSecond();
  double GetWorldAccelX_G();
  double GetWorldAccelY_G();
  double GetRawAccelX_G();
  double GetRawAccelY_G();
  double GetRawAccelZ_G();
  double GetRawGyroX_Dps();
  double GetRawGyroY_Dps();
  double GetRawGyroZ_Dps();
  bool IsImuConnected();
  bool IsImuCalibrating();
  double GetImuUpdateCount() { return imu_.GetUpdateCount(); }

  void PingSonar(int index);
  double GetSonarRangeMM(int index) const;
  bool IsSonarRangeValid(int index) const;
  LineReading ReadLine();
  void StopManipulator();

 private:
  static bool TeleopMotorMotionAllowed();
  bool DriveMotorMotionAllowed() const;
  const WheelDriveController& ActiveWheelDrive() const { return teleopWheelActive_?teleopWheelDrive_:wheelDrive_; }
  bool autonomousDriveEnabled_=false;
  double liftCommand_=0;
  std::unique_ptr<frc::DigitalInput> liftUpperLimit_,liftLowerLimit_;
  std::unique_ptr<studica::Cobra> cobra_;
  WheelDriveController wheelDrive_;
  WheelDriveController teleopWheelDrive_;
  bool teleopWheelActive_=false;
  double wheelCommands_[localization_config::WHEEL_COUNT] = {0.0, 0.0, 0.0};
  studica::Servo manualServo_{manipulator_config::SERVO_PWM[0]};
  studica::Servo triangleServo_{manipulator_config::SERVO_PWM[1]};
  studica::TitanQuad leftRearMotor_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              localization_config::WHEEL_TITAN_PORT[0]};
  studica::TitanQuad rightRearMotor_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              localization_config::WHEEL_TITAN_PORT[1]};
  studica::TitanQuad frontMotor_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              localization_config::WHEEL_TITAN_PORT[2]};
  studica::TitanQuad liftMotor_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              manipulator_config::TITAN_MOTOR_PORT};
  // Raw counts only: lift gearing/travel conversion is not calibrated yet.
  studica::TitanQuadEncoder liftEncoder_{liftMotor_,manipulator_config::TITAN_MOTOR_PORT,1.0};

  studica::TitanQuadEncoder leftRearEncoder_{
      leftRearMotor_, localization_config::WHEEL_TITAN_PORT[0],
      localization_config::ENCODER_MM_PER_COUNT};
  studica::TitanQuadEncoder rightRearEncoder_{
      rightRearMotor_, localization_config::WHEEL_TITAN_PORT[1],
      localization_config::ENCODER_MM_PER_COUNT};
  studica::TitanQuadEncoder frontEncoder_{
      frontMotor_, localization_config::WHEEL_TITAN_PORT[2],
      localization_config::ENCODER_MM_PER_COUNT};

  frc::Ultrasonic leftSonar_{localization_config::SONARS[0].triggerDio,
                           localization_config::SONARS[0].echoDio};
  frc::Ultrasonic rearSonar_{localization_config::SONARS[1].triggerDio,
                           localization_config::SONARS[1].echoDio};
  frc::Ultrasonic rightSonar_{localization_config::SONARS[2].triggerDio,
                            localization_config::SONARS[2].echoDio};
  frc::Ultrasonic frontSonar_{localization_config::SONARS[3].triggerDio,
                            localization_config::SONARS[3].echoDio};

  AHRS imu_{frc::SPI::Port::kMXP}; // preserve the original device sampling configuration
};

// Mode-isolated pose observers. Only Autonomous uses the simplified odometry.
class Localization {
 public:
  explicit Localization(Hardware* hardware);
  void Reset(double x_MM,double y_MM,double headingDeg);
  void UpdateOdometry(double dtSeconds);
  void SetAutonomousMode(bool autonomous);
  bool IsHealthy() const { return autonomousMode_?odometry_.Healthy():teleopTelemetry_.Healthy(); }
  void PublishTelemetry() const;
  const RobotPose& GetPose() const { return autonomousMode_?odometry_.Pose():teleopTelemetry_.Pose(); }
  double GetBodyVelocityX_MMps() const { return autonomousMode_?odometry_.ForwardSpeed():teleopTelemetry_.ForwardSpeed(); }
  double GetBodyVelocityY_MMps() const { return autonomousMode_?odometry_.LeftSpeed():teleopTelemetry_.LeftSpeed(); }
  std::string GetStatus() const;
  double GetImuOffsetDegrees() const { return autonomousMode_?odometry_.OffsetDegrees():teleopTelemetry_.OffsetDegrees(); }
 private:
  Hardware* hardware_;
  drive_kinematics::EncoderGyroOdometry odometry_;
  drive_kinematics::TeleopPoseTelemetry teleopTelemetry_;
  bool autonomousMode_=false;
};
// Robot scheduler, modes and dashboard integration.
class CameraCapture;
class RobotSubsystem : public frc2::SubsystemBase {
 public:
  explicit RobotSubsystem(CameraCapture* camera=nullptr);
  void Periodic() override;
  // Explicit lifecycle entry/exit: no autonomous state survives a mode change.
  void EnterAutonomous();
  void StopAll();
  bool HasSimpleTaskSelection() { return taskChooser_.GetSelected()!=static_cast<int>(SimpleTask::None); }

 private:
  std::string PreflightError() const;
  void InitializeDashboard();
  void InitializeTeleopDashboard();
  void InitializeCobraDashboard();
  void InitializeTaskDashboard();
  void UpdateSimpleTask(double now,double dt);
  void UpdateTeleopDrive(double dt);
  void UpdateSonarPoll(double now, bool enabled);
  void UpdateAutonomous(double now, double dt);
  void UpdateArenaTest(double now, double dt);
  void PublishDiagnostics();
  bool SonarMotionSafe(double now) const;

  Hardware hardware_;
  Localization localization_{&hardware_};
  AutonomousController autonomous_;
  ArenaTestController arenaTest_;
  SimpleTaskController simpleTask_;
  CameraCapture* camera_;
  frc::SendableChooser<int> taskChooser_;
  nt::NetworkTableEntry runTask_,stopTask_;
  bool runTaskRequested_=false,stopTaskRequested_=false,simpleSession_=false;
  unsigned colourRequest_=0;
  std::string taskNotice_="Choose a task; requests run only in enabled Autonomous";
  frc::Joystick driver_{localization_config::DRIVER_JOYSTICK_PORT};
  TeleopDriveController teleopDrive_;
  bool driverConnected_=false;
  double driverRawX_=0,driverRawY_=0;
  double driverRightX_=0;
  bool driverPovPresent_=false;
  int driverPov_=-1;
  LiftTeleopController liftTeleop_;
  ServoTeleopController servoTeleop_;
  bool driverTriangle_=false;
  bool driverL1_=false,driverR1_=false;
  bool autoSession_=false;
  bool poseAnchored_=false;
  LineReading line_;
  CobraCheck cobraCheck_;
  nt::NetworkTableEntry captureCobraWhite_,captureCobraBlack_;
  nt::NetworkTableEntry startRoutine_,stopRoutine_;
  nt::NetworkTableEntry setSpReference_;
  std::string poseReferenceStatus_="Disabled only: place at SP facing +X, then Set SP reference";
  bool startRoutineRequested_=false,stopRoutineRequested_=false;
  double lastLoopTime_=0;
  double controlDt_=localization_config::CONTROL_PERIOD_S,maxControlDt_=0;
  double imuRateWindowTime_=0,imuRateWindowCount_=0,imuMeasuredHz_=0;
  unsigned lateUpdates_=0;
  int nextSonarIndex_=0;
  bool sonarAwaitingEcho_=false;
  double sonarPingTime_=0,lastSonarPingTime_=0;
  double sonarRanges_MM_[4]={-1,-1,-1,-1};
  double sonarReadTime_[4]={-1,-1,-1,-1};
  bool sonarRangeValid_[4]={false,false,false,false};
};
