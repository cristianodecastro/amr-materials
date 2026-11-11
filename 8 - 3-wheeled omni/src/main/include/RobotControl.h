#pragma once

#include <memory>
#include <string>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <frc/SPI.h>
#include <frc/Ultrasonic.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>
#include <studica/Cobra.h>
#include <studica/Servo.h>
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
  void SetWheelPwm(int wheel, double output);
  double GetWheelCommand(int wheel) const;
  // Explicit callers keep autonomous output treatment out of manual driving.
  void DriveTeleopRobotRelative(double forward, double left, double yawRadps,
                               double maxOutput, double dt);
  void DriveAutonomousRobotRelative(double forward, double left, double yawRadps,
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
  IrReading ReadIr(int index);
  LineReading ReadLine();
  void StopManipulator();

 private:
  static bool TeleopMotorMotionAllowed();
  bool AutonomousMotorMotionAllowed() const;
  bool DriveMotorMotionAllowed() const;
  void DriveWithContext(double forward,double left,double yawRadps,
                        double maxOutput,double dt,WheelDriveContext context);
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

  frc::Ultrasonic frontSonar_{localization_config::SONARS[0].triggerDio,
                            localization_config::SONARS[0].echoDio};
  frc::Ultrasonic rearSonar_{localization_config::SONARS[1].triggerDio,
                           localization_config::SONARS[1].echoDio};
  frc::AnalogInput leftIr_{localization_config::IR_ANALOG_CHANNEL[0]};
  frc::AnalogInput rightIr_{localization_config::IR_ANALOG_CHANNEL[1]};

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
  void UpdateTeleopDrive(double dt);
  void UpdateSonarPoll(double now, bool enabled);
  void UpdateTapeRoutine(double now,double dt,bool start,bool stop);

  Hardware hardware_;
  Localization localization_{&hardware_};
  TapeAlignmentRoutine tapeRoutine_;
  nt::NetworkTableEntry startTape_,stopTape_;
  bool autonomousSession_=false;
  std::string routineNotice_;
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
  LineReading line_;
  CobraCheck cobraCheck_;
  unsigned savedCobraRevision_=0;
  std::string cobraCalibrationNotice_;
  nt::NetworkTableEntry captureCobraWhite_,captureCobraBlack_;
  double lastLoopTime_=0;
  double controlDt_=localization_config::CONTROL_PERIOD_S;
  double lastSensorTime_=-1;
  IrReading irReading_[localization_config::IR_COUNT];
  int nextSonarIndex_=0;
  bool sonarAwaitingEcho_=false;
  double sonarPingTime_=0,lastSonarPingTime_=0;
  double sonarRanges_MM_[localization_config::SONAR_COUNT]={-1,-1};
  double sonarReadTime_[localization_config::SONAR_COUNT]={-1,-1};
  bool sonarRangeValid_[localization_config::SONAR_COUNT]={false,false};
};
