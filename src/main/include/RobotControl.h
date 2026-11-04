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
  double GetWheelTargetMMps(int i) const { return wheelDrive_.TargetMMps(i); }
  double GetWheelMeasuredMMps(int i) const { return wheelDrive_.MeasuredMMps(i); }
  bool HasDriveFault() const { return wheelDrive_.HasFault(); }
  bool IsDriveStarting() const { return wheelDrive_.IsStarting(); }
  std::string GetDriveStatus() const;
  void StopDrive();
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

  void PingSonar(int index);
  double GetSonarRangeMM(int index) const;
  bool IsSonarRangeValid(int index) const;
  LineReading ReadLine();
  void StopManipulator();

 private:
  static bool TeleopMotorMotionAllowed();
  double liftCommand_=0;
  std::unique_ptr<frc::DigitalInput> liftUpperLimit_,liftLowerLimit_;
  std::unique_ptr<studica::Cobra> cobra_;
  WheelDriveController wheelDrive_;
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
      (2.0 * robot_geometry::PI * localization_config::WHEEL_RADIUS_MM) /
          (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV *
           localization_config::MOTOR_TO_WHEEL_RATIO)};
  studica::TitanQuadEncoder rightRearEncoder_{
      rightRearMotor_, localization_config::WHEEL_TITAN_PORT[1],
      (2.0 * robot_geometry::PI * localization_config::WHEEL_RADIUS_MM) /
          (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV *
           localization_config::MOTOR_TO_WHEEL_RATIO)};
  studica::TitanQuadEncoder frontEncoder_{
      frontMotor_, localization_config::WHEEL_TITAN_PORT[2],
      (2.0 * robot_geometry::PI * localization_config::WHEEL_RADIUS_MM) /
          (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV *
           localization_config::MOTOR_TO_WHEEL_RATIO)};

  frc::Ultrasonic backLeftSonar_{localization_config::SONARS[0].triggerDio,
                                  localization_config::SONARS[0].echoDio};
  frc::Ultrasonic backRightSonar_{localization_config::SONARS[1].triggerDio,
                                   localization_config::SONARS[1].echoDio};
  frc::Ultrasonic leftSonar_{localization_config::SONARS[2].triggerDio,
                              localization_config::SONARS[2].echoDio};
  frc::Ultrasonic rightSonar_{localization_config::SONARS[3].triggerDio,
                               localization_config::SONARS[3].echoDio};

  AHRS imu_{frc::SPI::Port::kMXP};
};

// Odometry and sensor fusion.
class Localization {
 public:
  explicit Localization(Hardware* hardware);

  void Reset(double x_MM, double y_MM, double headingDeg);
  void UpdateOdometry(double dtSeconds);
  bool CorrectFromSonar(int sensorIndex, double measuredRange_MM);
  bool CorrectFromLine(const autonomous_config::Station& station, double tapeLeftError_MM);
  bool IsHealthy() const;
  double GetPositionSigmaMM() const;
  void PublishTelemetry() const;

  const RobotPose& GetPose() const { return pose_; }
  double GetBodyVelocityX_MMps() const { return lastBodyVelocityX_MMps_; }
  double GetBodyVelocityY_MMps() const { return lastBodyVelocityY_MMps_; }

 private:
  double PredictSonarRange(int sensorIndex, const RobotPose& pose) const;
  ArenaRayHit PredictSonarHit(int sensorIndex, const RobotPose& pose, double angleOffset=0) const;
  void ApplyScalarCorrection(const double jacobian[3], double innovation,
                             double variance);

  Hardware* hardware_;
  ArenaMap arena_;
  RobotPose pose_;
  double covariance_[3][3];
  double previousEncoder_[localization_config::WHEEL_COUNT];
  double fusedWorldVelocityX_MMps_;
  double fusedWorldVelocityY_MMps_;
  double imuYawOffsetRad_;
  double lastBodyVelocityX_MMps_;
  double lastBodyVelocityY_MMps_;
  double lastAngularVelocityRadps_;
  double lastAccelX_G_;
  double lastAccelY_G_;
  double lastRawAccelX_G_;
  double lastRawAccelY_G_;
  double lastRawAccelZ_G_;
  double lastRawGyroX_Dps_;
  double lastRawGyroY_Dps_;
  double lastRawGyroZ_Dps_;
  double lastYawDegrees_;
  double lastYawRateDps_;
};

// Robot scheduler, modes and dashboard integration.
class RobotSubsystem : public frc2::SubsystemBase {
 public:
  RobotSubsystem();
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
