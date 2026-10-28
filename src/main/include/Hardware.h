#pragma once

#include <frc/DigitalInput.h>
#include <frc/SPI.h>
#include <frc/Ultrasonic.h>

#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>
#include <studica/Cobra.h>
#include <studica/Servo.h>
#include <memory>

#include "AHRS.h"
#include "LocalizationConstants.h"
#include "LineTracker.h"
#include "ObjectHandler.h"
#include "ManipulatorConfig.h"

class Hardware {
 public:
  Hardware();

  void ResetDriveEncoders();
  double GetWheelEncoderRaw(int wheel);
  void SetWheelPwm(int wheel, double output);
  double GetWheelCommand(int wheel) const;
  void DriveRobotRelative(double vx_MMps, double vy_MMps,
                          double yawRate_Radps, double maxOutput = 1.0);
  void DriveWheelsMMps(const double (&wheelSpeed)[3], double maxOutput);
  void DriveRobotRelativeFeedback(double forward, double left, double yawRadps,
                                   double maxOutput, double dt);
  void StopDrive();
  void SetLiftPwm(double output);
  double GetLiftCommand() const { return liftCommand_; }
  double GetLiftEncoderRaw();
  bool GetLiftUpperLimit() const;
  bool GetLiftLowerLimit() const;
  void SetServoAngle(int servo, double degrees, bool enabled);
  void StopServos();

  bool GetStartPressed() const;
  void ZeroYaw();
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
  bool IsObjectHandlerReady() const { return false; } // implement with actual mechanism
  void BeginObjectAction(ObjectAction action);
  ObjectActionStatus GetObjectActionStatus() const;
  void StopObjectHandler();

 private:
  static bool TeleopMotorMotionAllowed();
  double liftCommand_=0;
  std::unique_ptr<frc::DigitalInput> liftUpperLimit_,liftLowerLimit_;
  std::unique_ptr<studica::Cobra> cobra_;
  double speedPreviousCounts_[3]={0,0,0};
  double speedIntegral_[3]={0,0,0};
  bool speedBaselineValid_=false;
  double wheelCommands_[localization_config::WHEEL_COUNT] = {0.0, 0.0, 0.0};
  studica::Servo crossServo_{manipulator_config::SERVO_PWM[0]};
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
      (2.0 * localization_config::PI * localization_config::WHEEL_RADIUS_MM) /
          (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV *
           localization_config::MOTOR_TO_WHEEL_RATIO)};
  studica::TitanQuadEncoder rightRearEncoder_{
      rightRearMotor_, localization_config::WHEEL_TITAN_PORT[1],
      (2.0 * localization_config::PI * localization_config::WHEEL_RADIUS_MM) /
          (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV *
           localization_config::MOTOR_TO_WHEEL_RATIO)};
  studica::TitanQuadEncoder frontEncoder_{
      frontMotor_, localization_config::WHEEL_TITAN_PORT[2],
      (2.0 * localization_config::PI * localization_config::WHEEL_RADIUS_MM) /
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
