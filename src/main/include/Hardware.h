#pragma once

#include <frc/DigitalInput.h>
#include <frc/SPI.h>
#include <frc/Ultrasonic.h>

#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>

#include "AHRS.h"
#include "LocalizationConstants.h"

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
  void StopDrive();

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

 private:
  double wheelCommands_[localization_config::WHEEL_COUNT] = {0.0, 0.0, 0.0};
  studica::TitanQuad motor0_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              localization_config::WHEEL_TITAN_PORT[0]};
  studica::TitanQuad motor1_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              localization_config::WHEEL_TITAN_PORT[1]};
  studica::TitanQuad motor2_{localization_config::TITAN_CAN_ID,
                              localization_config::TITAN_MOTOR_FREQUENCY_HZ,
                              localization_config::WHEEL_TITAN_PORT[2]};

  studica::TitanQuadEncoder encoder0_{
      motor0_, localization_config::WHEEL_TITAN_PORT[0],
      (2.0 * localization_config::PI * localization_config::WHEEL_RADIUS_MM) /
          (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV *
           localization_config::MOTOR_TO_WHEEL_RATIO)};
  studica::TitanQuadEncoder encoder1_{
      motor1_, localization_config::WHEEL_TITAN_PORT[1],
      (2.0 * localization_config::PI * localization_config::WHEEL_RADIUS_MM) /
          (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV *
           localization_config::MOTOR_TO_WHEEL_RATIO)};
  studica::TitanQuadEncoder encoder2_{
      motor2_, localization_config::WHEEL_TITAN_PORT[2],
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

  frc::DigitalInput startButton_{localization_config::START_BUTTON_DIO};
  AHRS imu_{frc::SPI::Port::kMXP};
};
