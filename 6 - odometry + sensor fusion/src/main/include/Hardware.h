#pragma once

#include <frc/DigitalInput.h>
#include <frc/Ultrasonic.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>
#include "AHRS.h"

#include "Constants.h"

class Hardware {
 public:
  Hardware();

  void ResetDriveEncoders();
  double GetWheelEncoderRaw(int wheel);
  void SetWheelPwm(int wheel, double output);
  void DriveRobotRelative(double vx_Mps, double vy_Mps, double yawRate_Radps);
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
  double GetSonarRangeMetres(int index) const;
  bool IsSonarRangeValid(int index) const;

 private:
  studica::TitanQuad motor0_{config::TITAN_CAN_ID,
                              config::TITAN_MOTOR_FREQUENCY_HZ,
                              config::WHEEL_TITAN_PORT[0]};
  studica::TitanQuad motor1_{config::TITAN_CAN_ID,
                              config::TITAN_MOTOR_FREQUENCY_HZ,
                              config::WHEEL_TITAN_PORT[1]};
  studica::TitanQuad motor2_{config::TITAN_CAN_ID,
                              config::TITAN_MOTOR_FREQUENCY_HZ,
                              config::WHEEL_TITAN_PORT[2]};

  studica::TitanQuadEncoder encoder0_{
      motor0_, config::WHEEL_TITAN_PORT[0],
      (2.0 * config::PI * config::WHEEL_RADIUS_M) /
          config::ENCODER_COUNTS_PER_OUTPUT_REV};
  studica::TitanQuadEncoder encoder1_{
      motor1_, config::WHEEL_TITAN_PORT[1],
      (2.0 * config::PI * config::WHEEL_RADIUS_M) /
          config::ENCODER_COUNTS_PER_OUTPUT_REV};
  studica::TitanQuadEncoder encoder2_{
      motor2_, config::WHEEL_TITAN_PORT[2],
      (2.0 * config::PI * config::WHEEL_RADIUS_M) /
          config::ENCODER_COUNTS_PER_OUTPUT_REV};

  frc::Ultrasonic backLeftSonar_{config::SONARS[0].triggerDio,
                                  config::SONARS[0].echoDio};
  frc::Ultrasonic backRightSonar_{config::SONARS[1].triggerDio,
                                   config::SONARS[1].echoDio};
  frc::Ultrasonic leftSonar_{config::SONARS[2].triggerDio,
                              config::SONARS[2].echoDio};
  frc::Ultrasonic rightSonar_{config::SONARS[3].triggerDio,
                               config::SONARS[3].echoDio};

  frc::DigitalInput startButton_{config::START_BUTTON_DIO};
  AHRS imu_{frc::SPI::Port::kMXP};
};
