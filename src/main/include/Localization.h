#pragma once

#include "Hardware.h"
#include "ArenaMap.h"

class Localization {
 public:
  explicit Localization(Hardware* hardware);

  void Reset(double x_MM, double y_MM, double headingDeg);
  void UpdateOdometry(double dtSeconds);
  bool CorrectFromSonar(int sensorIndex, double measuredRange_MM);
  bool CorrectFromLine(const autonomous_config::Station& station, double tapeLeftError_MM);
  bool IsHealthy() const;
  double GetPositionSigmaMM() const;
  void PublishTelemetry(double sonarRange_MM, int sonarIndex) const;

  const RobotPose& GetPose() const { return pose_; }
  double GetBodyVelocityX_MMps() const { return lastBodyVelocityX_MMps_; }
  double GetBodyVelocityY_MMps() const { return lastBodyVelocityY_MMps_; }

 private:
  double PredictSonarRange(int sensorIndex, const RobotPose& pose) const;
  ArenaRayHit PredictSonarHit(int sensorIndex, const RobotPose& pose, double angleOffset=0) const;
  void ApplyScalarCorrection(const double jacobian[3], double innovation,
                             double variance);
  static double WrapRadians(double angleRad);
  static double DegreesToRadians(double angleDeg);

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
