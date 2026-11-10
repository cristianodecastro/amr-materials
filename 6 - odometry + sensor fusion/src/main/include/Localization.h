#pragma once

#include "Hardware.h"

struct RobotPose {
  double x_M;
  double y_M;
  double headingRad;
};

class Localization {
 public:
  explicit Localization(Hardware* hardware);

  void Reset(double x_M, double y_M, double headingDeg);
  void UpdateOdometry(double dtSeconds);
  bool CorrectFromSonar(int sensorIndex, double measuredRange_M);
  void PublishTelemetry(double sonarRange_M, int sonarIndex) const;

  const RobotPose& GetPose() const { return pose_; }

 private:
  double PredictSonarRange(int sensorIndex, const RobotPose& pose) const;
  void ApplyScalarCorrection(const double jacobian[3], double innovation,
                             double variance);
  static double WrapRadians(double angleRad);
  static double DegreesToRadians(double angleDeg);

  Hardware* hardware_;
  RobotPose pose_;
  double covariance_[3][3];
  double previousEncoder_[config::WHEEL_COUNT];
  double fusedWorldVelocityX_Mps_;
  double fusedWorldVelocityY_Mps_;
  double imuYawOffsetRad_;
  double lastBodyVelocityX_Mps_;
  double lastBodyVelocityY_Mps_;
  double lastAngularVelocityRadps_;
  double lastAccelX_G_;
  double lastAccelY_G_;
  double lastRawAccelX_G_;
  double lastRawAccelY_G_;
  double lastRawAccelZ_G_;
  double lastRawGyroX_Dps_;
  double lastRawGyroY_Dps_;
  double lastRawGyroZ_Dps_;
  double lastYawRateDps_;
};
