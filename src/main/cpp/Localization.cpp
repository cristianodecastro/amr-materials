#include "Localization.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kAccelDampingPerSecond = 0.98;
constexpr double kPositionProcessNoiseMM2PerSecond = 400.0;
constexpr double kHeadingProcessNoiseRad2PerSecond = 0.0008;
constexpr double kYawMeasurementStdDevRad = 0.035;
constexpr double kJacobianPositionStepMM = 1.0;
constexpr double kJacobianHeadingStepRad = 0.5 * localization_config::PI / 180.0;

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}
}  // namespace

Localization::Localization(Hardware* hardware)
    : hardware_(hardware),
      pose_{localization_config::START_X_MM, localization_config::START_Y_MM,
            DegreesToRadians(localization_config::START_HEADING_DEG)},
      fusedWorldVelocityX_MMps_(0.0),
      fusedWorldVelocityY_MMps_(0.0),
      imuYawOffsetRad_(0.0),
      lastBodyVelocityX_MMps_(0.0),
      lastBodyVelocityY_MMps_(0.0),
      lastAngularVelocityRadps_(0.0),
      lastAccelX_G_(0.0),
      lastAccelY_G_(0.0),
      lastRawAccelX_G_(0.0),
      lastRawAccelY_G_(0.0),
      lastRawAccelZ_G_(0.0),
      lastRawGyroX_Dps_(0.0),
      lastRawGyroY_Dps_(0.0),
      lastRawGyroZ_Dps_(0.0),
      lastYawDegrees_(0.0),
      lastYawRateDps_(0.0) {
  Reset(localization_config::START_X_MM, localization_config::START_Y_MM,
        localization_config::START_HEADING_DEG);
}

void Localization::Reset(double x_MM, double y_MM, double headingDeg) {
  pose_.x_MM = x_MM;
  pose_.y_MM = y_MM;
  pose_.headingRad = WrapRadians(DegreesToRadians(headingDeg));
  imuYawOffsetRad_ = WrapRadians(
      pose_.headingRad - DegreesToRadians(hardware_->GetYawDegrees()));

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) covariance_[row][col] = 0.0;
  }
  covariance_[0][0] = 250000.0;
  covariance_[1][1] = 250000.0;
  covariance_[2][2] = 0.12;

  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    previousEncoder_[wheel] = hardware_->GetWheelEncoderRaw(wheel);
  }
  fusedWorldVelocityX_MMps_ = 0.0;
  fusedWorldVelocityY_MMps_ = 0.0;
  lastBodyVelocityX_MMps_ = 0.0;
  lastBodyVelocityY_MMps_ = 0.0;
  lastAngularVelocityRadps_ = 0.0;
}

void Localization::UpdateOdometry(double dtSeconds) {
  if (dtSeconds <= 0.0) return;
  const double dt = Clamp(dtSeconds, 0.001, 0.10);

  double wheelSpeed[localization_config::WHEEL_COUNT] = {0.0, 0.0, 0.0};
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    const double current = hardware_->GetWheelEncoderRaw(wheel);
    const double deltaCounts = (current - previousEncoder_[wheel]) *
                               localization_config::WHEEL_ENCODER_SIGN[wheel];
    previousEncoder_[wheel] = current;

    const double millimetresPerCount =
        (2.0 * localization_config::PI * localization_config::WHEEL_RADIUS_MM) /
        (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV * localization_config::MOTOR_TO_WHEEL_RATIO);
    wheelSpeed[wheel] = deltaCounts * millimetresPerCount / dt;
  }

  // For equally spaced omni wheels, each wheel rolls tangent to the circle
  // around the robot centre. These are the inverse kinematics of that layout.
  double kinematics[3][4];
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    const double alpha = DegreesToRadians(localization_config::WHEEL_ANGLE_DEG[wheel]);
    kinematics[wheel][0] = -std::sin(alpha);
    kinematics[wheel][1] =  std::cos(alpha);
    kinematics[wheel][2] = localization_config::WHEEL_CENTER_RADIUS_MM[wheel];
    kinematics[wheel][3] = wheelSpeed[wheel];
  }

  // Solve the 3x3 wheel model so measured per-wheel centre radii can differ.
  bool kinematicsValid = true;
  for (int column = 0; column < 3; ++column) {
    int pivot = column;
    for (int row = column + 1; row < 3; ++row) {
      if (std::fabs(kinematics[row][column]) >
          std::fabs(kinematics[pivot][column])) pivot = row;
    }
    if (std::fabs(kinematics[pivot][column]) < 1e-8) {
      kinematicsValid = false;
      break;
    }
    if (pivot != column) {
      for (int index = column; index < 4; ++index)
        std::swap(kinematics[column][index], kinematics[pivot][index]);
    }
    const double divisor = kinematics[column][column];
    for (int index = column; index < 4; ++index)
      kinematics[column][index] /= divisor;
    for (int row = 0; row < 3; ++row) {
      if (row == column) continue;
      const double scale = kinematics[row][column];
      for (int index = column; index < 4; ++index)
        kinematics[row][index] -= scale * kinematics[column][index];
    }
  }
  const double bodyVx = kinematicsValid ? kinematics[0][3] : 0.0;
  const double bodyVy = kinematicsValid ? kinematics[1][3] : 0.0;
  const double wheelYawRate = kinematicsValid ? kinematics[2][3] : 0.0;

  // Blend the VMX gyro with encoder-derived wheel rotation for the yaw
  // prediction. The AHRS yaw corrects drift; sonar wall geometry supplies an
  // independent heading update.
  lastYawRateDps_ = hardware_->GetYawRateDegreesPerSecond();
  lastRawGyroX_Dps_ = hardware_->GetRawGyroX_Dps();
  lastRawGyroY_Dps_ = hardware_->GetRawGyroY_Dps();
  lastRawGyroZ_Dps_ = hardware_->GetRawGyroZ_Dps();
  const double gyroRateRadps = DegreesToRadians(lastYawRateDps_);
  const double encoderYawWeight = localization_config::ENCODER_YAW_RATE_BLEND;
  const double yawRateRadps =
      (1.0 - encoderYawWeight) * gyroRateRadps +
      encoderYawWeight * wheelYawRate;
  const double gyroPrediction =
      WrapRadians(pose_.headingRad + yawRateRadps * dt);
  lastYawDegrees_ = hardware_->GetYawDegrees();
  const double ahrsYaw = WrapRadians(
      DegreesToRadians(lastYawDegrees_) + imuYawOffsetRad_);
  const double yawInnovation = WrapRadians(ahrsYaw - gyroPrediction);
  pose_.headingRad = WrapRadians(
      gyroPrediction + localization_config::YAW_AHRS_CORRECTION * yawInnovation);

  // Encoder velocity is the stable baseline. A small, damped acceleration
  // integration helps during short transients without allowing accelerometer
  // bias to dominate position. The navX reports world-linear acceleration in G.
  lastAccelX_G_ = hardware_->GetWorldAccelX_G();
  lastAccelY_G_ = hardware_->GetWorldAccelY_G();
  lastRawAccelX_G_ = hardware_->GetRawAccelX_G();
  lastRawAccelY_G_ = hardware_->GetRawAccelY_G();
  lastRawAccelZ_G_ = hardware_->GetRawAccelZ_G();
  double accelX = std::fabs(lastAccelX_G_) < localization_config::ACCEL_DEADBAND_G
                      ? 0.0 : lastAccelX_G_ * localization_config::GRAVITY_MM_PER_S2;
  double accelY = std::fabs(lastAccelY_G_) < localization_config::ACCEL_DEADBAND_G
                      ? 0.0 : lastAccelY_G_ * localization_config::GRAVITY_MM_PER_S2;
  const double offsetCos = std::cos(imuYawOffsetRad_);
  const double offsetSin = std::sin(imuYawOffsetRad_);
  const double arenaAccelX = offsetCos * accelX - offsetSin * accelY;
  const double arenaAccelY = offsetSin * accelX + offsetCos * accelY;
  const double damping = std::pow(kAccelDampingPerSecond, dt);
  const double predictedWorldVx =
      fusedWorldVelocityX_MMps_ * damping + arenaAccelX * dt;
  const double predictedWorldVy =
      fusedWorldVelocityY_MMps_ * damping + arenaAccelY * dt;

  const double c = std::cos(pose_.headingRad);
  const double s = std::sin(pose_.headingRad);
  const double encoderWorldVx = c * bodyVx - s * bodyVy;
  const double encoderWorldVy = s * bodyVx + c * bodyVy;
  const double accelWeight = localization_config::ACCEL_VELOCITY_BLEND;
  fusedWorldVelocityX_MMps_ = (1.0 - accelWeight) * encoderWorldVx +
                             accelWeight * predictedWorldVx;
  fusedWorldVelocityY_MMps_ = (1.0 - accelWeight) * encoderWorldVy +
                             accelWeight * predictedWorldVy;

  pose_.x_MM += fusedWorldVelocityX_MMps_ * dt;
  pose_.y_MM += fusedWorldVelocityY_MMps_ * dt;

  lastBodyVelocityX_MMps_ = bodyVx;
  lastBodyVelocityY_MMps_ = bodyVy;
  lastAngularVelocityRadps_ = wheelYawRate;

  covariance_[0][0] += kPositionProcessNoiseMM2PerSecond * dt;
  covariance_[1][1] += kPositionProcessNoiseMM2PerSecond * dt;
  covariance_[2][2] += kHeadingProcessNoiseRad2PerSecond * dt;

  // Keep a small non-zero yaw uncertainty after the AHRS correction. Range
  // updates can then refine heading using the known arena and sensor offsets.
  const double yawVariance = kYawMeasurementStdDevRad * kYawMeasurementStdDevRad;
  const double alpha = localization_config::YAW_AHRS_CORRECTION;
  covariance_[2][2] = (1.0 - alpha) * (1.0 - alpha) * covariance_[2][2] +
                      alpha * alpha * yawVariance;
}

double Localization::PredictSonarRange(int sensorIndex,
                                       const RobotPose& pose) const {
  if (sensorIndex < 0 || sensorIndex >= 4 ||
      localization_config::ARENA_WIDTH_MM <= 0.0 ||
      localization_config::ARENA_HEIGHT_MM <= 0.0) {
    return -1.0;
  }

  const localization_config::SonarConfig& sensor =
      localization_config::SONARS[sensorIndex];
  const double mountAngle = DegreesToRadians(sensor.beamAngleDeg);
  const double c = std::cos(pose.headingRad);
  const double s = std::sin(pose.headingRad);
  const double sensorX = pose.x_MM + c * sensor.mountX_MM - s * sensor.mountY_MM;
  const double sensorY = pose.y_MM + s * sensor.mountX_MM + c * sensor.mountY_MM;
  const double rayAngle = pose.headingRad + mountAngle;
  const double rayX = std::cos(rayAngle);
  const double rayY = std::sin(rayAngle);
  const double infinity = std::numeric_limits<double>::infinity();
  double nearest = infinity;

  if (rayX > 1e-8) {
    const double range = (localization_config::ARENA_WIDTH_MM - sensorX) / rayX;
    const double hitY = sensorY + range * rayY;
    if (range > 0.0 && hitY >= 0.0 &&
        hitY <= localization_config::ARENA_HEIGHT_MM)
      nearest = std::min(nearest, range);
  } else if (rayX < -1e-8) {
    const double range = (0.0 - sensorX) / rayX;
    const double hitY = sensorY + range * rayY;
    if (range > 0.0 && hitY >= 0.0 &&
        hitY <= localization_config::ARENA_HEIGHT_MM)
      nearest = std::min(nearest, range);
  }

  if (rayY > 1e-8) {
    const double range = (localization_config::ARENA_HEIGHT_MM - sensorY) / rayY;
    const double hitX = sensorX + range * rayX;
    if (range > 0.0 && hitX >= 0.0 &&
        hitX <= localization_config::ARENA_WIDTH_MM)
      nearest = std::min(nearest, range);
  } else if (rayY < -1e-8) {
    const double range = (0.0 - sensorY) / rayY;
    const double hitX = sensorX + range * rayX;
    if (range > 0.0 && hitX >= 0.0 &&
        hitX <= localization_config::ARENA_WIDTH_MM)
      nearest = std::min(nearest, range);
  }

  return std::isfinite(nearest) ? nearest : -1.0;
}

void Localization::ApplyScalarCorrection(const double jacobian[3],
                                         double innovation,
                                         double variance) {
  double hp[3] = {0.0, 0.0, 0.0};
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row)
      hp[col] += jacobian[row] * covariance_[row][col];
  }

  double innovationVariance = variance;
  for (int index = 0; index < 3; ++index)
    innovationVariance += hp[index] * jacobian[index];
  if (innovationVariance <= 1e-12) return;

  double gain[3] = {0.0, 0.0, 0.0};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col)
      gain[row] += covariance_[row][col] * jacobian[col];
    gain[row] /= innovationVariance;
  }

  pose_.x_MM += gain[0] * innovation;
  pose_.y_MM += gain[1] * innovation;
  pose_.headingRad = WrapRadians(pose_.headingRad + gain[2] * innovation);

  double updated[3][3];
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col)
      updated[row][col] = covariance_[row][col] - gain[row] * hp[col];
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col)
      covariance_[row][col] = 0.5 * (updated[row][col] + updated[col][row]);
  }
}

bool Localization::CorrectFromSonar(int sensorIndex, double measuredRange_MM) {
  if (sensorIndex < 0 || sensorIndex >= 4 ||
      measuredRange_MM < localization_config::SONAR_MIN_RANGE_MM ||
      measuredRange_MM > localization_config::SONAR_MAX_RANGE_MM) {
    return false;
  }

  const double predicted = PredictSonarRange(sensorIndex, pose_);
  if (predicted < 0.0 ||
      predicted > localization_config::SONAR_MAX_RANGE_MM) {
    return false;
  }
  const double innovation = measuredRange_MM - predicted;
  if (std::fabs(innovation) >
      localization_config::SONAR_MAX_INNOVATION_MM) {
    return false;
  }

  const double xPlus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_MM + kJacobianPositionStepMM,
                             pose_.y_MM, pose_.headingRad});
  const double xMinus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_MM - kJacobianPositionStepMM,
                             pose_.y_MM, pose_.headingRad});
  const double yPlus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_MM,
                             pose_.y_MM + kJacobianPositionStepMM,
                             pose_.headingRad});
  const double yMinus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_MM,
                             pose_.y_MM - kJacobianPositionStepMM,
                             pose_.headingRad});
  const double thetaPlus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_MM, pose_.y_MM,
                             pose_.headingRad + kJacobianHeadingStepRad});
  const double thetaMinus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_MM, pose_.y_MM,
                             pose_.headingRad - kJacobianHeadingStepRad});

  if (xPlus < 0.0 || xMinus < 0.0 || yPlus < 0.0 || yMinus < 0.0 ||
      thetaPlus < 0.0 || thetaMinus < 0.0) {
    return false;
  }

  const double jacobian[3] = {
      (xPlus - xMinus) / (2.0 * kJacobianPositionStepMM),
      (yPlus - yMinus) / (2.0 * kJacobianPositionStepMM),
      (thetaPlus - thetaMinus) /
          (2.0 * kJacobianHeadingStepRad)};
  const double variance = localization_config::SONAR_RANGE_STDDEV_MM *
                          localization_config::SONAR_RANGE_STDDEV_MM;
  ApplyScalarCorrection(jacobian, innovation, variance);
  return true;
}

void Localization::PublishTelemetry(double sonarRange_MM,
                                    int sonarIndex) const {
  if (!localization_config::ENABLE_DETAILED_TELEMETRY) return;

  frc::SmartDashboard::PutNumber("Localization/Pose/x_mm", pose_.x_MM);
  frc::SmartDashboard::PutNumber("Localization/Pose/y_mm", pose_.y_MM);
  frc::SmartDashboard::PutNumber(
      "Localization/Pose/heading_deg",
      pose_.headingRad * 180.0 / localization_config::PI);
  frc::SmartDashboard::PutNumber("Localization/Velocity/body_vx_mmps",
                                  lastBodyVelocityX_MMps_);
  frc::SmartDashboard::PutNumber("Localization/Velocity/body_vy_mmps",
                                  lastBodyVelocityY_MMps_);
  frc::SmartDashboard::PutNumber(
      "Localization/Velocity/wheel_yaw_rate_radps",
      lastAngularVelocityRadps_);
  frc::SmartDashboard::PutNumber("Localization/Sonar/last_range_mm",
                                  sonarRange_MM);
  frc::SmartDashboard::PutNumber("Localization/Sonar/last_sensor_index",
                                  sonarIndex);
  frc::SmartDashboard::PutNumber("Localization/Uncertainty/sigma_x_mm",
                                 std::sqrt(std::max(0.0, covariance_[0][0])));
  frc::SmartDashboard::PutNumber("Localization/Uncertainty/sigma_y_mm",
                                 std::sqrt(std::max(0.0, covariance_[1][1])));
  frc::SmartDashboard::PutNumber(
      "Localization/Uncertainty/sigma_heading_deg",
                                 std::sqrt(std::max(0.0, covariance_[2][2])) *
                                     180.0 / localization_config::PI);
}

double Localization::WrapRadians(double angleRad) {
  while (angleRad > localization_config::PI) angleRad -= 2.0 * localization_config::PI;
  while (angleRad < -localization_config::PI) angleRad += 2.0 * localization_config::PI;
  return angleRad;
}

double Localization::DegreesToRadians(double angleDeg) {
  return angleDeg * localization_config::PI / 180.0;
}
