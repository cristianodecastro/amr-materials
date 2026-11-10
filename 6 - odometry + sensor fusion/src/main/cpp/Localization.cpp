#include "Localization.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kAccelDampingPerSecond = 0.98;
constexpr double kPositionProcessNoiseM2PerSecond = 0.0004;
constexpr double kHeadingProcessNoiseRad2PerSecond = 0.0008;
constexpr double kYawMeasurementStdDevRad = 0.035;
constexpr double kJacobianPositionStepM = 0.001;
constexpr double kJacobianHeadingStepRad = 0.5 * config::PI / 180.0;

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}
}  // namespace

Localization::Localization(Hardware* hardware)
    : hardware_(hardware),
      pose_{config::START_X_M, config::START_Y_M,
            DegreesToRadians(config::START_HEADING_DEG)},
      fusedWorldVelocityX_Mps_(0.0),
      fusedWorldVelocityY_Mps_(0.0),
      imuYawOffsetRad_(0.0),
      lastBodyVelocityX_Mps_(0.0),
      lastBodyVelocityY_Mps_(0.0),
      lastAngularVelocityRadps_(0.0),
      lastAccelX_G_(0.0),
      lastAccelY_G_(0.0),
      lastRawAccelX_G_(0.0),
      lastRawAccelY_G_(0.0),
      lastRawAccelZ_G_(0.0),
      lastRawGyroX_Dps_(0.0),
      lastRawGyroY_Dps_(0.0),
      lastRawGyroZ_Dps_(0.0),
      lastYawRateDps_(0.0) {
  Reset(config::START_X_M, config::START_Y_M, config::START_HEADING_DEG);
}

void Localization::Reset(double x_M, double y_M, double headingDeg) {
  pose_.x_M = x_M;
  pose_.y_M = y_M;
  pose_.headingRad = WrapRadians(DegreesToRadians(headingDeg));
  imuYawOffsetRad_ = WrapRadians(
      pose_.headingRad - DegreesToRadians(hardware_->GetYawDegrees()));

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) covariance_[row][col] = 0.0;
  }
  covariance_[0][0] = 0.25;
  covariance_[1][1] = 0.25;
  covariance_[2][2] = 0.12;

  for (int wheel = 0; wheel < config::WHEEL_COUNT; ++wheel) {
    previousEncoder_[wheel] = hardware_->GetWheelEncoderRaw(wheel);
  }
  fusedWorldVelocityX_Mps_ = 0.0;
  fusedWorldVelocityY_Mps_ = 0.0;
  lastBodyVelocityX_Mps_ = 0.0;
  lastBodyVelocityY_Mps_ = 0.0;
  lastAngularVelocityRadps_ = 0.0;
}

void Localization::UpdateOdometry(double dtSeconds) {
  if (dtSeconds <= 0.0) return;
  const double dt = Clamp(dtSeconds, 0.001, 0.10);

  double wheelSpeed[config::WHEEL_COUNT] = {0.0, 0.0, 0.0};
  for (int wheel = 0; wheel < config::WHEEL_COUNT; ++wheel) {
    const double current = hardware_->GetWheelEncoderRaw(wheel);
    const double deltaCounts = (current - previousEncoder_[wheel]) *
                               config::WHEEL_ENCODER_SIGN[wheel];
    previousEncoder_[wheel] = current;

    const double metresPerCount =
        (2.0 * config::PI * config::WHEEL_RADIUS_M) /
        (config::ENCODER_COUNTS_PER_OUTPUT_REV * config::MOTOR_TO_WHEEL_RATIO);
    wheelSpeed[wheel] = deltaCounts * metresPerCount / dt;
  }

  // For equally spaced omni wheels, each wheel rolls tangent to the circle
  // around the robot centre. These are the inverse kinematics of that layout.
  double kinematics[3][4];
  for (int wheel = 0; wheel < config::WHEEL_COUNT; ++wheel) {
    const double alpha = DegreesToRadians(config::WHEEL_ANGLE_DEG[wheel]);
    kinematics[wheel][0] = -std::sin(alpha);
    kinematics[wheel][1] =  std::cos(alpha);
    kinematics[wheel][2] = config::WHEEL_CENTER_RADIUS_M[wheel];
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

  // The VMX AHRS rate predicts yaw; its fused yaw corrects integrated gyro
  // drift. Sonar wall geometry supplies a second, independent heading update.
  lastYawRateDps_ = hardware_->GetYawRateDegreesPerSecond();
  lastRawGyroX_Dps_ = hardware_->GetRawGyroX_Dps();
  lastRawGyroY_Dps_ = hardware_->GetRawGyroY_Dps();
  lastRawGyroZ_Dps_ = hardware_->GetRawGyroZ_Dps();
  const double gyroPrediction = WrapRadians(
      pose_.headingRad + DegreesToRadians(lastYawRateDps_) * dt);
  const double ahrsYaw = WrapRadians(
      DegreesToRadians(hardware_->GetYawDegrees()) + imuYawOffsetRad_);
  const double yawInnovation = WrapRadians(ahrsYaw - gyroPrediction);
  pose_.headingRad = WrapRadians(
      gyroPrediction + config::YAW_AHRS_CORRECTION * yawInnovation);

  // Encoder velocity is the stable baseline. A small, damped acceleration
  // integration helps during short transients without allowing accelerometer
  // bias to dominate position. The navX reports world-linear acceleration in G.
  lastAccelX_G_ = hardware_->GetWorldAccelX_G();
  lastAccelY_G_ = hardware_->GetWorldAccelY_G();
  lastRawAccelX_G_ = hardware_->GetRawAccelX_G();
  lastRawAccelY_G_ = hardware_->GetRawAccelY_G();
  lastRawAccelZ_G_ = hardware_->GetRawAccelZ_G();
  double accelX = std::fabs(lastAccelX_G_) < config::ACCEL_DEADBAND_G
                      ? 0.0 : lastAccelX_G_ * config::GRAVITY_M_PER_S2;
  double accelY = std::fabs(lastAccelY_G_) < config::ACCEL_DEADBAND_G
                      ? 0.0 : lastAccelY_G_ * config::GRAVITY_M_PER_S2;
  const double offsetCos = std::cos(imuYawOffsetRad_);
  const double offsetSin = std::sin(imuYawOffsetRad_);
  const double arenaAccelX = offsetCos * accelX - offsetSin * accelY;
  const double arenaAccelY = offsetSin * accelX + offsetCos * accelY;
  const double damping = std::pow(kAccelDampingPerSecond, dt);
  const double predictedWorldVx =
      fusedWorldVelocityX_Mps_ * damping + arenaAccelX * dt;
  const double predictedWorldVy =
      fusedWorldVelocityY_Mps_ * damping + arenaAccelY * dt;

  const double c = std::cos(pose_.headingRad);
  const double s = std::sin(pose_.headingRad);
  const double encoderWorldVx = c * bodyVx - s * bodyVy;
  const double encoderWorldVy = s * bodyVx + c * bodyVy;
  const double accelWeight = config::ACCEL_VELOCITY_BLEND;
  fusedWorldVelocityX_Mps_ = (1.0 - accelWeight) * encoderWorldVx +
                             accelWeight * predictedWorldVx;
  fusedWorldVelocityY_Mps_ = (1.0 - accelWeight) * encoderWorldVy +
                             accelWeight * predictedWorldVy;

  pose_.x_M += fusedWorldVelocityX_Mps_ * dt;
  pose_.y_M += fusedWorldVelocityY_Mps_ * dt;

  lastBodyVelocityX_Mps_ = bodyVx;
  lastBodyVelocityY_Mps_ = bodyVy;
  lastAngularVelocityRadps_ = wheelYawRate;

  covariance_[0][0] += kPositionProcessNoiseM2PerSecond * dt;
  covariance_[1][1] += kPositionProcessNoiseM2PerSecond * dt;
  covariance_[2][2] += kHeadingProcessNoiseRad2PerSecond * dt;

  // Keep a small non-zero yaw uncertainty after the AHRS correction. Range
  // updates can then refine heading using the known arena and sensor offsets.
  const double yawVariance = kYawMeasurementStdDevRad * kYawMeasurementStdDevRad;
  const double alpha = config::YAW_AHRS_CORRECTION;
  covariance_[2][2] = (1.0 - alpha) * (1.0 - alpha) * covariance_[2][2] +
                      alpha * alpha * yawVariance;
}

double Localization::PredictSonarRange(int sensorIndex,
                                       const RobotPose& pose) const {
  if (sensorIndex < 0 || sensorIndex >= 4 ||
      config::ARENA_WIDTH_M <= 0.0 || config::ARENA_HEIGHT_M <= 0.0) {
    return -1.0;
  }

  const config::SonarConfig& sensor = config::SONARS[sensorIndex];
  const double mountAngle = DegreesToRadians(sensor.beamAngleDeg);
  const double c = std::cos(pose.headingRad);
  const double s = std::sin(pose.headingRad);
  const double sensorX = pose.x_M + c * sensor.mountX_M - s * sensor.mountY_M;
  const double sensorY = pose.y_M + s * sensor.mountX_M + c * sensor.mountY_M;
  const double rayAngle = pose.headingRad + mountAngle;
  const double rayX = std::cos(rayAngle);
  const double rayY = std::sin(rayAngle);
  const double infinity = std::numeric_limits<double>::infinity();
  double nearest = infinity;

  if (rayX > 1e-8) {
    const double range = (config::ARENA_WIDTH_M - sensorX) / rayX;
    const double hitY = sensorY + range * rayY;
    if (range > 0.0 && hitY >= 0.0 && hitY <= config::ARENA_HEIGHT_M)
      nearest = std::min(nearest, range);
  } else if (rayX < -1e-8) {
    const double range = (0.0 - sensorX) / rayX;
    const double hitY = sensorY + range * rayY;
    if (range > 0.0 && hitY >= 0.0 && hitY <= config::ARENA_HEIGHT_M)
      nearest = std::min(nearest, range);
  }

  if (rayY > 1e-8) {
    const double range = (config::ARENA_HEIGHT_M - sensorY) / rayY;
    const double hitX = sensorX + range * rayX;
    if (range > 0.0 && hitX >= 0.0 && hitX <= config::ARENA_WIDTH_M)
      nearest = std::min(nearest, range);
  } else if (rayY < -1e-8) {
    const double range = (0.0 - sensorY) / rayY;
    const double hitX = sensorX + range * rayX;
    if (range > 0.0 && hitX >= 0.0 && hitX <= config::ARENA_WIDTH_M)
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

  pose_.x_M += gain[0] * innovation;
  pose_.y_M += gain[1] * innovation;
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

bool Localization::CorrectFromSonar(int sensorIndex, double measuredRange_M) {
  if (sensorIndex < 0 || sensorIndex >= 4 ||
      measuredRange_M < config::SONAR_MIN_RANGE_M ||
      measuredRange_M > config::SONAR_MAX_RANGE_M) {
    return false;
  }

  const double predicted = PredictSonarRange(sensorIndex, pose_);
  if (predicted < 0.0 || predicted > config::SONAR_MAX_RANGE_M) return false;
  const double innovation = measuredRange_M - predicted;
  if (std::fabs(innovation) > config::SONAR_MAX_INNOVATION_M) return false;

  const double xPlus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_M + kJacobianPositionStepM,
                             pose_.y_M, pose_.headingRad});
  const double xMinus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_M - kJacobianPositionStepM,
                             pose_.y_M, pose_.headingRad});
  const double yPlus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_M,
                             pose_.y_M + kJacobianPositionStepM,
                             pose_.headingRad});
  const double yMinus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_M,
                             pose_.y_M - kJacobianPositionStepM,
                             pose_.headingRad});
  const double thetaPlus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_M, pose_.y_M,
                             pose_.headingRad + kJacobianHeadingStepRad});
  const double thetaMinus = PredictSonarRange(
      sensorIndex, RobotPose{pose_.x_M, pose_.y_M,
                             pose_.headingRad - kJacobianHeadingStepRad});

  if (xPlus < 0.0 || xMinus < 0.0 || yPlus < 0.0 || yMinus < 0.0 ||
      thetaPlus < 0.0 || thetaMinus < 0.0) {
    return false;
  }

  const double jacobian[3] = {
      (xPlus - xMinus) / (2.0 * kJacobianPositionStepM),
      (yPlus - yMinus) / (2.0 * kJacobianPositionStepM),
      WrapRadians(thetaPlus - thetaMinus) /
          (2.0 * kJacobianHeadingStepRad)};
  const double variance = config::SONAR_RANGE_STDDEV_M *
                          config::SONAR_RANGE_STDDEV_M;
  ApplyScalarCorrection(jacobian, innovation, variance);
  return true;
}

void Localization::PublishTelemetry(double sonarRange_M,
                                    int sonarIndex) const {
  frc::SmartDashboard::PutNumber("pose_x_m", pose_.x_M);
  frc::SmartDashboard::PutNumber("pose_y_m", pose_.y_M);
  frc::SmartDashboard::PutNumber("pose_heading_deg",
                                 pose_.headingRad * 180.0 / config::PI);
  frc::SmartDashboard::PutNumber("odom_body_vx_mps", lastBodyVelocityX_Mps_);
  frc::SmartDashboard::PutNumber("odom_body_vy_mps", lastBodyVelocityY_Mps_);
  frc::SmartDashboard::PutNumber("odom_wheel_yaw_rate_radps",
                                 lastAngularVelocityRadps_);
  frc::SmartDashboard::PutNumber("imu_yaw_rate_dps", lastYawRateDps_);
  frc::SmartDashboard::PutNumber("imu_world_accel_x_g", lastAccelX_G_);
  frc::SmartDashboard::PutNumber("imu_world_accel_y_g", lastAccelY_G_);
  frc::SmartDashboard::PutNumber("imu_raw_accel_x_g", lastRawAccelX_G_);
  frc::SmartDashboard::PutNumber("imu_raw_accel_y_g", lastRawAccelY_G_);
  frc::SmartDashboard::PutNumber("imu_raw_accel_z_g", lastRawAccelZ_G_);
  frc::SmartDashboard::PutNumber("imu_raw_gyro_x_dps", lastRawGyroX_Dps_);
  frc::SmartDashboard::PutNumber("imu_raw_gyro_y_dps", lastRawGyroY_Dps_);
  frc::SmartDashboard::PutNumber("imu_raw_gyro_z_dps", lastRawGyroZ_Dps_);
  frc::SmartDashboard::PutNumber("sonar_last_index", sonarIndex);
  frc::SmartDashboard::PutNumber("sonar_last_range_m", sonarRange_M);
  frc::SmartDashboard::PutNumber("pose_sigma_x_m",
                                 std::sqrt(std::max(0.0, covariance_[0][0])));
  frc::SmartDashboard::PutNumber("pose_sigma_y_m",
                                 std::sqrt(std::max(0.0, covariance_[1][1])));
  frc::SmartDashboard::PutNumber("pose_sigma_heading_deg",
                                 std::sqrt(std::max(0.0, covariance_[2][2])) *
                                     180.0 / config::PI);
}

double Localization::WrapRadians(double angleRad) {
  while (angleRad > config::PI) angleRad -= 2.0 * config::PI;
  while (angleRad < -config::PI) angleRad += 2.0 * config::PI;
  return angleRad;
}

double Localization::DegreesToRadians(double angleDeg) {
  return angleDeg * config::PI / 180.0;
}
