#include "RobotControl.h"
#include "TeleopControl.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kPositionProcessNoiseMM2PerSecond = 400.0;
constexpr double kHeadingProcessNoiseRad2PerSecond = 0.0008;
constexpr double kYawMeasurementStdDevRad = 0.035;
constexpr double kJacobianPositionStepMM = 1.0;
constexpr double kJacobianHeadingStepRad = 0.5 * robot_geometry::PI / 180.0;

}  // namespace

Localization::Localization(Hardware* hardware)
    : hardware_(hardware),
      pose_{localization_config::START_X_MM, localization_config::START_Y_MM,
            robot_geometry::Radians(localization_config::START_HEADING_DEG)},
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
  pose_.headingRad = robot_geometry::Wrap(robot_geometry::Radians(headingDeg));
  lastYawDegrees_=hardware_->GetYawDegrees();
  lastYawRateDps_=hardware_->GetYawRateDegreesPerSecond();
  odometrySampleHealthy_=std::isfinite(lastYawDegrees_) && std::isfinite(lastYawRateDps_);
  imuYawOffsetRad_=odometrySampleHealthy_?robot_geometry::Wrap(
      pose_.headingRad-robot_geometry::Radians(lastYawDegrees_)):0.0;
  lastAccelX_G_=hardware_->GetWorldAccelX_G();
  lastAccelY_G_=hardware_->GetWorldAccelY_G();
  accelSampleValid_=std::isfinite(lastAccelX_G_) && std::isfinite(lastAccelY_G_) &&
      std::hypot(lastAccelX_G_,lastAccelY_G_)<=localization_config::ACCEL_MAX_PLAUSIBLE_G;
  // Start is pressed with the robot resting at SP. Capture the stationary
  // residual, then refine it only during subsequent confirmed zero motion.
  accelBiasX_G_=accelSampleValid_?lastAccelX_G_:0.0;
  accelBiasY_G_=accelSampleValid_?lastAccelY_G_:0.0;
  stationaryUpdate_=true;

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) covariance_[row][col] = 0.0;
  }
  // Assumes the robot was placed at the measured start pose, within 20 mm.
  // A single sonar return must never drag an unknown pose across the arena.
  covariance_[0][0] = 400.0;
  covariance_[1][1] = 400.0;
  covariance_[2][2] = 0.0012;

  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    previousEncoder_[wheel] = hardware_->GetWheelEncoderRaw(wheel);
    if (!std::isfinite(previousEncoder_[wheel])) {
      previousEncoder_[wheel]=0;
      odometrySampleHealthy_=false;
    }
  }
  fusedWorldVelocityX_MMps_ = 0.0;
  fusedWorldVelocityY_MMps_ = 0.0;
  lastBodyVelocityX_MMps_ = 0.0;
  lastBodyVelocityY_MMps_ = 0.0;
  lastAngularVelocityRadps_ = 0.0;
}

void Localization::UpdateOdometry(double dtSeconds) {
  odometrySampleHealthy_=false;
  if (!std::isfinite(dtSeconds) || dtSeconds<=0.0 ||
      dtSeconds>autonomous_config::MAX_CONTROL_GAP_S) return;
  const double dt=dtSeconds;
  const double oldHeading=pose_.headingRad;

  double wheelSpeed[localization_config::WHEEL_COUNT] = {0.0, 0.0, 0.0};
  double counts[localization_config::WHEEL_COUNT]={};
  double maximumWheelSpeed=0;
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    const double current = hardware_->GetWheelEncoderRaw(wheel);
    if (!std::isfinite(current)) return;
    counts[wheel]=current;
    const double deltaCounts = (current - previousEncoder_[wheel]) *
                               localization_config::WHEEL_ENCODER_SIGN[wheel];

    const double millimetresPerCount =
        (2.0 * robot_geometry::PI * localization_config::WHEEL_RADIUS_MM) /
        (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV * localization_config::MOTOR_TO_WHEEL_RATIO);
    wheelSpeed[wheel] = deltaCounts * millimetresPerCount / dt;
    if (!std::isfinite(wheelSpeed[wheel]) ||
        std::fabs(wheelSpeed[wheel])>3*localization_config::MAX_WHEEL_SPEED_MMPS) return;
    maximumWheelSpeed=std::max(maximumWheelSpeed,std::fabs(wheelSpeed[wheel]));
  }
  for (int wheel=0;wheel<localization_config::WHEEL_COUNT;++wheel) previousEncoder_[wheel]=counts[wheel];

  // For equally spaced omni wheels, each wheel rolls tangent to the circle
  // around the robot centre. These are the inverse kinematics of that layout.
  double kinematics[3][4];
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    const double alpha = robot_geometry::Radians(localization_config::WHEEL_ANGLE_DEG[wheel]);
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
  const double bodyVx = kinematicsValid ? drive_kinematics::BodyForward(kinematics[0][3],kinematics[1][3]) : 0.0;
  const double bodyVy = kinematicsValid ? drive_kinematics::BodyLeft(kinematics[0][3],kinematics[1][3]) : 0.0;
  const double wheelYawRate = kinematicsValid ? kinematics[2][3] : 0.0;
  if (!kinematicsValid) return;

  // Gyro rate predicts heading; AHRS yaw corrects it. The optional encoder-yaw
  // weight defaults to zero so slipping wheels cannot rotate the estimate.
  // Valid stationary sonar geometry can supply an independent heading update.
  lastYawRateDps_ = hardware_->GetYawRateDegreesPerSecond();
  lastYawDegrees_ = hardware_->GetYawDegrees();
  if (!std::isfinite(lastYawRateDps_) || !std::isfinite(lastYawDegrees_)) return;
  lastRawGyroX_Dps_ = hardware_->GetRawGyroX_Dps();
  lastRawGyroY_Dps_ = hardware_->GetRawGyroY_Dps();
  lastRawGyroZ_Dps_ = hardware_->GetRawGyroZ_Dps();
  const double gyroRateRadps = robot_geometry::Radians(lastYawRateDps_);
  const double encoderYawWeight = localization_config::ENCODER_YAW_RATE_BLEND;
  const double yawRateRadps =
      (1.0 - encoderYawWeight) * gyroRateRadps +
      encoderYawWeight * wheelYawRate;
  const double gyroPrediction =
      robot_geometry::Wrap(pose_.headingRad + yawRateRadps * dt);
  const double ahrsYaw = robot_geometry::Wrap(
      robot_geometry::Radians(lastYawDegrees_) + imuYawOffsetRad_);
  const double yawInnovation = robot_geometry::Wrap(ahrsYaw - gyroPrediction);
  pose_.headingRad = robot_geometry::Wrap(
      gyroPrediction + localization_config::YAW_AHRS_CORRECTION * yawInnovation);

  // Acceleration assists short-term velocity prediction; encoders anchor it
  // on every update. No unbounded acceleration-only displacement integration.
  lastAccelX_G_ = hardware_->GetWorldAccelX_G();
  lastAccelY_G_ = hardware_->GetWorldAccelY_G();
  lastRawAccelX_G_ = hardware_->GetRawAccelX_G();
  lastRawAccelY_G_ = hardware_->GetRawAccelY_G();
  lastRawAccelZ_G_ = hardware_->GetRawAccelZ_G();
  accelSampleValid_=std::isfinite(lastAccelX_G_) && std::isfinite(lastAccelY_G_) &&
      std::hypot(lastAccelX_G_,lastAccelY_G_)<=localization_config::ACCEL_MAX_PLAUSIBLE_G;
  double residualX=accelSampleValid_?lastAccelX_G_-accelBiasX_G_:0;
  double residualY=accelSampleValid_?lastAccelY_G_-accelBiasY_G_:0;
  stationaryUpdate_=maximumWheelSpeed<=localization_config::ENCODER_STATIONARY_MMPS &&
      std::fabs(lastYawRateDps_)<=localization_config::GYRO_STATIONARY_DPS;
  // Use independent wheel/gyro evidence for zero velocity. Accelerometer bias
  // must not prevent stationary updates and create false position drift.
  if (stationaryUpdate_ && accelSampleValid_) {
    const double gain=dt/(localization_config::ACCEL_BIAS_TRACK_TAU_S+dt);
    accelBiasX_G_+=gain*residualX; accelBiasY_G_+=gain*residualY;
    residualX=lastAccelX_G_-accelBiasX_G_; residualY=lastAccelY_G_-accelBiasY_G_;
  }
  const double accelX=std::fabs(residualX)<localization_config::ACCEL_DEADBAND_G?0:
      residualX*localization_config::GRAVITY_MM_PER_S2;
  const double accelY=std::fabs(residualY)<localization_config::ACCEL_DEADBAND_G?0:
      residualY*localization_config::GRAVITY_MM_PER_S2;
  const double offsetCos = std::cos(imuYawOffsetRad_);
  const double offsetSin = std::sin(imuYawOffsetRad_);
  const double arenaAccelX = offsetCos * accelX - offsetSin * accelY;
  const double arenaAccelY = offsetSin * accelX + offsetCos * accelY;
  const double predictedWorldVx =
      fusedWorldVelocityX_MMps_ + arenaAccelX * dt;
  const double predictedWorldVy =
      fusedWorldVelocityY_MMps_ + arenaAccelY * dt;

  const double c = std::cos(pose_.headingRad);
  const double s = std::sin(pose_.headingRad);
  const double midpoint=oldHeading+0.5*robot_geometry::Wrap(pose_.headingRad-oldHeading);
  const double encoderWorldVx=std::cos(midpoint)*bodyVx-std::sin(midpoint)*bodyVy;
  const double encoderWorldVy=std::sin(midpoint)*bodyVx+std::cos(midpoint)*bodyVy;
  const double accelWeight=accelSampleValid_?std::pow(localization_config::ACCEL_VELOCITY_BLEND,
      dt/localization_config::CONTROL_PERIOD_S):0;
  double correctionX=accelWeight*(predictedWorldVx-encoderWorldVx);
  double correctionY=accelWeight*(predictedWorldVy-encoderWorldVy);
  const double correction=std::hypot(correctionX,correctionY);
  if (correction>localization_config::ACCEL_MAX_VELOCITY_CORRECTION_MMPS) {
    const double scale=localization_config::ACCEL_MAX_VELOCITY_CORRECTION_MMPS/correction;
    correctionX*=scale; correctionY*=scale;
  }
  fusedWorldVelocityX_MMps_=stationaryUpdate_?0:encoderWorldVx+correctionX;
  fusedWorldVelocityY_MMps_=stationaryUpdate_?0:encoderWorldVy+correctionY;

  pose_.x_MM += fusedWorldVelocityX_MMps_ * dt;
  pose_.y_MM += fusedWorldVelocityY_MMps_ * dt;

  lastBodyVelocityX_MMps_=c*fusedWorldVelocityX_MMps_+s*fusedWorldVelocityY_MMps_;
  lastBodyVelocityY_MMps_=-s*fusedWorldVelocityX_MMps_+c*fusedWorldVelocityY_MMps_;
  lastAngularVelocityRadps_=yawRateRadps;
  odometrySampleHealthy_=robot_geometry::Finite(pose_) && std::isfinite(fusedWorldVelocityX_MMps_) &&
      std::isfinite(fusedWorldVelocityY_MMps_);

  // Motion-dependent uncertainty: waiting still for Start must not exhaust the
  // same drift budget as driving. Noise parameters require arena calibration.
  const double motionScale=std::min(1.0,std::hypot(bodyVx,bodyVy)/250.0);
  const double positionNoise=2.0+kPositionProcessNoiseMM2PerSecond*motionScale;
  covariance_[0][0] += positionNoise * dt;
  covariance_[1][1] += positionNoise * dt;
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
  const auto hit=PredictSonarHit(sensorIndex,pose);
  return hit.trustedWall?hit.range:-1.0;
}

ArenaRayHit Localization::PredictSonarHit(int sensorIndex,const RobotPose& pose,double angleOffset) const {
  return arena_.SonarHit(sensorIndex,pose,angleOffset);
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
  const double headingCorrection=gain[2]*innovation;
  pose_.headingRad = robot_geometry::Wrap(pose_.headingRad + headingCorrection);
  // Retain external wall/tape heading corrections relative to AHRS; otherwise
  // the next yaw update would pull the estimate back to its old reference.
  imuYawOffsetRad_=robot_geometry::Wrap(imuYawOffsetRad_+headingCorrection);

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

bool Localization::CorrectFromSonar(int sensorIndex, double measuredRange_MM, int expectedSurface) {
  if (sensorIndex < 0 || sensorIndex >= 4 ||
      !std::isfinite(measuredRange_MM) ||
      measuredRange_MM < localization_config::SONAR_MIN_RANGE_MM ||
      measuredRange_MM > localization_config::SONAR_MAX_RANGE_MM) {
    return false;
  }

  const double predicted = PredictSonarRange(sensorIndex, pose_);
  if (predicted < 0.0 ||
      predicted > localization_config::SONAR_MAX_RANGE_MM) {
    return false;
  }
  const auto centerHit=PredictSonarHit(sensorIndex,pose_);
  // A known reference must hit the associated physical wall, never a different
  // divider/outer wall selected after pose drift or an unexpected heading.
  if (expectedSurface>=0 && (!centerHit.trustedWall || centerHit.surface!=expectedSurface)) return false;
  // Reject corners, occluders and broad-cone ambiguity. The sensor returns the
  // nearest echo within a cone, not necessarily the center-ray intersection.
  for (const double offset : {-autonomous_config::SONAR_CONE_HALF_ANGLE_RAD,
                               autonomous_config::SONAR_CONE_HALF_ANGLE_RAD}) {
    const auto edge=PredictSonarHit(sensorIndex,pose_,offset);
    if (!edge.trustedWall || edge.surface!=centerHit.surface ||
        std::fabs(edge.range-predicted)>autonomous_config::SONAR_CONE_RANGE_SPREAD_MM) return false;
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
  // Reject a changing surface in the finite-difference neighborhood.
  if (std::fabs(jacobian[0])>4 || std::fabs(jacobian[1])>4 ||
      std::fabs(jacobian[2])>5000) return false;
  const double variance = localization_config::SONAR_RANGE_STDDEV_MM *
                          localization_config::SONAR_RANGE_STDDEV_MM;
  // Bound the actual proposed update, rather than clipping after updating P.
  double ph[3]={};
  double innovationVariance=variance;
  for (int r=0;r<3;++r) for (int c=0;c<3;++c) ph[r]+=covariance_[r][c]*jacobian[c];
  for (int i=0;i<3;++i) innovationVariance+=jacobian[i]*ph[i];
  if (!std::isfinite(innovationVariance) || innovationVariance<=0 ||
      innovation*innovation>9*innovationVariance ||
      std::hypot(ph[0],ph[1])*std::fabs(innovation)/innovationVariance>autonomous_config::SONAR_MAX_POSE_STEP_MM ||
      std::fabs(ph[2]*innovation/innovationVariance)>0.035) return false;
  ApplyScalarCorrection(jacobian, innovation, variance);
  return true;
}

bool Localization::CorrectFromLine(const autonomous_config::Station& station,double tapeLeftError_MM) {
  using namespace autonomous_config;
  if (!std::isfinite(tapeLeftError_MM)) return false;
  const double h=robot_geometry::Radians(station.headingDeg),theta=pose_.headingRad;
  if (std::fabs(robot_geometry::Wrap(theta-h))>0.08 || std::fabs(tapeLeftError_MM)>30) return false;
  const double sensorLeft=COBRA_LEFT_MM+tapeLeftError_MM;
  const double sensorX=pose_.x_MM+std::cos(theta)*COBRA_FORWARD_MM-std::sin(theta)*sensorLeft;
  const double sensorY=pose_.y_MM+std::sin(theta)*COBRA_FORWARD_MM+std::cos(theta)*sensorLeft;
  const double innovation= -(-std::sin(h)*(sensorX-station.center.x)+std::cos(h)*(sensorY-station.center.y));
  if (std::fabs(innovation)>25) return false;
  const double derivativeX=-std::sin(theta)*COBRA_FORWARD_MM-std::cos(theta)*sensorLeft;
  const double derivativeY=std::cos(theta)*COBRA_FORWARD_MM-std::sin(theta)*sensorLeft;
  const double jacobian[3]={-std::sin(h),std::cos(h),-std::sin(h)*derivativeX+std::cos(h)*derivativeY};
  ApplyScalarCorrection(jacobian,innovation,64.0);
  return true;
}

double Localization::GetPositionSigmaMM() const {
  return std::sqrt(std::max(covariance_[0][0],covariance_[1][1]));
}
std::string Localization::GetFusionStatus() const {
  if (!odometrySampleHealthy_) return "Unhealthy gyro/encoder sample - autonomous stops";
  if (!accelSampleValid_) return "Encoders + gyro/AHRS; invalid acceleration ignored";
  if (stationaryUpdate_) return "Stationary: zero velocity; updating accelerometer bias";
  return "Encoders + gyro/AHRS + bounded accelerometer prediction";
}

bool Localization::IsHealthy(double maxPositionSigmaMM) const {
  return odometrySampleHealthy_ && robot_geometry::Finite(pose_) && std::isfinite(GetPositionSigmaMM()) &&
      GetPositionSigmaMM()<=maxPositionSigmaMM &&
      std::isfinite(covariance_[2][2]) && covariance_[2][2]>=0 &&
      std::sqrt(covariance_[2][2])<=autonomous_config::MAX_HEADING_SIGMA_RAD &&
      std::isfinite(lastBodyVelocityX_MMps_) && std::isfinite(lastBodyVelocityY_MMps_);
}

void Localization::PublishTelemetry() const {
  if (!localization_config::ENABLE_DETAILED_TELEMETRY) return;

  frc::SmartDashboard::PutNumber("Localization/Pose/x_mm", pose_.x_MM);
  frc::SmartDashboard::PutNumber("Localization/Pose/y_mm", pose_.y_MM);
  frc::SmartDashboard::PutNumber(
      "Localization/Pose/heading_deg",
      pose_.headingRad * 180.0 / robot_geometry::PI);
  frc::SmartDashboard::PutNumber("Localization/Velocity/body_vx_mmps",
                                  lastBodyVelocityX_MMps_);
  frc::SmartDashboard::PutNumber("Localization/Velocity/body_vy_mmps",
                                  lastBodyVelocityY_MMps_);
  frc::SmartDashboard::PutNumber(
      "Localization/Velocity/wheel_yaw_rate_radps",
      lastAngularVelocityRadps_);
  frc::SmartDashboard::PutNumber("Localization/Uncertainty/sigma_x_mm",
                                 std::sqrt(std::max(0.0, covariance_[0][0])));
  frc::SmartDashboard::PutNumber("Localization/Uncertainty/sigma_y_mm",
                                 std::sqrt(std::max(0.0, covariance_[1][1])));
  frc::SmartDashboard::PutNumber(
      "Localization/Uncertainty/sigma_heading_deg",
                                 std::sqrt(std::max(0.0, covariance_[2][2])) *
                                     180.0 / robot_geometry::PI);
}
