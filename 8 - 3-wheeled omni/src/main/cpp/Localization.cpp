#include "RobotControl.h"

Localization::Localization(Hardware* hardware):hardware_(hardware) {
  Reset(localization_config::START_X_MM,localization_config::START_Y_MM,localization_config::START_HEADING_DEG);
}
void Localization::Reset(double x_MM,double y_MM,double headingDeg) {
  double counts[3];
  for (int i=0;i<3;++i) counts[i]=hardware_->GetWheelEncoderRaw(i);
  const double yaw=hardware_->GetYawDegrees();
  odometry_.Reset({x_MM,y_MM,robot_geometry::Radians(headingDeg)},counts,yaw);
  teleopTelemetry_.Reset({x_MM,y_MM,robot_geometry::Radians(headingDeg)},counts,yaw,
      hardware_->GetWorldAccelX_G(),hardware_->GetWorldAccelY_G());
  imuSynchronized_=hardware_->IsImuReady();
}
void Localization::SetAutonomousMode(bool autonomous) {
  if (autonomous==autonomousMode_) return;
  const auto pose=GetPose();
  autonomousMode_=autonomous;
  Reset(pose.x_MM,pose.y_MM,pose.headingRad*180/robot_geometry::PI);
}
void Localization::ApplyReference(RobotPose pose,bool resetHeading) {
  if (resetHeading) { Reset(pose.x_MM,pose.y_MM,pose.headingRad*180/robot_geometry::PI); return; }
  // Preserve yaw offset, encoder baselines and feedback timing for heading '-'.
  odometry_.CorrectPosition(pose.x_MM-odometry_.Pose().x_MM,pose.y_MM-odometry_.Pose().y_MM);
  teleopTelemetry_.SetPosition(pose.x_MM,pose.y_MM);
}
void Localization::UpdateOdometry(double dtSeconds) {
  if (!hardware_->IsImuReady()) { imuSynchronized_=false; return; }
  if (!imuSynchronized_) {
    // Bind the arena reference to a post-calibration sample, never constructor
    // zeros or the sensor's still-changing initial yaw-offset phase.
    const auto pose=GetPose(); Reset(pose.x_MM,pose.y_MM,pose.headingRad*180/robot_geometry::PI); return;
  }
  double counts[3];
  for (int i=0;i<3;++i) counts[i]=hardware_->GetWheelEncoderRaw(i);
  if (autonomousMode_)
    odometry_.Update(counts,hardware_->GetYawDegrees(),hardware_->GetYawRateDegreesPerSecond(),dtSeconds);
  else teleopTelemetry_.Update(counts,hardware_->GetYawDegrees(),hardware_->GetYawRateDegreesPerSecond(),
      hardware_->GetWorldAccelX_G(),hardware_->GetWorldAccelY_G(),dtSeconds);
}
