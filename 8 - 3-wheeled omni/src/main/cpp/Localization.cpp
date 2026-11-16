#include "RobotControl.h"

Localization::Localization(Hardware* hardware):hardware_(hardware) {
  Reset(localization_config::START_X_MM,localization_config::START_Y_MM,localization_config::START_HEADING_DEG);
}
void Localization::Reset(double x_MM,double y_MM,double headingDeg) {
  double counts[3];
  for (int i=0;i<3;++i) counts[i]=hardware_->GetWheelEncoderRaw(i);
  odometry_.Reset({x_MM,y_MM,robot_geometry::Radians(headingDeg)},counts,hardware_->GetYawDegrees());
  teleopTelemetry_.Reset({x_MM,y_MM,robot_geometry::Radians(headingDeg)},counts,hardware_->GetYawDegrees(),
      hardware_->GetWorldAccelX_G(),hardware_->GetWorldAccelY_G());
}
void Localization::SetAutonomousMode(bool autonomous) {
  if (autonomous==autonomousMode_) return;
  const auto pose=GetPose();
  autonomousMode_=autonomous;
  Reset(pose.x_MM,pose.y_MM,pose.headingRad*180/robot_geometry::PI);
}
void Localization::UpdateOdometry(double dtSeconds) {
  double counts[3];
  for (int i=0;i<3;++i) counts[i]=hardware_->GetWheelEncoderRaw(i);
  if (autonomousMode_)
    odometry_.Update(counts,hardware_->GetYawDegrees(),hardware_->GetYawRateDegreesPerSecond(),dtSeconds);
  else teleopTelemetry_.Update(counts,hardware_->GetYawDegrees(),hardware_->GetYawRateDegreesPerSecond(),
      hardware_->GetWorldAccelX_G(),hardware_->GetWorldAccelY_G(),dtSeconds);
}
