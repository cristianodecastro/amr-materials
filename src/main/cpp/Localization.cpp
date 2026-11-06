#include "RobotControl.h"
#include <frc/smartdashboard/SmartDashboard.h>

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
std::string Localization::GetStatus() const {
  if (!IsHealthy()) return "Invalid encoder/gyro sample or control timing";
  return autonomousMode_?"Autonomous: encoder increments + reset-relative gyro heading; position corrections off":
      "Teleop pose telemetry: legacy encoder/gyro/accelerometer blend; no steering correction";
}
void Localization::PublishTelemetry() const {
  if (!localization_config::ENABLE_DETAILED_TELEMETRY) return;
  const auto pose=GetPose();
  frc::SmartDashboard::PutNumber("Localization/Pose/x_mm",pose.x_MM);
  frc::SmartDashboard::PutNumber("Localization/Pose/y_mm",pose.y_MM);
  frc::SmartDashboard::PutNumber("Localization/Pose/heading_deg",pose.headingRad*180/robot_geometry::PI);
  frc::SmartDashboard::PutNumber("Localization/Velocity/body_vx_mmps",GetBodyVelocityX_MMps());
  frc::SmartDashboard::PutNumber("Localization/Velocity/body_vy_mmps",GetBodyVelocityY_MMps());
}
