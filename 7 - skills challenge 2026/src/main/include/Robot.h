#pragma once
#include <frc/TimedRobot.h>
#include "RobotControl.h"
#include "CameraCapture.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void AutonomousInit() override;
  void TeleopInit() override;
  void TestInit() override;
 private:
  CameraCapture camera_;
  RobotSubsystem subsystem_{&camera_};
};
