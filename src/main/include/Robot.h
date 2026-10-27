#pragma once
#include <frc/TimedRobot.h>
#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void AutonomousInit() override;
  void TeleopInit() override;
  void TestInit() override;
 private:
  RobotContainer container_;
};
