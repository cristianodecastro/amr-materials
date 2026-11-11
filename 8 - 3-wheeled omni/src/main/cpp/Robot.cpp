#include "Robot.h"
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace {
void ClearOldTelemetry(nt::NetworkTable& table) {
  for (const auto& name:table.GetSubTables()) ClearOldTelemetry(*table.GetSubTable(name));
  for (const auto& key:table.GetKeys()) table.Delete(key);
}
}
void Robot::RobotInit() {
  frc::LiveWindow::GetInstance()->DisableAllTelemetry();
  // Clear old published entries and tab metadata before building the three tabs.
  auto network=nt::NetworkTableInstance::GetDefault();
  ClearOldTelemetry(*network.GetTable("Shuffleboard"));
  ClearOldTelemetry(*network.GetTable("SmartDashboard"));
  subsystem_.InitializeDashboard();
  frc::Shuffleboard::Update();
  frc::Shuffleboard::SelectTab("Motion");
}
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}
void Robot::DisabledInit() { subsystem_.StopAll(); }
void Robot::AutonomousInit() {
  subsystem_.EnterAutonomous();
  frc::Shuffleboard::SelectTab("Motion");
}
void Robot::TeleopInit() {
  subsystem_.StopAll();
  frc::Shuffleboard::SelectTab("Motion");
}
void Robot::TestInit() { subsystem_.StopAll(); }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
