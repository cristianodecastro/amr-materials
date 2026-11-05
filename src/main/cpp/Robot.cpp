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
  camera_.Start();
  if (!localization_config::ENABLE_DETAILED_TELEMETRY) {
    auto dashboard=nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
    ClearOldTelemetry(*dashboard->GetSubTable("Diagnostics"));
    ClearOldTelemetry(*dashboard->GetSubTable("Localization"));
  }
  frc::Shuffleboard::Update();
  frc::Shuffleboard::SelectTab("Teleop");
}
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  camera_.Periodic();
}
void Robot::DisabledInit() { subsystem_.StopAll(); }
void Robot::AutonomousInit() {
  subsystem_.EnterAutonomous();
  frc::Shuffleboard::SelectTab(subsystem_.HasSimpleTaskSelection()?"Simple tasks":"Autonomous");
}
void Robot::TeleopInit() {
  subsystem_.StopAll();
  frc::Shuffleboard::SelectTab("Teleop");
}
void Robot::TestInit() { subsystem_.StopAll(); }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
