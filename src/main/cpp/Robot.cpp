/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace {
// Remove only this program's former diagnostic trees. Saved dashboard widgets
// are client-side; the new Robot tab provides a fresh, fixed layout.
void ClearOldTelemetry(nt::NetworkTable& table) {
  for (const auto& name : table.GetSubTables()) {
    ClearOldTelemetry(*table.GetSubTable(name));
  }
  for (const auto& key : table.GetKeys()) table.Delete(key);
}
}  // namespace

void Robot::RobotInit() {
  if (localization_config::TELEOP_DRIVE_ENABLED ||
      localization_config::FORWARD_TEST_ENABLED ||
      !localization_config::ENABLE_DETAILED_TELEMETRY) {
    frc::LiveWindow::GetInstance()->DisableAllTelemetry();
    auto dashboard = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
    ClearOldTelemetry(*dashboard->GetSubTable("Diagnostics"));
    ClearOldTelemetry(*dashboard->GetSubTable("Localization"));
  }
  frc::Shuffleboard::Update();
  frc::Shuffleboard::SelectTab(
      localization_config::TELEOP_DRIVE_ENABLED ? "Teleop" :
      (localization_config::FORWARD_TEST_ENABLED ? "Forward 1m" : "Robot"));
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
