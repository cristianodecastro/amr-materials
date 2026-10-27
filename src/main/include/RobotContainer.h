#pragma once
#include "subsystems/LocalizationSubsystem.h"

class RobotContainer {
 public:
  void EnterAutonomous() { localization_.EnterAutonomous(); }
  void StopAll() { localization_.StopAll(); }
 private:
  LocalizationSubsystem localization_;
};
