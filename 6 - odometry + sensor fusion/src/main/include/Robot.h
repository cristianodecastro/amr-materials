#pragma once

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <studica/MockDS.h>

#include "Hardware.h"
#include "Localization.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

 private:
  void BeginLocalization();
  void UpdateSonarPoll(double nowSeconds);

  studica::MockDS mockDriverStation_;
  Hardware hardware_;
  Localization localization_{&hardware_};
  bool started_ = false;
  int startButtonDebounceCount_ = 0;
  int nextSonarIndex_ = 0;
  bool sonarAwaitingEcho_ = false;
  double sonarPingTime_ = 0.0;
  double lastSonarPingTime_ = 0.0;
  double lastLoopTime_ = 0.0;
  double lastSonarRange_M_ = -1.0;
  int lastSonarIndex_ = -1;
  double sonarRanges_M_[4] = {-1.0, -1.0, -1.0, -1.0};
  bool sonarRangeValid_[4] = {false, false, false, false};
  bool sonarCorrectionAccepted_[4] = {false, false, false, false};
};
