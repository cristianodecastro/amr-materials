#pragma once

#include "ManipulatorConfig.h"

// Independent press toggles, with release required after enable/reconnect.
// Targets survive disabling; disabling never commands a return movement.
class ServoTeleopController {
 public:
  constexpr void Update(bool allowed, bool inputsPresent, bool cross, bool triangle) {
    enabled_=allowed && inputsPresent;
    if (!enabled_) { Disarm(); return; }
    const bool pressed[2]={cross,triangle};
    for (int i=0;i<2;++i) {
      if (!pressed[i]) armed_[i]=true;
      if (armed_[i] && pressed[i] && !previous_[i]) active_[i]=!active_[i];
      previous_[i]=pressed[i];
    }
  }
  constexpr void Disarm() {
    enabled_=false;
    for (int i=0;i<2;++i) { armed_[i]=false; previous_[i]=false; }
  }
  constexpr bool OutputEnabled(int i) const {
    return i>=0 && i<2 && enabled_ && armed_[i];
  }
  constexpr double TargetDegrees(int i) const {
    return i>=0 && i<2 && active_[i]
        ? manipulator_config::SERVO_ACTIVE_DEG[i] : manipulator_config::SERVO_HOME_DEG;
  }
 private:
  bool active_[2]={false,false};
  bool armed_[2]={false,false};
  bool previous_[2]={false,false};
  bool enabled_=false;
};
