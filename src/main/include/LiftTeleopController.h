#pragma once

#include <cmath>
#include "ManipulatorConfig.h"

// Hold-to-run manual lift. Release after enable/reconnect/limit before moving.
class LiftTeleopController {
 public:
  constexpr void Update(bool allowed, bool inputsPresent, bool up, bool down,
              bool upperLimit, bool lowerLimit) {
    output_=0;
    if (!allowed || !inputsPresent) {
      armed_=false;
      status_=!allowed?"Stopped - enabled Teleop and neutral controls required":"D-pad POV missing";
      return;
    }
    if (!(manipulator_config::LIFT_UP_SIGN==1 || manipulator_config::LIFT_UP_SIGN==-1) ||
        !(manipulator_config::LIFT_OUTPUT>0 && manipulator_config::LIFT_OUTPUT<=1)) {
      armed_=false; status_="Invalid lift configuration"; return;
    }
    if (!armed_) {
      if (up || down) { status_="Release D-pad to arm lift"; return; }
      armed_=true;
    }
    if (up && down) { status_="Up + down cancelled"; return; }
    if ((up && upperLimit) || (down && lowerLimit)) {
      armed_=false; status_=up?"Upper limit - release D-pad":"Lower limit - release D-pad"; return;
    }
    output_=(up?1.0:(down?-1.0:0.0))*manipulator_config::LIFT_UP_SIGN*manipulator_config::LIFT_OUTPUT;
    status_=up?"Raising - hold D-pad up":(down?"Lowering - hold D-pad down":"Ready - D-pad up/down");
  }
  constexpr double Output() const { return output_; }
  constexpr const char* Status() const { return status_; }
 private:
  double output_=0;
  bool armed_=false;
  const char* status_="Stopped";
};
