/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#pragma once

#include <string>
#include <iostream>

#include <frc/TimedRobot.h>
#include <studica/MockDS.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Hardware.h"
#include "Movement.h"
#include "Oms.h"

class Robot : public frc::TimedRobot {
 public:
  studica::MockDS ds;
};


bool STOP = false;
void robot_interface();
