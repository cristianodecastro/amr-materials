/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

namespace constant
{
    //Motors
    static constexpr int TITAN_ID   = 42;
    static constexpr int LEFT_MOTOR = 3;    
    static constexpr int BACK_MOTOR = 2;    //BACK
    static constexpr int RIGHT_MOTOR = 1;
    static constexpr int ELEVATOR_MOTOR = 0;    //ELEVATOR

    //Encoder
    static constexpr double WHEEL_RADIUS    = 5.1; // Wheels Radius [cm]
    static constexpr double FRAME_RADIUS    = 22;  // Frame Radius [cm]
    static constexpr double PULSE_PER_REV   = 1464;
    static constexpr double GEAR_RATIO      = 1/1;
    static constexpr double ENCODER_PULSE_RATIO = PULSE_PER_REV * GEAR_RATIO;
    static constexpr double DIST_PER_TICK   =   (M_PI * 2 * WHEEL_RADIUS) / ENCODER_PULSE_RATIO;

    //Inputs
    static constexpr int START_BUTTON   = 11;
    static constexpr int STOP_BUTTON    = 10;

    //Outputs
    static constexpr int RUNNING_LED    = 13;
    static constexpr int STOPPED_LED    = 12;

    // Sharp
    static constexpr int RIGHT_SHARP    = 2;
    static constexpr int LEFT_SHARP     = 3;


}