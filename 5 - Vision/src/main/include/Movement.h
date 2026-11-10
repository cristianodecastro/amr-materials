/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "Functions.h"
#include "Hardware.h"

#include <string>

#include <cmath>

class Movement
{
    public:
        Movement( Hardware * h ) : hardware{h}{}

        void InverseKinematics(double x, double y, double z);
        void ForwardKinematics( double vl, double vr, double vb );
        void SetPosition( double x, double y, double th );
        
        void PositionDriver( double desired_x, double desired_y, double desired_th );

        double WheelSpeed( int encoder, double time );     // Returns the wheel speed based on the Encoder Ticks
        void ShuffleBoardUpdate();
        double get_x();
        double get_y();
        double get_th();

        void sensor_drive( double dist, std::string direction );
        void line_align( std::string direction );

        double desired_back_speed; 
        double desired_left_speed; 
        double desired_right_speed;

    private:

        Hardware * hardware;

        double x_global  = 0;   //Robot Global Position on the X  axis  [cm]
        double y_global  = 0;   //Robot Global Position on the Y  axis  [cm]
        double th_global = 0;   //Robot Global Position on the Th axis  [degrees]

        int current_enc_l ;
        int previous_enc_l;

        int current_enc_r ;
        int previous_enc_r;

        int current_enc_b ;
        int previous_enc_b;

        const double linear_tolerance  = 4.0;    // [cm]
        const double angular_tolerance = 3.0;    // [degrees]

        const double max_linear_speed = 30.0;    // cm/s
        const double max_ang_speed = 0.75;        // rad/s

        double leftVelocity;
        double rightVelocity;
        double backVelocity; 

        double desired_vx;
        double desired_vy;
        double desired_vth;

        double vx;
        double vy; 
        double vth;

        double offset_th;

};