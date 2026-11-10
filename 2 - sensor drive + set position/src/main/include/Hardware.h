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
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>

#include "frc/AnalogInput.h"
#include "frc/Ultrasonic.h"



#include "studica/TitanQuad.h"
#include "studica/TitanQuadEncoder.h"
#include "studica/Servo.h"
#include "studica/Cobra.h"

#include "Constants.h"

#include "AHRS.h"
#include <math.h>


class Hardware
{
    public:
        Hardware();
        double GetLeftEncoder(void);
        double GetBackEncoder(void);
        double GetRightEncoder(void);
        double GetElevatorEncoder(void);
        double GetYaw(void);
        double GetAngle(void);
        double GetCobra( int channel );
        void ResetYaw(void);
        void ResetEncoders(void);
        void Elevation( double servo, double elevator );
        void SetRunningLED(bool on);
        void SetStoppedLED(bool on);
        void SetLeft( double pwm );
        void SetRight( double pwm );
        void SetBack( double pwm );
        
        bool GetStopButton();
        bool GetStartButton();

        double getRightDistance( );
        double getLeftDistance( );
        double getFrontDistance( );

    private:
        studica::TitanQuad LeftMotor     {constant::TITAN_ID, 15600, constant::LEFT_MOTOR    };
        studica::TitanQuad BackMotor     {constant::TITAN_ID, 15600, constant::BACK_MOTOR    };
        studica::TitanQuad RightMotor    {constant::TITAN_ID, 15600, constant::RIGHT_MOTOR   };
        studica::TitanQuad ElevatorMotor {constant::TITAN_ID, 15600, constant::ELEVATOR_MOTOR};

        studica::TitanQuadEncoder LeftEncoder     {LeftMotor,     constant::LEFT_MOTOR,     constant::DIST_PER_TICK};
        studica::TitanQuadEncoder BackEncoder     {BackMotor,     constant::BACK_MOTOR,     constant::DIST_PER_TICK};
        studica::TitanQuadEncoder RightEncoder    {RightMotor,    constant::RIGHT_MOTOR,    constant::DIST_PER_TICK};
        studica::TitanQuadEncoder ElevatorEncoder {ElevatorMotor, constant::ELEVATOR_MOTOR, constant::DIST_PER_TICK};

        studica::Servo servo{9};

        AHRS navX{frc::SPI::Port::kMXP};

        frc::DigitalInput startButton{constant::START_BUTTON};
        frc::DigitalInput  stopButton{constant::STOP_BUTTON};

        frc::AnalogInput right_sharp{2};
        frc::AnalogInput left_sharp{3};


        frc::Ultrasonic us_r{0, 1};
        frc::Ultrasonic us_l{2, 3};



        frc::DigitalOutput runningLED{constant::RUNNING_LED};
        frc::DigitalOutput stoppedLED{constant::STOPPED_LED};

        studica::Cobra cobra{};

};

