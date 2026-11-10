/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#include "Robot.h"

int main() { 

    // Start MockDS
    Robot r;
    r.ds.Enable();

    Hardware hard;
    Movement move( &hard );

    // Wait until the Start Button is pressed
    // while( hard.GetStartButton() ){ delay(150); }

    hard.SetRunningLED(true);

    move.SetPosition( 0, 0, 0 );            // Defines the Initial Position of the Robot
    // move.PositionDriver( 100,   100,   0 );
    // move.PositionDriver( 100,   100,  90 );
    // move.PositionDriver( 100, 100,  90 );
    // move.PositionDriver( 100, 100, 270 );
    // move.PositionDriver(   0, 100, 270 );
    // move.PositionDriver(   0, 100,   0 );
    // move.PositionDriver(   0,   0,   0 );

    move.sensor_drive( 15 );
    
    hard.SetRunningLED(false);

    return 0; 
}
