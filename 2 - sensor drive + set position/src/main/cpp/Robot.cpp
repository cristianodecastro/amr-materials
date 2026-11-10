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
    while( hard.GetStartButton() ){ delay(150); }

    hard.SetRunningLED(true);

    move.SetPosition( 30, 30, 270 );            // Defines the Initial Position of the Robot

    move.PositionDriver( 100,  30, 270 );
    move.PositionDriver( 100, 100, 270 );
    move.PositionDriver( 50,  150, 270 );
    move.PositionDriver( 50,  150, 90  );
    move.PositionDriver( 25,  170, 90  );     // 30, 160


    double robot_x = hard.getLeftDistance() + 10;        // 30
    double robot_y = 200 - hard.getFrontDistance() - 15; // 160   

    move.SetPosition( robot_x, robot_y, move.get_th() );



    // move.sensor_drive( 15, "right" );
    move.line_align( "right" );

    
    hard.SetRunningLED(false);

    return 0; 
}
