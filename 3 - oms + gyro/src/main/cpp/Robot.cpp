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
    Oms oms( &hard );

    delay(1000);

    // Wait until the Start Button is pressed
    while( hard.GetStartButton() ){ delay(150); }

    hard.SetRunningLED(true);
    

    move.SetPosition( 0, 0, 0 );            // Defines the Initial Position of the Robot
/* while(true){
    frc::SmartDashboard::PutBoolean("limitswitch",  hard.GetLimitLow() ); // limit switch checking

frc::SmartDashboard::PutBoolean("limitswitchhigh",  hard.GetLimitHigh());
} */
      

    // double robot_x = hard.getRightDistance() + 10; 
    // double robot_y = hard.getFrontDistance() + 15; 

    // move.SetPosition( robot_x, robot_y, move.get_th() );            


     //move.PositionDriver( 10,   0,   0 );
    // move.PositionDriver( 100,   0,  90 );
    // move.PositionDriver( 100, 100,  90 );
    // move.PositionDriver( 100, 100, 180 );
    // move.PositionDriver(   0, 100, 180 );
    // move.PositionDriver(   0, 100, 270 );
    // move.PositionDriver(   0,   0,   0 );

    // move.sensor_drive( 20, "right" );
   // move.line_align( "left" );

     oms.oms_driver(20);

    //delay(500);
    //oms.oms_driver( 20 );

    //oms.oms_driver( 35 );

    //oms.SetGripper(20 ); 
    
    //oms.SetGripper(20 ); 
    //delay (500);
   // oms.SetGripper(25); // 0-300
    
    // move.line_align( "left" );

    hard.SetRunningLED(false);

    return 0; 
}
