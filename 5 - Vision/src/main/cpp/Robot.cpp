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
    Camera cam( &move, &hard );
    Oms oms( &hard );

    cam.TrackObject();
    // std::string obj = cam.DetectObject();

    // frc::SmartDashboard::PutString("Detected Object", obj );

    delay(1000);


    // Wait until the Start Button is pressed
    while( hard.GetStartButton() ){ delay(150); }

    hard.SetRunningLED(true);
    
    move.SetPosition( 0, 0, 0 );            // Defines the Initial Position of the Robot

    oms.oms_driver(20);

    hard.SetRunningLED(false);

    return 0; 
}
