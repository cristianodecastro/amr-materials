#include "Oms.h"

void Oms::oms_driver( double desired_height ){

    double delta_time = 0.2; // [s]

    delay(100);

    double current_enc  = hardware->GetElevatorEncoder();
    double previous_enc = hardware->GetElevatorEncoder();

    do{

        current_enc = hardware->GetElevatorEncoder();
        double delta_enc = current_enc - previous_enc;
        previous_enc = current_enc;

        // Speed per iteration
        double elevatorVelocity = (((2 * M_PI * ( 1.25 ) * delta_enc) / (constant::PULSE_PER_REV * delta_time)));   // [cm/s]

        double delta_elev = elevatorVelocity * delta_time;  // Displacement per iteration

        height = height + delta_elev;

        if( !hardware->GetLimitHigh() ){ height = 40; }
        if( !hardware->GetLimitLow()  ){ height = 10; }


        double elev_diff = desired_height - height;

        double max_linear_speed = 10; // cm/s

        double desired_v = (elev_diff / 5.0) * max_linear_speed; // [cm/s]
        if     ( desired_v >  max_linear_speed ){ desired_v =  max_linear_speed; }
        else if( desired_v < -max_linear_speed ){ desired_v = -max_linear_speed; }

        if( abs(elev_diff) < 2 )  { desired_v = 0; break; }


        if ( !hardware->GetStopButton() ){  // Stop the Motors when the Stop Button is pressed
            hardware->SetElevator ( 0 );
        }else{
            hardware->SetElevator ( desired_v / 70.0 );
        }

        frc::SmartDashboard::PutNumber("height",  height );


        delay( delta_time * 1000 );


    }while( true );
    
    hardware->SetElevator ( 0 );
    delay(500);

}

void Oms::SetGripper( double angle ){
    hardware->SetGripper( angle );
}
