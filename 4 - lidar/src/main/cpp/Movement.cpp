/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#include "Movement.h"

void Movement::PositionDriver( double desired_x, double desired_y, double desired_th ) {

    double delta_time = 0.2; // [s]

    delay(100);

    previous_enc_l = hardware->GetLeftEncoder();
    previous_enc_r = hardware->GetRightEncoder();
    previous_enc_b = hardware->GetBackEncoder();

    do{

        current_enc_l = hardware->GetLeftEncoder();
        double delta_enc_l = current_enc_l - previous_enc_l;
        previous_enc_l = current_enc_l;

        current_enc_r = hardware->GetRightEncoder();
        double delta_enc_r = current_enc_r - previous_enc_r;
        previous_enc_r = current_enc_r;

        current_enc_b = hardware->GetBackEncoder();
        double delta_enc_b = current_enc_b - previous_enc_b;
        previous_enc_b = current_enc_b;


        //Wheels Velocity
        leftVelocity  = WheelSpeed(delta_enc_l, delta_time );     // [cm/s]
        rightVelocity = WheelSpeed(delta_enc_r, delta_time );     // [cm/s]
        backVelocity  = WheelSpeed(delta_enc_b, delta_time );     // [cm/s]
        
        //Forward Kinematics
        ForwardKinematics( leftVelocity, rightVelocity, backVelocity );

        //Robot Displacement
        double delta_x  = vx  * delta_time;
        double delta_y  = vy  * delta_time;
        double delta_th = vth * delta_time;

        //Robot Position update
        x_global  = x_global  + delta_x;
        y_global  = y_global  + delta_y;
        // th_global = th_global + ((delta_th / M_PI) * 180); 

        th_global = -hardware->GetYaw() - offset_th;            // Angle based on the Gyro

        if      ( th_global <  0  ) { th_global = th_global + 360; }
        else if ( th_global > 360 ) { th_global = th_global - 360; } 

        double desired_position[3] = { desired_x, desired_y, desired_th };   // [cm], [cm], [degrees]
        double current_position[3] = { x_global , y_global, th_global };     // [cm], [cm], [degrees]

        double x_diff  = desired_position[0] - current_position[0];
        double y_diff  = desired_position[1] - current_position[1];
        double th_diff = desired_position[2] - current_position[2];

        if      ( th_diff < -180 ) { th_diff = th_diff + 360; }
        else if ( th_diff >  180 ) { th_diff = th_diff - 360; }

        frc::SmartDashboard::PutNumber("th_diff", th_diff);

        desired_vx = (x_diff / 10.0) * max_linear_speed; // [cm/s]
        if     ( desired_vx >  max_linear_speed ){ desired_vx =  max_linear_speed; }
        else if( desired_vx < -max_linear_speed ){ desired_vx = -max_linear_speed; }

        desired_vy = (y_diff / 10.0) * max_linear_speed; // [cm/s]
        if     ( desired_vy >  max_linear_speed ){ desired_vy =  max_linear_speed; }
        else if( desired_vy < -max_linear_speed ){ desired_vy = -max_linear_speed; }

        desired_vth = (th_diff / 10.0) * max_ang_speed;  // [rad/s]
        if     ( desired_vth >  max_ang_speed ){ desired_vth =  max_ang_speed; }
        else if( desired_vth < -max_ang_speed ){ desired_vth = -max_ang_speed; }


        if( abs(x_diff) < linear_tolerance )  { desired_vx = 0;  }
        if( abs(y_diff) < linear_tolerance )  { desired_vy = 0;  }
        if( abs(th_diff) < angular_tolerance ){ desired_vth = 0; }


        double th_radius = th_global * ( M_PI / 180.0 );

        // From Global to Local
        double x_l = desired_vx * cos( -th_radius ) - desired_vy * sin( -th_radius );
        double y_l = desired_vx * sin( -th_radius ) + desired_vy * cos( -th_radius );

        //Inverse Kinematics
        InverseKinematics( x_l, y_l, desired_vth );


        if ( !hardware->GetStopButton() ){  // Stop the Motors when the Stop Button is pressed
            hardware->SetLeft ( 0 );
            hardware->SetBack ( 0 );
            hardware->SetRight( 0 );
            // break;
        }else{
            hardware->SetLeft ( desired_left_speed );
            hardware->SetBack ( desired_back_speed );
            hardware->SetRight( desired_right_speed );
        }

        ShuffleBoardUpdate();
      
        delay( delta_time * 1000 ); 

    }while( desired_vx != 0 || desired_vy != 0 || desired_vth != 0 );

    hardware->SetLeft ( 0 );
    hardware->SetBack ( 0 );
    hardware->SetRight( 0 );

    delay(250);
}

void Movement::InverseKinematics(double x, double y, double z){
    
    desired_back_speed  = (( x * cos( M_PI*( 90.0/180.0))) + ( -y * sin(M_PI*( 90.0/180.0))) + (z * constant::FRAME_RADIUS) );
    desired_left_speed  = (( x * cos( M_PI*(210.0/180.0))) + ( -y * sin(M_PI*(210.0/180.0))) + (z * constant::FRAME_RADIUS) );
    desired_right_speed = (( x * cos( M_PI*(330.0/180.0))) + ( -y * sin(M_PI*(330.0/180.0))) + (z * constant::FRAME_RADIUS) );

    desired_back_speed  = -desired_back_speed  / 70.0;   // cm/s to PWM [0-1]
    desired_left_speed  = -desired_left_speed  / 55.0;   // cm/s to PWM [0-1]    
    desired_right_speed = -desired_right_speed / 55.0;   // cm/s to PWM [0-1]
   
}

void Movement::ForwardKinematics( double vl, double vr, double vb ){
    
    double vx_l   = ( (      0     * vb ) + ( (1.0/sqrt(3.0)) * vr) + ( (-1.0/sqrt(3.0)) * vl) );   // [cm/s]
    double vy_l   = ( ( (-2.0/3.0) * vb ) + (    (1.0/3.0)    * vr) + (     (1.0/3.0)    * vl) );   // [cm/s]
    double vth_l  = ( ( ( 1.0/3.0) * vb ) + (    (1.0/3.0)    * vr) + (     (1.0/3.0)    * vl) ) / constant::FRAME_RADIUS ; // [rad/s]

    double th_radius = th_global * ( M_PI / 180.0 );

    // From Local to Global
    vx = vx_l * cos( th_radius ) - vy_l * sin( th_radius );
    vy = vx_l * sin( th_radius ) + vy_l * cos( th_radius );
    vth = vth_l;
}

double Movement::WheelSpeed( int encoder, double time ){
        double speed  = -1 * (((2 * M_PI * constant::WHEEL_RADIUS * encoder) / (constant::PULSE_PER_REV * time)));   // [cm/s]

        if ( time == 0 ) { speed  = 0; }

        return speed;
}

void Movement::SetPosition( double x, double y, double th ){
    x_global  = x;
    y_global  = y;
    offset_th = -hardware->GetYaw() - th;
    th_global = th;
}

void Movement::ShuffleBoardUpdate(){

    th_global = -hardware->GetYaw() - offset_th;            // Angle based on the Gyro

    if      ( th_global <  0  ) { th_global = th_global + 360; }
    else if ( th_global > 360 ) { th_global = th_global - 360; } 

    frc::SmartDashboard::PutNumber("desired_back_speed",  desired_back_speed );
    frc::SmartDashboard::PutNumber("desired_left_speed",  desired_left_speed );
    frc::SmartDashboard::PutNumber("desired_right_speed", desired_right_speed);

    frc::SmartDashboard::PutNumber("robot_x",  x_global );
    frc::SmartDashboard::PutNumber("robot_y",  y_global );
    frc::SmartDashboard::PutNumber("robot_th", th_global);

    frc::SmartDashboard::PutNumber("vx",  vx );
    frc::SmartDashboard::PutNumber("vy",  vy );
    frc::SmartDashboard::PutNumber("vth", vth);
    frc::SmartDashboard::PutBoolean("Start Button",  hardware->GetStartButton() );
    frc::SmartDashboard::PutBoolean("Stop Button",  hardware->GetStopButton() );

    frc::SmartDashboard::PutNumber("leftVelocity",  leftVelocity  );
    frc::SmartDashboard::PutNumber("rightVelocity", rightVelocity );
    frc::SmartDashboard::PutNumber("backVelocity",  backVelocity  );

    frc::SmartDashboard::PutNumber("desired_vx",  desired_vx );
    frc::SmartDashboard::PutNumber("desired_vy",  desired_vy );
    frc::SmartDashboard::PutNumber("desired_vth", desired_vth );

}

double Movement::get_x() { return x_global;  }

double Movement::get_y() { return y_global;  }

double Movement::get_th(){ return th_global; }



void Movement::sensor_drive( double dist, std::string direction ){

    double sensor_reading = 0;

    while( sensor_reading > dist + 2 || sensor_reading < dist - 2 ){

        if      ( direction.compare( "front" ) == 0 ){
            sensor_reading = hardware->getFrontDistance();
            frc::SmartDashboard::PutNumber("front sensor", sensor_reading ); 

            if( sensor_reading > dist ){
                InverseKinematics(  20, 0, 0 );
            }else{
                InverseKinematics( -20, 0, 0 );
            }
        }else if( direction.compare( "left" ) == 0 ){
            sensor_reading = hardware->getLeftDistance();
            frc::SmartDashboard::PutNumber("left sensor", sensor_reading ); 

            if( sensor_reading > dist ){
                InverseKinematics( 0,  20, 0 );
            }else{
                InverseKinematics( 0, -20, 0 );
            }
        }else if( direction.compare( "right" ) == 0 ){
            sensor_reading = hardware->getRightDistance();
            frc::SmartDashboard::PutNumber("right sensor", sensor_reading ); 

            if( sensor_reading > dist ){
                InverseKinematics( 0, -20, 0 );
            }else{
                InverseKinematics( 0,  20, 0 );
            }
        }

        if ( !hardware->GetStopButton() ){  // Stop the Motors when the Stop Button is pressed
            hardware->SetLeft ( 0 );
            hardware->SetBack ( 0 );
            hardware->SetRight( 0 );
            // break;
        }else{
            hardware->SetLeft ( desired_left_speed );
            hardware->SetBack ( desired_back_speed );
            hardware->SetRight( desired_right_speed );
        }

        ShuffleBoardUpdate();

        delay( 50 );

    }

    hardware->SetLeft ( 0 );
    hardware->SetBack ( 0 );
    hardware->SetRight( 0 );

    delay( 150 );
}

void Movement::line_align( std::string direction ){

    frc::SmartDashboard::PutString("Process",  "Cobra Align" );

    bool cobra_l  = false;
    bool cobra_r  = false;
    bool cobra_cl = false;
    bool cobra_cr = false;


    while( !cobra_cl || !cobra_cr ){
        
        cobra_l  = hardware->GetCobra(0) > 2.5;
        cobra_r  = hardware->GetCobra(3) > 2.5;
        cobra_cl = hardware->GetCobra(1) > 2.5;
        cobra_cr = hardware->GetCobra(2) > 2.5;

        frc::SmartDashboard::PutNumber("cobra_0", hardware->GetCobra(0) );
        frc::SmartDashboard::PutNumber("cobra_1", hardware->GetCobra(1) );
        frc::SmartDashboard::PutNumber("cobra_2", hardware->GetCobra(2) );
        frc::SmartDashboard::PutNumber("cobra_3", hardware->GetCobra(3) ); 

        if      ( direction.compare( "left" ) == 0){
            InverseKinematics( 0,  20, 0 );  
        }else if( direction.compare( "right" ) == 0 ){
            InverseKinematics( 0, -20, 0 );  
        }else{
            break;
        }

        
        if ( !hardware->GetStopButton() ){  // Stop the Motors when the Stop Button is pressed
            hardware->SetLeft ( 0 );
            hardware->SetBack ( 0 );
            hardware->SetRight( 0 );
            // break;
        }else{
            hardware->SetLeft ( desired_left_speed );
            hardware->SetBack ( desired_back_speed );
            hardware->SetRight( desired_right_speed );
        }

        delay(50);
    }
    hardware->SetLeft ( 0 );
    hardware->SetBack ( 0 );
    hardware->SetRight( 0 );

    delay(200);
}