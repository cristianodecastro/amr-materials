/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#include "Hardware.h"

#define DEBUG true

Hardware::Hardware()
{
    ResetEncoders();
    ResetYaw();
}

void Hardware::ResetEncoders()
{
    LeftEncoder.Reset();
    BackEncoder.Reset();
    RightEncoder.Reset();
    ElevatorEncoder.Reset();
}

void Hardware::ResetYaw()
{
    navX.ZeroYaw();
}
void Hardware::SetLeft( double pwm ){
    LeftMotor.Set( pwm );
}
void Hardware::SetRight( double pwm ){
    RightMotor.Set( pwm );
}
void Hardware::SetBack( double pwm ){
    BackMotor.Set( pwm );
}

void Hardware::Elevation( double angle, double elevator ){
    ElevatorMotor.Set( elevator );
    servo.SetAngle( angle );
}

void Hardware::SetRunningLED(bool on)
{
    runningLED.Set(on);
}

void Hardware::SetStoppedLED(bool on)
{
    stoppedLED.Set(on);
}

double Hardware::GetLeftEncoder()
{
    return LeftEncoder.GetRaw();
}

double Hardware::GetBackEncoder()
{
    return BackEncoder.GetRaw();
}

double Hardware::GetRightEncoder()
{
    return RightEncoder.GetRaw();
}

double Hardware::GetElevatorEncoder()
{
    return ElevatorEncoder.GetRaw();
}

double Hardware::GetYaw()
{
    return navX.GetYaw();
}

double Hardware::GetAngle()
{
    return navX.GetAngle();
}

bool Hardware::GetStopButton(){
    return stopButton.Get();
}
bool Hardware::GetStartButton(){
    return startButton.Get();
}

double Hardware::GetCobra( int channel ){
    return cobra.GetVoltage(channel);
}


double Hardware::getRightDistance( ){
    return (pow(right_sharp.GetAverageVoltage(), -1.2045)) * 27.726;
}

double Hardware::getLeftDistance( ){
    return (pow(left_sharp.GetAverageVoltage(), -1.2045)) * 27.726;
}