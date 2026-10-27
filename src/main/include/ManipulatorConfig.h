#pragma once

// Planned hardware confirmed by user: one Maverick 12 V DC motor, two servos,
// and possibly one or two limit switches. This file reserves configuration;
// it does NOT construct outputs or claim a working pickup mechanism.
namespace manipulator_config {
static constexpr bool CONFIGURED = false;
static constexpr int TITAN_MOTOR_PORT = 3; // proposed spare port, confirm wiring
static constexpr int SERVO_PWM[2] = {-1,-1}; // PLACEHOLDER WPI PWM channels
static constexpr int LIMIT_DIO[2] = {-1,-1}; // PLACEHOLDER; -1 = not assigned
static constexpr double MOTOR_UP_OUTPUT = 0.0; // PLACEHOLDER polarity/power
static constexpr double MOTOR_DOWN_OUTPUT = 0.0;
static constexpr double SERVO_PICKUP_DEG[2] = {0,0}; // PLACEHOLDER safe positions
static constexpr double SERVO_RELEASE_DEG[2] = {0,0};
// Heights, homing direction, travel bounds and successful-object detection
// depend on the actual linkage and must accompany its future implementation.
}
