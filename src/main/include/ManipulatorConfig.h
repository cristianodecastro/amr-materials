#pragma once

// Manual lift and servo configuration. Autonomous pickup remains unimplemented.
namespace manipulator_config {
static constexpr bool CONFIGURED = false; // autonomous mechanism only; manual lift is independent
// Titan Quad's printed M1 connector and its matching encoder channel.
static constexpr int TITAN_MOTOR_PORT = 1;
// Conservative manual open-loop power cap; there are currently NO end switches.
static constexpr double LIFT_OUTPUT = 0.20;
static constexpr double LIFT_UP_SIGN = 1.0; // reverse if installed motor polarity lowers on positive
static constexpr int DRIVER_POV_INDEX = 0;
static constexpr int DRIVER_POV_UP_DEG = 0;
static constexpr int DRIVER_POV_DOWN_DEG = 180;
static constexpr bool LIMIT_ACTIVE_LOW = true;
// Servo order: X button / physical FlexDIO 8, triangle / physical FlexDIO 9.
// VMX PWM addressing differs from the physical FlexDIO and DigitalInput labels.
static constexpr int SERVO_PWM[2] = {18,19};
static constexpr int SERVO_BUTTON[2] = {2,4}; // PS-style X/cross and triangle; WPILib is 1-based
static constexpr double SERVO_HOME_DEG = 0.0;
static constexpr double SERVO_ACTIVE_DEG[2] = {100.0,180.0}; // X/cross, triangle
static constexpr int LIMIT_DIO[2] = {-1,-1}; // PLACEHOLDER; -1 = not assigned
// Heights, homing direction, travel bounds and successful-object detection
// depend on the actual linkage and must accompany its future implementation.
}
