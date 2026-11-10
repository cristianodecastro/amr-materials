#pragma once

#include <math.h>

namespace config {

static constexpr int TITAN_CAN_ID = 42;
static constexpr int TITAN_MOTOR_FREQUENCY_HZ = 15600;

// Titan M0/M1/M2 are assigned to the three wheels. Change this table if the
// physical motor-to-wheel assignment differs.
static constexpr int WHEEL_COUNT = 3;
static constexpr int WHEEL_TITAN_PORT[WHEEL_COUNT] = {0, 1, 2};

// Studica 75001 (61:1 D-shaft) is specified at 1,464 quadrature counts per
// gearbox-output revolution. Confirm the Titan encoder CPR configuration.
static constexpr double ENCODER_COUNTS_PER_OUTPUT_REV = 1464.0;
static constexpr double MOTOR_TO_WHEEL_RATIO = 1.0;

// PLACEHOLDERS: replace with measured robot dimensions, in metres.
static constexpr double WHEEL_RADIUS_M = 0.051;
static constexpr double WHEEL_CENTER_RADIUS_M[WHEEL_COUNT] = {
    0.150, 0.150, 0.150};  // PLACEHOLDER: centre to each wheel contact point
static constexpr double WHEEL_ANGLE_DEG[WHEEL_COUNT] = {0.0, 120.0, 240.0};
static constexpr double WHEEL_ENCODER_SIGN[WHEEL_COUNT] = {1.0, 1.0, 1.0};
static constexpr double WHEEL_MOTOR_SIGN[WHEEL_COUNT] = {1.0, 1.0, 1.0};
static constexpr double MAX_WHEEL_SPEED_MPS = 1.0;  // PLACEHOLDER

static constexpr double ARENA_WIDTH_M = 2.0;   // PLACEHOLDER
static constexpr double ARENA_HEIGHT_M = 2.0;  // PLACEHOLDER
static constexpr double START_X_M = 0.50;      // PLACEHOLDER start pose
static constexpr double START_Y_M = 0.25;      // PLACEHOLDER start pose
static constexpr double START_HEADING_DEG = 0.0;

// Coordinate convention: +X is robot-forward at yaw=0, +Y is robot-left,
// positive yaw is counter-clockwise. Sensor offsets are from robot centre.
// PLACEHOLDERS: measure the sensor face positions from the robot centre.
static constexpr double REAR_SENSOR_X_M = -0.10;
static constexpr double REAR_SENSOR_HALF_SPACING_M = 0.08;
static constexpr double SIDE_SENSOR_Y_M = 0.12;

struct SonarConfig {
    int triggerDio;
    int echoDio;
    double mountX_M;
    double mountY_M;
    double beamAngleDeg;
    const char* name;
};

// Each Studica ultrasonic uses one FlexDIO connector: signal A = trigger,
// signal B = echo. The valid channel pairs are 0/1 through 6/7.
static const SonarConfig SONARS[4] = {
    {0, 1, REAR_SENSOR_X_M,  REAR_SENSOR_HALF_SPACING_M, 180.0, "sonar_back_left"},
    {2, 3, REAR_SENSOR_X_M, -REAR_SENSOR_HALF_SPACING_M, 180.0, "sonar_back_right"},
    {4, 5, 0.0,  SIDE_SENSOR_Y_M,  90.0, "sonar_left"},
    {6, 7, 0.0, -SIDE_SENSOR_Y_M, -90.0, "sonar_right"}
};

static constexpr int START_BUTTON_DIO = 8;
static constexpr bool START_BUTTON_ACTIVE_LOW = true;

static constexpr double CONTROL_PERIOD_S = 0.020;
static constexpr double SONAR_PING_SPACING_S = 0.040;
static constexpr double SONAR_ECHO_SETTLE_S = 0.035;
static constexpr double SONAR_MIN_RANGE_M = 0.02;
static constexpr double SONAR_MAX_RANGE_M = 4.00;

static constexpr double ACCEL_DEADBAND_G = 0.025;
static constexpr double ACCEL_VELOCITY_BLEND = 0.10;
static constexpr double YAW_AHRS_CORRECTION = 0.25;
static constexpr double SONAR_RANGE_STDDEV_M = 0.04;
static constexpr double SONAR_MAX_INNOVATION_M = 0.40;

static constexpr double PI = M_PI;
static constexpr double GRAVITY_M_PER_S2 = 9.80665;

}  // namespace config
