#pragma once

#include <math.h>

namespace localization_config {

// The active Shuffleboard tab shows the essentials. Enable this only for
// extra SmartDashboard troubleshooting fields in localization mode.
static constexpr bool ENABLE_DETAILED_TELEMETRY = false;

// Left-stick translation and L2/R2 rotation in Control Station Teleoperated
// mode. Autonomous motion is currently explicitly disabled.
static constexpr bool TELEOP_DRIVE_ENABLED = true;
static constexpr int DRIVER_JOYSTICK_PORT = 0;
// Common gamepad mapping; confirm with the Teleop tab/Windows game controller
// panel because USB/Bluetooth operating modes can expose different axes.
static constexpr int DRIVER_LEFT_X_AXIS = 0;
static constexpr int DRIVER_LEFT_Y_AXIS = 1;
// Reverse both translation axes following the installed robot's movement test.
// These signs affect translation only; trigger rotation polarity is independent.
static constexpr double DRIVER_LEFT_X_SIGN = -1.0;
static constexpr double DRIVER_LEFT_Y_SIGN = 1.0;
static constexpr double DRIVER_STICK_DEADBAND = 0.12;
// PS-style digital trigger buttons, numbered from 1 by WPILib. Check the
// L2/R2 dashboard lamps while Disabled; USB/Bluetooth host mappings can differ.
// Button identities: L2 requests CW, R2 requests CCW in TeleopDriveController.
static constexpr int DRIVER_L2_BUTTON = 7;
static constexpr int DRIVER_R2_BUTTON = 8;
// Cap manual translation at half the estimated no-load wheel speed and ramp
// the velocity vector gently to reduce sharp starts and diagonal changes.
static constexpr double TELEOP_TRANSLATION_SPEED_SCALE = 0.50;
static constexpr double TELEOP_ACCELERATION_MMPS2 = 400.0;
static constexpr double TELEOP_MAX_MOTOR_OUTPUT = 0.90;
static constexpr double TELEOP_MAX_YAW_RATE_DPS = 90.0;
static constexpr double TELEOP_YAW_ACCELERATION_DPS2 = 180.0;
// The tangent wheel model is counterclockwise-positive. Flip only if a
// physical rotation check establishes opposite installed motor polarity.
static constexpr double TELEOP_YAW_SIGN = 1.0;

// Wheel angles below are now directly in the robot body frame: +X forward,
// +Y left. The previous layout's -90 degree offset is therefore no longer used.
static constexpr double DRIVE_FORWARD_DIRECTION_DEG = 0.0;
static constexpr double TELEOP_FORWARD_DIRECTION_DEG = DRIVE_FORWARD_DIRECTION_DEG;
static constexpr double START_BUTTON_DEBOUNCE_S = 0.060;

static constexpr int TITAN_CAN_ID = 42;
static constexpr int TITAN_MOTOR_FREQUENCY_HZ = 15600;

static constexpr int WHEEL_COUNT = 3;
// Use the Titan Quad's printed M0..M3 labels directly as API port numbers.
// Wheel order: left rear (M0), right rear (M2), front (M3). Lift uses M1.
static constexpr int WHEEL_TITAN_PORT[WHEEL_COUNT] = {0, 2, 3};
static constexpr const char* WHEEL_NAME[WHEEL_COUNT] = {"Left rear", "Right rear", "Front"};

// Studica specifies 6 CPR * 61 gearbox ratio * 4 quadrature = 1464/output rev.
static constexpr double ENCODER_COUNTS_PER_OUTPUT_REV = 1464.0;
static constexpr double MOTOR_TO_WHEEL_RATIO = 1.0;  // PLACEHOLDER if gearing differs

// User-specified wheel diameter 102 mm and center radius 168 mm.
static constexpr double WHEEL_RADIUS_MM = 51.0;
static constexpr double WHEEL_CENTER_RADIUS_MM[WHEEL_COUNT] = {
    168.0, 168.0, 168.0};

// Same 120-degree spacing, expressed relative to the robot front (+X):
// left rear 120 degrees, right rear 240 degrees, front 0 degrees.
// Positive rolling direction is tangent in the counterclockwise direction.
static constexpr double WHEEL_ANGLE_DEG[WHEEL_COUNT] = {120.0, 240.0, 0.0};
static constexpr double WHEEL_ENCODER_SIGN[WHEEL_COUNT] = {1.0, 1.0, 1.0};
static constexpr double WHEEL_MOTOR_SIGN[WHEEL_COUNT] = {1.0, 1.0, 1.0};
// Maverick 75001 speeds are already at the 61:1 gearbox OUTPUT shaft.
// MOTOR_TO_WHEEL_RATIO represents external gearing only; do not divide by 61
// again. This is a feedforward estimate, not an encoder speed-control loop.
static constexpr double MAVERICK_MAX_NO_LOAD_RPM = 112.0;
static constexpr double MAVERICK_NOMINAL_NO_LOAD_RPM = 100.0;
static constexpr double MAVERICK_NOMINAL_RATED_RPM = 80.0;
static constexpr double MAX_WHEEL_SPEED_MMPS =
    2.0 * M_PI * WHEEL_RADIUS_MM * MAVERICK_MAX_NO_LOAD_RPM /
    (60.0 * MOTOR_TO_WHEEL_RATIO);
static constexpr double TELEOP_SPEED_MMPS =
    MAX_WHEEL_SPEED_MMPS * TELEOP_TRANSLATION_SPEED_SCALE;

// Arena coordinates span X=[0,width], Y=[0,height].
static constexpr double ARENA_WIDTH_MM = 2000.0;
static constexpr double ARENA_HEIGHT_MM = 2000.0;
static constexpr double START_X_MM = 1000.0;       // PLACEHOLDER center of start square
static constexpr double START_Y_MM = 900.0;        // PLACEHOLDER center of start square
static constexpr double START_HEADING_DEG = 90.0;  // PLACEHOLDER facing upper arena

// At heading zero, robot +X points toward arena +X and +Y points left.
// Sensor positions are their sensing-face coordinates from robot center.
static constexpr double REAR_SENSOR_X_MM = -100.0;  // PLACEHOLDER
static constexpr double REAR_SENSOR_HALF_SPACING_MM = 80.0;  // PLACEHOLDER
static constexpr double SIDE_SENSOR_Y_MM = 120.0;  // PLACEHOLDER

struct SonarConfig {
  int triggerDio;
  int echoDio;
  double mountX_MM;
  double mountY_MM;
  double beamAngleDeg;
  const char* name;
};

// Studica ultrasonic trigger/echo DIO pairs are 0/1, 2/3, 4/5, and 6/7.
static const SonarConfig SONARS[4] = {
    {0, 1, REAR_SENSOR_X_MM,  REAR_SENSOR_HALF_SPACING_MM, 180.0,
     "sonar_back_left"},
    {2, 3, REAR_SENSOR_X_MM, -REAR_SENSOR_HALF_SPACING_MM, 180.0,
     "sonar_back_right"},
    {4, 5, 0.0,  SIDE_SENSOR_Y_MM,  90.0, "sonar_left"},
    {6, 7, 0.0, -SIDE_SENSOR_Y_MM, -90.0, "sonar_right"}};

// No physical start button is installed. FlexDIO 8 is now a servo output.

static constexpr double CONTROL_PERIOD_S = 0.020;
// Quiet time after reading one sensor, before triggering the next.
static constexpr double SONAR_PING_SPACING_S = 0.040;
// A 4000 mm echo takes about 24 ms round trip. Allow additional time for
// VMX's captured measurement to reach the Raspberry Pi through the HAL.
static constexpr double SONAR_ECHO_SETTLE_S = 0.060;
static constexpr double SONAR_MIN_RANGE_MM = 20.0;
static constexpr double SONAR_MAX_RANGE_MM = 4000.0;

static constexpr double ACCEL_DEADBAND_G = 0.025;
// Still capture accelerometer data, but do not integrate its bias into position.
// Encoders are the translation baseline; AHRS uses accelerometer internally.
static constexpr double ACCEL_VELOCITY_BLEND = 0.0;
static constexpr double ENCODER_YAW_RATE_BLEND = 0.10;
static constexpr double YAW_AHRS_CORRECTION = 0.25;
static constexpr double SONAR_RANGE_STDDEV_MM = 40.0;
static constexpr double SONAR_MAX_INNOVATION_MM = 100.0;

static constexpr double PI = M_PI;
static constexpr double GRAVITY_MM_PER_S2 = 9806.65;

}  // namespace localization_config
