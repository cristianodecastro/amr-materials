#pragma once

#include <math.h>

namespace localization_config {

// The active Shuffleboard tab shows the essentials. Enable this only for
// extra SmartDashboard troubleshooting fields in localization mode.
static constexpr bool ENABLE_DETAILED_TELEMETRY = false;

// Left-stick robot-relative translation. This mode takes priority over the
// previous arena exercise; no physical Start press is needed for manual drive.
static constexpr bool TELEOP_DRIVE_ENABLED = true;
static constexpr int DRIVER_JOYSTICK_PORT = 0;
// Common gamepad mapping; confirm with the Teleop tab/Windows game controller
// panel because USB/Bluetooth operating modes can expose different axes.
static constexpr int DRIVER_LEFT_X_AXIS = 0;
static constexpr int DRIVER_LEFT_Y_AXIS = 1;
static constexpr double DRIVER_LEFT_X_SIGN = 1.0;  // positive raw X = right
static constexpr double DRIVER_LEFT_Y_SIGN = -1.0; // negative raw Y = forward
static constexpr double DRIVER_STICK_DEADBAND = 0.12;
static constexpr double TELEOP_SPEED_MMPS = 250.0;
static constexpr double TELEOP_ACCELERATION_MMPS2 = 500.0;
static constexpr double TELEOP_MAX_MOTOR_OUTPUT = 0.25;

// Current arena exercise: encoder-only forward travel, started by the button
// in enabled Teleoperated mode. Set false to restore localization diagnostics.
static constexpr bool FORWARD_TEST_ENABLED = false;
// Physical Titan-port/wheel mapping was supplied by the user. The direction
// offset below is an arena-observation correction awaiting the next run.
static constexpr bool FORWARD_WHEEL_MAPPING_CONFIRMED = true;
static constexpr double FORWARD_TARGET_MM = 1000.0;
static constexpr double FORWARD_CRUISE_SPEED_MMPS = 250.0;
static constexpr double FORWARD_MIN_SPEED_MMPS = 150.0;
static constexpr double FORWARD_ACCELERATION_MMPS2 = 250.0;
static constexpr double FORWARD_DISTANCE_KP_PER_S = 2.0;
static constexpr double FORWARD_MAX_MOTOR_OUTPUT = 0.25;
// Arena observation: the original +X command translated physically left.
// Rotate this exercise's travel vector clockwise 90 degrees in the existing
// wheel-layout frame. Use this same vector for commands and encoder distance.
static constexpr double FORWARD_TEST_DIRECTION_DEG = -90.0;
// Keep manual forward consistent with the most recent arena correction.
// This mounting-frame alignment still needs confirmation on the real robot.
static constexpr double TELEOP_FORWARD_DIRECTION_DEG = FORWARD_TEST_DIRECTION_DEG;
static constexpr double FORWARD_WHEEL_SYNC_KP_PER_S = 2.0;
static constexpr double FORWARD_MAX_SYNC_MMPS = 80.0;
static constexpr double FORWARD_TIMEOUT_S = 20.0;
static constexpr double FORWARD_STALL_TIMEOUT_S = 2.0;
static constexpr double FORWARD_PROGRESS_STEP_MM = 2.0;
static constexpr double FORWARD_MAX_WHEEL_DISAGREEMENT_MM = 100.0;
static constexpr double FORWARD_REVERSE_LIMIT_MM = 20.0;
static constexpr double START_BUTTON_DEBOUNCE_S = 0.060;

static constexpr int TITAN_CAN_ID = 42;
static constexpr int TITAN_MOTOR_FREQUENCY_HZ = 15600;

static constexpr int WHEEL_COUNT = 3;
static constexpr int WHEEL_TITAN_PORT[WHEEL_COUNT] = {0, 1, 2};

// Verify the Titan encoder's configured count convention for the 75001.
static constexpr double ENCODER_COUNTS_PER_OUTPUT_REV = 1464.0;
static constexpr double MOTOR_TO_WHEEL_RATIO = 1.0;  // PLACEHOLDER if gearing differs

// User-specified wheel diameter 102 mm and center radius 168 mm.
static constexpr double WHEEL_RADIUS_MM = 51.0;
static constexpr double WHEEL_CENTER_RADIUS_MM[WHEEL_COUNT] = {
    168.0, 168.0, 168.0};

// Confirmed top-view radial positions: M0 in quadrant II (150 deg),
// M1 in quadrant I (30 deg), M2 on -Y (270 deg) in the supplied layout frame.
// The first run exposed a mismatch with physical forward; the test's direction
// offset above handles that observation without rewriting localization geometry.
// Positive rolling direction is tangent in the counterclockwise direction.
static constexpr double WHEEL_ANGLE_DEG[WHEEL_COUNT] = {150.0, 30.0, 270.0};
static constexpr double WHEEL_ENCODER_SIGN[WHEEL_COUNT] = {1.0, 1.0, 1.0};
static constexpr double WHEEL_MOTOR_SIGN[WHEEL_COUNT] = {1.0, 1.0, 1.0};
static constexpr double MAX_WHEEL_SPEED_MMPS = 1000.0;  // PLACEHOLDER

// Arena coordinates span X=[0,width], Y=[0,height].
static constexpr double ARENA_WIDTH_MM = 2000.0;   // PLACEHOLDER
static constexpr double ARENA_HEIGHT_MM = 2000.0;  // PLACEHOLDER
static constexpr double START_X_MM = 500.0;        // PLACEHOLDER
static constexpr double START_Y_MM = 250.0;        // PLACEHOLDER
static constexpr double START_HEADING_DEG = 0.0;

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

static constexpr int START_BUTTON_DIO = 8;
static constexpr bool START_BUTTON_ACTIVE_LOW = true;

static constexpr double CONTROL_PERIOD_S = 0.020;
// Quiet time after reading one sensor, before triggering the next.
static constexpr double SONAR_PING_SPACING_S = 0.040;
// A 4000 mm echo takes about 24 ms round trip. Allow additional time for
// VMX's captured measurement to reach the Raspberry Pi through the HAL.
static constexpr double SONAR_ECHO_SETTLE_S = 0.060;
static constexpr double SONAR_MIN_RANGE_MM = 20.0;
static constexpr double SONAR_MAX_RANGE_MM = 4000.0;

static constexpr double ACCEL_DEADBAND_G = 0.025;
static constexpr double ACCEL_VELOCITY_BLEND = 0.10;
static constexpr double ENCODER_YAW_RATE_BLEND = 0.10;
static constexpr double YAW_AHRS_CORRECTION = 0.25;
static constexpr double SONAR_RANGE_STDDEV_MM = 40.0;
static constexpr double SONAR_MAX_INNOVATION_MM = 400.0;

static constexpr double PI = M_PI;
static constexpr double GRAVITY_MM_PER_S2 = 9806.65;

}  // namespace localization_config
