#pragma once
#include <cmath>
#include "RobotTypes.h"

// On-demand USB still photos; no colour-processing or continuous capture.
namespace camera_config {
static constexpr int DEVICE=0,WIDTH=640,HEIGHT=480,FPS=15,PREVIEW_FPS=1;
static constexpr double WARMUP_S=0.30,REQUEST_TIMEOUT_S=8.0;
static constexpr const char* CAPTURE_DIRECTORY="/home/pi/SkillsChallenge/captures";
}

namespace localization_config {
static constexpr bool TELEOP_DRIVE_ENABLED=true;
static constexpr int DRIVER_JOYSTICK_PORT=0;
static constexpr int DRIVER_LEFT_X_AXIS=0,DRIVER_LEFT_Y_AXIS=1,DRIVER_RIGHT_X_AXIS=2;
static constexpr double DRIVER_LEFT_X_SIGN=1,DRIVER_LEFT_Y_SIGN=-1,DRIVER_RIGHT_X_SIGN=1;
static constexpr double DRIVER_STICK_DEADBAND=0.12;
// Operator speed/acceleration limits, not individual motor corrections.
static constexpr double TELEOP_ACCELERATION_MMPS2=400;
static constexpr double TELEOP_MAX_MOTOR_OUTPUT=0.90;
static constexpr double TELEOP_MAX_YAW_RATE_DPS=90;
static constexpr double TELEOP_YAW_ACCELERATION_DPS2=45;
static constexpr double TELEOP_YAW_SIGN=1;
static constexpr int TITAN_CAN_ID=42,TITAN_MOTOR_FREQUENCY_HZ=15600;
static constexpr int WHEEL_COUNT=3;
// Top view: M0 upper-left, M1 rear/bottom, M2 upper-right. M3 is unused.
static constexpr int WHEEL_TITAN_PORT[WHEEL_COUNT]={0,1,2};
static constexpr const char* WHEEL_NAME[WHEEL_COUNT]={"Front left","Rear","Front right"};
// Body +X is forward (TOP of supplied picture), +Y is left; yaw is CCW.
// Picture radial angles 150/270/30 become 60/180/300 in this body frame.
static constexpr double WHEEL_ANGLE_DEG[WHEEL_COUNT]={60,180,300};
// Wiring polarity only: positive output/count follows each CCW tangent arrow.
static constexpr double WHEEL_MOTOR_SIGN[WHEEL_COUNT]={1,1,1};
static constexpr double WHEEL_ENCODER_SIGN[WHEEL_COUNT]={1,1,1};
static constexpr double WHEEL_CENTER_RADIUS_MM[WHEEL_COUNT]={159.74,159.74,159.74};
static constexpr double WHEEL_RADIUS_MM=51;
static constexpr double ENCODER_COUNTS_PER_OUTPUT_REV=1464; // 6 * 61 * quadrature 4
static constexpr double ENCODER_MM_PER_COUNT=2*robot_geometry::PI*WHEEL_RADIUS_MM/ENCODER_COUNTS_PER_OUTPUT_REV;
static constexpr double MAVERICK_MAX_NO_LOAD_RPM=112;
static constexpr double MAX_WHEEL_SPEED_MMPS=2*robot_geometry::PI*WHEEL_RADIUS_MM*MAVERICK_MAX_NO_LOAD_RPM/60;
// Retain the requested half-speed chassis limit; no wheel trim or distance scale.
static constexpr double TELEOP_SPEED_MMPS=MAX_WHEEL_SPEED_MMPS/2;
static constexpr double CONTROL_PERIOD_S=0.020,MAX_CONTROL_GAP_S=0.10;
static constexpr double ARENA_WIDTH_MM=2000,ARENA_HEIGHT_MM=2000;
static constexpr double START_X_MM=950,START_Y_MM=900,START_HEADING_DEG=0;

static constexpr double FRONT_SENSOR_OFFSET_MM=171.6,REAR_SENSOR_OFFSET_MM=173;
static constexpr double LEFT_SENSOR_OFFSET_MM=60,RIGHT_SENSOR_OFFSET_MM=60;
struct RangeSensorMount { double mountX_MM,mountY_MM,beamAngleDeg; };
static constexpr RangeSensorMount RANGE_SENSOR_MOUNTS[4]={
  {0,LEFT_SENSOR_OFFSET_MM,90},{-REAR_SENSOR_OFFSET_MM,0,180},
  {0,-RIGHT_SENSOR_OFFSET_MM,-90},{FRONT_SENSOR_OFFSET_MM,0,0}};
struct SonarConfig { int triggerDio,echoDio; const char* name; };
static constexpr int SONAR_COUNT=2;
// FlexDIO: even channel = trigger (A), odd channel = echo (B).
// Keep logical order front/rear for the dashboard and wall references.
static constexpr SonarConfig SONARS[SONAR_COUNT]={{2,3,"Front"},{0,1,"Rear"}};
static constexpr double SONAR_PING_SPACING_S=0.040,SONAR_ECHO_SETTLE_S=0.060;
static constexpr double SONAR_MIN_RANGE_MM=20,SONAR_MAX_RANGE_MM=4000,SONAR_READING_STALE_S=0.350;
// Sharp GP2Y0A21YK analog voltage: VMX Analog In 0 left, 1 right.
static constexpr int IR_COUNT=2,IR_ANALOG_CHANNEL[IR_COUNT]={0,1};
static constexpr double IR_MIN_RANGE_MM=100,IR_MAX_RANGE_MM=800,IR_MAX_INPUT_V=5;
static constexpr double SENSOR_UPDATE_PERIOD_S=0.050,SENSOR_STALE_S=0.200;
}

namespace wheel_drive_config {
// Encoders monitor motion only. They never trim/boost/hold an individual wheel.
static constexpr double SPEED_FILTER_S=0.060,SPEED_WINDOW_S=0.060,MAX_SAMPLE_S=0.10;
static constexpr double ZERO_SPEED_MMPS=0.5;
static constexpr double STALL_REQUEST_MMPS=20,STALL_MEASURED_MMPS=3,STALL_OUTPUT=0.12,STALL_TIMEOUT_S=1;
static constexpr double DIRECTION_GRACE_S=0.30,WRONG_DIRECTION_TIMEOUT_S=0.25;
}

// Observational manual-mode pose display; never steers Teleop or feeds Auto.
namespace teleop_odometry_config {
static constexpr double ACCEL_DEADBAND_G=0.005,ACCEL_VELOCITY_BLEND=0.15;
static constexpr double ACCEL_MAX_PLAUSIBLE_G=0.5,ACCEL_BIAS_TRACK_TAU_S=0.75;
static constexpr double ENCODER_STATIONARY_MMPS=3,GYRO_STATIONARY_DPS=0.5;
static constexpr double ACCEL_MAX_VELOCITY_CORRECTION_MMPS=20,YAW_AHRS_CORRECTION=0.25;
static constexpr double GRAVITY_MM_PER_S2=9806.65;
}

namespace cobra_config {
static constexpr double COBRA_REFERENCE_V=5;
static constexpr int COBRA_I2C_ADDRESS=0x48;
// Authoritative measured references: CH0 leftmost to CH3 rightmost.
static constexpr double COBRA_FLOOR_V[4]={2.172852,1.005859,1.257324,2.868652};
static constexpr double COBRA_TAPE_V[4]={4.997559,4.997559,4.997559,4.997559};
}

namespace heading_control_config {
static constexpr double HEADING_TOLERANCE_RAD=robot_geometry::PI/180;
static constexpr double STATIONARY_SPEED_MMPS=8,STATIONARY_YAW_DPS=0.5;
static constexpr double HOLD_HEADING_DEADBAND_RAD=0.25*robot_geometry::PI/180;
static constexpr double HOLD_HEADING_KP=2.4,HOLD_YAW_RATE_DAMPING=0.65;
static constexpr double HOLD_YAW_ACCEL_RADPS2=180*robot_geometry::PI/180;
static constexpr double HOLD_BRAKE_RESPONSE_S=0.18,HOLD_RECHECK_S=0.060;
static constexpr double HOLD_RECHECK_WINDOW_RAD=2*robot_geometry::PI/180;
static constexpr double HOLD_HEADING_MAX_YAW_RADPS=20*robot_geometry::PI/180;
static constexpr double YAW_DIRECTION_ERROR_RAD=5*robot_geometry::PI/180;
static constexpr double YAW_DIRECTION_CHECK_MIN_RADPS=0.1*robot_geometry::PI/180;
}

namespace navigation_config {
static constexpr double BODY_RADIUS_MM=159.74+12.0;
static constexpr double MANIPULATOR_EXTENSION_MM=120.0;
static constexpr double MIN_CLEARANCE_MM=40.0;
static constexpr double POSITION_TOLERANCE_MM=10.0;
static constexpr double GRID_MM=25.0;
static constexpr double SPEED_MMPS=180.0,ACCEL_MMPS2=200.0;
static constexpr double OBSERVATION_SPACING_MM=300.0,OBSERVATION_S=0.65;
static constexpr double MISSION_TIMEOUT_S=180.0;
static constexpr double DEFAULT_GOAL_X_MM=1600,DEFAULT_GOAL_Y_MM=1600,DEFAULT_GOAL_HEADING_DEG=0;
// Physical keep-out regions, without robot clearance. Add rows as needed;
// count is inferred automatically. Central square and other tape stay free.
static constexpr ForbiddenRegion FORBIDDEN[]={
    {780,1220,1220,2000}, // 440 x 780 upper block, centered at X=1000
    {0,0,2000,220},     // bottom delivery band
    {985,0,1015,600}};  // 30 x 600 lower central strip
}

// Explicit-start corner reference and tape approach. Arena +X east, +Y north.
namespace tape_test_config {
static constexpr double START_HEADING_DEG=0; // north, under the new arena convention
static constexpr ArenaPoint TARGET_FROM_LEFT={220,500},TARGET_FROM_RIGHT={580,500};
static constexpr double LEFT_WALL_X_MM=0,REAR_WALL_Y_MM=0;
static constexpr double DRIVE_SPEED_MMPS=localization_config::TELEOP_SPEED_MMPS;
static constexpr double DRIVE_ACCEL_MMPS2=localization_config::TELEOP_ACCELERATION_MMPS2;
static constexpr double POSITION_TOLERANCE_MM=navigation_config::POSITION_TOLERANCE_MM;
static constexpr double POSITION_KP=1.5,MIN_APPROACH_MMPS=120;
static constexpr double CAPTURE_RADIUS_MM=7,NEAR_TARGET_MM=60;
static constexpr double BRAKE_ACCEL_MMPS2=DRIVE_ACCEL_MMPS2,BRAKE_RESPONSE_S=0.18,VELOCITY_FILTER_S=0.08;
static constexpr double CORRECTION_MAX_S=0.50,CORRECTION_REST_S=0.18,WAYPOINT_SETTLE_S=0.30,DRIVE_TIMEOUT_S=35;
static constexpr double SCAN_SPEED_MMPS=60,SCAN_ACCEL_MMPS2=80,CENTER_MIN_SPEED_MMPS=30,CENTER_KP_MMPS=100;
static constexpr double SCAN_MAX_TRAVEL_MM=300,SCAN_TIMEOUT_S=20,TAPE_ACQUIRE_LEVEL=0.40;
static constexpr double CENTER_BLACK_MIN=0.55,CENTER_LEVEL_TOLERANCE=0.10,CENTER_VOLTAGE_TOLERANCE_V=0.20;
static constexpr double CENTER_OUTER_CONTRAST=0.20,TAPE_SETTLE_S=0.30,TAPE_LOST_S=0.40;
static constexpr double NO_PROGRESS_TIMEOUT_S=2,PROGRESS_MM=2;
}
