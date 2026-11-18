#pragma once
#include <cmath>
#include "RobotTypes.h"

// MOTION TUNING INDEX (all control values live in this header):
// localization_config: geometry, Teleop limits, feedback timing, sensor mounts.
// wheel_drive_config: PWM floor, output scaling and encoder fault thresholds.
// heading_control_config: shared gyro controller and final heading tolerance.
// navigation_config: transit/approach behavior, route smoothing and sensor blend.
// position_control_config: shared final XY braking/settling/correction behavior.
// tape_test_config: tape approach and scan settings.

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
static constexpr double DRIVER_MAX_VALID_AXIS=1.01,TELEOP_MAX_CONTROL_STEP_S=0.050;
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
// Wiring polarity only: canonical positive wheel travel follows the CCW tangent.
// Installed motors run opposite that tangent for positive Titan electrical duty
// (confirmed by reversed forward/left/CCW Teleop tests), in BOTH drive modes.
static constexpr double WHEEL_MOTOR_SIGN[WHEEL_COUNT]={-1,-1,-1};
// Encoder wiring is independent: passive CW rolling increases all raw counts.
// Convert once to the canonical CCW-tangent convention used by odometry/monitoring.
static constexpr double WHEEL_ENCODER_SIGN[WHEEL_COUNT]={-1,-1,-1};
static constexpr double WHEEL_CENTER_RADIUS_MM[WHEEL_COUNT]={159.74,159.74,159.74};
static constexpr double WHEEL_RADIUS_MM=51;
static constexpr double ENCODER_COUNTS_PER_OUTPUT_REV=1464; // 6 * 61 * quadrature 4
static constexpr double ENCODER_MM_PER_COUNT=2*robot_geometry::PI*WHEEL_RADIUS_MM/ENCODER_COUNTS_PER_OUTPUT_REV;
static constexpr double MAVERICK_MAX_NO_LOAD_RPM=112;
static constexpr double MAX_WHEEL_SPEED_MMPS=2*robot_geometry::PI*WHEEL_RADIUS_MM*MAVERICK_MAX_NO_LOAD_RPM/60;
// Retain the requested half-speed chassis limit; no wheel trim or distance scale.
static constexpr double TELEOP_SPEED_MMPS=MAX_WHEEL_SPEED_MMPS/2;
static constexpr double CONTROL_PERIOD_S=0.020,MAX_CONTROL_GAP_S=0.10;
// Autonomous can integrate a bounded late sample using the real elapsed time.
// Controller ramps use a capped step; Teleop keeps its original timing limits.
static constexpr double AUTO_MAX_SAMPLE_GAP_S=0.50,AUTO_MAX_CONTROL_STEP_S=0.04;
static constexpr double ARENA_WIDTH_MM=2000,ARENA_HEIGHT_MM=2000;
static constexpr double START_X_MM=950,START_Y_MM=900,START_HEADING_DEG=-90;

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
// Installed drivetrain cannot reliably move below this electrical duty.
// Applies to all nonzero drive commands in both modes; zero still means stop.
static constexpr double MIN_RUNNING_OUTPUT=0.15;
static constexpr double VECTOR_PEAK_FLOOR_MULTIPLIER=2;
static constexpr double COMMAND_STOP_MMPS=0.01,COMMAND_STOP_RADPS=0.0001;
static constexpr double MAX_PLAUSIBLE_SPEED_FACTOR=3;
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
static constexpr double FINAL_HEADING_TOLERANCE_DEG=1;
static constexpr double HEADING_TOLERANCE_RAD=FINAL_HEADING_TOLERANCE_DEG*robot_geometry::PI/180;
static constexpr double STATIONARY_SPEED_MMPS=8,STATIONARY_YAW_DPS=0.5;
static constexpr double HOLD_HEADING_DEADBAND_DEG=0.25;
static constexpr double HOLD_HEADING_DEADBAND_RAD=HOLD_HEADING_DEADBAND_DEG*robot_geometry::PI/180;
static constexpr double HOLD_HEADING_KP=2.4,HOLD_YAW_RATE_DAMPING=0.65;
static constexpr double HOLD_YAW_ACCEL_DPS2=180;
static constexpr double HOLD_YAW_ACCEL_RADPS2=HOLD_YAW_ACCEL_DPS2*robot_geometry::PI/180;
static constexpr double HOLD_BRAKE_RESPONSE_S=0.18,HOLD_RECHECK_S=0.060;
static constexpr double HOLD_RECHECK_WINDOW_DEG=2,HOLD_HEADING_MAX_YAW_DPS=20;
static constexpr double HOLD_RECHECK_WINDOW_RAD=HOLD_RECHECK_WINDOW_DEG*robot_geometry::PI/180;
static constexpr double HOLD_HEADING_MAX_YAW_RADPS=HOLD_HEADING_MAX_YAW_DPS*robot_geometry::PI/180;
static constexpr double YAW_DIRECTION_ERROR_DEG=5,YAW_DIRECTION_CHECK_MIN_DPS=0.1;
static constexpr double YAW_DIRECTION_ERROR_RAD=YAW_DIRECTION_ERROR_DEG*robot_geometry::PI/180;
static constexpr double YAW_DIRECTION_CHECK_MIN_RADPS=YAW_DIRECTION_CHECK_MIN_DPS*robot_geometry::PI/180;
// Motor inertia and cached IMU samples can briefly oppose a NEW yaw request.
static constexpr double YAW_RESPONSE_SETTLE_S=0.30;
}

// Layout from docs/court_barriers.png. Written dimensions take precedence over
// the drawing's scale. Divider centreline is X=1000, independently of Start X.
// These describe the scene, not a route-blocking/footprint feasibility layer.
namespace arena_config {
static constexpr double DIVIDER_X_MM=1000;
struct RestrictedRegion { double minX,minY,maxX,maxY; };
static constexpr RestrictedRegion RESTRICTED_REGIONS[]={
    {DIVIDER_X_MM-115,1220,DIVIDER_X_MM+115,2000}, // upper: 230 x 780
    {0,0,2000,230},                            // bottom: 2000 x 230
    {DIVIDER_X_MM-15,230,DIVIDER_X_MM+15,600}}; // lower strip: 30 x 370
// The horizontal wooden wall is separate from the upper restricted rectangle;
// the centre square itself is floor tape. Wall faces are inferred from the
// 600 mm span and 780 mm north-wall offset, approximately confirmed by the user.
// Keep the faces editable for refinement of the physical measurements.
static constexpr bool HORIZONTAL_WALL_REFERENCE_ENABLED=true;
static constexpr double HORIZONTAL_WALL_MIN_X_MM=DIVIDER_X_MM-300;
static constexpr double HORIZONTAL_WALL_MAX_X_MM=DIVIDER_X_MM+300;
static constexpr double HORIZONTAL_WALL_SOUTH_Y_MM=1200,HORIZONTAL_WALL_NORTH_Y_MM=1220;
}

// Minimum version: manually supplied intermediate poses and one final pose.
namespace navigation_config {
// FINAL POSE
static constexpr double POSITION_TOLERANCE_MM=10;
static constexpr double FINAL_SETTLE_S=0.30,FINAL_APPROACH_DISTANCE_MM=200;
// TRANSLATION
static constexpr double SPEED_MMPS=350,ACCEL_MMPS2=300;
// TRANSIT HEADING: hysteresis prevents repeated small corrections.
static constexpr double TRANSIT_HEADING_START_DEG=12,TRANSIT_HEADING_RELEASE_DEG=6;
static constexpr double TRANSIT_HEADING_KP=0.8;
static constexpr double TRANSIT_HEADING_START_RAD=TRANSIT_HEADING_START_DEG*robot_geometry::PI/180;
static constexpr double TRANSIT_HEADING_RELEASE_RAD=TRANSIT_HEADING_RELEASE_DEG*robot_geometry::PI/180;
// NEAR FINAL GOAL: tighter hold; final stopped yaw uses heading_control_config.
static constexpr double APPROACH_HEADING_DEADBAND_DEG=3,APPROACH_HEADING_KP=1.2;
static constexpr double APPROACH_HEADING_DEADBAND_RAD=APPROACH_HEADING_DEADBAND_DEG*robot_geometry::PI/180;
// INTERMEDIATE POSES / SMOOTHING
static constexpr double INTERMEDIATE_POSITION_TOLERANCE_MM=25,MIN_PASS_SPEED_MMPS=120;
static constexpr double INTERMEDIATE_HEADING_TOLERANCE_DEG=10;
static constexpr double INTERMEDIATE_HEADING_TOLERANCE_RAD=INTERMEDIATE_HEADING_TOLERANCE_DEG*robot_geometry::PI/180;
static constexpr double LOOKAHEAD_MM=100,CORNER_BLEND_MM=25;
static constexpr double MIN_SEGMENT_MM=1;
static constexpr unsigned MAX_INTERMEDIATE_POINTS=32;
// Conservative wall association and complementary XY correction, Auto only.
static constexpr double WALL_MAX_INNOVATION_MM=100,WALL_BLEND=0.25,WALL_MAX_STEP_MM=5;
static constexpr double WALL_HISTORY_S=0.5;
static constexpr double BRAKE_PREVIEW_S=0.25;
// Named poses and shared pass-through guides.
enum GoalId { STORAGE_RIGHT,STORAGE_LEFT,DELIVERY_RIGHT,DELIVERY_LEFT,START,GOAL_COUNT };
static constexpr double SOURCE_MATCH_RADIUS_MM=75;
static constexpr double REFERENCE_MIN_COMPONENT=0.70;
struct GoalPreset { const char* name; double xMM,yMM,headingDeg; };
static constexpr GoalPreset GOALS[GOAL_COUNT]={
    {"Storage right",1700,1700,90}, {"Storage left",300,1700,-90},
    {"Delivery right",1700,400,90}, {"Delivery left",300,400,-0},
    {"Start",localization_config::START_X_MM,localization_config::START_Y_MM,localization_config::START_HEADING_DEG}};
// Default WALL coordinates [X,Y], editable on Navigation. Sensor identity is
// calculated from the current/requested heading, not a fixed area profile.
static constexpr double STARTING_WALLS[GOAL_COUNT][2]={
    {2000,2000},{0,2000},{2000,0},{0,0},{0,arena_config::HORIZONTAL_WALL_SOUTH_Y_MM}};
enum GuideId { NO_GUIDE=-1,LEFT_GUIDE,RIGHT_GUIDE };
static constexpr GoalPreset INTERMEDIATES[2]={
    {"Intermediate left",500,900,-90}, {"Intermediate right",1500,900,-90}};
// Row = source, column = destination, in GOALS order above. NO_GUIDE means
// direct. Same-side storage/delivery transfers are direct; crossing sides uses
// source-side then destination-side guides; Start uses the other pose's side.
// Translation and rotation overlap; guides do not introduce mandatory stops.
struct RoutePreset { int guide[2]; };
static constexpr RoutePreset ROUTES[GOAL_COUNT][GOAL_COUNT]={
  { // from Storage right -> Storage right, Storage left, Delivery right, Delivery left, Start
    {{NO_GUIDE,NO_GUIDE}}, {{RIGHT_GUIDE,LEFT_GUIDE}}, {{NO_GUIDE,NO_GUIDE}},
    {{RIGHT_GUIDE,LEFT_GUIDE}}, {{RIGHT_GUIDE,NO_GUIDE}}},
  { // from Storage left
    {{LEFT_GUIDE,RIGHT_GUIDE}}, {{NO_GUIDE,NO_GUIDE}}, {{LEFT_GUIDE,RIGHT_GUIDE}},
    {{NO_GUIDE,NO_GUIDE}}, {{LEFT_GUIDE,NO_GUIDE}}},
  { // from Delivery right
    {{NO_GUIDE,NO_GUIDE}}, {{RIGHT_GUIDE,LEFT_GUIDE}}, {{NO_GUIDE,NO_GUIDE}},
    {{RIGHT_GUIDE,LEFT_GUIDE}}, {{RIGHT_GUIDE,NO_GUIDE}}},
  { // from Delivery left
    {{LEFT_GUIDE,RIGHT_GUIDE}}, {{NO_GUIDE,NO_GUIDE}}, {{LEFT_GUIDE,RIGHT_GUIDE}},
    {{NO_GUIDE,NO_GUIDE}}, {{LEFT_GUIDE,NO_GUIDE}}},
  { // from Start
    {{RIGHT_GUIDE,NO_GUIDE}}, {{LEFT_GUIDE,NO_GUIDE}}, {{RIGHT_GUIDE,NO_GUIDE}},
    {{LEFT_GUIDE,NO_GUIDE}}, {{NO_GUIDE,NO_GUIDE}}}};
static constexpr double DEFAULT_GOAL_X_MM=GOALS[0].xMM,DEFAULT_GOAL_Y_MM=GOALS[0].yMM,DEFAULT_GOAL_HEADING_DEG=GOALS[0].headingDeg;
}
// Shared final XY controller for Navigation and the tape approach.
namespace position_control_config {
static constexpr double POSITION_TOLERANCE_MM=navigation_config::POSITION_TOLERANCE_MM;
static constexpr double POSITION_KP=1.5,MIN_APPROACH_MMPS=120;
static constexpr double NEAR_TARGET_MM=60,CAPTURE_RADIUS_RATIO=0.7;
static constexpr double BRAKE_RESPONSE_S=0.18,VELOCITY_FILTER_S=0.08;
static constexpr double CORRECTION_MAX_S=0.50,CORRECTION_REST_S=0.18,WAYPOINT_SETTLE_S=0.30;
}
// Explicit-start corner reference and tape approach. Arena +X east, +Y north.
namespace tape_test_config {
static constexpr double START_HEADING_DEG=0; // north, under the new arena convention
static constexpr ArenaPoint TARGET_FROM_LEFT={220,500},TARGET_FROM_RIGHT={580,500};
static constexpr double LEFT_WALL_X_MM=0,REAR_WALL_Y_MM=0;
static constexpr double DRIVE_SPEED_MMPS=localization_config::TELEOP_SPEED_MMPS;
static constexpr double DRIVE_ACCEL_MMPS2=localization_config::TELEOP_ACCELERATION_MMPS2;
static constexpr double DRIVE_TIMEOUT_S=35;
static constexpr double SCAN_SPEED_MMPS=60,SCAN_ACCEL_MMPS2=80,CENTER_MIN_SPEED_MMPS=30,CENTER_KP_MMPS=100;
static constexpr double SCAN_MAX_TRAVEL_MM=300,SCAN_TIMEOUT_S=20,TAPE_ACQUIRE_LEVEL=0.40;
static constexpr double CENTER_BLACK_MIN=0.55,CENTER_LEVEL_TOLERANCE=0.10,CENTER_VOLTAGE_TOLERANCE_V=0.20;
static constexpr double CENTER_OUTER_CONTRAST=0.20,TAPE_SETTLE_S=0.30,TAPE_LOST_S=0.40;
static constexpr double NO_PROGRESS_TIMEOUT_S=2,PROGRESS_MM=2;
static constexpr double PROGRESS_EFFORT_MMPS=20,REFERENCE_NORMAL_COMPONENT=0.98;
}
