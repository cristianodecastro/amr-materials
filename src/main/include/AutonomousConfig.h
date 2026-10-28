#pragma once

#include "RobotGeometry.h"

// Drawing coordinates: bottom-left origin, X right, Y up, CCW heading.
// Every value marked PLACEHOLDER must be measured before autonomous operation.
namespace autonomous_config {
// Explicit operator request: autonomous motion is disabled, regardless of
// individual calibration flags or the physical Start button.
static constexpr bool AUTONOMOUS_ENABLED = false;
static constexpr bool ARENA_AND_FOOTPRINT_CONFIRMED = false;
static constexpr bool DRIVE_AND_IMU_FRAME_CONFIRMED = false;
static constexpr bool WHEEL_SPEED_CONTROL_CONFIRMED = false;
static constexpr bool SONAR_MOUNTS_CONFIRMED = false;
static constexpr bool STATION_ASSIGNMENTS_CONFIRMED = false;
static constexpr bool START_POSE_CONFIRMED = false;

static constexpr double DIVIDER_X_MIN_MM = 990.0;
static constexpr double DIVIDER_X_MAX_MM = 1010.0; // PLACEHOLDER thickness interpretation
static constexpr double LOWER_DIVIDER_END_Y_MM = 600.0;
static constexpr double UPPER_DIVIDER_START_Y_MM = 1220.0; // 2000 - 780
// Circular envelope MUST contain wheels, chassis, gripper AND carried object.
static constexpr double ROBOT_ENVELOPE_RADIUS_MM = 225.0; // PLACEHOLDER
static constexpr double CLEARANCE_MM = 40.0;
static constexpr double STATION_SIDE_MM = 100.0; // PLACEHOLDER; not dimensioned in drawing
static constexpr double STAGING_DISTANCE_MM = 480.0; // center to center; PLACEHOLDER
static constexpr double DOCK_DISTANCE_MM = 330.0; // gripper reach dependent; PLACEHOLDER

struct Station {
  const char* name;
  ArenaPoint center;
  // Forward direction of robot when facing the station, in arena coordinates.
  double headingDeg;
};
// Inferred from drawing, subject to station size and dimension reference faces.
static constexpr Station STORAGES[4] = {
    {"S1 upper left", {865.0,1675.0}, 0.0},
    {"S2 lower left", {865.0,1545.0}, 0.0},
    {"S3 upper right",{1135.0,1675.0},180.0},
    {"S4 lower right",{1135.0,1545.0},180.0}};
static constexpr Station DELIVERIES[4] = {
    {"D1",{325.0,100.0},-90.0}, {"D2",{625.0,100.0},-90.0},
    {"D3",{1375.0,100.0},-90.0},{"D4",{1675.0,100.0},-90.0}};
struct Job { int storage; int delivery; };
static constexpr Job JOBS[4] = {{0,0},{1,1},{2,2},{3,3}}; // PLACEHOLDER assignment/order
static constexpr int JOB_COUNT = 4;

// Legacy Cobra + analog module API supplied by Studica 1.1.28. No I2C access
// is attempted when absent. Confirm Cobra generation before enabling.
static constexpr bool COBRA_INSTALLED = true; // user connected analog module J1 to VMX I2C
static constexpr bool COBRA_CALIBRATED = false;
static constexpr double COBRA_REFERENCE_V = 5.0;
static constexpr double COBRA_FORWARD_MM = 180.0; // PLACEHOLDER from robot center
static constexpr double COBRA_LEFT_MM = 0.0; // PLACEHOLDER array center
static constexpr double COBRA_CHANNEL_LEFT_MM[4] = {22.5,7.5,-7.5,-22.5}; // PLACEHOLDER order/spacing
static constexpr double COBRA_FLOOR_V[4] = {4.0,4.0,4.0,4.0}; // PLACEHOLDER measured on floor
static constexpr double COBRA_TAPE_V[4] = {1.0,1.0,1.0,1.0}; // PLACEHOLDER measured on tape
static constexpr double LINE_MIN_STRENGTH = 0.55;
static constexpr double LINE_MIN_PEAK = 0.40;
static constexpr double LINE_MAX_WIDTH_MM = 32.0;
static constexpr double LINE_WINDOW_MM = 45.0; // expected-station association gate
static constexpr double LINE_KP_PER_S = 2.5;
static constexpr double LINE_MAX_LATERAL_MMPS = 70.0;
static constexpr double LINE_SEARCH_MM = 30.0;
static constexpr double LINE_SEARCH_MMPS = 30.0;
static constexpr double LINE_LOST_TIMEOUT_S = 0.25;
static constexpr double LINE_ACQUIRE_TIMEOUT_S = 6.0;

static constexpr double CRUISE_MMPS = 220.0;
static constexpr double DOCK_MMPS = 70.0;
static constexpr double ACCEL_MMPS2 = 300.0;
static constexpr double MAX_OUTPUT = 0.45;
// PLACEHOLDER PI/feedforward tuning: output fractions, measured speed in mm/s.
// Autonomous only; manual drive retains its original open-loop mixer.
static constexpr double WHEEL_KS = 0.04;
static constexpr double WHEEL_KP = 0.0008;
static constexpr double WHEEL_KI = 0.0020;
static constexpr double WHEEL_INTEGRAL_LIMIT = 0.18;
static constexpr double POSITION_KP_PER_S = 1.5;
static constexpr double HEADING_KP_PER_S = 2.5;
static constexpr double MAX_YAW_RADPS = 0.6;
static constexpr double YAW_ACCEL_RADPS2 = 1.0;
static constexpr double POSITION_TOLERANCE_MM = 12.0;
static constexpr double HEADING_TOLERANCE_RAD = 0.035;
static constexpr double SETTLE_S = 0.20;
static constexpr double WAYPOINT_OBSERVATION_S = 0.70;
static constexpr double LEG_TIMEOUT_S = 35.0;
static constexpr double ACTION_TIMEOUT_S = 8.0;
static constexpr double MISSION_TIMEOUT_S = 180.0;
static constexpr double STALL_TIMEOUT_S = 2.5;
static constexpr double PROGRESS_MM = 5.0;
static constexpr double MAX_CONTROL_GAP_S = 0.10;
static constexpr double MAX_POSITION_SIGMA_MM = 60.0;
static constexpr double MAX_HEADING_SIGMA_RAD = 0.09;
// Sonar age is handled conservatively: fuse only if slow at ping AND receipt.
static constexpr double SONAR_FUSION_MAX_SPEED_MMPS = 60.0;
static constexpr double SONAR_FUSION_MAX_YAW_DPS = 8.0;
static constexpr double SONAR_FUSION_MAX_AGE_S = 0.12;
static constexpr double SONAR_MAX_POSE_STEP_MM = 20.0;
static constexpr double SONAR_CONE_HALF_ANGLE_RAD = 0.0873;
static constexpr double SONAR_CONE_RANGE_SPREAD_MM = 60.0;
static constexpr double SONAR_STALE_S = 0.60;

inline ArenaPoint Approach(const Station& s, double distance) {
  const double a = robot_geometry::Radians(s.headingDeg);
  return {s.center.x-distance*std::cos(a), s.center.y-distance*std::sin(a)};
}
}
