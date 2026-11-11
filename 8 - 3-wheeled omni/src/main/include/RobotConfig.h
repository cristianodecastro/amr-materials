#pragma once

#include <cmath>
#include "RobotTypes.h"

// Robot motion, IO and localization settings.
namespace localization_config {

// The active Shuffleboard tab shows the essentials. Enable this only for
// extra SmartDashboard troubleshooting fields in localization mode.
static constexpr bool ENABLE_DETAILED_TELEMETRY = false;

// Left-stick translation (including diagonals) and right-stick rotation in Teleoperated
// mode. The SP -> MW -> RDW autonomous test has its own enable/start gate below.
static constexpr bool TELEOP_DRIVE_ENABLED = true;
static constexpr int DRIVER_JOYSTICK_PORT = 0;
// Common gamepad mapping; confirm with the Teleop tab/Windows game controller
// panel because USB/Bluetooth operating modes can expose different axes.
static constexpr int DRIVER_LEFT_X_AXIS = 0;
static constexpr int DRIVER_LEFT_Y_AXIS = 1;
static constexpr int DRIVER_RIGHT_X_AXIS = 2;
// Canonical body commands: +forward and +left. The installed translation
// reversal is handled once in DRIVE_FORWARD_DIRECTION_DEG for ALL callers.
// Flipping these with the shared frame preserves existing teleop wheel outputs.
static constexpr double DRIVER_LEFT_X_SIGN = 1.0;
static constexpr double DRIVER_LEFT_Y_SIGN = -1.0;
static constexpr double DRIVER_STICK_DEADBAND = 0.12;
// Paired with DRIVE_YAW_DIRECTION_SIGN below so the existing tested Teleop
// motor outputs are preserved while yaw requests use the actual CCW frame.
static constexpr double DRIVER_RIGHT_X_SIGN = 1.0;
// Cap manual translation at half the estimated no-load wheel speed and ramp
// the translation vector gently, including changes between directions.
static constexpr double TELEOP_TRANSLATION_SPEED_SCALE = 0.50;
static constexpr double TELEOP_ACCELERATION_MMPS2 = 400.0;
static constexpr double TELEOP_MAX_MOTOR_OUTPUT = 0.90;
static constexpr double TELEOP_MAX_YAW_RATE_DPS = 90.0;
// Start at zero and take two seconds to reach the 90 degree/s yaw limit.
static constexpr double TELEOP_YAW_ACCELERATION_DPS2 = 45.0;
// Joystick-side yaw multiplier, separate from installed wheel yaw polarity.
static constexpr double TELEOP_YAW_SIGN = 1.0;

// Installed wheel-layout translation is reversed relative to robot +X forward,
// +Y left. Apply this same rotation to commands AND inverse encoder odometry.
// It is a wheel-frame calibration, never an offset added to the gyro heading.
static constexpr double DRIVE_FORWARD_DIRECTION_DEG = 180.0;
// The arena run showed negative wheel yaw producing increasing gyro heading.
// Installed positive wheel rotation is therefore CW. Apply this polarity once
// in BOTH the wheel command model and its encoder inverse; never invert the
// already verified CCW gyro heading or change the -90 degree waypoint targets.
static constexpr double DRIVE_YAW_DIRECTION_SIGN = -1.0;
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
// Studica encoder distance formula. GetRaw() is counts: convert exactly once.
static constexpr double ENCODER_MM_PER_COUNT = 2.0*robot_geometry::PI*WHEEL_RADIUS_MM/
    (ENCODER_COUNTS_PER_OUTPUT_REV*MOTOR_TO_WHEEL_RATIO);
static constexpr double WHEEL_CENTER_RADIUS_MM[WHEEL_COUNT] = {
    168.0, 168.0, 168.0};

// Same 120-degree spacing in the wheel-layout frame (see shared rotation above):
// left rear 120 degrees, right rear 240 degrees, front 0 degrees.
// These angles parameterize the translation layout. Installed motor rotation
// uses DRIVE_YAW_DIRECTION_SIGN rather than assuming a CCW tangent polarity.
static constexpr double WHEEL_ANGLE_DEG[WHEEL_COUNT] = {120.0, 240.0, 0.0};
static constexpr double WHEEL_ENCODER_SIGN[WHEEL_COUNT] = {1.0, 1.0, 1.0};
static constexpr double WHEEL_MOTOR_SIGN[WHEEL_COUNT] = {1.0, 1.0, 1.0};
// Maverick 75001 speeds are already at the 61:1 gearbox OUTPUT shaft.
// MOTOR_TO_WHEEL_RATIO represents external gearing only; do not divide by 61
// again. This estimates the speed/PWM scale. Drive is currently open-loop;
// Both modes use the shared static-friction output floor. Teleop startup and smoothing settings
// are isolated in teleop_wheel_config below. Neither mode uses motor trim.
static constexpr double MAVERICK_MAX_NO_LOAD_RPM = 112.0;
static constexpr double MAVERICK_NOMINAL_NO_LOAD_RPM = 100.0;
static constexpr double MAVERICK_NOMINAL_RATED_RPM = 80.0;
static constexpr double MAX_WHEEL_SPEED_MMPS =
    2.0 * robot_geometry::PI * WHEEL_RADIUS_MM * MAVERICK_MAX_NO_LOAD_RPM /
    (60.0 * MOTOR_TO_WHEEL_RATIO);
static constexpr double TELEOP_SPEED_MMPS =
    MAX_WHEEL_SPEED_MMPS * TELEOP_TRANSLATION_SPEED_SCALE;

// Arena coordinates span X=[0,width], Y=[0,height].
static constexpr double ARENA_WIDTH_MM = 2000.0;
static constexpr double ARENA_HEIGHT_MM = 2000.0;
static constexpr double START_X_MM = 950.0;
static constexpr double START_Y_MM = 900.0;
static constexpr double START_HEADING_DEG = 0.0; // editable SP heading

// Body frame: +X forward, +Y left. User-provided center-to-sensor distances
// are treated as sensing-face offsets on the four cardinal centerlines.
static constexpr double FRONT_SENSOR_OFFSET_MM = 160.0;
static constexpr double REAR_SENSOR_OFFSET_MM = 175.0;
static constexpr double LEFT_SENSOR_OFFSET_MM = 150.0;
static constexpr double RIGHT_SENSOR_OFFSET_MM = 150.0;

// Geometry used by the retained route library, independent of sensor type.
struct RangeSensorMount {
  double mountX_MM;
  double mountY_MM;
  double beamAngleDeg;
  const char* name;
};

static constexpr RangeSensorMount RANGE_SENSOR_MOUNTS[4] = {
    {0.0, LEFT_SENSOR_OFFSET_MM, 90.0, "Left"},
    {-REAR_SENSOR_OFFSET_MM, 0.0, 180.0, "Rear"},
    {0.0, -RIGHT_SENSOR_OFFSET_MM, -90.0, "Right"},
    {FRONT_SENSOR_OFFSET_MM, 0.0, 0.0, "Front"}};

struct SonarConfig { int triggerDio,echoDio; const char* name; };
static constexpr int SONAR_COUNT=2;
// Only these two physical ultrasonic sensors are instantiated and triggered.
static constexpr SonarConfig SONARS[SONAR_COUNT] = {{6,7,"Front"},{2,3,"Rear"}};

// Sharp 2Y0A21 / GP2Y0A21YK0F: ANALOG output, 100..800 mm range.
// VMX Analog In 0 = left, Analog In 1 = right. FlexDIO 10/11 are not used.
static constexpr int IR_COUNT=2;
static constexpr int IR_ANALOG_CHANNEL[IR_COUNT]={0,1};
static_assert(IR_ANALOG_CHANNEL[0]>=0 && IR_ANALOG_CHANNEL[0]<4 &&
    IR_ANALOG_CHANNEL[1]>=0 && IR_ANALOG_CHANNEL[1]<4 &&
    IR_ANALOG_CHANNEL[0]!=IR_ANALOG_CHANNEL[1],"IR sensors require distinct VMX Analog In channels 0..3");
static constexpr double IR_MIN_RANGE_MM=100.0;
static constexpr double IR_MAX_RANGE_MM=800.0;
static constexpr double IR_MAX_INPUT_V=5.0;
static constexpr double SENSOR_UPDATE_PERIOD_S=0.050;
static constexpr double SENSOR_STALE_S=0.200;

// No physical start button is installed. FlexDIO 8 is now a servo output.

static constexpr double CONTROL_PERIOD_S = 0.020;
// Quiet time after reading one sensor, before triggering the next.
static constexpr double SONAR_PING_SPACING_S = 0.040;
// A 4000 mm echo takes about 24 ms round trip. Allow additional time for
// VMX's captured measurement to reach the Raspberry Pi through the HAL.
static constexpr double SONAR_ECHO_SETTLE_S = 0.060;
static constexpr double SONAR_MIN_RANGE_MM = 20.0;
static constexpr double SONAR_MAX_RANGE_MM = 4000.0;
static constexpr double SONAR_READING_STALE_S = 0.350;

// Autonomous position uses encoder increments only and reset-relative gyro
// heading. Teleop pose telemetry retains its separate legacy blend below.

}  // namespace localization_config

// Empirical linear-distance calibration for Autonomous ONLY. With scale 0.91,
// the latest 1000 mm task measured 1050 mm: new scale = 0.91*1050/1000.
// Keep nominal wheel geometry, motor outputs and Teleop unchanged.
namespace autonomous_odometry_config {
static constexpr double DISTANCE_SCALE = 0.9555;
static_assert(DISTANCE_SCALE>0 && DISTANCE_SCALE<=2.0,"Invalid autonomous odometry distance scale");
}

// Shared minimum for Titan DC motor commands, in BOTH operating modes.
// 15% is an initial loaded-robot calibration, not a Studica motor specification.
// A stop is always exactly zero. Servos use positional pulses, not this duty.
namespace motor_output_config {
static constexpr double MIN_RUNNING_OUTPUT = 0.15;
static_assert(MIN_RUNNING_OUTPUT>0 && MIN_RUNNING_OUTPUT<1,
    "Motor output floor must be between zero and full output");
}

// Lift and servo settings.
// Manual lift and servo configuration. Autonomous pickup remains unimplemented.
namespace manipulator_config {
// Titan Quad's printed M1 connector and its matching encoder channel.
static constexpr int TITAN_MOTOR_PORT = 1;
// Conservative manual open-loop power cap; there are currently NO end switches.
static constexpr double LIFT_OUTPUT = 0.20;
static_assert(LIFT_OUTPUT>=motor_output_config::MIN_RUNNING_OUTPUT,
    "Lift cap must allow the shared motor output floor");
static constexpr double LIFT_UP_SIGN = 1.0; // reverse if installed motor polarity lowers on positive
static constexpr int DRIVER_POV_INDEX = 0;
static constexpr int DRIVER_POV_UP_DEG = 0;
static constexpr int DRIVER_POV_DOWN_DEG = 180;
static constexpr bool LIMIT_ACTIVE_LOW = true;
// Servo order: L1/R1 manual servo / physical FlexDIO 8, triangle / physical FlexDIO 9.
// VMX PWM addressing differs from the physical FlexDIO and DigitalInput labels.
static constexpr int SERVO_PWM[2] = {18,19};
static constexpr int SERVO_TRIANGLE_BUTTON = 4; // WPILib button numbering
static constexpr double SERVO_HOME_DEG = 0.0;
static constexpr double SERVO_TRIANGLE_ANGLE_DEG = 180.0;
static constexpr int SERVO_STEP_BUTTON[2] = {5,6}; // L1 increases, R1 decreases the manual servo target
static constexpr double SERVO_STEP_DEG = 15.0;
static constexpr double SERVO_MIN_DEG = 0.0;
static constexpr double SERVO_MAX_DEG = 300.0;
static constexpr int LIMIT_DIO[2] = {-1,-1}; // PLACEHOLDER; -1 = not assigned
// Heights, homing direction, travel bounds and successful-object detection
// depend on the actual linkage and must accompany its future implementation.
}

// Motor ramp, encoder monitoring and startup settings.
namespace wheel_drive_config {
static constexpr double SPEED_FILTER_S=0.060;
static constexpr double SPEED_WINDOW_S=0.060;
static constexpr double MAX_SAMPLE_S=0.10;
static constexpr double ZERO_SPEED_MMPS=0.5;
static constexpr double STALL_REQUEST_MMPS=20.0;
static constexpr double STALL_MEASURED_MMPS=3.0;
static constexpr double STALL_OUTPUT=0.12;
static constexpr double STALL_TIMEOUT_S=1.0;
static constexpr double DIRECTION_GRACE_S=0.30;
static constexpr double WRONG_DIRECTION_TIMEOUT_S=0.25;
}

// Preserve the driver's pre-autonomous-revision wheel behaviour exclusively
// in Teleop. Autonomous bypasses this startup gate and the second output ramp.
namespace teleop_wheel_config {
static constexpr double PWM_RISE_PER_S=0.75;
static constexpr double PWM_FALL_PER_S=1.50;
static constexpr double STARTUP_CONFIRM_COUNTS=2.0;
static constexpr double STARTUP_MIN_WHEEL_RATIO=0.10;
static constexpr double STARTUP_TIMEOUT_S=2.0;
}
// Legacy manual-mode pose telemetry. It is never a steering input for Teleop
// and never contributes to Autonomous's encoder-only position estimate.
namespace teleop_odometry_config {
static constexpr double ACCEL_DEADBAND_G=0.005;
static constexpr double ACCEL_VELOCITY_BLEND=0.15;
static constexpr double ACCEL_MAX_PLAUSIBLE_G=0.5;
static constexpr double ACCEL_BIAS_TRACK_TAU_S=0.75;
static constexpr double ENCODER_STATIONARY_MMPS=3.0;
static constexpr double GYRO_STATIONARY_DPS=0.5;
static constexpr double ACCEL_MAX_VELOCITY_CORRECTION_MMPS=20.0;
static constexpr double YAW_AHRS_CORRECTION=0.25;
static constexpr double GRAVITY_MM_PER_S2=9806.65;
}

// USB still-photo settings.
namespace camera_config {
// USB video-device index, not the physical USB socket number.
static constexpr int DEVICE = 0;
static constexpr int WIDTH = 640;
static constexpr int HEIGHT = 480;
static constexpr int FPS = 15;
// Camera open/auto-exposure time is paid only after a photo request.
static constexpr double WARMUP_S = 0.30;
static constexpr double REQUEST_TIMEOUT_S = 8.0;
// Repeat the cached still at low rate for dashboard viewers/reconnections.
// This does not acquire new USB frames or save additional files.
static constexpr int PREVIEW_FPS = 1;
static constexpr const char* CAPTURE_DIRECTORY = "/home/pi/SkillsChallenge/captures";
// ROI coordinates refer to the already upright 640x480 photo, scaled for any
// other capture resolution. This patch is inside all three docs/*.png pieces,
// clear of the gripper, edges and floor. Recalibrate after camera/mount changes.
static constexpr ImageRegion COLOUR_ROI = {260,200,100,50};
// Euclidean distance between mean RGB values (0..255 per channel). These are
// initial acceptance limits, not probabilities; unknown/ambiguous crops reject.
static constexpr double COLOUR_MAX_DISTANCE = 35.0;
static constexpr double COLOUR_MIN_MARGIN = 12.0;
static constexpr double COLOUR_MAX_STDDEV = 25.0;
static constexpr int COLOUR_REFERENCE_COUNT = 8; // increase to reserve more slots
// Fixed dashboard cards map to the existing persisted slots; never reorder the
// reference file just to change the display order. Remaining slots stay available
// through the advanced Colour calibration tab.
static constexpr int COLOUR_CARD_COUNT = 5;
static constexpr int COLOUR_CARD_SLOT[COLOUR_CARD_COUNT] = {1,2,3,0,4};
static constexpr const char* COLOUR_CARD_TITLE[COLOUR_CARD_COUNT] = {
    "Green","Yellow","Blue","Red","Surprise colour"};
// RGB averages measured over the ROI in docs. Red label confirmed by user.
// Blue has no supplied sample and must be calibrated; spare slots accept ANY
// name and colour, including a competition surprise colour, without new code.
static const ColourReference DEFAULT_COLOUR_REFERENCES[COLOUR_REFERENCE_COUNT] = {
    {"Red",{105.0694,33.7314,7.8134},true},  // piece-1750888872955429-1562-3.png
    {"Green",{17.7432,73.8264,35.2924},true}, // piece-1750888647575289-1562-2.png
    {"Yellow",{166.1108,136.0396,33.9726},true}, // piece-1750888096651251-1562-1.png
    {"Blue",{0,0,0},false},
    {"Surprise colour",{0,0,0},false},{"Surprise 2",{0,0,0},false},
    {"Surprise 3",{0,0,0},false},{"Surprise 4",{0,0,0},false}};
static constexpr const char* COLOUR_CALIBRATION_FILE = "/home/pi/SkillsChallenge/colour-calibration.txt";
}

// Disabled autonomous arena, mission and correction settings.
// Drawing coordinates: bottom-left origin, X right, Y up, CCW heading.
// Every value marked PLACEHOLDER must be measured before autonomous operation.
namespace autonomous_config {
// Explicit operator request: autonomous motion is disabled, regardless of
// individual calibration flags or the physical Start button.
static constexpr bool AUTONOMOUS_ENABLED = false;
static constexpr bool MANIPULATOR_READY = false; // automatic pickup/release not implemented
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
// Center-path clearance only; no assumed circular robot footprint.
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

// Installed Cobra analog module J1 uses VMX I2C through Studica 1.1.28.
// Runtime always uses the supplied measured calibration below. No file or
// dashboard capture overrides it; fresh valid I2C readings are still required.
static constexpr bool COBRA_INSTALLED = true; // user connected analog module J1 to VMX I2C
static constexpr bool COBRA_CALIBRATED = true;
static constexpr double COBRA_REFERENCE_V = 5.0;
static constexpr int COBRA_I2C_ADDRESS = 0x48; // analog module default address
static constexpr double COBRA_FORWARD_MM = 180.0; // PLACEHOLDER from robot center
static constexpr double COBRA_LEFT_MM = 0.0; // PLACEHOLDER array center
static constexpr double COBRA_CHANNEL_LEFT_MM[4] = {22.5,7.5,-7.5,-22.5}; // PLACEHOLDER order/spacing
// Authoritative user measurements, CH0 leftmost -> CH3 rightmost.
static constexpr double COBRA_FLOOR_V[4] = {2.172852,1.005859,1.257324,2.868652};
static constexpr double COBRA_TAPE_V[4] = {4.997559,4.997559,4.997559,4.997559};
static constexpr const char* COBRA_CALIBRATION_DIRECTORY="/home/pi/SkillsChallenge";
static constexpr const char* COBRA_CALIBRATION_FILE="/home/pi/SkillsChallenge/cobra-calibration.txt";
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
// Command acceleration limits and encoder monitoring live in TeleopControl.h.
static constexpr double POSITION_KP_PER_S = 1.5;
static constexpr double HEADING_KP_PER_S = 2.5;
static constexpr double MAX_YAW_RADPS = 0.6;
static constexpr double YAW_ACCEL_RADPS2 = 1.0;
static constexpr double POSITION_TOLERANCE_MM = 15.0;
static constexpr double HEADING_TOLERANCE_RAD = robot_geometry::PI/180.0;
static constexpr double SETTLE_S = 0.20;
static constexpr double WAYPOINT_OBSERVATION_S = 0.70;
static constexpr double LEG_TIMEOUT_S = 35.0;
static constexpr double ACTION_TIMEOUT_S = 8.0;
static constexpr double MISSION_TIMEOUT_S = 180.0;
static constexpr double STALL_TIMEOUT_S = 2.5;
static constexpr double PROGRESS_MM = 5.0;
static constexpr double MAX_CONTROL_GAP_S = 0.10;
static constexpr double SONAR_STALE_S = 0.60;

inline ArenaPoint Approach(const Station& s, double distance) {
  const double a = robot_geometry::Radians(s.headingDeg);
  return {s.center.x-distance*std::cos(a), s.center.y-distance*std::sin(a)};
}
} // namespace autonomous_config

// First autonomous test; independent of the future pickup/delivery mission.
namespace arena_test_config {
static constexpr bool ENABLED = true;
struct Waypoint { double x_MM, y_MM, headingDeg; };
static constexpr Waypoint SP = {950.0,900.0,0.0};
static constexpr Waypoint MW = {250.0,900.0,-90.0};
static constexpr Waypoint RDW = {250.0,1780.0,-90.0};
// Headings use [-180, 180] degrees: 0 faces east, -90 faces south.
// Travel SP -> MW at SP heading, then turn to MW heading. Travel MW -> RDW
// at MW heading, then settle at RDW heading. All coordinates are millimeters.
// Share the tested teleop limits instead of maintaining slower duplicate values.
// The autonomous position controller still slows down when approaching a target.
static constexpr double SPEED_MMPS = localization_config::TELEOP_SPEED_MMPS;
static constexpr double ACCEL_MMPS2 = localization_config::TELEOP_ACCELERATION_MMPS2;
static constexpr double MAX_OUTPUT = localization_config::TELEOP_MAX_MOTOR_OUTPUT;
static constexpr double MAX_YAW_RADPS = localization_config::TELEOP_MAX_YAW_RATE_DPS *
    robot_geometry::PI / 180.0;
static constexpr double YAW_ACCEL_RADPS2 = localization_config::TELEOP_YAW_ACCELERATION_DPS2 *
    robot_geometry::PI / 180.0;
static constexpr double POSITION_KP = 1.0;
static constexpr double HEADING_KP = 1.5;
// Moving motors need a usable request near the goal. These floors apply only
// outside the completion tolerances, and never bypass startup slew limits.
static constexpr double MIN_APPROACH_MMPS = 120.0;
static constexpr double MIN_TURN_YAW_RADPS = 30.0*robot_geometry::PI/180.0;
// Gyro-based turn stopping prediction: use the angular acceleration limit as
// the braking deceleration, plus a short allowance for sampling/motor response.
// A minimum moving request must never override the need to stop near the goal.
static constexpr double TURN_BRAKE_RESPONSE_S = 0.10;
// Stop a reversed feedback loop after accumulated opposing gyro rotation,
// including short attempts separated by braking. No extra rotation is issued
// for this check, and no automatic sign changes are made while running.
static constexpr double YAW_DIRECTION_ERROR_RAD = 5.0*robot_geometry::PI/180.0;
static constexpr double YAW_DIRECTION_CHECK_MIN_RADPS = 0.1*robot_geometry::PI/180.0;
static constexpr double POSITION_TOLERANCE_MM = autonomous_config::POSITION_TOLERANCE_MM;
static constexpr double HEADING_TOLERANCE_RAD = autonomous_config::HEADING_TOLERANCE_RAD;
// Autonomous hold reacts before the 1 degree completion limit is exceeded.
// It must not inherit the driver's deliberately slow rotation ramp.
static constexpr double HOLD_HEADING_DEADBAND_RAD = 0.25*robot_geometry::PI/180.0;
static constexpr double HOLD_HEADING_KP = 3.0;
static constexpr double HOLD_YAW_RATE_DAMPING = 0.65;
static constexpr double HOLD_YAW_ACCEL_RADPS2 = 180.0*robot_geometry::PI/180.0;
static constexpr double HOLD_BRAKE_RESPONSE_S = 0.18;
// After releasing turning power near the reference, allow a fresh IMU pose
// before another minimum-PWM correction. No delay before the first correction.
static constexpr double HOLD_RECHECK_S = 0.060;
static constexpr double HOLD_RECHECK_WINDOW_RAD = 2.0*robot_geometry::PI/180.0;
static constexpr double HOLD_HEADING_MAX_YAW_RADPS = 20.0*robot_geometry::PI/180.0;
static constexpr double SETTLE_S = 0.40;
static constexpr double LEG_TIMEOUT_S = 40.0;
static constexpr double ROUTINE_TIMEOUT_S = 120.0;
static constexpr double STATIONARY_SPEED_MMPS = 8.0;
static constexpr double STATIONARY_YAW_DPS = 0.5;
} // namespace arena_test_config

// Project 8 corner-reference / Cobra alignment test. Arena +X is right/east,
// +Y forward/north on the drawing, with CCW-positive heading. The operator
// aligns the robot facing +Y; gyro yaw is reset-relative to this known heading.
namespace tape_test_config {
static constexpr double START_HEADING_DEG=90.0;
static constexpr ArenaPoint TARGET_FROM_LEFT={220.0,500.0};
static constexpr ArenaPoint TARGET_FROM_RIGHT={580.0,500.0};
static constexpr double LEFT_WALL_X_MM=0,REAR_WALL_Y_MM=0;
static constexpr int REFERENCE_SAMPLES=3;
static constexpr double REFERENCE_MAX_SPREAD_MM=15.0;
static constexpr double REFERENCE_TIMEOUT_S=6.0;
static constexpr double REFERENCE_MAX_PAIR_AGE_S=0.10;
static constexpr double DRIVE_SPEED_MMPS=localization_config::TELEOP_SPEED_MMPS;
static constexpr double DRIVE_ACCEL_MMPS2=localization_config::TELEOP_ACCELERATION_MMPS2;
static constexpr double POSITION_TOLERANCE_MM=15.0;
static constexpr double POSITION_KP=1.5;
static constexpr double MIN_APPROACH_MMPS=120.0;
static constexpr double CAPTURE_RADIUS_MM=7.0; // aim inside the 15 mm acceptance radius
static constexpr double NEAR_TARGET_MM=60.0;
// Use the achievable commanded deceleration, not the previous optimistic
// 600 mm/s^2. Include velocity-estimate and motor response in stopping distance.
static constexpr double BRAKE_ACCEL_MMPS2=DRIVE_ACCEL_MMPS2;
static constexpr double BRAKE_RESPONSE_S=0.18;
static constexpr double VELOCITY_FILTER_S=0.08;
static constexpr double CORRECTION_MAX_S=0.50;
static constexpr double CORRECTION_REST_S=0.18;
static constexpr double WAYPOINT_SETTLE_S=0.30;
static constexpr double DRIVE_TIMEOUT_S=35.0;
static constexpr double SCAN_SPEED_MMPS=60.0;
static constexpr double SCAN_ACCEL_MMPS2=80.0;
static constexpr double CENTER_MIN_SPEED_MMPS=30.0;
static constexpr double CENTER_KP_MMPS=100.0;
static constexpr double SCAN_MAX_TRAVEL_MM=300.0;
static constexpr double SCAN_TIMEOUT_S=20.0;
static constexpr double TAPE_ACQUIRE_LEVEL=0.40;
static constexpr double CENTER_BLACK_MIN=0.55;
static constexpr double CENTER_LEVEL_TOLERANCE=0.10;
static constexpr double CENTER_VOLTAGE_TOLERANCE_V=0.20;
static constexpr double CENTER_OUTER_CONTRAST=0.20;
static constexpr double TAPE_SETTLE_S=0.30;
static constexpr double TAPE_LOST_S=0.40;
static constexpr double NO_PROGRESS_TIMEOUT_S=2.0;
static constexpr double PROGRESS_MM=2.0;
}
