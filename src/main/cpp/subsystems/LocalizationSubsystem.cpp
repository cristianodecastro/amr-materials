#include "subsystems/LocalizationSubsystem.h"
#include "DriveKinematics.h"

#include <frc/RobotState.h>
#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <algorithm>
#include <string>

namespace {
ForwardDriveCalibration MakeForwardCalibration() {
  ForwardDriveCalibration calibration{};
  calibration.millimetresPerCount =
      (2.0 * localization_config::PI * localization_config::WHEEL_RADIUS_MM) /
      (localization_config::ENCODER_COUNTS_PER_OUTPUT_REV *
       localization_config::MOTOR_TO_WHEEL_RATIO);
  calibration.mappingConfirmed =
      localization_config::FORWARD_WHEEL_MAPPING_CONFIRMED;
  for (int wheel = 0; wheel < 3; ++wheel) {
    calibration.forwardCoefficient[wheel] = drive_kinematics::TravelCoefficient(
        wheel, localization_config::FORWARD_TEST_DIRECTION_DEG);
    calibration.encoderSign[wheel] = localization_config::WHEEL_ENCODER_SIGN[wheel];
  }
  return calibration;
}
}  // namespace

LocalizationSubsystem::LocalizationSubsystem()
    : forwardDrive_(MakeForwardCalibration()) {
  lastLoopTime_ = frc::Timer::GetFPGATimestamp();
  lastSonarPingTime_ = lastLoopTime_;
  hardware_.StopDrive();
  InitializeDashboard();
  if (!localization_config::TELEOP_DRIVE_ENABLED &&
      !localization_config::FORWARD_TEST_ENABLED) PublishDiagnostics();
}

void LocalizationSubsystem::InitializeDashboard() {
  if (localization_config::TELEOP_DRIVE_ENABLED) {
    InitializeTeleopDashboard();
    return;
  }
  if (localization_config::FORWARD_TEST_ENABLED) {
    InitializeForwardDashboard();
    return;
  }
  auto& tab = frc::Shuffleboard::GetTab("Robot");

  // Four fixed rows keep the initial evaluation screen within one view.
  // Row 0: robot readiness. No dashboard widget commands robot motion.
  tab.AddString("Robot state", [this]() -> std::string {
    if (!frc::RobotState::IsEnabled()) return "Disabled - enable for sonars";
    if (!hardware_.IsImuConnected()) return "Waiting for IMU connection";
    if (hardware_.IsImuCalibrating()) return "Keep still: IMU calibrating";
    return started_ ? "Localization running" : "Sensors active - pose not started";
  }).WithPosition(0, 0).WithSize(4, 1);
  tab.AddBoolean("Enabled", [] { return frc::RobotState::IsEnabled(); })
      .WithPosition(4, 0).WithSize(2, 1);
  tab.AddBoolean("IMU ready", [this] {
    return hardware_.IsImuConnected() && !hardware_.IsImuCalibrating();
  }).WithPosition(6, 0).WithSize(2, 1);
  tab.AddBoolean("Start button", [this] { return hardware_.GetStartPressed(); })
      .WithPosition(8, 0).WithSize(2, 1);
  tab.AddString("Sonars", [this]() -> std::string {
    if (!frc::RobotState::IsEnabled()) return "Disabled";
    int valid = 0;
    int firstInvalid = -1;
    for (int sensor = 0; sensor < 4; ++sensor) {
      if (sonarRangeValid_[sensor]) {
        ++valid;
      } else if (firstInvalid < 0) {
        firstInvalid = sensor;
      }
    }
    if (firstInvalid < 0) return "4/4 valid";

    const char* names[4] = {"Rear left", "Rear right", "Left", "Right"};
    const char* reason = sonarLastReadTime_S_[firstInvalid] < 0.0
        ? "waiting"
        : (sonarEchoReceived_[firstInvalid] ? "out of range" : "no echo");
    return std::to_string(valid) + "/4; " + names[firstInvalid] + ": " + reason;
  }).WithPosition(10, 0).WithSize(2, 1);

  // Row 1: a distance of -1 means disabled, no echo, or outside usable range.
  const char* sonarTitles[4] = {
      "Rear left (mm)", "Rear right (mm)", "Left (mm)", "Right (mm)"};
  for (int sensor = 0; sensor < 4; ++sensor) {
    tab.AddNumber(sonarTitles[sensor], [this, sensor] {
      return sonarRangeValid_[sensor] ? sonarRanges_MM_[sensor] : -1.0;
    }).WithPosition(sensor * 3, 1).WithSize(3, 1);
  }

  // Row 2: arena pose and body motion. Pose uses configured start values
  // until the physical Start button begins localization.
  tab.AddNumber("X (mm)", [this] { return localization_.GetPose().x_MM; })
      .WithPosition(0, 2).WithSize(2, 1);
  tab.AddNumber("Y (mm)", [this] { return localization_.GetPose().y_MM; })
      .WithPosition(2, 2).WithSize(2, 1);
  tab.AddNumber("Heading (deg)", [this] {
    return localization_.GetPose().headingRad * 180.0 / localization_config::PI;
  }).WithPosition(4, 2).WithSize(2, 1);
  tab.AddNumber("Vx (mm/s)", [this] {
    return localization_.GetBodyVelocityX_MMps();
  }).WithPosition(6, 2).WithSize(2, 1);
  tab.AddNumber("Vy (mm/s)", [this] {
    return localization_.GetBodyVelocityY_MMps();
  }).WithPosition(8, 2).WithSize(2, 1);
  tab.AddNumber("Yaw rate (deg/s)", [this] {
    return hardware_.GetYawRateDegreesPerSecond();
  }).WithPosition(10, 2).WithSize(2, 1);

  // Row 3: raw wheel counts and the IMU inputs used by localization.
  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    tab.AddNumber("Encoder " + std::to_string(wheel) + " (counts)",
                  [this, wheel] { return hardware_.GetWheelEncoderRaw(wheel); })
        .WithPosition(wheel * 2, 3).WithSize(2, 1);
  }
  tab.AddNumber("IMU yaw (deg)", [this] { return hardware_.GetYawDegrees(); })
      .WithPosition(6, 3).WithSize(2, 1);
  tab.AddNumber("Accel X (g)", [this] { return hardware_.GetWorldAccelX_G(); })
      .WithPosition(8, 3).WithSize(2, 1);
  tab.AddNumber("Accel Y (g)", [this] { return hardware_.GetWorldAccelY_G(); })
      .WithPosition(10, 3).WithSize(2, 1);
}

void LocalizationSubsystem::InitializeTeleopDashboard() {
  auto& tab = frc::Shuffleboard::GetTab("Teleop");
  tab.AddString("Drive status", [this] { return teleopStatus_; })
      .WithPosition(0, 0).WithSize(6, 1);
  tab.AddBoolean("Enabled", [] { return frc::RobotState::IsEnabled(); })
      .WithPosition(6, 0).WithSize(2, 1);
  tab.AddBoolean("Controller", [this] { return driverConnected_; })
      .WithPosition(8, 0).WithSize(2, 1);
  tab.AddBoolean("Stick armed", [this] { return teleopArmed_; })
      .WithPosition(10, 0).WithSize(2, 1);
  tab.AddNumber("Left stick X", [this] { return driverRawX_; })
      .WithPosition(0, 1).WithSize(3, 1);
  tab.AddNumber("Left stick Y", [this] { return driverRawY_; })
      .WithPosition(3, 1).WithSize(3, 1);
  tab.AddNumber("Forward request (mm/s)", [this] { return teleopForward_MMps_; })
      .WithPosition(6, 1).WithSize(3, 1);
  tab.AddNumber("Left request (mm/s)", [this] { return teleopLeft_MMps_; })
      .WithPosition(9, 1).WithSize(3, 1);
  for (int wheel = 0; wheel < 3; ++wheel) {
    const std::string port = "M" + std::to_string(
        localization_config::WHEEL_TITAN_PORT[wheel]);
    tab.AddNumber(port + " encoder (counts)",
                  [this, wheel] { return hardware_.GetWheelEncoderRaw(wheel); })
        .WithPosition(wheel * 2, 2).WithSize(2, 1);
    tab.AddNumber(port + " output (%)",
                  [this, wheel] { return hardware_.GetWheelCommand(wheel) * 100.0; })
        .WithPosition(6 + wheel * 2, 2).WithSize(2, 1);
  }
}

void LocalizationSubsystem::UpdateTeleopDrive(double dt) {
  auto stop = [this](const char* message, bool disarm) {
    hardware_.StopDrive();
    teleopForward_MMps_ = 0.0;
    teleopLeft_MMps_ = 0.0;
    if (disarm) teleopArmed_ = false;
    teleopStatus_ = message;
  };
  auto& ds = frc::DriverStation::GetInstance();
  const bool attached = ds.IsDSAttached();
  const int xAxis = localization_config::DRIVER_LEFT_X_AXIS;
  const int yAxis = localization_config::DRIVER_LEFT_Y_AXIS;
  const bool validAxes = xAxis >= 0 && yAxis >= 0 && xAxis != yAxis;
  const int axisCount = attached ? driver_.GetAxisCount() : 0;
  driverConnected_ = validAxes && axisCount > std::max(xAxis, yAxis);
  driverRawX_ = driverConnected_ ? driver_.GetRawAxis(xAxis) : 0.0;
  driverRawY_ = driverConnected_ ? driver_.GetRawAxis(yAxis) : 0.0;
  if (!attached || !driverConnected_) {
    stop(attached ? "Controller missing - check PC connection/port 0"
                  : "Control Station disconnected", true);
    return;
  }
  if (!std::isfinite(driverRawX_) || !std::isfinite(driverRawY_) ||
      std::fabs(driverRawX_) > 1.01 || std::fabs(driverRawY_) > 1.01) {
    stop("Invalid stick input", true);
    return;
  }
  if (!frc::RobotState::IsEnabled() || !frc::RobotState::IsOperatorControl() ||
      frc::RobotState::IsTest()) {
    stop("Disabled - select and enable Teleoperated mode", true);
    return;
  }
  const double deadband = localization_config::DRIVER_STICK_DEADBAND;
  if (!(deadband >= 0.0 && deadband < 1.0) ||
      !std::isfinite(localization_config::TELEOP_FORWARD_DIRECTION_DEG) ||
      !(localization_config::TELEOP_SPEED_MMPS > 0.0) ||
      !std::isfinite(localization_config::TELEOP_SPEED_MMPS) ||
      !(localization_config::TELEOP_ACCELERATION_MMPS2 > 0.0) ||
      !std::isfinite(localization_config::TELEOP_ACCELERATION_MMPS2) ||
      std::fabs(localization_config::DRIVER_LEFT_X_SIGN) != 1.0 ||
      std::fabs(localization_config::DRIVER_LEFT_Y_SIGN) != 1.0) {
    stop("Invalid teleop configuration", true);
    return;
  }
  const double magnitude = std::hypot(driverRawX_, driverRawY_);
  if (!teleopArmed_) {
    if (magnitude > deadband) {
      stop("Center left stick to arm", true);
      return;
    }
    teleopArmed_ = true;
  }
  if (magnitude <= deadband) {
    // Releasing the stick commands zero immediately, without a ramp-down.
    stop("Ready - move left stick", false);
    return;
  }
  // Radial deadband preserves diagonal direction. Clamp vector magnitude so
  // a square-range gamepad cannot request sqrt(2) times the straight speed.
  const double speed = (std::min(1.0, magnitude) - deadband) /
      (1.0 - deadband) * localization_config::TELEOP_SPEED_MMPS;
  const double requestedForward = driverRawY_ *
      localization_config::DRIVER_LEFT_Y_SIGN / magnitude * speed;
  const double requestedLeft = -driverRawX_ *
      localization_config::DRIVER_LEFT_X_SIGN / magnitude * speed;
  const double deltaForward = requestedForward - teleopForward_MMps_;
  const double deltaLeft = requestedLeft - teleopLeft_MMps_;
  const double delta = std::hypot(deltaForward, deltaLeft);
  const double step = localization_config::TELEOP_ACCELERATION_MMPS2 *
      (std::isfinite(dt) ? std::max(0.0, std::min(0.050, dt)) : 0.0);
  const double fraction = delta > step && delta > 0.0 ? step / delta : 1.0;
  teleopForward_MMps_ += deltaForward * fraction;
  teleopLeft_MMps_ += deltaLeft * fraction;
  const double direction = localization_config::TELEOP_FORWARD_DIRECTION_DEG *
      localization_config::PI / 180.0;
  const double c = std::cos(direction);
  const double s = std::sin(direction);
  // Driver frame: +forward, +left. Rotate both axes into the wheel-layout
  // frame using the same mounting correction as the last forward exercise.
  hardware_.DriveRobotRelative(
      c * teleopForward_MMps_ - s * teleopLeft_MMps_,
      s * teleopForward_MMps_ + c * teleopLeft_MMps_, 0.0,
      localization_config::TELEOP_MAX_MOTOR_OUTPUT);
  teleopStatus_ = "Driving - left stick translation";
}

void LocalizationSubsystem::InitializeForwardDashboard() {
  auto& tab = frc::Shuffleboard::GetTab("Forward 1m");
  tab.AddString("Run status", [this]() -> std::string {
    std::string status = forwardDrive_.GetStatus();
    if (forwardDrive_.GetFaultWheel() >= 0)
      status += " (M" + std::to_string(
          localization_config::WHEEL_TITAN_PORT[forwardDrive_.GetFaultWheel()]) + ")";
    if (!frc::RobotState::IsEnabled()) {
      return forwardDrive_.GetState() == ForwardDriveController::State::Ready
          ? "Disabled - enable Teleoperated mode" : status + " | Disabled";
    }
    if (!frc::RobotState::IsOperatorControl() || frc::RobotState::IsTest())
      return status + " | Select Teleoperated mode";
    return status;
  }).WithPosition(0, 0).WithSize(6, 1);
  tab.AddBoolean("Enabled", [] { return frc::RobotState::IsEnabled(); })
      .WithPosition(6, 0).WithSize(2, 1);
  tab.AddBoolean("Start button", [this] { return hardware_.GetStartPressed(); })
      .WithPosition(8, 0).WithSize(2, 1);
  tab.AddBoolean("Complete", [this] { return forwardDrive_.IsComplete(); })
      .WithPosition(10, 0).WithSize(2, 1);

  tab.AddNumber("Travel (mm)", [this] { return forwardDrive_.GetDistanceMM(); })
      .WithPosition(0, 1).WithSize(3, 1);
  tab.AddNumber("Target (mm)", [] { return localization_config::FORWARD_TARGET_MM; })
      .WithPosition(3, 1).WithSize(3, 1);
  tab.AddNumber("Remaining (mm)", [this] { return forwardDrive_.GetRemainingMM(); })
      .WithPosition(6, 1).WithSize(3, 1);
  tab.AddNumber("Elapsed (s)", [this] { return forwardDrive_.GetElapsedSeconds(); })
      .WithPosition(9, 1).WithSize(3, 1);

  for (int wheel = 0; wheel < 3; ++wheel) {
    const std::string port = "M" + std::to_string(
        localization_config::WHEEL_TITAN_PORT[wheel]);
    tab.AddNumber(port + " encoder (counts)",
                  [this, wheel] { return hardware_.GetWheelEncoderRaw(wheel); })
        .WithPosition(wheel * 2, 2).WithSize(2, 1);
    tab.AddNumber(port + " output (%)",
                  [this, wheel] { return hardware_.GetWheelCommand(wheel) * 100.0; })
        .WithPosition(6 + wheel * 2, 2).WithSize(2, 1);
  }
}

void LocalizationSubsystem::Periodic() {
  const double now = frc::Timer::GetFPGATimestamp();
  const double dt = now - lastLoopTime_;
  lastLoopTime_ = now;
  if (localization_config::TELEOP_DRIVE_ENABLED) {
    UpdateTeleopDrive(dt);
    return;
  }
  if (localization_config::FORWARD_TEST_ENABLED) {
    const bool motionAllowed = frc::RobotState::IsEnabled() &&
        frc::RobotState::IsOperatorControl() && !frc::RobotState::IsTest();
    const double counts[3] = {hardware_.GetWheelEncoderRaw(0),
                              hardware_.GetWheelEncoderRaw(1),
                              hardware_.GetWheelEncoderRaw(2)};
    forwardDrive_.Update(motionAllowed, hardware_.GetStartPressed(), now, counts);
    if (motionAllowed && forwardDrive_.IsDriving()) {
      const double wheelSpeed[3] = {forwardDrive_.GetWheelCommandMMps(0),
                                     forwardDrive_.GetWheelCommandMMps(1),
                                     forwardDrive_.GetWheelCommandMMps(2)};
      hardware_.DriveWheelsMMps(wheelSpeed,
                                localization_config::FORWARD_MAX_MOTOR_OUTPUT);
    } else {
      hardware_.StopDrive();
    }
    // No sonar polling, sensor-fusion update, or IMU-readiness requirement
    // participates in this exercise. Distance comes only from Titan counts.
    return;
  }
  hardware_.StopDrive();  // Localization evaluation mode holds the motors still.

  if (!started_) {
    const bool imuReady = hardware_.IsImuConnected() &&
                          !hardware_.IsImuCalibrating();
    if (hardware_.GetStartPressed() && imuReady) {
      ++startButtonDebounceCount_;
    } else {
      startButtonDebounceCount_ = 0;
    }

    if (startButtonDebounceCount_ >= 3) BeginLocalization();
  }

  if (started_) localization_.UpdateOdometry(dt);

  // VMX's HAL watchdog blocks FlexDIO outputs while the robot is disabled.
  // Enable in the Control Station to produce trigger pulses. The physical
  // start button is independent: before start, ranges are diagnostic only.
  UpdateSonarPoll(now, frc::RobotState::IsEnabled());
  PublishDiagnostics();
  if (started_) localization_.PublishTelemetry(lastSonarRange_MM_, lastSonarIndex_);
}

void LocalizationSubsystem::BeginLocalization() {
  hardware_.ZeroYaw();
  hardware_.ResetDriveEncoders();
  sonarAwaitingEcho_ = false;
  nextSonarIndex_ = 0;
  for (int sensor = 0; sensor < 4; ++sensor) {
    sonarRanges_MM_[sensor] = -1.0;
    sonarRangeValid_[sensor] = false;
    sonarEchoReceived_[sensor] = false;
    sonarCorrectionAccepted_[sensor] = false;
    sonarLastReadTime_S_[sensor] = -1.0;
    sonarStatus_[sensor] = "Waiting for ping";
  }
  localization_.Reset(localization_config::START_X_MM,
                      localization_config::START_Y_MM,
                      localization_config::START_HEADING_DEG);
  started_ = true;
  lastSonarPingTime_ = frc::Timer::GetFPGATimestamp();
}

void LocalizationSubsystem::UpdateSonarPoll(double nowSeconds,
                                             bool robotEnabled) {
  if (!robotEnabled) {
    // Discard an interrupted measurement; never reuse its counter after
    // re-enabling. The next Ping() resets that counter before triggering.
    sonarAwaitingEcho_ = false;
    sonarPollingEnabled_ = false;
    nextSonarIndex_ = 0;
    lastSonarPingTime_ = nowSeconds;
    for (int sensor = 0; sensor < 4; ++sensor) {
      sonarRanges_MM_[sensor] = -1.0;
      sonarRangeValid_[sensor] = false;
      sonarEchoReceived_[sensor] = false;
      sonarCorrectionAccepted_[sensor] = false;
      sonarStatus_[sensor] = "Blocked: robot disabled (VMX IO watchdog)";
    }
    return;
  }

  if (!sonarPollingEnabled_) {
    sonarPollingEnabled_ = true;
    for (int sensor = 0; sensor < 4; ++sensor) {
      sonarStatus_[sensor] = "Waiting for ping";
    }
  }

  if (sonarAwaitingEcho_) {
    if (nowSeconds - sonarPingTime_ <
        localization_config::SONAR_ECHO_SETTLE_S) {
      return;
    }

    sonarEchoReceived_[nextSonarIndex_] =
        hardware_.IsSonarRangeValid(nextSonarIndex_);
    sonarRangeValid_[nextSonarIndex_] = false;
    sonarCorrectionAccepted_[nextSonarIndex_] = false;
    sonarLastReadTime_S_[nextSonarIndex_] = nowSeconds;
    if (sonarEchoReceived_[nextSonarIndex_]) {
      ++sonarEchoResponses_[nextSonarIndex_];
      const double range_MM = hardware_.GetSonarRangeMM(nextSonarIndex_);
      sonarRanges_MM_[nextSonarIndex_] =
          std::isfinite(range_MM) ? range_MM : -1.0;
      sonarRangeValid_[nextSonarIndex_] =
          std::isfinite(range_MM) &&
          range_MM >= localization_config::SONAR_MIN_RANGE_MM &&
          range_MM <= localization_config::SONAR_MAX_RANGE_MM;
      sonarStatus_[nextSonarIndex_] = sonarRangeValid_[nextSonarIndex_]
          ? "OK" : "Echo outside sensor range (20..4000 mm)";
      if (started_ && sonarRangeValid_[nextSonarIndex_]) {
        sonarCorrectionAccepted_[nextSonarIndex_] =
            localization_.CorrectFromSonar(nextSonarIndex_, range_MM);
      }
      if (sonarCorrectionAccepted_[nextSonarIndex_]) {
        lastSonarRange_MM_ = range_MM;
        lastSonarIndex_ = nextSonarIndex_;
      }
    } else {
      sonarRanges_MM_[nextSonarIndex_] = -1.0;
      ++sonarTimeouts_[nextSonarIndex_];
      sonarStatus_[nextSonarIndex_] = "No echo after trigger request";
    }

    nextSonarIndex_ = (nextSonarIndex_ + 1) % 4;
    sonarAwaitingEcho_ = false;
    lastSonarPingTime_ = nowSeconds;
    return;
  }

  if (nowSeconds - lastSonarPingTime_ >=
      localization_config::SONAR_PING_SPACING_S) {
    hardware_.PingSonar(nextSonarIndex_);
    ++sonarPingRequests_[nextSonarIndex_];
    sonarStatus_[nextSonarIndex_] = "Waiting for echo";
    sonarPingTime_ = nowSeconds;
    sonarAwaitingEcho_ = true;
  }
}

void LocalizationSubsystem::PublishDiagnostics() {
  if (!localization_config::ENABLE_DETAILED_TELEMETRY) return;

  const bool imuConnected = hardware_.IsImuConnected();
  const bool imuCalibrating = hardware_.IsImuCalibrating();
  const bool startPressed = hardware_.GetStartPressed();
  const bool robotEnabled = frc::RobotState::IsEnabled();
  const double now = frc::Timer::GetFPGATimestamp();

  frc::SmartDashboard::PutBoolean("Diagnostics/Status/code_alive", true);
  frc::SmartDashboard::PutString("Diagnostics/Status/software_version",
                                  "sonar-watchdog-v1");
  frc::SmartDashboard::PutNumber("Diagnostics/Status/uptime_s", now);
  frc::SmartDashboard::PutBoolean("Diagnostics/Status/robot_enabled",
                                  robotEnabled);
  frc::SmartDashboard::PutString("Diagnostics/Ultrasonic/status",
      robotEnabled ? "Polling; start button not required"
                   : "Enable robot in Control Station: VMX blocks triggers while disabled");
  frc::SmartDashboard::PutString(
      "Diagnostics/Status/localization",
      started_ ? "Running" :
          (!imuConnected ? "Waiting for IMU connection" :
           (imuCalibrating ? "Waiting for IMU calibration" :
                             "Waiting for start button")));
  frc::SmartDashboard::PutBoolean("Diagnostics/Inputs/start_button_pressed",
                                  startPressed);
  frc::SmartDashboard::PutBoolean("Diagnostics/IMU/connected", imuConnected);
  frc::SmartDashboard::PutBoolean("Diagnostics/IMU/calibrating",
                                  imuCalibrating);
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/yaw_deg",
                                  hardware_.GetYawDegrees());
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/yaw_rate_dps",
                                  hardware_.GetYawRateDegreesPerSecond());
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/world_accel_x_g",
                                  hardware_.GetWorldAccelX_G());
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/world_accel_y_g",
                                  hardware_.GetWorldAccelY_G());
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/raw_accel_x_g",
                                  hardware_.GetRawAccelX_G());
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/raw_accel_y_g",
                                  hardware_.GetRawAccelY_G());
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/raw_accel_z_g",
                                  hardware_.GetRawAccelZ_G());
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/raw_gyro_x_dps",
                                  hardware_.GetRawGyroX_Dps());
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/raw_gyro_y_dps",
                                  hardware_.GetRawGyroY_Dps());
  frc::SmartDashboard::PutNumber("Diagnostics/IMU/raw_gyro_z_dps",
                                  hardware_.GetRawGyroZ_Dps());

  for (int wheel = 0; wheel < localization_config::WHEEL_COUNT; ++wheel) {
    const std::string key = "Diagnostics/Encoders/wheel_" +
                            std::to_string(wheel) + "_raw_count";
    frc::SmartDashboard::PutNumber(key, hardware_.GetWheelEncoderRaw(wheel));
    frc::SmartDashboard::PutNumber(
        "Diagnostics/Motors/wheel_" + std::to_string(wheel) +
            "_command_pct",
        hardware_.GetWheelCommand(wheel) * 100.0);
  }

  for (int sensor = 0; sensor < 4; ++sensor) {
    const std::string key = "Diagnostics/Ultrasonic/" +
                            std::string(localization_config::SONARS[sensor].name);
    frc::SmartDashboard::PutNumber(key + "_range_mm", sonarRanges_MM_[sensor]);
    frc::SmartDashboard::PutBoolean(key + "_valid", sonarRangeValid_[sensor]);
    frc::SmartDashboard::PutBoolean(key + "_echo_received",
                                    sonarEchoReceived_[sensor]);
    frc::SmartDashboard::PutString(key + "_status", sonarStatus_[sensor]);
    // A request count proves the software called Ping(), not that a pulse
    // has been measured electrically at the connector.
    frc::SmartDashboard::PutNumber(key + "_ping_requests",
                                   sonarPingRequests_[sensor]);
    frc::SmartDashboard::PutNumber(key + "_echo_responses",
                                   sonarEchoResponses_[sensor]);
    frc::SmartDashboard::PutNumber(key + "_timeouts", sonarTimeouts_[sensor]);
    frc::SmartDashboard::PutNumber(key + "_last_read_age_ms",
        sonarLastReadTime_S_[sensor] < 0.0 ? -1.0 :
            (now - sonarLastReadTime_S_[sensor]) * 1000.0);
    frc::SmartDashboard::PutNumber(key + "_trigger_dio",
        localization_config::SONARS[sensor].triggerDio);
    frc::SmartDashboard::PutNumber(key + "_echo_dio",
        localization_config::SONARS[sensor].echoDio);
    frc::SmartDashboard::PutBoolean(
        key + "_used_for_correction", sonarCorrectionAccepted_[sensor]);
  }
}
