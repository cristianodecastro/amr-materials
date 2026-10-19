#pragma once

#include "LocalizationConstants.h"

// A pure controller: no VMX, Titan, ultrasonic, IMU, or dashboard dependency.
// Radial wheel angles are converted to forward rolling coefficients outside
// this class. For pure forward travel d_wheel = coefficient * d_robot.
struct ForwardDriveCalibration {
  double millimetresPerCount;
  double forwardCoefficient[3];
  double encoderSign[3];
  bool mappingConfirmed;
};

class ForwardDriveController {
 public:
  enum class State { Ready, Driving, Completed, Aborted, TimedOut,
                     EncoderFault, ConfigurationFault };

  explicit constexpr ForwardDriveController(ForwardDriveCalibration calibration)
      : calibration_(calibration) {}

  constexpr void Update(bool motionAllowed, bool startPressed, double now,
                        const double (&counts)[3]) {
    const double dt = Clamp(now - lastUpdateTime_, 0.0, 0.050);
    lastUpdateTime_ = now;

    if (!motionAllowed) {
      if (state_ == State::Driving)
        Finish(State::Aborted, "Stopped: disabled or left Teleoperated mode", now);
      armed_ = false;
      pressing_ = false;
      command_MMps_ = 0.0;
      return;
    }

    if (state_ != State::Driving) {
      command_MMps_ = 0.0;
      if (!startPressed) {
        armed_ = true;
        pressing_ = false;
      } else if (armed_) {
        if (!pressing_) {
          pressing_ = true;
          pressStartTime_ = now;
        } else if (now - pressStartTime_ >=
                   localization_config::START_BUTTON_DEBOUNCE_S) {
          Begin(now, counts);
        }
      }
      return;
    }

    // A press during a run cannot queue another run after completion.
    armed_ = false;
    pressing_ = false;
    elapsed_S_ = now - runStartTime_;
    if (elapsed_S_ >= localization_config::FORWARD_TIMEOUT_S) {
      Finish(State::TimedOut, "Stopped: travel timeout", now);
      return;
    }

    double total = 0.0;
    double coefficientSquaredSum = 0.0;
    double smallest = 1.0e12;
    double largest = -1.0e12;
    for (int wheel = 0; wheel < 3; ++wheel) {
      const double coefficient = calibration_.forwardCoefficient[wheel];
      if (Abs(coefficient) < 0.05) continue;  // wheel is idle for pure forward
      if (!Finite(counts[wheel])) {
        faultWheel_ = wheel;
        Finish(State::EncoderFault, "Stopped: invalid encoder reading", now);
        return;
      }
      const double projectedTravel =
          (counts[wheel] - initialCounts_[wheel]) *
          calibration_.millimetresPerCount * calibration_.encoderSign[wheel] /
          coefficient;
      projectedTravel_MM_[wheel] = projectedTravel;
      if (projectedTravel < -localization_config::FORWARD_REVERSE_LIMIT_MM) {
        faultWheel_ = wheel;
        Finish(State::EncoderFault, "Stopped: encoder direction mismatch", now);
        return;
      }
      if (projectedTravel >= lastProgress_MM_[wheel] +
                                 localization_config::FORWARD_PROGRESS_STEP_MM) {
        lastProgress_MM_[wheel] = projectedTravel;
        lastProgressTime_[wheel] = now;
      } else if (now - lastProgressTime_[wheel] >=
                 localization_config::FORWARD_STALL_TIMEOUT_S) {
        faultWheel_ = wheel;
        Finish(State::EncoderFault, "Stopped: encoder not advancing", now);
        return;
      }
      // Least-squares forward component. For the symmetric 120-degree
      // layout this also cancels the common wheel displacement from a small
      // rotation; an unweighted average of projected distances would not.
      total += projectedTravel * coefficient * coefficient;
      coefficientSquaredSum += coefficient * coefficient;
      smallest = Min(smallest, projectedTravel);
      largest = Max(largest, projectedTravel);
    }
    distance_MM_ = total / coefficientSquaredSum;
    if (largest - smallest >
        localization_config::FORWARD_MAX_WHEEL_DISAGREEMENT_MM) {
      Finish(State::EncoderFault, "Stopped: wheel distances disagree", now);
      return;
    }
    if (distance_MM_ >= localization_config::FORWARD_TARGET_MM) {
      Finish(State::Completed, "Complete - target reached", now);
      return;
    }

    const double requested = Clamp(
        (localization_config::FORWARD_TARGET_MM - distance_MM_) *
            localization_config::FORWARD_DISTANCE_KP_PER_S,
        localization_config::FORWARD_MIN_SPEED_MMPS,
        localization_config::FORWARD_CRUISE_SPEED_MMPS);
    command_MMps_ = Min(requested, command_MMps_ +
        localization_config::FORWARD_ACCELERATION_MMPS2 * dt);
  }

  constexpr State GetState() const { return state_; }
  constexpr bool IsDriving() const { return state_ == State::Driving; }
  constexpr bool IsComplete() const { return state_ == State::Completed; }
  constexpr double GetDistanceMM() const { return distance_MM_; }
  constexpr double GetRemainingMM() const {
    return Max(0.0, localization_config::FORWARD_TARGET_MM - distance_MM_);
  }
  constexpr double GetCommandMMps() const { return command_MMps_; }
  constexpr double GetWheelProjectedTravelMM(int wheel) const {
    return wheel >= 0 && wheel < 3 ? projectedTravel_MM_[wheel] : 0.0;
  }
  constexpr double GetWheelCommandMMps(int wheel) const {
    if (!IsDriving() || wheel < 0 || wheel >= 3 ||
        Abs(calibration_.forwardCoefficient[wheel]) < 0.05) return 0.0;
    // Close the synchronization loop on measured wheel travel. A lagging
    // driven wheel receives more command; a leading wheel receives less.
    const double correction = Clamp(
        (distance_MM_ - projectedTravel_MM_[wheel]) *
            localization_config::FORWARD_WHEEL_SYNC_KP_PER_S,
        -localization_config::FORWARD_MAX_SYNC_MMPS,
        localization_config::FORWARD_MAX_SYNC_MMPS);
    return calibration_.forwardCoefficient[wheel] * Clamp(
        command_MMps_ + correction, 0.0,
        localization_config::FORWARD_CRUISE_SPEED_MMPS +
            localization_config::FORWARD_MAX_SYNC_MMPS);
  }
  constexpr double GetElapsedSeconds() const { return elapsed_S_; }
  constexpr int GetFaultWheel() const { return faultWheel_; }
  constexpr const char* GetStatus() const {
    return state_ == State::Ready
        ? (armed_ ? "Ready - press Start" : "Release Start to arm") : status_;
  }

 private:
  static constexpr double Min(double a, double b) { return a < b ? a : b; }
  static constexpr double Max(double a, double b) { return a > b ? a : b; }
  static constexpr double Abs(double value) { return value < 0.0 ? -value : value; }
  static constexpr double Clamp(double value, double low, double high) {
    return Min(high, Max(low, value));
  }
  static constexpr bool Finite(double value) {
    return value >= -1.0e15 && value <= 1.0e15;
  }

  constexpr void Begin(double now, const double (&counts)[3]) {
    armed_ = false;
    pressing_ = false;
    faultWheel_ = -1;
    int drivenWheels = 0;
    bool valid = calibration_.mappingConfirmed &&
                 calibration_.millimetresPerCount > 0.0 &&
                 Finite(calibration_.millimetresPerCount);
    for (int wheel = 0; wheel < 3; ++wheel) {
      const double coefficient = calibration_.forwardCoefficient[wheel];
      valid = valid && Finite(counts[wheel]) && Finite(coefficient) &&
              Abs(coefficient) <= 1.000001 &&
              Abs(calibration_.encoderSign[wheel]) == 1.0;
      if (Abs(coefficient) >= 0.05) ++drivenWheels;
    }
    if (!valid || drivenWheels < 2) {
      state_ = State::ConfigurationFault;
      status_ = "Stopped: confirm wheel mapping and calibration";
      return;
    }
    for (int wheel = 0; wheel < 3; ++wheel) {
      initialCounts_[wheel] = counts[wheel];
      lastProgress_MM_[wheel] = 0.0;
      lastProgressTime_[wheel] = now;
      projectedTravel_MM_[wheel] = 0.0;
    }
    // Snapshot counts instead of issuing asynchronous Titan encoder resets.
    distance_MM_ = 0.0;
    command_MMps_ = 0.0;
    elapsed_S_ = 0.0;
    runStartTime_ = now;
    state_ = State::Driving;
    status_ = "Driving forward - encoders only";
  }

  constexpr void Finish(State state, const char* message, double now) {
    state_ = state;
    status_ = message;
    command_MMps_ = 0.0;
    elapsed_S_ = now - runStartTime_;
  }

  ForwardDriveCalibration calibration_;
  State state_ = State::Ready;
  const char* status_ = "Ready";
  bool armed_ = false;
  bool pressing_ = false;
  double lastUpdateTime_ = 0.0;
  double pressStartTime_ = 0.0;
  double runStartTime_ = 0.0;
  double elapsed_S_ = 0.0;
  double distance_MM_ = 0.0;
  double command_MMps_ = 0.0;
  double initialCounts_[3] = {0.0, 0.0, 0.0};
  double projectedTravel_MM_[3] = {0.0, 0.0, 0.0};
  double lastProgress_MM_[3] = {0.0, 0.0, 0.0};
  double lastProgressTime_[3] = {0.0, 0.0, 0.0};
  int faultWheel_ = -1;
};
