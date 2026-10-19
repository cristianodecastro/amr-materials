#include "ForwardDriveController.h"

#include <limits>

// Execute the real controller at compile time so these regression checks run
// on the development PC even though the executable targets Raspberry Pi ARM.
namespace {
using Controller = ForwardDriveController;
using State = Controller::State;
constexpr ForwardDriveCalibration kCalibration{
    1.0, {-0.5, -0.5, 1.0}, {1.0, 1.0, 1.0}, true};
constexpr double kInitial[3] = {100.0, 200.0, 300.0};

constexpr Controller Started() {
  Controller controller(kCalibration);
  controller.Update(true, false, 0.0, kInitial);
  controller.Update(true, true, 0.020, kInitial);
  controller.Update(true, true, 0.100, kInitial);
  return controller;
}

constexpr bool HeldStartCannotLaunch() {
  Controller controller(kCalibration);
  controller.Update(false, true, 0.0, kInitial);
  controller.Update(true, true, 0.1, kInitial);
  controller.Update(true, true, 5.0, kInitial);
  return controller.GetState() == State::Ready &&
         controller.GetCommandMMps() == 0.0;
}
static_assert(HeldStartCannotLaunch(), "Held Start must not launch on enable");

constexpr bool DebounceAndBaseline() {
  Controller controller(kCalibration);
  controller.Update(true, false, 0.0, kInitial);
  controller.Update(true, true, 0.02, kInitial);
  controller.Update(true, true, 0.04, kInitial);
  if (controller.IsDriving()) return false;
  controller.Update(true, false, 0.06, kInitial);  // contact bounce
  controller.Update(true, true, 0.08, kInitial);
  controller.Update(true, true, 0.16, kInitial);
  return controller.IsDriving() && controller.GetDistanceMM() == 0.0;
}
static_assert(DebounceAndBaseline(), "Start must debounce and snapshot counts");

constexpr bool DistanceAndCompletion() {
  auto controller = Started();
  const double half[3] = {-150.0, -50.0, 800.0};
  controller.Update(true, true, 0.5, half);
  if (controller.GetDistanceMM() != 500.0 || !controller.IsDriving()) return false;
  const double almost[3] = {-399.5, -299.5, 1299.0};
  controller.Update(true, true, 0.8, almost);
  if (!controller.IsDriving()) return false;
  const double goal[3] = {-400.0, -300.0, 1300.0};
  controller.Update(true, true, 1.0, goal);
  if (!controller.IsComplete() || controller.GetCommandMMps() != 0.0 ||
      controller.GetRemainingMM() != 0.0) return false;
  controller.Update(true, true, 1.5, goal);  // held button cannot repeat
  return controller.IsComplete() && controller.GetDistanceMM() == 1000.0;
}
static_assert(DistanceAndCompletion(), "Stop at 1000 mm without held-button repeat");

constexpr bool RepeatRequiresNewPress() {
  auto controller = Started();
  const double goal[3] = {-400.0, -300.0, 1300.0};
  controller.Update(true, true, 1.0, goal);
  controller.Update(true, false, 1.1, goal);
  controller.Update(true, true, 1.2, goal);
  controller.Update(true, true, 1.3, goal);
  return controller.IsDriving() && controller.GetDistanceMM() == 0.0;
}
static_assert(RepeatRequiresNewPress(), "A fresh press must start a new baseline");

constexpr bool DisableStopsWithoutResume() {
  auto controller = Started();
  const double moved[3] = {95.0, 195.0, 310.0};
  controller.Update(true, false, 0.2, moved);
  if (controller.GetCommandMMps() <= 0.0) return false;
  controller.Update(false, true, 0.3, moved);
  if (controller.GetState() != State::Aborted ||
      controller.GetCommandMMps() != 0.0) return false;
  controller.Update(true, true, 0.4, moved);
  controller.Update(true, true, 1.0, moved);
  return controller.GetState() == State::Aborted &&
         controller.GetCommandMMps() == 0.0 &&
         controller.GetWheelCommandMMps(0) == 0.0 &&
         controller.GetWheelCommandMMps(1) == 0.0 &&
         controller.GetWheelCommandMMps(2) == 0.0;
}
static_assert(DisableStopsWithoutResume(), "Disable/mode change must stop and not resume");

constexpr bool FrozenEncoderStops() {
  auto controller = Started();
  controller.Update(true, false, 2.2, kInitial);
  return controller.GetState() == State::EncoderFault &&
         controller.GetCommandMMps() == 0.0;
}
static_assert(FrozenEncoderStops(), "Frozen feedback must stop the motors");

constexpr bool OneFailedEncoderStops() {
  auto controller = Started();
  // The aggregate would pass the target, but one encoder has not moved.
  const double oneStuck[3] = {100.0, -550.0, 1800.0};
  controller.Update(true, false, 0.5, oneStuck);
  return controller.GetState() == State::EncoderFault &&
         controller.GetCommandMMps() == 0.0;
}
static_assert(OneFailedEncoderStops(), "A failed wheel must not double the travel distance");

constexpr bool ReverseFeedbackStops() {
  auto controller = Started();
  const double reversed[3] = {115.0, 185.0, 330.0};
  controller.Update(true, false, 0.5, reversed);
  return controller.GetState() == State::EncoderFault &&
         controller.GetFaultWheel() == 0 && controller.GetCommandMMps() == 0.0;
}
static_assert(ReverseFeedbackStops(), "Reversed encoder polarity must stop the run");

constexpr bool InvalidFeedbackStops() {
  auto controller = Started();
  const double invalid[3] = {std::numeric_limits<double>::infinity(), 200.0, 300.0};
  controller.Update(true, false, 0.2, invalid);
  return controller.GetState() == State::EncoderFault &&
         controller.GetCommandMMps() == 0.0;
}
static_assert(InvalidFeedbackStops(), "Invalid feedback must not reach motor outputs");

constexpr bool TotalTimeoutStops() {
  auto controller = Started();
  for (int second = 1; second <= 20; ++second) {
    const double travel = second * 5.0;
    const double counts[3] = {100.0 - travel * 0.5,
                              200.0 - travel * 0.5, 300.0 + travel};
    controller.Update(true, false, 0.11 + second, counts);
  }
  return controller.GetState() == State::TimedOut &&
         controller.GetCommandMMps() == 0.0;
}
static_assert(TotalTimeoutStops(), "Slow but moving feedback must still time out");

constexpr bool InvalidMappingCannotLaunch() {
  ForwardDriveCalibration invalid = kCalibration;
  invalid.mappingConfirmed = false;
  Controller controller(invalid);
  controller.Update(true, false, 0.0, kInitial);
  controller.Update(true, true, 0.02, kInitial);
  controller.Update(true, true, 0.1, kInitial);
  return controller.GetState() == State::ConfigurationFault &&
         controller.GetCommandMMps() == 0.0;
}
static_assert(InvalidMappingCannotLaunch(), "Unconfirmed geometry must not drive");

constexpr bool RampAndSlowdown() {
  auto controller = Started();
  const double moved[3] = {99.0, 199.0, 302.0};
  controller.Update(true, false, 0.12, moved);
  if (controller.GetCommandMMps() > 5.000001) return false;
  for (int tick = 1; tick <= 60; ++tick) {
    const double travel = tick * 10.0;
    const double counts[3] = {100.0 - travel * 0.5,
                              200.0 - travel * 0.5, 300.0 + travel};
    controller.Update(true, false, 0.12 + tick * 0.02, counts);
    if (controller.GetCommandMMps() >
        localization_config::FORWARD_CRUISE_SPEED_MMPS) return false;
  }
  const double nearGoal[3] = {-375.0, -275.0, 1250.0};
  controller.Update(true, false, 1.4, nearGoal);
  return controller.IsDriving() && controller.GetCommandMMps() ==
      localization_config::FORWARD_MIN_SPEED_MMPS;
}
static_assert(RampAndSlowdown(), "Ramp-up, cruise limit and final slowdown must apply");

constexpr bool IdleWheelIsNotTreatedAsStalled() {
  constexpr ForwardDriveCalibration twoDriven{
      1.0, {0.0, -0.5, 0.5}, {1.0, 1.0, 1.0}, true};
  Controller controller(twoDriven);
  controller.Update(true, false, 0.0, kInitial);
  controller.Update(true, true, 0.02, kInitial);
  controller.Update(true, true, 0.1, kInitial);
  for (int second = 1; second <= 3; ++second) {
    const double travel = second * 20.0;
    const double counts[3] = {100.0, 200.0 - travel * 0.5, 300.0 + travel * 0.5};
    controller.Update(true, false, 0.1 + second, counts);
  }
  return controller.IsDriving() && controller.GetDistanceMM() == 60.0;
}
static_assert(IdleWheelIsNotTreatedAsStalled(), "Only commanded wheels require progress");

constexpr bool PhysicalScaleAndPolarity() {
  constexpr double scale = (2.0 * localization_config::PI * 51.0) / 1464.0;
  constexpr ForwardDriveCalibration calibrated{
      scale, {-0.5, -0.5, 1.0}, {1.0, -1.0, 1.0}, true};
  Controller controller(calibrated);
  controller.Update(true, false, 0.0, kInitial);
  controller.Update(true, true, 0.02, kInitial);
  controller.Update(true, true, 0.1, kInitial);
  const double counts[3] = {100.0 - 732.0, 200.0 + 732.0, 300.0 + 1464.0};
  controller.Update(true, false, 0.5, counts);
  const double circumference = 2.0 * localization_config::PI * 51.0;
  return controller.IsDriving() &&
      controller.GetDistanceMM() > circumference - 1.0e-6 &&
      controller.GetDistanceMM() < circumference + 1.0e-6;
}
static_assert(PhysicalScaleAndPolarity(), "Wheel radius, count scale and polarity must convert to mm");

constexpr bool SmallRotationDoesNotBiasForwardComponent() {
  auto controller = Started();
  // 100 mm forward plus a common +1 mm rotational wheel displacement.
  const double counts[3] = {51.0, 151.0, 401.0};
  controller.Update(true, false, 0.5, counts);
  return controller.IsDriving() && controller.GetDistanceMM() == 100.0;
}
static_assert(SmallRotationDoesNotBiasForwardComponent(),
              "Forward projection must cancel common rotational wheel motion");

// Corrected travel basis for the arena-observed clockwise 90-degree change:
// M0 advances, M1 reverses, M2 is idle. Verify command AND measurement use it.
constexpr double kSqrtThreeOverTwo = 0.8660254037844386;
constexpr ForwardDriveCalibration kCorrectedCalibration{
    1.0, {kSqrtThreeOverTwo, -kSqrtThreeOverTwo, 0.0},
    {1.0, 1.0, 1.0}, true};

constexpr Controller CorrectedStarted() {
  Controller controller(kCorrectedCalibration);
  controller.Update(true, false, 0.0, kInitial);
  controller.Update(true, true, 0.02, kInitial);
  controller.Update(true, true, 0.1, kInitial);
  return controller;
}

constexpr bool CorrectedDirectionAndDistanceAgree() {
  auto controller = CorrectedStarted();
  const double counts[3] = {100.0 + 500.0 * kSqrtThreeOverTwo,
                             200.0 - 500.0 * kSqrtThreeOverTwo, 300.0};
  controller.Update(true, false, 0.5, counts);
  return controller.IsDriving() && controller.GetDistanceMM() > 499.99999 &&
      controller.GetDistanceMM() < 500.00001 &&
      controller.GetWheelCommandMMps(0) > 0.0 &&
      controller.GetWheelCommandMMps(1) < 0.0 &&
      controller.GetWheelCommandMMps(2) == 0.0;
}
static_assert(CorrectedDirectionAndDistanceAgree(),
              "Corrected drive basis must also measure the corrected travel direction");

constexpr bool SynchronizationHelpsLaggingWheel() {
  auto controller = CorrectedStarted();
  const double counts[3] = {100.0 + 90.0 * kSqrtThreeOverTwo,
                             200.0 - 110.0 * kSqrtThreeOverTwo, 300.0};
  controller.Update(true, false, 0.5, counts);
  const double m0 = controller.GetWheelCommandMMps(0) / kSqrtThreeOverTwo;
  const double m1 = controller.GetWheelCommandMMps(1) / -kSqrtThreeOverTwo;
  return controller.IsDriving() && m0 > m1 &&
      m0 > controller.GetCommandMMps() && m1 >= 0.0 &&
      controller.GetWheelCommandMMps(2) == 0.0;
}
static_assert(SynchronizationHelpsLaggingWheel(),
              "Lagging driven wheels must receive more command than leading wheels");

constexpr bool SynchronizedWheelsKeepEqualCommands() {
  auto controller = CorrectedStarted();
  const double counts[3] = {100.0 + 50.0 * kSqrtThreeOverTwo,
                             200.0 - 50.0 * kSqrtThreeOverTwo, 300.0};
  controller.Update(true, false, 0.5, counts);
  const double sum = controller.GetWheelCommandMMps(0) +
                     controller.GetWheelCommandMMps(1);
  return controller.IsDriving() && sum > -1.0e-8 && sum < 1.0e-8;
}
static_assert(SynchronizedWheelsKeepEqualCommands(),
              "No synchronization correction is needed when both wheels track together");

constexpr bool CorrectedGoalStopsEveryMotor() {
  auto controller = CorrectedStarted();
  const double counts[3] = {100.0 + 1001.0 * kSqrtThreeOverTwo,
                             200.0 - 1001.0 * kSqrtThreeOverTwo, 300.0};
  controller.Update(true, false, 1.0, counts);
  return controller.IsComplete() && controller.GetWheelCommandMMps(0) == 0.0 &&
      controller.GetWheelCommandMMps(1) == 0.0 &&
      controller.GetWheelCommandMMps(2) == 0.0;
}
static_assert(CorrectedGoalStopsEveryMotor(),
              "Reaching the goal in the corrected direction must stop all three motors");

constexpr bool PartialRunDoesNotCountAsComplete() {
  auto controller = CorrectedStarted();
  const double counts[3] = {100.0 + 526.0 * kSqrtThreeOverTwo,
                             200.0 - 526.0 * kSqrtThreeOverTwo, 300.0};
  controller.Update(true, false, 5.2, counts);
  controller.Update(false, false, 5.22, counts);
  return controller.GetState() == State::Aborted && !controller.IsComplete() &&
      controller.GetDistanceMM() > 525.99999 &&
      controller.GetDistanceMM() < 526.00001;
}
static_assert(PartialRunDoesNotCountAsComplete(),
              "A disabled run at 526 mm must remain incomplete without rescaling its distance");
}  // namespace
