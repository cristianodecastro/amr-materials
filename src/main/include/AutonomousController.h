#pragma once

#include <string>
#include <vector>
#include "ArenaMap.h"
#include "LineTracker.h"
#include "ObjectHandler.h"

struct AutonomousInput {
  RobotPose pose;
  LineReading line;
  double nowSeconds;
  double dtSeconds;
  bool startPressed;
  bool sensorsHealthy;
  ObjectActionStatus objectStatus;
};

// Nonblocking: one update per scheduler tick. Only outputs requests; never
// touches hardware, sleeps, resets encoders, or changes Control Station mode.
class AutonomousController {
 public:
  enum class State { Disabled, Blocked, WaitingStart, NavigateStorage, AcquireStorage,
    DockStorage, Pickup, RetreatStorage, NavigateDelivery, AcquireDelivery,
    DockDelivery, Release, RetreatDelivery, Complete, Fault, Aborted };
  void Enter(double now, const std::string& preflightError);
  void Abort();
  void SafetyStop(const char* reason) { Fault(reason); }
  void Update(const AutonomousInput& in);
  State GetState() const { return state_; }
  const std::string& GetStatus() const { return status_; }
  double GetForwardMMps() const { return forward_; }
  double GetLeftMMps() const { return left_; }
  double GetYawRadps() const { return yaw_; }
  int GetJobNumber() const { return job_+1; }
  int GetCompletedJobs() const { return completedJobs_; }
  const autonomous_config::Station& GetStation() const;
  ArenaPoint GetTarget() const { return target_; }
  ObjectAction GetObjectRequest() const;
  bool IsRunning() const;
  bool CanCorrectFromLine() const;
 private:
  void Stop();
  void Fault(const char* reason);
  void Transition(State state, double now, const char* status);
  bool BeginRoute(const AutonomousInput& in, bool storage);
  std::string ValidateMissionGeometry() const;
  bool Navigate(const AutonomousInput& in);
  bool MoveTo(const AutonomousInput& in, ArenaPoint target, double heading, double speed);
  void Dock(const AutonomousInput& in, bool acquiring);
  void LimitTranslation(double oldForward, double oldLeft, double dt);
  State state_ = State::Disabled;
  std::string status_ = "Disabled";
  ArenaMap map_;
  std::vector<ArenaPoint> route_;
  unsigned waypoint_ = 0;
  int job_ = 0, completedJobs_ = 0, lineSamples_ = 0;
  bool startReleased_ = false, stationIsStorage_ = true;
  double pressTime_ = -1, stateTime_ = 0, missionTime_ = 0, settleTime_ = -1;
  double progressTime_ = 0, lastLineTime_ = 0;
  ArenaPoint progressPoint_{0,0}, target_{0,0};
  double forward_ = 0, left_ = 0, yaw_ = 0;
};
