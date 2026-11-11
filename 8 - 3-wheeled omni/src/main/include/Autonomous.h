#pragma once

#include <string>
#include <vector>
#include "RobotConfig.h"

// Arena model and route planning.
struct ArenaRect { double x0,y0,x1,y1; bool trustedWall; };
struct ArenaRayHit { double range = -1.0; int surface = -1; bool trustedWall = false; };

// Fixed map of solid divider segments and station/object envelopes. The large
// central start square and tape are floor markings, not solid obstacles.
class ArenaMap {
 public:
  explicit ArenaMap(bool includeStations = true);
  bool IsFree(ArenaPoint p) const;
  bool SegmentFree(ArenaPoint a, ArenaPoint b) const;
  bool Plan(ArenaPoint start, ArenaPoint goal, std::vector<ArenaPoint>& path) const;
  ArenaRayHit Raycast(ArenaPoint origin, double angle) const;
  ArenaRayHit SonarHit(int sensor, const RobotPose& pose, double beamOffset=0) const;
 private:
  std::vector<ArenaRect> obstacles_;
};

// Independent SP -> MW -> RDW test. Planned turns follow XY travel, which
// uses gyro heading hold. Wall/tape position referencing is currently disabled.
struct ArenaTestInput {
  RobotPose pose;
  double now, dt;
  bool healthy, stationary, startRequested, stopRequested;
  double yawRateRadps=0; // signed CCW gyro rate, independent of the arena yaw offset
};
// Shared gyro turn profile and direction checks for routes and individual tasks.
class HeadingMotionControl {
 public:
  void Reset() { braking_=false; opposed_=0; direction_=0; holdRecheck_=0; }
  double Request(double error,double oldYaw,double measuredYaw,double dt,bool hold);
  std::string CheckFeedback(double previousRequest,double measuredYaw,double dt);
 private:
  bool braking_=false;
  double opposed_=0;
  double holdRecheck_=0;
  int direction_=0;
};

enum class TapeApproach { FromLeft, FromRight };

struct TapeRoutineInput {
  ArenaTestInput motion;
  double leftRangeMM=-1,rearRangeMM=-1,leftTime=-1,rearTime=-1,cobraTime=-1;
  bool leftValid=false,rearValid=false,cobraCalibrated=false;
  LineReading line;
  double blackLevel[4]={-1,-1,-1,-1}; // CH0 leftmost -> CH3 rightmost
  double bodyForwardMMps=0,bodyLeftMMps=0;
  TapeApproach approach=TapeApproach::FromLeft; // latched only on accepted Start
};

class TapeAlignmentRoutine {
 public:
  enum class State { Waiting, Reference, ReferenceReady, DriveToPoint, SearchTape,
    CenterTape, VerifyTape, Complete, Fault, Aborted };
  void Enter();
  void Update(const TapeRoutineInput& in);
  void ConfirmPoseReset(double now);
  void Abort();
  void Fail(const std::string& reason);
  bool IsRunning() const { return state_>=State::Reference && state_<=State::VerifyTape; }
  bool NeedsPoseReset() const { return state_==State::ReferenceReady; }
  State GetState() const { return state_; }
  const RobotPose& GetReferencePose() const { return referencePose_; }
  TapeApproach GetApproach() const { return approach_; }
  ArenaPoint GetTarget() const {
    return approach_==TapeApproach::FromLeft?tape_test_config::TARGET_FROM_LEFT:tape_test_config::TARGET_FROM_RIGHT;
  }
  const std::string& GetStatus() const { return status_; }
  double GetForwardMMps() const { return forward_; }
  double GetLeftMMps() const { return left_; }
  double GetYawRadps() const { return yaw_; }
  double GetScanTravelMM() const { return scanTravel_; }
  static bool PoseFromWalls(double leftGap,double rearGap,RobotPose& pose);
 private:
  void Stop() { forward_=left_=yaw_=0; }
  void Begin(const TapeRoutineInput& in);
  void SampleReference(const TapeRoutineInput& in);
  bool CobraHealthy(const TapeRoutineInput& in) const;
  bool Centered(const TapeRoutineInput& in) const;
  void LimitTranslation(double oldForward,double oldLeft,double acceleration,double dt);
  void DriveWaypoint(const TapeRoutineInput& in,double oldForward,double oldLeft,double headingError);
  const char* ApproachName() const { return approach_==TapeApproach::FromLeft?"From left: ":"From right: "; }
  TapeApproach approach_=TapeApproach::FromLeft;
  enum class WaypointPhase { Travel, Brake, Settle, Correct };
  WaypointPhase waypointPhase_=WaypointPhase::Travel;
  State state_=State::Waiting;
  std::string status_="Supplied Cobra calibration ready; select Autonomous, enable, then choose a tape approach";
  RobotPose referencePose_{0,0,0},scanStart_{0,0,0},progressPose_{0,0,0};
  double forward_=0,left_=0,yaw_=0,stateTime_=0,scanTime_=0,settleTime_=-1,lostTime_=-1;
  double motionEffortTime_=0,scanTravel_=0;
  double phaseTime_=0,filteredForward_=0,filteredLeft_=0,correctionX_=0,correctionY_=0;
  double lastLeftTime_=-1,lastRearTime_=-1;
  double sumLeft_=0,sumRear_=0,minLeft_=0,maxLeft_=0,minRear_=0,maxRear_=0;
  int referenceSamples_=0;
  HeadingMotionControl heading_;
};

enum class SimpleTask { None, Forward1000, Backward1000, Ccw90, Cw90, Ccw180, Cw180, Colour };
const char* SimpleTaskName(SimpleTask task);
// Relative tasks snapshot their start; there are no arena/map prerequisites.
class SimpleTaskController {
 public:
  enum class State { Idle, Driving, Turning, Photograph, Complete, Fault, Aborted };
  bool Start(SimpleTask task,const ArenaTestInput& in);
  void Update(const ArenaTestInput& in);
  void CompletePhoto(bool success,const std::string& result);
  void Abort();
  void Fail(const std::string& reason);
  bool IsRunning() const { return state_==State::Driving || state_==State::Turning || state_==State::Photograph; }
  bool IsMotionRunning() const { return state_==State::Driving || state_==State::Turning; }
  State GetState() const { return state_; }
  const std::string& GetStatus() const { return status_; }
  SimpleTask GetTask() const { return task_; }
  double GetForwardMMps() const { return forward_; }
  double GetLeftMMps() const { return left_; }
  double GetYawRadps() const { return yaw_; }
  double GetTravelMM() const { return travel_; }
  double GetTurnDegrees() const { return turned_*180/robot_geometry::PI; }
 private:
  void Stop() { forward_=left_=yaw_=0; }
  State state_=State::Idle;
  SimpleTask task_=SimpleTask::None;
  std::string status_="Choose a task, enable Autonomous, then Run selected task";
  RobotPose start_{0,0,0};
  double targetX_=0,targetY_=0,targetTurn_=0,turned_=0,previousHeading_=0;
  double started_=0,settled_=-1,travel_=0;
  double forward_=0,left_=0,yaw_=0;
  HeadingMotionControl heading_;
};
class ArenaTestController {
 public:
  enum class State { Disabled, WaitingStart, TravelMW, TurnMW, ReferenceMW, FinishMW, FinishHeadingMW,
    TravelRDW, TurnRDW, ReferenceRDW, FinishRDW, FinishHeadingRDW, Complete, Fault, Aborted };
  void Enter(double now);
  void Abort();
  void Fail(const std::string& reason);
  void Update(const ArenaTestInput& in);
  bool IsRunning() const;
  State GetState() const { return state_; }
  const std::string& GetStatus() const { return status_; }
  const std::string& GetReferenceReport() const { return referenceReport_; }
  const std::string& GetLastReferenceReport() const { return lastReferenceReport_; }
  double GetForwardMMps() const { return forward_; }
  double GetLeftMMps() const { return left_; }
  double GetYawRadps() const { return yaw_; }
  double GetHeadingTargetDeg() const;
  ArenaPoint GetTarget() const { return {target_.x_MM,target_.y_MM}; }
  std::string ValidateGeometry() const;
 private:
  void Stop();
  void Transition(State state,double now,const char* status);
  bool Settled(bool reached,double now);
  void Reference(const ArenaTestInput& in);
  ArenaMap map_{false}; // this test uses the drawing's walls, not old station placeholders
  State state_=State::Disabled;
  arena_test_config::Waypoint target_=arena_test_config::SP;
  std::string status_="Select Autonomous, enable, then press Start routine";
  std::string referenceReport_="No reference requested", lastReferenceReport_;
  double forward_=0,left_=0,yaw_=0;
  double travelHeadingRad_=0;
  double stoppedHeadingTargetDeg_=arena_test_config::SP.headingDeg;
  double stateTime_=0, missionTime_=0, settleTime_=-1;
  HeadingMotionControl heading_;
};

// Disabled mission state machine.
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
