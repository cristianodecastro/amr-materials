#pragma once
#include <string>
#include <vector>
#include "RobotConfig.h"

// Global planning is independent of motor IO. The real map and footprint must
// be supplied explicitly; painted tape is not an obstacle. During XY travel
// heading stays at start.headingRad, followed by a checked turn at the goal.
struct NavigationMap {
  double widthMM=2000,heightMM=2000;
  std::vector<ForbiddenRegion> forbidden;
};
struct NavigationFootprint {
  // Conservative body-frame rectangle, including every physical projection.
  double frontMM=0,rearMM=0,leftMM=0,rightMM=0;
};
NavigationMap DefaultNavigationMap();
NavigationFootprint DefaultNavigationFootprint();
struct WallRangeEstimate {
  bool valid=false;
  int axis=-1; // 0 = arena X, 1 = arena Y
  double coordinateMM=0;
};
// Pose is the estimate at acquisition time, not a later pose of a moving robot.
// Keep-outs conservatively exclude beams which may hit storage/delivery items.
WallRangeEstimate EstimateFromOuterWall(const NavigationMap& map,const RobotPose& acquiredPose,
    int sensor,double rangeMM,bool valid);
struct NavigationPlan {
  std::vector<ArenaPoint> points;
  double lengthMM=0,cost=0;
  std::string error;
  bool Valid() const { return error.empty() && !points.empty(); }
};
class ClearancePlanner {
 public:
  ClearancePlanner(NavigationMap map,NavigationFootprint footprint,
                   double clearanceMM=navigation_config::MIN_CLEARANCE_MM,
                   double gridMM=navigation_config::GRID_MM);
  NavigationPlan Plan(RobotPose start,RobotPose goal) const;
  bool SegmentFree(ArenaPoint a,ArenaPoint b,double headingRad) const;
  bool RotationFree(ArenaPoint point,double fromRad,double toRad) const;
 private:
  bool ConfigurationValid() const;
  bool SegmentFreeWithMargin(ArenaPoint a,ArenaPoint b,double heading,double margin) const;
  double Clearance(ArenaPoint point,double heading) const;
  double SegmentCost(ArenaPoint a,ArenaPoint b,double heading) const;
  NavigationMap map_;
  NavigationFootprint footprint_;
  double clearance_,grid_;
};

struct MotionInput {
  RobotPose pose;
  double now, dt;
  bool healthy, startRequested, stopRequested;
  double yawRateRadps=0; // signed CCW gyro rate, independent of the arena yaw offset
};
class HeadingMotionControl {
 public:
  void Reset() { opposed_=commandTime_=lastRequest_=lastMeasured_=0; direction_=0; holdRecheck_=0; }
  double Request(double error,double oldYaw,double measuredYaw,double dt);
  std::string CheckFeedback(double previousRequest,double measuredYaw,double dt);
  double FeedbackRequest() const { return lastRequest_; }
  double FeedbackMeasured() const { return lastMeasured_; }
 private:
  double opposed_=0;
  double commandTime_=0,lastRequest_=0,lastMeasured_=0;
  double holdRecheck_=0;
  int direction_=0;
};

// Shared XY braking/settling controller for tape tests and planned routes.
class WaypointDrive {
 public:
  void Reset(ArenaPoint target,double now,double speed,double acceleration);
  void Update(const MotionInput& motion,double measuredForward,double measuredLeft,double headingError);
  bool Reached() const { return reached_; }
  double Forward() const { return forward_; }
  double Left() const { return left_; }
  const std::string& Status() const { return status_; }
 private:
  void LimitTranslation(double oldForward,double oldLeft,double acceleration,double dt);
  enum class WaypointPhase { Travel, Brake, Settle, Correct };
  WaypointPhase waypointPhase_=WaypointPhase::Travel;
  ArenaPoint target_={0,0};
  double forward_=0,left_=0,phaseTime_=0,settleTime_=-1;
  double filteredForward_=0,filteredLeft_=0,correctionX_=0,correctionY_=0;
  double maxSpeed_=0,acceleration_=0;
  bool reached_=false;
  std::string status_;
};

struct NavigationInput {
  MotionInput motion;
  double forwardMMps=0,leftMMps=0;
  // Same order as RANGE_SENSOR_MOUNTS: left IR, rear sonar, right IR, front sonar.
  double rangeMM[4]={-1,-1,-1,-1},stamp[4]={-1,-1,-1,-1};
  bool valid[4]={false,false,false,false};
};
class GoalNavigationRoutine {
 public:
  enum class State { Unreferenced, Reference, ReferenceReady, Ready, NeedPlan, Planning,
    Following, Observing, PositionReady, FinalHeading, Complete, Aborted, Fault };
  void Enter();
  void ReferenceCorner(double now);
  bool StartGoal(RobotPose goal,const NavigationInput& in);
  void Update(const NavigationInput& in);
  void Abort();
  void Fail(const std::string& message);
  State GetState() const { return state_; }
  bool IsRunning() const { return state_>=State::Reference && state_<=State::FinalHeading && state_!=State::Ready; }
  bool HasReference() const { return referenced_; }
  bool NeedsPlan() const { return state_==State::NeedPlan; }
  void PlanningStarted() { if (NeedsPlan()) state_=State::Planning; }
  void AcceptPlan(NavigationPlan plan,const NavigationInput& in);
  bool NeedsPoseReset() const { return state_==State::ReferenceReady; }
  bool NeedsPositionCorrection() const { return state_==State::PositionReady; }
  const RobotPose& RequestedPose() const { return requestedPose_; }
  void ConfirmReference();
  void ConfirmPositionCorrection(const NavigationInput& in);
  RobotPose PlanStart(const NavigationInput& in) const { return {in.motion.pose.x_MM,in.motion.pose.y_MM,travelHeading_}; }
  const RobotPose& Goal() const { return goal_; }
  const NavigationMap& Map() const { return map_; }
  const std::string& Status() const { return status_; }
  double Forward() const { return forward_; }
  double Left() const { return left_; }
  double Yaw() const { return yaw_; }
  double FeedbackRequest() const { return heading_.FeedbackRequest(); }
  double FeedbackMeasured() const { return heading_.FeedbackMeasured(); }
  double PathLengthMM() const { return pathLength_; }
  unsigned WaypointCount() const { return path_.size(); }
  unsigned WaypointIndex() const { return waypoint_; }
 private:
  void Stop() { forward_=left_=yaw_=0; }
  void ContinueAfterObservation(const NavigationInput& in);
  void BeginObservation(double now);
  void NeedNewPlan(const NavigationInput& in);
  NavigationMap map_=DefaultNavigationMap();
  ClearancePlanner planner_{map_,DefaultNavigationFootprint()};
  State state_=State::Unreferenced;
  bool referenced_=false;
  RobotPose goal_{0,0,0},requestedPose_{0,0,0},progressPose_{0,0,0};
  double travelHeading_=0,started_=0,phaseStarted_=0,settled_=-1,effort_=0,pathLength_=0;
  double forward_=0,left_=0,yaw_=0;
  std::vector<ArenaPoint> path_;
  unsigned waypoint_=0;
  double lastStamp_[4]={-1,-1,-1,-1};
  std::vector<double> observations_[2];
  WaypointDrive point_;
  HeadingMotionControl heading_;
  std::string status_="Place in clear lower-left corner facing north, then Reference corner";
};

enum class TapeApproach { FromLeft, FromRight };

struct TapeRoutineInput {
  MotionInput motion;
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
  double FeedbackRequest() const { return heading_.FeedbackRequest(); }
  double FeedbackMeasured() const { return heading_.FeedbackMeasured(); }
  double GetScanTravelMM() const { return scanTravel_; }
  static bool PoseFromWalls(double leftGap,double rearGap,RobotPose& pose);
 private:
  void Stop() { forward_=left_=yaw_=0; }
  void Begin(const TapeRoutineInput& in);
  void SampleReference(const TapeRoutineInput& in);
  bool CobraHealthy(const TapeRoutineInput& in) const;
  bool Centered(const TapeRoutineInput& in) const;
  void LimitTranslation(double oldForward,double oldLeft,double acceleration,double dt);
  void DriveWaypoint(const TapeRoutineInput& in,double headingError);
  const char* ApproachName() const { return approach_==TapeApproach::FromLeft?"From left: ":"From right: "; }
  TapeApproach approach_=TapeApproach::FromLeft;
  State state_=State::Waiting;
  std::string status_="Supplied Cobra calibration ready; select Autonomous, enable, then choose a tape approach";
  RobotPose referencePose_{0,0,0},scanStart_{0,0,0},progressPose_{0,0,0};
  double forward_=0,left_=0,yaw_=0,stateTime_=0,scanTime_=0,settleTime_=-1,lostTime_=-1;
  double motionEffortTime_=0,scanTravel_=0;
  WaypointDrive point_;
  HeadingMotionControl heading_;
};
