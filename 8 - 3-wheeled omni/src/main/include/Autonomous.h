#pragma once
#include <string>
#include <vector>



#include "RobotConfig.h"

struct MotionInput {
  RobotPose pose;
  double now, dt;
  bool healthy, startRequested, stopRequested;
  double yawRateRadps=0; // signed CCW gyro rate, independent of the arena yaw offset
};
class HeadingMotionControl {
 public:
  void Reset() { opposed_=commandTime_=lastRequest_=lastMeasured_=0; direction_=0; holdRecheck_=0; }
  double Request(double error,double oldYaw,double measuredYaw,double dt,
      double deadband=heading_control_config::HOLD_HEADING_DEADBAND_RAD,
      double gain=heading_control_config::HOLD_HEADING_KP,
      double maxYaw=heading_control_config::HOLD_HEADING_MAX_YAW_RADPS,double minYaw=0);
  std::string CheckFeedback(double previousRequest,double measuredYaw,double dt);
  double FeedbackRequest() const { return lastRequest_; }
  double FeedbackMeasured() const { return lastMeasured_; }
 private:
  double opposed_=0;
  double commandTime_=0,lastRequest_=0,lastMeasured_=0;
  double holdRecheck_=0;
  int direction_=0;
};

// Stop/settle position controller retained for the tape test only.
class WaypointDrive {
 public:
  void Reset(ArenaPoint target,double now,double speed,double acceleration,
      double initialForward=0,double initialLeft=0);
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
  // left IR, rear sonar, right IR, front sonar (same order as sensor mounts).
  double rangeMM[4]={-1,-1,-1,-1},stamp[4]={-1,-1,-1,-1};
  bool valid[4]={false,false,false,false};
  std::string healthIssue;
};
struct PoseInitialization {
  // Numeric X/Y are WALL coordinates; '-' preserves that pose coordinate.
  std::string xMM="-",yMM="-";
  std::string headingDeg="-";
  bool useArea=false;
  RobotPose areaPose={0,0,0}; // approximate region, for choosing wall-facing rays only
};
// Editable pose route with optional range-based XY odometry support.
class GoalNavigationRoutine {
 public:
  enum class State { Unreferenced, Reference, ReferenceReady, Ready, Driving,
    Complete, Aborted, Fault };
  void Enter();
  bool InitializePose(const PoseInitialization& settings);
  bool StartGoal(RobotPose goal,const NavigationInput& in,const std::string& intermediatePoses="");
  static std::string DefaultIntermediates(int source,int destination);
  void Update(const NavigationInput& in);
  void Abort();
  void Fail(const std::string& message);
  State GetState() const { return state_; }
  bool IsRunning() const { return state_==State::Reference || state_==State::ReferenceReady ||
      state_==State::Driving; }
  bool HasReference() const { return referenced_; }
  bool NeedsPoseReset() const { return poseResetPending_; }
  bool NeedsHeadingReset() const { return initialHeadingPending_; }
  const RobotPose& RequestedPose() const { return requestedPose_; }
  void ConfirmReference();
  const RobotPose& Goal() const { return goal_; }
  const std::string& Status() const { return status_; }
  const std::string& InitializationStatus() const { return initializationStatus_; }
  const std::string& WallSupportStatus() const { return wallStatus_; }
  ArenaPoint PositionCorrection() const { return positionCorrection_; }
  const RobotPose& ControlPose() const { return controlPose_; }
  double Forward() const { return forward_; }
  double Left() const { return left_; }
  double Yaw() const { return yaw_; }
  double RemainingMM() const { return remaining_; }
  unsigned IntermediateCount() const { return route_.empty()?0:route_.size()-1; }
  unsigned LegIndex() const { return leg_; }
 private:
  void Stop() { forward_=left_=yaw_=0; }
  void DriveRoute(const NavigationInput& in,double previousYaw);
  void AdvanceRoute(const NavigationInput& in);
  void CorrectFromWalls(NavigationInput& in);
  RobotPose SamplePose(double stamp) const;
  State state_=State::Unreferenced;
  bool referenced_=false;
  RobotPose goal_{0,0,0},requestedPose_{0,0,0},controlPose_{0,0,0};
  double settled_=-1,remaining_=0;
  double forward_=0,left_=0,yaw_=0;
  std::vector<RobotPose> route_;
  unsigned leg_=0;
  ArenaPoint legStart_={0,0};
  bool carryHeading_=false;
  bool transitHeadingActive_=false;
  bool finalToleranceActive_=false;
  double transitHeadingTarget_=0;
  double carriedHeading_=0,worldVX_=0,worldVY_=0;
  struct WallReference { int axis=-1; double coordinate=0; };
  WallReference walls_[4];
  struct PoseSample { double time; RobotPose pose; };
  std::vector<PoseSample> history_;
  double consumedStamp_[4]={-1,-1,-1,-1},wallTime_[2]={-1,-1};
  double rangeOffset_[2]={0,0},rangeTarget_[2]={0,0},rangeStamp_[2]={-1,-1};
  unsigned rangeSources_[2]={0,0};
  ArenaPoint positionCorrection_={0,0};
  bool initialPending_[2]={false,false},initialRequested_[2]={false,false};
  double initialWall_[2]={0,0},initialHeading_=0;
  bool initialHeadingPending_=false,poseResetPending_=false,initialUseArea_=false;
  RobotPose initialArea_={0,0,0};
  HeadingMotionControl heading_;
  std::string status_="Set initialization, then Initialize pose";
  std::string initializationStatus_="Set wall X/Y or '-' and heading or '-', then Initialize pose";
  std::string wallStatus_="Encoders + gyro; ranges support XY when usable";
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
