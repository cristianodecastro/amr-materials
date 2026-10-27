#include "AutonomousController.h"

#include <algorithm>

using namespace autonomous_config;
using robot_geometry::Radians;
using robot_geometry::Wrap;
namespace {
double Clamp(double x,double limit) { return std::max(-limit,std::min(limit,x)); }
ArenaPoint Point(const RobotPose& p) { return {p.x_MM,p.y_MM}; }
}

void AutonomousController::Stop() { forward_=left_=yaw_=0; }
void AutonomousController::Fault(const char* reason) { state_=State::Fault; status_=reason; Stop(); }
bool AutonomousController::IsRunning() const {
  return state_>=State::NavigateStorage && state_<=State::RetreatDelivery;
}
void AutonomousController::Enter(double now,const std::string& error) {
  Stop(); route_.clear(); job_=completedJobs_=0; stationIsStorage_=true;
  startReleased_=false; pressTime_=-1; stateTime_=now;
  const std::string blocked=error.empty()?ValidateMissionGeometry():error;
  state_=blocked.empty()?State::WaitingStart:State::Blocked;
  status_=blocked.empty()?"Release Start, then press to begin":blocked;
}
std::string AutonomousController::ValidateMissionGeometry() const {
  if (!std::isfinite(ROBOT_ENVELOPE_RADIUS_MM) || ROBOT_ENVELOPE_RADIUS_MM<=0 ||
      !std::isfinite(CLEARANCE_MM) || CLEARANCE_MM<0 || !std::isfinite(STATION_SIDE_MM) || STATION_SIDE_MM<=0 ||
      !(STAGING_DISTANCE_MM>DOCK_DISTANCE_MM) || !(DOCK_DISTANCE_MM>0) ||
      UPPER_DIVIDER_START_Y_MM-LOWER_DIVIDER_END_Y_MM<=2*(ROBOT_ENVELOPE_RADIUS_MM+CLEARANCE_MM))
    return "Blocked: invalid dimensions or robot cannot fit through divider opening";
  ArenaPoint previous={localization_config::START_X_MM,localization_config::START_Y_MM};
  std::vector<ArenaPoint> route;
  for (const auto& job:JOBS) {
    if (job.storage<0 || job.storage>=4 || job.delivery<0 || job.delivery>=4)
      return "Blocked: invalid station assignment index";
    for (const auto station:{STORAGES[job.storage],DELIVERIES[job.delivery]}) {
      const auto stage=Approach(station,STAGING_DISTANCE_MM),dock=Approach(station,DOCK_DISTANCE_MM);
      if (!map_.Plan(previous,stage,route) || !map_.SegmentFree(stage,dock))
        return "Blocked: mission route/docking geometry is not traversable";
      previous=stage;
    }
  }
  return {};
}
void AutonomousController::Abort() {
  Stop();
  if (IsRunning() || state_==State::WaitingStart) {
    state_=State::Aborted; status_="Aborted: disabled or mode changed; reposition before restart";
  }
}
void AutonomousController::Transition(State state,double now,const char* status) {
  state_=state; stateTime_=now; settleTime_=-1; lineSamples_=0;
  lastLineTime_=now; status_=status; Stop();
}
const Station& AutonomousController::GetStation() const {
  const int index=stationIsStorage_?JOBS[job_].storage:JOBS[job_].delivery;
  const int safeIndex=std::max(0,std::min(3,index)); // dashboard remains safe on invalid config
  return stationIsStorage_?STORAGES[safeIndex]:DELIVERIES[safeIndex];
}
ObjectAction AutonomousController::GetObjectRequest() const {
  return state_==State::Pickup?ObjectAction::Pickup:
      (state_==State::Release?ObjectAction::Release:ObjectAction::None);
}
bool AutonomousController::CanCorrectFromLine() const {
  return (state_==State::DockStorage || state_==State::DockDelivery) && lineSamples_>=3;
}
bool AutonomousController::BeginRoute(const AutonomousInput& in,bool storage) {
  stationIsStorage_=storage;
  target_=Approach(GetStation(),STAGING_DISTANCE_MM);
  if (!map_.Plan(Point(in.pose),target_,route_)) { Fault("No collision-free route: check map, footprint and pose"); return false; }
  waypoint_=0; progressPoint_=Point(in.pose); progressTime_=in.nowSeconds;
  Transition(storage?State::NavigateStorage:State::NavigateDelivery,in.nowSeconds,
      storage?"Navigating to storage staging point":"Navigating to delivery staging point");
  return true;
}
bool AutonomousController::MoveTo(const AutonomousInput& in,ArenaPoint target,double heading,double speed) {
  target_=target;
  const double dx=target.x-in.pose.x_MM,dy=target.y-in.pose.y_MM;
  const double distance=std::hypot(dx,dy), error=Wrap(heading-in.pose.headingRad);
  yaw_=Clamp(error*HEADING_KP_PER_S,MAX_YAW_RADPS);
  if (distance<=POSITION_TOLERANCE_MM) forward_=left_=0;
  else {
    const double magnitude=std::min(speed,distance*POSITION_KP_PER_S);
    const double c=std::cos(in.pose.headingRad),s=std::sin(in.pose.headingRad);
    forward_=(c*dx+s*dy)*magnitude/distance;
    left_=(-s*dx+c*dy)*magnitude/distance;
  }
  return distance<=POSITION_TOLERANCE_MM && std::fabs(error)<=HEADING_TOLERANCE_RAD;
}
bool AutonomousController::Navigate(const AutonomousInput& in) {
  if (waypoint_>=route_.size()) { Fault("Invalid route state"); return false; }
  const auto goal=route_[waypoint_];
  // Revalidate the actual segment after every pose correction. No blind
  // corner cutting and no automatic replanning through uncertain objects.
  if (!map_.SegmentFree(Point(in.pose),goal)) { Fault("Route corridor lost after pose update"); return false; }
  const bool reached=MoveTo(in,goal,Radians(GetStation().headingDeg),CRUISE_MMPS);
  if (!reached) { settleTime_=-1; return false; }
  Stop(); // corners must be reached with zero commanded speed before turning
  if (settleTime_<0) settleTime_=in.nowSeconds;
  if (in.nowSeconds-settleTime_<WAYPOINT_OBSERVATION_S) return false;
  settleTime_=-1;
  if (++waypoint_<route_.size()) return false;
  return true;
}
void AutonomousController::Dock(const AutonomousInput& in,bool acquiring) {
  const auto& station=GetStation();
  const double h=Radians(station.headingDeg);
  const auto stage=Approach(station,STAGING_DISTANCE_MM);
  const auto dock=Approach(station,DOCK_DISTANCE_MM);
  // Cross-track association is limited to the expected station. Parallel
  // storage rows are only 130 mm apart; a wide search could select the wrong row.
  const double cross=-(in.pose.x_MM-stage.x)*std::sin(h)+(in.pose.y_MM-stage.y)*std::cos(h);
  if (std::fabs(cross)>LINE_WINDOW_MM) { Fault("Outside expected station line window"); return; }
  if (std::fabs(Wrap(h-in.pose.headingRad))>0.15) {
    MoveTo(in,Point(in.pose),h,0); lineSamples_=0; return;
  }
  if (!in.line.valid) {
    lineSamples_=0;
    if (!acquiring) {
      Stop();
      if (in.nowSeconds-lastLineTime_>LINE_LOST_TIMEOUT_S) Fault("Tape lost during docking");
      return;
    }
    if (in.nowSeconds-stateTime_>LINE_ACQUIRE_TIMEOUT_S) { Fault("Expected tape not acquired"); return; }
    // Bounded lateral sweep at staging, never an unrestricted forward search.
    const double offset=LINE_SEARCH_MM*std::sin((in.nowSeconds-stateTime_)*1.5);
    MoveTo(in,{stage.x-std::sin(h)*offset,stage.y+std::cos(h)*offset},h,LINE_SEARCH_MMPS);
    return;
  }
  lastLineTime_=in.nowSeconds;
  ++lineSamples_;
  if (acquiring) {
    Stop();
    if (lineSamples_>=3) {
      if (!map_.SegmentFree(Point(in.pose),dock)) { Fault("Dock approach is obstructed by configured geometry"); return; }
      Transition(stationIsStorage_?State::DockStorage:State::DockDelivery,in.nowSeconds,"Following station tape");
      progressPoint_=Point(in.pose); progressTime_=in.nowSeconds;
    }
    return;
  }
  if (!map_.SegmentFree(Point(in.pose),dock)) { Fault("Dock corridor lost"); return; }
  const double remaining=(dock.x-in.pose.x_MM)*std::cos(h)+(dock.y-in.pose.y_MM)*std::sin(h);
  if (remaining < -POSITION_TOLERANCE_MM) { Fault("Dock target overshot"); return; }
  const double along=std::max(0.0,std::min(DOCK_MMPS,remaining*POSITION_KP_PER_S));
  const double lateral=Clamp(LINE_KP_PER_S*in.line.leftErrorMM,LINE_MAX_LATERAL_MMPS);
  // Requested components are in station axes, then transformed into robot axes.
  const double delta=Wrap(h-in.pose.headingRad);
  forward_=std::cos(delta)*along-std::sin(delta)*lateral;
  left_=std::sin(delta)*along+std::cos(delta)*lateral;
  yaw_=Clamp(delta*HEADING_KP_PER_S,MAX_YAW_RADPS);
  if (remaining<=POSITION_TOLERANCE_MM && std::fabs(in.line.leftErrorMM)<=5 &&
      std::fabs(delta)<=HEADING_TOLERANCE_RAD) {
    Stop();
    if (settleTime_<0) settleTime_=in.nowSeconds;
    if (in.nowSeconds-settleTime_>=SETTLE_S)
      Transition(stationIsStorage_?State::Pickup:State::Release,in.nowSeconds,
          stationIsStorage_?"Waiting for confirmed pickup":"Waiting for confirmed release");
  } else settleTime_=-1;
}
void AutonomousController::LimitTranslation(double oldForward,double oldLeft,double dt) {
  const double dx=forward_-oldForward,dy=left_-oldLeft;
  const double change=std::hypot(dx,dy),limit=ACCEL_MMPS2*dt;
  if (change>limit && change>0) {
    forward_=oldForward+dx*limit/change; left_=oldLeft+dy*limit/change;
  }
}
void AutonomousController::Update(const AutonomousInput& in) {
  const double oldForward=forward_,oldLeft=left_,oldYaw=yaw_;
  Stop();
  if (state_==State::Blocked || state_==State::Disabled || state_==State::Fault ||
      state_==State::Aborted || state_==State::Complete) return;
  if (!robot_geometry::Finite(in.pose) || !std::isfinite(in.nowSeconds) ||
      !std::isfinite(in.dtSeconds) || in.dtSeconds<=0 || in.dtSeconds>MAX_CONTROL_GAP_S) {
    Fault("Unhealthy/stale odometry, IMU or control timing"); return;
  }
  if (!in.sensorsHealthy) {
    if (state_==State::WaitingStart) {
      startReleased_=false; pressTime_=-1;
      status_="Waiting for IMU, odometry and four fresh sonar readings";
    } else Fault("Unhealthy odometry or IMU");
    return;
  }
  if (!map_.IsFree(Point(in.pose))) { Fault("Estimated robot envelope intersects map"); return; }
  if (state_==State::WaitingStart) {
    status_="Release Start, then press to begin";
    if (!in.startPressed) { startReleased_=true; pressTime_=-1; }
    else if (startReleased_) {
      if (pressTime_<0) pressTime_=in.nowSeconds;
      if (in.nowSeconds-pressTime_>=localization_config::START_BUTTON_DEBOUNCE_S) {
        missionTime_=in.nowSeconds; BeginRoute(in,true);
      }
    }
    return;
  }
  if (in.nowSeconds-missionTime_>MISSION_TIMEOUT_S) { Fault("Mission timeout"); return; }
  if (in.nowSeconds-stateTime_>LEG_TIMEOUT_S) { Fault("Motion leg timeout"); return; }
  const State before=state_;
  switch (state_) {
    case State::NavigateStorage: case State::NavigateDelivery:
      if (Navigate(in)) Transition(stationIsStorage_?State::AcquireStorage:State::AcquireDelivery,
          in.nowSeconds,"Acquiring expected station tape");
      break;
    case State::AcquireStorage: case State::AcquireDelivery: Dock(in,true); break;
    case State::DockStorage: case State::DockDelivery: Dock(in,false); break;
    case State::Pickup: case State::Release:
      if (in.objectStatus==ObjectActionStatus::Unavailable) Fault("Pickup/release hardware is not implemented");
      else if (in.objectStatus==ObjectActionStatus::Failed) Fault("Pickup/release failed");
      else if (in.objectStatus==ObjectActionStatus::Complete) {
        Transition(stationIsStorage_?State::RetreatStorage:State::RetreatDelivery,in.nowSeconds,"Retreating along verified approach");
        progressPoint_=Point(in.pose); progressTime_=in.nowSeconds;
      } else if (in.nowSeconds-stateTime_>ACTION_TIMEOUT_S) Fault("Pickup/release confirmation timeout");
      break;
    case State::RetreatStorage: case State::RetreatDelivery: {
      const auto stage=Approach(GetStation(),STAGING_DISTANCE_MM);
      if (!map_.SegmentFree(Point(in.pose),stage)) { Fault("Retreat corridor obstructed"); break; }
      if (MoveTo(in,stage,Radians(GetStation().headingDeg),DOCK_MMPS)) {
        Stop();
        if (settleTime_<0) settleTime_=in.nowSeconds;
        if (in.nowSeconds-settleTime_>=SETTLE_S) {
          if (stationIsStorage_) BeginRoute(in,false);
          else {
            ++completedJobs_;
            if (completedJobs_==JOB_COUNT) Transition(State::Complete,in.nowSeconds,"All pickup and delivery actions confirmed");
            else { ++job_; BeginRoute(in,true); }
          }
        }
      } else settleTime_=-1;
      break;
    }
    default: break;
  }
  // Faults, state changes and loss of tape stop immediately, bypassing ramps.
  if (!IsRunning() || before!=state_) { Stop(); return; }
  const double magnitude=std::hypot(forward_,left_);
  if (magnitude>0) LimitTranslation(oldForward,oldLeft,in.dtSeconds);
  if (yaw_!=0) yaw_=oldYaw+Clamp(yaw_-oldYaw,YAW_ACCEL_RADPS2*in.dtSeconds);
  if (robot_geometry::Distance(Point(in.pose),progressPoint_)>=PROGRESS_MM || magnitude<30 ||
      state_==State::AcquireStorage || state_==State::AcquireDelivery) {
    progressPoint_=Point(in.pose); progressTime_=in.nowSeconds;
  } else if (in.nowSeconds-progressTime_>STALL_TIMEOUT_S) Fault("No encoder position progress while commanded to move");
}
