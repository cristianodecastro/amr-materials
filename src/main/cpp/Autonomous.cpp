#include "Autonomous.h"

#include <algorithm>
#include <limits>

// Arena geometry and route planning.
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
double Padding() { return autonomous_config::CLEARANCE_MM; }
ArenaRect Inflate(ArenaRect r) {
  const double p=Padding();
  return {r.x0-p,r.y0-p,r.x1+p,r.y1+p,r.trustedWall};
}
// Slab intersection, including tangency. Works for axis-aligned rays/segments.
bool Intersect(ArenaPoint p, ArenaPoint d, const ArenaRect& r, double& lo, double& hi) {
  const double origin[2]={p.x,p.y}, direction[2]={d.x,d.y};
  const double minimum[2]={r.x0,r.y0}, maximum[2]={r.x1,r.y1};
  for (int i=0;i<2;++i) {
    if (std::fabs(direction[i])<1e-9) {
      if (origin[i]<minimum[i] || origin[i]>maximum[i]) return false;
    } else {
      double a=(minimum[i]-origin[i])/direction[i];
      double b=(maximum[i]-origin[i])/direction[i];
      if (a>b) std::swap(a,b);
      lo=std::max(lo,a); hi=std::min(hi,b);
      if (lo>hi) return false;
    }
  }
  return hi>=lo;
}
}

ArenaMap::ArenaMap(bool includeStations) {
  using namespace autonomous_config;
  obstacles_.push_back({DIVIDER_X_MIN_MM,0,DIVIDER_X_MAX_MM,LOWER_DIVIDER_END_Y_MM,true});
  obstacles_.push_back({DIVIDER_X_MIN_MM,UPPER_DIVIDER_START_Y_MM,DIVIDER_X_MAX_MM,localization_config::ARENA_HEIGHT_MM,true});
  if (!includeStations) return;
  const double h=STATION_SIDE_MM/2.0;
  for (const auto& s:STORAGES) obstacles_.push_back({s.center.x-h,s.center.y-h,s.center.x+h,s.center.y+h,false});
  for (const auto& s:DELIVERIES) obstacles_.push_back({s.center.x-h,s.center.y-h,s.center.x+h,s.center.y+h,false});
}

bool ArenaMap::IsFree(ArenaPoint p) const {
  const double pad=Padding();
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x<pad || p.y<pad ||
      p.x>localization_config::ARENA_WIDTH_MM-pad || p.y>localization_config::ARENA_HEIGHT_MM-pad) return false;
  for (const auto& obstacle:obstacles_) {
    const auto r=Inflate(obstacle);
    if (p.x>=r.x0 && p.x<=r.x1 && p.y>=r.y0 && p.y<=r.y1) return false;
  }
  return true;
}

bool ArenaMap::SegmentFree(ArenaPoint a, ArenaPoint b) const {
  if (!IsFree(a) || !IsFree(b)) return false;
  for (const auto& obstacle:obstacles_) {
    double lo=0,hi=1;
    if (Intersect(a,{b.x-a.x,b.y-a.y},Inflate(obstacle),lo,hi)) return false;
  }
  return true;
}

bool ArenaMap::Plan(ArenaPoint start, ArenaPoint goal, std::vector<ArenaPoint>& path) const {
  path.clear();
  if (!IsFree(start) || !IsFree(goal)) return false;
  // Visibility graph over inflated obstacle corners; at most 42 nodes.
  // Leave room for waypoint tolerance; a waypoint must not permit corner cuts.
  std::vector<ArenaPoint> nodes={start,goal};
  for (const auto& obstacle:obstacles_) {
    const auto r=Inflate(obstacle);
    const double extra=autonomous_config::POSITION_TOLERANCE_MM+10.0;
    for (auto p : {ArenaPoint{r.x0-extra,r.y0-extra},ArenaPoint{r.x0-extra,r.y1+extra},
                   ArenaPoint{r.x1+extra,r.y0-extra},ArenaPoint{r.x1+extra,r.y1+extra}})
      if (IsFree(p)) nodes.push_back(p);
  }
  const int n=static_cast<int>(nodes.size());
  std::vector<double> cost(n,kInfinity);
  std::vector<int> parent(n,-1);
  std::vector<bool> visited(n,false);
  cost[0]=0;
  for (int iteration=0;iteration<n;++iteration) {
    int best=-1;
    for (int i=0;i<n;++i)
      if (!visited[i] && (best<0 || cost[i]<cost[best])) best=i;
    if (best<0 || !std::isfinite(cost[best])) break;
    if (best==1) break;
    visited[best]=true;
    for (int next=0;next<n;++next) {
      if (visited[next] || next==best || !SegmentFree(nodes[best],nodes[next])) continue;
      const double candidate=cost[best]+robot_geometry::Distance(nodes[best],nodes[next]);
      if (candidate<cost[next]) { cost[next]=candidate; parent[next]=best; }
    }
  }
  if (!std::isfinite(cost[1])) return false;
  for (int i=1;i!=0;i=parent[i]) {
    if (i<0) { path.clear(); return false; }
    path.push_back(nodes[i]);
  }
  std::reverse(path.begin(),path.end());
  // Regular stationary observation opportunities, including long straight legs.
  std::vector<ArenaPoint> observations;
  ArenaPoint previous=start;
  for (auto p:path) {
    const int steps=std::max(1,static_cast<int>(std::ceil(robot_geometry::Distance(previous,p)/600.0)));
    for (int i=1;i<=steps;++i) {
      const double fraction=static_cast<double>(i)/steps;
      observations.push_back({previous.x+(p.x-previous.x)*fraction,previous.y+(p.y-previous.y)*fraction});
    }
    previous=p;
  }
  path=std::move(observations);
  return true;
}

ArenaRayHit ArenaMap::Raycast(ArenaPoint origin, double angle) const {
  ArenaRayHit hit;
  if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(angle) ||
      origin.x<=0 || origin.y<=0 || origin.x>=localization_config::ARENA_WIDTH_MM ||
      origin.y>=localization_config::ARENA_HEIGHT_MM) return hit;
  const ArenaPoint d={std::cos(angle),std::sin(angle)};
  double nearest=kInfinity;
  // Outer-wall IDs 0..3; divider IDs 4,5; untrusted objects IDs >=6.
  const double width=localization_config::ARENA_WIDTH_MM, height=localization_config::ARENA_HEIGHT_MM;
  const double ranges[4]={d.x<-1e-9?-origin.x/d.x:kInfinity,
      d.x>1e-9?(width-origin.x)/d.x:kInfinity,
      d.y<-1e-9?-origin.y/d.y:kInfinity,
      d.y>1e-9?(height-origin.y)/d.y:kInfinity};
  for (int i=0;i<4;++i) if (ranges[i]<nearest) {
    nearest=ranges[i]; hit={nearest,i,true};
  }
  for (unsigned i=0;i<obstacles_.size();++i) {
    double lo=0,hi=kInfinity;
    if (Intersect(origin,d,obstacles_[i],lo,hi) && lo<nearest) {
      nearest=lo; hit={lo,static_cast<int>(i)+4,obstacles_[i].trustedWall};
    }
  }
  return hit;
}

ArenaRayHit ArenaMap::SonarHit(int sensor,const RobotPose& pose,double offset) const {
  if (sensor<0 || sensor>=4 || !robot_geometry::Finite(pose)) return {};
  const auto& mount=localization_config::SONARS[sensor];
  const double c=std::cos(pose.headingRad),s=std::sin(pose.headingRad);
  return Raycast({pose.x_MM+c*mount.mountX_MM-s*mount.mountY_MM,
      pose.y_MM+s*mount.mountX_MM+c*mount.mountY_MM},
      pose.headingRad+robot_geometry::Radians(mount.beamAngleDeg)+offset);
}

// SP -> MW -> RDW test routine.
namespace at = arena_test_config;
void ArenaTestController::Stop() { forward_=left_=yaw_=0; }
bool ArenaTestController::IsRunning() const {
  return state_>=State::TravelMW && state_<=State::FinishHeadingRDW;
}
void ArenaTestController::Fail(const std::string& reason) {
  Stop(); state_=State::Fault; status_=reason;
}
std::string ArenaTestController::ValidateGeometry() const {
  for (const auto& p:{at::SP,at::MW,at::RDW}) {
    if (!std::isfinite(p.headingDeg) || !map_.IsFree({p.x_MM,p.y_MM}))
      return "Invalid waypoint: outside arena or mapped center-path clearance";
  }
  if (!map_.SegmentFree({at::SP.x_MM,at::SP.y_MM},{at::MW.x_MM,at::MW.y_MM}) ||
      !map_.SegmentFree({at::MW.x_MM,at::MW.y_MM},{at::RDW.x_MM,at::RDW.y_MM}))
    return "SP -> MW -> RDW crosses a mapped wall";
  if (!(at::SPEED_MMPS>0 && at::ACCEL_MMPS2>0 && at::MAX_OUTPUT>0 && at::MAX_OUTPUT<=1 &&
        at::MAX_YAW_RADPS>0 && at::YAW_ACCEL_RADPS2>0 && at::POSITION_KP>0 && at::HEADING_KP>0 &&
        at::POSITION_TOLERANCE_MM>0 && at::HEADING_TOLERANCE_RAD>0 && at::SETTLE_S>=0 &&
        at::HOLD_HEADING_DEADBAND_RAD>=0 && at::HOLD_HEADING_DEADBAND_RAD<=at::HEADING_TOLERANCE_RAD &&
        at::HOLD_HEADING_MAX_YAW_RADPS>0 &&
        at::MIN_APPROACH_MMPS>0 && at::MIN_APPROACH_MMPS<=at::SPEED_MMPS &&
        at::MIN_TURN_YAW_RADPS>0 && at::MIN_TURN_YAW_RADPS<=at::MAX_YAW_RADPS &&
        at::REFERENCE_TIMEOUT_S>at::SETTLE_S && at::REFERENCE_SAMPLES>0 &&
        at::LEG_TIMEOUT_S>0 && at::ROUTINE_TIMEOUT_S>0))
    return "Invalid test speed, tolerance or timing configuration";
  return {};
}
void ArenaTestController::Enter(double now) {
  Stop(); target_=at::SP; stateTime_=now; missionTime_=now; settleTime_=-1;
  travelHeadingRad_=robot_geometry::Radians(at::SP.headingDeg);
  referencesSkipped_=false; lastReferenceReport_.clear();
  referenceReport_="No reference requested";
  for (int i=0;i<4;++i) { referenceSamples_[i]=0; previousCorrection_[i]=-1; }
  state_=at::ENABLED?State::WaitingStart:State::Disabled;
  status_=at::ENABLED?"Ready at SP - press Start routine":"Arena test disabled in configuration";
  const auto error=ValidateGeometry();
  if (at::ENABLED && !error.empty()) Fail(error);
}
void ArenaTestController::Abort() {
  Stop();
  if (IsRunning() || state_==State::WaitingStart) {
    state_=State::Aborted;
    status_="Stopped - select Autonomous again and place robot at SP before restart";
  }
}
void ArenaTestController::Transition(State state,double now,const char* status) {
  Stop(); state_=state; stateTime_=now; settleTime_=-1; status_=status;
  if (state==State::TravelMW) travelHeadingRad_=robot_geometry::Radians(at::SP.headingDeg);
  if (state==State::TravelRDW) travelHeadingRad_=robot_geometry::Radians(at::MW.headingDeg);
  if (state==State::FinishRDW) travelHeadingRad_=robot_geometry::Radians(at::RDW.headingDeg);
  for (int i=0;i<4;++i) { referenceSamples_[i]=0; previousCorrection_[i]=-1; }
}
bool ArenaTestController::Settled(bool reached,double now) {
  if (!reached) { settleTime_=-1; return false; }
  if (settleTime_<0) settleTime_=now;
  return now-settleTime_>=at::SETTLE_S;
}
unsigned ArenaTestController::ReferenceMask() const {
  return state_==State::ReferenceMW?at::MW_REFERENCE_MASK:
      (state_==State::ReferenceRDW?at::RDW_REFERENCE_MASK:0u);
}
double ArenaTestController::GetHeadingTargetDeg() const {
  if (state_==State::TravelMW || state_==State::TravelRDW || state_==State::FinishRDW)
    return travelHeadingRad_*180.0/robot_geometry::PI;
  return target_.headingDeg;
}
double ArenaTestController::HeadingRequest(double error,double oldYaw,double dt,bool hold) const {
  const double deadband=hold?at::HOLD_HEADING_DEADBAND_RAD:at::HEADING_TOLERANCE_RAD;
  if (std::fabs(error)<=deadband) return 0;
  const double limit=hold?std::min(at::MAX_YAW_RADPS,at::HOLD_HEADING_MAX_YAW_RADPS):at::MAX_YAW_RADPS;
  double request=std::max(-limit,std::min(limit,error*at::HEADING_KP));
  if (!hold && std::fabs(request)<at::MIN_TURN_YAW_RADPS)
    request=error<0?-at::MIN_TURN_YAW_RADPS:at::MIN_TURN_YAW_RADPS;
  const double step=at::YAW_ACCEL_RADPS2*dt;
  return oldYaw+std::max(-step,std::min(step,request-oldYaw));
}
int ArenaTestController::ReferenceSurface(int sensor) const {
  if (sensor<0 || sensor>=4 || !(ReferenceMask()&(1u<<sensor))) return -1;
  const RobotPose p={target_.x_MM,target_.y_MM,robot_geometry::Radians(target_.headingDeg)};
  const auto hit=map_.SonarHit(sensor,p);
  return hit.trustedWall?hit.surface:-1;
}
void ArenaTestController::Reference(const ArenaTestInput& in) {
  const unsigned mask=ReferenceMask();
  if (in.stationary && in.now-stateTime_>=at::SETTLE_S) {
    for (int i=0;i<4;++i) if ((mask&(1u<<i)) &&
        std::isfinite(in.correctionTime[i]) && in.correctionTime[i]>stateTime_+at::SETTLE_S &&
        in.correctionTime[i]>previousCorrection_[i] && in.correctionTime[i]<=in.now &&
        in.now-in.correctionTime[i]<=autonomous_config::SONAR_STALE_S) {
      previousCorrection_[i]=in.correctionTime[i];
      if (referenceSamples_[i]<at::REFERENCE_SAMPLES) ++referenceSamples_[i];
    }
  }
  bool done=true;
  referenceReport_=state_==State::ReferenceMW?"MW: ":"RDW: ";
  for (int i=0;i<4;++i) if (mask&(1u<<i)) {
    done=done && referenceSamples_[i]>=at::REFERENCE_SAMPLES;
    referenceReport_+=std::string(localization_config::SONARS[i].name)+" "+
        std::to_string(referenceSamples_[i])+"/"+std::to_string(at::REFERENCE_SAMPLES)+" accepted; ";
  }
  if (!done && in.now-stateTime_<at::REFERENCE_TIMEOUT_S) return;
  if (!done) {
    referencesSkipped_=true;
    referenceReport_+="Incomplete/invalid references skipped; retain odometry on those axes";
  }
  if (!lastReferenceReport_.empty()) lastReferenceReport_+=" | ";
  lastReferenceReport_+=referenceReport_;
  if (state_==State::ReferenceMW) {
    target_=at::RDW;
    Transition(State::TravelRDW,in.now,"MW observed - XY to RDW with gyro heading hold");
  } else {
    Transition(State::FinishRDW,in.now,"RDW observed - finish XY using the corrected pose");
  }
}
void ArenaTestController::Update(const ArenaTestInput& in) {
  const double oldForward=forward_,oldLeft=left_,oldYaw=yaw_;
  Stop();
  if (in.stopRequested) { Abort(); return; }
  if (!IsRunning() && state_!=State::WaitingStart) return;
  if (!robot_geometry::Finite(in.pose) || !std::isfinite(in.now) || !std::isfinite(in.dt) ||
      in.dt<=0 || in.dt>autonomous_config::MAX_CONTROL_GAP_S) {
    Fail("Invalid pose or stale control timing - stopped"); return;
  }
  if (!in.healthy) {
    if (IsRunning()) Fail("IMU/odometry unhealthy - stopped");
    else status_="Wait for IMU calibration/healthy odometry, then press Start routine";
    return;
  }
  if (state_==State::WaitingStart) {
    status_="Ready at SP - press Start routine";
    if (in.startRequested) {
      missionTime_=in.now; target_=at::MW;
      Transition(State::TravelMW,in.now,"XY: SP -> MW with gyro heading hold");
    }
    return;
  }
  if (in.now-missionTime_>at::ROUTINE_TIMEOUT_S) { Fail("Routine timeout - stopped"); return; }
  if (in.now-stateTime_>at::LEG_TIMEOUT_S) { Fail("Motion leg timeout - stopped"); return; }
  if (!map_.IsFree({in.pose.x_MM,in.pose.y_MM})) { Fail("Pose outside mapped center-path clearance"); return; }
  if (state_==State::ReferenceMW || state_==State::ReferenceRDW) { Reference(in); return; }
  if (state_==State::TravelMW || state_==State::TravelRDW || state_==State::FinishRDW) {
    const ArenaPoint current={in.pose.x_MM,in.pose.y_MM};
    if (!map_.SegmentFree(current,{target_.x_MM,target_.y_MM})) { Fail("XY route crosses a mapped wall"); return; }
    const double dx=target_.x_MM-in.pose.x_MM,dy=target_.y_MM-in.pose.y_MM;
    const double distance=std::hypot(dx,dy);
    const bool reached=distance<=at::POSITION_TOLERANCE_MM;
    // Correct small disturbances around the travel heading. The planned turn
    // to the waypoint's new heading still waits until XY motion has finished.
    const double holdError=robot_geometry::Wrap(travelHeadingRad_-in.pose.headingRad);
    yaw_=reached?0:HeadingRequest(holdError,oldYaw,in.dt,true);
    // At the XY goal, stop and let the following turn state correct heading.
    // Do not stall here requesting tiny yaw outputs against static friction.
    if (Settled(reached && in.stationary,in.now)) {
      const State next=state_==State::TravelMW?State::TurnMW:
          (state_==State::FinishRDW?State::FinishHeadingRDW:State::TurnRDW);
      Transition(next,in.now,
          state_==State::TravelMW?"MW XY complete - turn to MW heading":"RDW XY complete - turn to RDW heading");
      return;
    }
    if (reached) return;
    const double brakingSpeed=std::sqrt(2*at::ACCEL_MMPS2*
        std::max(0.0,distance-at::POSITION_TOLERANCE_MM));
    const double speed=std::min(at::SPEED_MMPS,std::max(at::MIN_APPROACH_MMPS,
        std::min(distance*at::POSITION_KP,brakingSpeed)));
    const double c=std::cos(in.pose.headingRad),s=std::sin(in.pose.headingRad);
    forward_=(c*dx+s*dy)*speed/distance;
    left_=(-s*dx+c*dy)*speed/distance;
    const double df=forward_-oldForward,dl=left_-oldLeft;
    const double delta=std::hypot(df,dl),step=at::ACCEL_MMPS2*in.dt;
    if (delta>step) { forward_=oldForward+df*step/delta; left_=oldLeft+dl*step/delta; }
    return;
  }
  const double error=robot_geometry::Wrap(robot_geometry::Radians(target_.headingDeg)-in.pose.headingRad);
  const bool reached=std::fabs(error)<=at::HEADING_TOLERANCE_RAD;
  if (Settled(reached && in.stationary,in.now)) {
    if (state_==State::FinishHeadingRDW) {
      Transition(State::Complete,in.now,referencesSkipped_?
          "SP -> MW -> RDW complete; some sensor corrections were unavailable":
          "SP -> MW -> RDW complete; MW and RDW references accepted");
      return;
    }
    Transition(state_==State::TurnMW?State::ReferenceMW:State::ReferenceRDW,in.now,
        state_==State::TurnMW?"MW: stationary right-sensor reference":"RDW: stationary right/rear reference");
    return;
  }
  if (reached) return;
  yaw_=HeadingRequest(error,oldYaw,in.dt,false);
  // Deliberately no XY command while turning.
}

// Future pickup/delivery mission (disabled in RobotConfig.h).
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
  if (!std::isfinite(CLEARANCE_MM) || CLEARANCE_MM<0 || !std::isfinite(STATION_SIDE_MM) || STATION_SIDE_MM<=0 ||
      !(STAGING_DISTANCE_MM>DOCK_DISTANCE_MM) || !(DOCK_DISTANCE_MM>0) ||
      UPPER_DIVIDER_START_Y_MM-LOWER_DIVIDER_END_Y_MM<=2*CLEARANCE_MM)
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
  if (!map_.IsFree(Point(in.pose))) { Fault("Estimated center leaves mapped path clearance"); return; }
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
