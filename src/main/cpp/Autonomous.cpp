#include "Autonomous.h"

#include <algorithm>
#include <limits>

// Arena geometry and route planning.
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
double Padding() { return autonomous_config::ROBOT_ENVELOPE_RADIUS_MM + autonomous_config::CLEARANCE_MM; }
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

ArenaMap::ArenaMap() {
  using namespace autonomous_config;
  obstacles_.push_back({DIVIDER_X_MIN_MM,0,DIVIDER_X_MAX_MM,LOWER_DIVIDER_END_Y_MM,true});
  obstacles_.push_back({DIVIDER_X_MIN_MM,UPPER_DIVIDER_START_Y_MM,DIVIDER_X_MAX_MM,localization_config::ARENA_HEIGHT_MM,true});
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

// Mission state machine (disabled in RobotConfig.h).
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
