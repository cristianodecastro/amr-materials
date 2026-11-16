#include "Autonomous.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace {
double Dot(ArenaPoint a,ArenaPoint b) { return a.x*b.x+a.y*b.y; }
bool FinitePoint(ArenaPoint p) { return std::isfinite(p.x) && std::isfinite(p.y); }
struct FootprintBox { ArenaPoint center,xaxis,yaxis; double halfx,halfy; };
FootprintBox BoxAt(ArenaPoint p,double heading,const NavigationFootprint& f) {
  const double angle=robot_geometry::ArenaAngle(heading);
  const ArenaPoint x={std::cos(angle),std::sin(angle)},y={-x.y,x.x};
  const double offsetx=(f.frontMM-f.rearMM)/2,offsety=(f.leftMM-f.rightMM)/2;
  return {{p.x+x.x*offsetx+y.x*offsety,p.y+x.y*offsetx+y.y*offsety},x,y,
      (f.frontMM+f.rearMM)/2,(f.leftMM+f.rightMM)/2};
}
double BoxRadius(const FootprintBox& box,ArenaPoint axis) {
  return box.halfx*std::fabs(Dot(box.xaxis,axis))+box.halfy*std::fabs(Dot(box.yaxis,axis));
}
// SAT on the swept rectangle: axes include its edges, obstacle edges and
// travel direction. Checks the WHOLE segment, including thin walls/corners.
bool SweepHits(const FootprintBox& first,const FootprintBox& last,const ForbiddenRegion& r,double margin) {
  const ArenaPoint center={(r.xmin+r.xmax)/2,(r.ymin+r.ymax)/2};
  const double hx=(r.xmax-r.xmin)/2+margin,hy=(r.ymax-r.ymin)/2+margin;
  const ArenaPoint travel={last.center.x-first.center.x,last.center.y-first.center.y};
  const ArenaPoint axes[5]={{1,0},{0,1},first.xaxis,first.yaxis,{-travel.y,travel.x}};
  for (auto axis:axes) {
    if (std::hypot(axis.x,axis.y)<1e-9) continue;
    const double radius=BoxRadius(first,axis),obstacle=Dot(center,axis);
    const double obstacleRadius=hx*std::fabs(axis.x)+hy*std::fabs(axis.y);
    const double a=Dot(first.center,axis),b=Dot(last.center,axis);
    if (std::max(a,b)+radius<obstacle-obstacleRadius ||
        std::min(a,b)-radius>obstacle+obstacleRadius) return false;
  }
  return true;
}
}

NavigationMap DefaultNavigationMap() {
  NavigationMap map;
  map.widthMM=localization_config::ARENA_WIDTH_MM;
  map.heightMM=localization_config::ARENA_HEIGHT_MM;
  for (auto rect:navigation_config::FORBIDDEN) map.forbidden.push_back(rect);
  return map;
}
NavigationFootprint DefaultNavigationFootprint() {
  const double r=navigation_config::BODY_RADIUS_MM;
  return {r+navigation_config::MANIPULATOR_EXTENSION_MM,
      std::max(r,localization_config::REAR_SENSOR_OFFSET_MM),r,r};
}

WallRangeEstimate EstimateFromOuterWall(const NavigationMap& map,const RobotPose& pose,
    int sensor,double range,bool valid) {
  WallRangeEstimate result;
  if (!valid || sensor<0 || sensor>=4 || !robot_geometry::Finite(pose) || !std::isfinite(range) ||
      !std::isfinite(map.widthMM) || !std::isfinite(map.heightMM) || map.widthMM<=0 || map.heightMM<=0) return result;
  const bool ir=sensor==0 || sensor==2;
  const double minimum=ir?localization_config::IR_MIN_RANGE_MM:localization_config::SONAR_MIN_RANGE_MM;
  const double maximum=ir?localization_config::IR_MAX_RANGE_MM:localization_config::SONAR_MAX_RANGE_MM;
  if (range<minimum || range>maximum) return result;
  const auto& mount=localization_config::RANGE_SENSOR_MOUNTS[sensor];
  const double angle=robot_geometry::ArenaAngle(pose.headingRad),c=std::cos(angle),s=std::sin(angle);
  const ArenaPoint origin={pose.x_MM+c*mount.mountX_MM-s*mount.mountY_MM,
      pose.y_MM+s*mount.mountX_MM+c*mount.mountY_MM};
  if (origin.x<=0 || origin.x>=map.widthMM || origin.y<=0 || origin.y>=map.heightMM) return result;
  int wall=-1; double predicted=0; ArenaPoint direction={};
  // This is a conservative association guard, not a claimed sensor FOV.
  for (double guard:{-15.0,0.0,15.0}) {
    const double ray=angle+robot_geometry::Radians(mount.beamAngleDeg+guard);
    const ArenaPoint d={std::cos(ray),std::sin(ray)};
    const double inf=std::numeric_limits<double>::infinity();
    const double distances[4]={d.x< -1e-9?-origin.x/d.x:inf,d.x>1e-9?(map.widthMM-origin.x)/d.x:inf,
        d.y< -1e-9?-origin.y/d.y:inf,d.y>1e-9?(map.heightMM-origin.y)/d.y:inf};
    int hit=0;
    for (int i=1;i<4;++i) if (distances[i]<distances[hit]) hit=i;
    if (wall>=0 && wall!=hit) return result;
    wall=hit;
    const ArenaPoint end={origin.x+d.x*distances[hit],origin.y+d.y*distances[hit]};
    // A point swept along the beam intersects all red regions which could
    // contain an intervening object. These readings never correct position.
    const FootprintBox a={origin,{1,0},{0,1},0,0},b={end,{1,0},{0,1},0,0};
    for (auto region:map.forbidden) if (SweepHits(a,b,region,0)) return result;
    if (guard==0) { predicted=distances[hit]; direction=d; }
  }
  if (std::fabs(range-predicted)>60) return result;
  result.axis=wall<2?0:1;
  const double component=result.axis==0?direction.x:direction.y;
  if (std::fabs(component)<0.98) return {}; // near-normal views only
  result.coordinateMM=(result.axis==0?pose.x_MM:pose.y_MM)+(predicted-range)*component;
  result.valid=true;
  return result;
}

ClearancePlanner::ClearancePlanner(NavigationMap map,NavigationFootprint footprint,double clearanceMM,double gridMM)
    :map_(std::move(map)),footprint_(footprint),clearance_(clearanceMM),grid_(gridMM) {}
bool ClearancePlanner::ConfigurationValid() const {
  if (!std::isfinite(map_.widthMM) || !std::isfinite(map_.heightMM) ||
      !std::isfinite(clearance_) || !std::isfinite(grid_) ||
      map_.widthMM<=0 || map_.heightMM<=0 || clearance_<0 || grid_<5 ||
      (std::ceil(map_.widthMM/grid_)+1)*(std::ceil(map_.heightMM/grid_)+1)>1000000) return false;
  for (double v:{footprint_.frontMM,footprint_.rearMM,footprint_.leftMM,footprint_.rightMM})
    if (!std::isfinite(v) || v<=0) return false;
  for (const auto& r:map_.forbidden)
    if (!std::isfinite(r.xmin) || !std::isfinite(r.ymin) || !std::isfinite(r.xmax) || !std::isfinite(r.ymax) ||
        r.xmin<0 || r.ymin<0 || r.xmax>map_.widthMM || r.ymax>map_.heightMM ||
        r.xmin>=r.xmax || r.ymin>=r.ymax) return false;
  return true;
}
bool ClearancePlanner::SegmentFreeWithMargin(ArenaPoint a,ArenaPoint b,double heading,double margin) const {
  if (!ConfigurationValid() || !FinitePoint(a) || !FinitePoint(b) || !std::isfinite(heading)) return false;
  const auto first=BoxAt(a,heading,footprint_),last=BoxAt(b,heading,footprint_);
  const double rx=BoxRadius(first,{1,0}),ry=BoxRadius(first,{0,1});
  if (std::min(first.center.x,last.center.x)-rx<margin ||
      std::max(first.center.x,last.center.x)+rx>map_.widthMM-margin ||
      std::min(first.center.y,last.center.y)-ry<margin ||
      std::max(first.center.y,last.center.y)+ry>map_.heightMM-margin) return false;
  for (const auto& r:map_.forbidden) if (SweepHits(first,last,r,margin)) return false;
  return true;
}
bool ClearancePlanner::SegmentFree(ArenaPoint a,ArenaPoint b,double heading) const {
  return SegmentFreeWithMargin(a,b,heading,clearance_);
}
bool ClearancePlanner::RotationFree(ArenaPoint p,double from,double to) const {
  if (!ConfigurationValid() || !std::isfinite(from) || !std::isfinite(to)) return false;
  const double delta=robot_geometry::Wrap(to-from);
  const int steps=std::max(1,static_cast<int>(std::ceil(std::fabs(delta)/robot_geometry::Radians(0.25))));
  const double radius=std::hypot(std::max(footprint_.frontMM,footprint_.rearMM),
      std::max(footprint_.leftMM,footprint_.rightMM));
  // Every corner moves <= radius*half-step between samples. This added
  // margin covers the continuous rotation rather than just sampled angles.
  const double margin=clearance_+radius*std::fabs(delta)/(2*steps);
  for (int i=0;i<=steps;++i)
    if (!SegmentFreeWithMargin(p,p,from+delta*i/steps,margin)) return false;
  return true;
}
double ClearancePlanner::Clearance(ArenaPoint p,double heading) const {
  const auto box=BoxAt(p,heading,footprint_);
  const double rx=BoxRadius(box,{1,0}),ry=BoxRadius(box,{0,1});
  double gap=std::min(std::min(box.center.x-rx,map_.widthMM-box.center.x-rx),
      std::min(box.center.y-ry,map_.heightMM-box.center.y-ry));
  for (const auto& r:map_.forbidden) {
    const ArenaPoint center={(r.xmin+r.xmax)/2,(r.ymin+r.ymax)/2};
    const double hx=(r.xmax-r.xmin)/2,hy=(r.ymax-r.ymin)/2;
    double separation=0;
    for (auto axis:{ArenaPoint{1,0},ArenaPoint{0,1},box.xaxis,box.yaxis}) {
      const double d=std::fabs(Dot({box.center.x-center.x,box.center.y-center.y},axis))-
          BoxRadius(box,axis)-hx*std::fabs(axis.x)-hy*std::fabs(axis.y);
      separation=std::max(separation,d);
    }
    gap=std::min(gap,separation); // conservative clearance score for route cost
  }
  return gap;
}
double ClearancePlanner::SegmentCost(ArenaPoint a,ArenaPoint b,double heading) const {
  if (!SegmentFree(a,b,heading)) return std::numeric_limits<double>::infinity();
  const double length=std::hypot(b.x-a.x,b.y-a.y);
  const int samples=std::max(1,static_cast<int>(std::ceil(length/grid_)));
  double sum=0;
  for (int i=0;i<samples;++i) {
    const double t=(i+0.5)/samples;
    const double gap=std::max(0.0,Clearance({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)},heading)-clearance_);
    const double penalty=150/(150+gap);
    sum+=1+4*penalty*penalty;
  }
  return length*sum/samples;
}
NavigationPlan ClearancePlanner::Plan(RobotPose start,RobotPose goal) const {
  NavigationPlan result;
  const auto fail=[&result](const char* message) { result.error=message; return result; };
  if (!ConfigurationValid() || !robot_geometry::Finite(start) || !robot_geometry::Finite(goal))
    return fail("Invalid map, footprint or goal");
  const ArenaPoint begin={start.x_MM,start.y_MM},end={goal.x_MM,goal.y_MM};
  if (!SegmentFree(begin,begin,start.headingRad)) return fail("Starting footprint is outside free space");
  if (!SegmentFree(end,end,start.headingRad) || !RotationFree(end,start.headingRad,goal.headingRad))
    return fail("Goal footprint or final rotation is outside free space");
  if (std::hypot(end.x-begin.x,end.y-begin.y)<1e-8) { result.points={begin}; return result; }
  const int nx=static_cast<int>(std::floor(map_.widthMM/grid_))+1;
  const int ny=static_cast<int>(std::floor(map_.heightMM/grid_))+1,terminal=nx*ny;
  const auto point=[&](int i) { return i==terminal?end:ArenaPoint{(i%nx)*grid_,(i/nx)*grid_}; };
  const double inf=std::numeric_limits<double>::infinity();
  std::vector<double> cost(terminal+1,inf);
  std::vector<int> parent(terminal+1,-1);
  using QueueItem=std::pair<double,int>;
  std::priority_queue<QueueItem,std::vector<QueueItem>,std::greater<QueueItem>> open;
  const auto heuristic=[&](ArenaPoint p) { return std::hypot(p.x-end.x,p.y-end.y); };
  const int sx=static_cast<int>(std::round(begin.x/grid_)),sy=static_cast<int>(std::round(begin.y/grid_));
  for (int y=std::max(0,sy-2);y<=std::min(ny-1,sy+2);++y)
    for (int x=std::max(0,sx-2);x<=std::min(nx-1,sx+2);++x) {
      const int i=y*nx+x;
      cost[i]=SegmentCost(begin,point(i),start.headingRad);
      if (std::isfinite(cost[i])) open.push({cost[i]+heuristic(point(i)),i});
    }
  if (std::hypot(end.x-begin.x,end.y-begin.y)<=3*grid_) {
    cost[terminal]=SegmentCost(begin,end,start.headingRad);
    if (std::isfinite(cost[terminal])) open.push({cost[terminal],terminal});
  }
  while (!open.empty()) {
    const auto item=open.top(); open.pop(); const int current=item.second;
    const auto p=point(current);
    if (item.first>cost[current]+heuristic(p)+1e-8) continue;
    if (current==terminal) break;
    const auto relax=[&](int next) {
      const auto q=point(next);
      const double candidate=cost[current]+SegmentCost(p,q,start.headingRad);
      if (candidate<cost[next]) { cost[next]=candidate; parent[next]=current; open.push({candidate+heuristic(q),next}); }
    };
    if (heuristic(p)<=3*grid_) relax(terminal);
    for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx) {
      if (dx==0 && dy==0) continue;
      const int x=current%nx+dx,y=current/nx+dy;
      if (x>=0 && x<nx && y>=0 && y<ny) relax(y*nx+x);
    }
  }
  if (!std::isfinite(cost[terminal])) return fail("No route with the requested footprint and clearance");
  std::vector<ArenaPoint> raw;
  for (int i=terminal;i>=0;i=parent[i]) raw.push_back(point(i));
  raw.push_back(begin); std::reverse(raw.begin(),raw.end());
  // Clearance-aware shortcutting removes grid zigzags only when the entire
  // new segment is free and its weighted cost does not worsen the route.
  std::vector<double> prefix(raw.size(),0);
  for (unsigned i=1;i<raw.size();++i) prefix[i]=prefix[i-1]+SegmentCost(raw[i-1],raw[i],start.headingRad);
  result.points.push_back(begin);
  for (unsigned i=0;i+1<raw.size();) {
    unsigned chosen=i+1;
    for (unsigned j=raw.size()-1;j>i+1;--j)
      if (SegmentCost(raw[i],raw[j],start.headingRad)<=prefix[j]-prefix[i]+1e-8) { chosen=j; break; }
    const auto a=raw[i],b=raw[chosen];
    result.lengthMM+=std::hypot(b.x-a.x,b.y-a.y);
    result.cost+=SegmentCost(a,b,start.headingRad);
    result.points.push_back(b); i=chosen;
  }
  return result;
}

namespace at=heading_control_config;
double HeadingMotionControl::Request(double error,double oldYaw,double measuredYaw,double dt,
                                     double deadband,double gain) {
  holdRecheck_=std::max(0.0,holdRecheck_-dt);
  if (holdRecheck_>1e-9 && std::fabs(error)<at::HOLD_RECHECK_WINDOW_RAD) return 0;
  const double toward=(error<0?-1:1)*measuredYaw;
  // Stop applying turning power BEFORE coasting carries us through the
  // reference. A zero correction stays exactly zero through the output path.
  const bool coastingToward=toward>robot_geometry::Radians(at::STATIONARY_YAW_DPS) &&
      toward*(at::HOLD_BRAKE_RESPONSE_S+dt)>=std::fabs(error)-deadband;
  const double limit=at::HOLD_HEADING_MAX_YAW_RADPS;
  const double request=std::max(-limit,std::min(limit,
      error*gain-measuredYaw*at::HOLD_YAW_RATE_DAMPING));
  // Rate damping releases power; it must not command an opposite turn while
  // still on the same side of the reference. Once the error crosses zero,
  // discard the old turn ramp so it cannot push farther past the reference.
  if (std::fabs(error)<=deadband || coastingToward || request*error<=0) {
    // Allow a fresh heading sample before applying another correction.
    if (std::fabs(oldYaw)>0.0001) holdRecheck_=at::HOLD_RECHECK_S;
    return 0;
  }
  if (oldYaw*error<0) oldYaw=0;
  const double step=at::HOLD_YAW_ACCEL_RADPS2*dt;
  return oldYaw+std::max(-step,std::min(step,request-oldYaw));
}
std::string HeadingMotionControl::CheckFeedback(double previousRequest,double measuredYaw,double dt) {
  lastRequest_=previousRequest; lastMeasured_=measuredYaw;
  if (std::fabs(previousRequest)<=0.0001) { direction_=0; opposed_=commandTime_=0; return {}; }
  const int requestedDirection=previousRequest<0?-1:1;
  if (requestedDirection!=direction_) opposed_=commandTime_=0;
  direction_=requestedDirection;
  commandTime_+=dt;
  // Allow motor reversal and IMU acquisition latency before diagnosing a
  // sustained wrong-sign response. Do not count coasting into the fault angle.
  if (commandTime_<=at::YAW_RESPONSE_SETTLE_S+1e-9) return {};
  if (direction_!=0 && std::fabs(measuredYaw)>=at::YAW_DIRECTION_CHECK_MIN_RADPS) {
    if (direction_*measuredYaw<0) opposed_+=std::fabs(measuredYaw)*dt;
    else opposed_=0;
    if (opposed_>=at::YAW_DIRECTION_ERROR_RAD)
      return direction_<0?"Yaw direction mismatch: commanded CW, gyro reported CCW - stopped":
          "Yaw direction mismatch: commanded CCW, gyro reported CW - stopped";
  }
  return {};
}

namespace tt=tape_test_config;
namespace {
bool FreshSample(double now,double stamp,double maximumAge) {
  return std::isfinite(stamp) && stamp>=0 && now>=stamp && now-stamp<=maximumAge;
}
}
void TapeAlignmentRoutine::Enter() {
  *this=TapeAlignmentRoutine{};
}
void TapeAlignmentRoutine::Abort() {
  Stop();
  if (IsRunning() || state_==State::Waiting) {
    state_=State::Aborted; status_="Stopped; return to the reference corner before starting again";
  }
}
void TapeAlignmentRoutine::Fail(const std::string& reason) {
  Stop(); state_=State::Fault; status_=reason;
}
bool TapeAlignmentRoutine::PoseFromWalls(double leftGap,double rearGap,RobotPose& pose) {
  if (!std::isfinite(leftGap) || !std::isfinite(rearGap) ||
      leftGap<localization_config::IR_MIN_RANGE_MM || leftGap>localization_config::IR_MAX_RANGE_MM ||
      rearGap<localization_config::SONAR_MIN_RANGE_MM || rearGap>localization_config::SONAR_MAX_RANGE_MM) return false;
  const double heading=robot_geometry::Radians(tt::START_HEADING_DEG);
  const double theta=robot_geometry::ArenaAngle(heading),c=std::cos(theta),s=std::sin(theta);
  const auto& left=localization_config::RANGE_SENSOR_MOUNTS[0];
  const auto& rear=localization_config::RANGE_SENSOR_MOUNTS[1];
  const double leftRay=std::cos(theta+robot_geometry::Radians(left.beamAngleDeg));
  const double rearRay=std::sin(theta+robot_geometry::Radians(rear.beamAngleDeg));
  // These particular walls require left looking west and rear looking south.
  if (leftRay> -0.98 || rearRay> -0.98) return false;
  pose={tt::LEFT_WALL_X_MM-(c*left.mountX_MM-s*left.mountY_MM)-leftRay*leftGap,
      tt::REAR_WALL_Y_MM-(s*rear.mountX_MM+c*rear.mountY_MM)-rearRay*rearGap,
      robot_geometry::Wrap(heading)};
  return robot_geometry::Finite(pose) && pose.x_MM>=0 && pose.x_MM<=localization_config::ARENA_WIDTH_MM &&
      pose.y_MM>=0 && pose.y_MM<=localization_config::ARENA_HEIGHT_MM;
}
bool TapeAlignmentRoutine::CobraHealthy(const TapeRoutineInput& in) const {
  if (!in.cobraCalibrated || !in.line.communicationValid ||
      !FreshSample(in.motion.now,in.cobraTime,localization_config::SENSOR_STALE_S)) return false;
  for (int i=0;i<4;++i) {
    if (!std::isfinite(in.line.volts[i]) || in.line.volts[i]<0 ||
        in.line.volts[i]>cobra_config::COBRA_REFERENCE_V+0.05 ||
        !std::isfinite(in.blackLevel[i]) || in.blackLevel[i]<0 || in.blackLevel[i]>1) return false;
  }
  return true;
}
bool TapeAlignmentRoutine::Centered(const TapeRoutineInput& in) const {
  const auto& b=in.blackLevel;
  // Equality alone also occurs on all-white/all-black background. Require a
  // dark centre, lighter outside, and agreement of BOTH raw and calibrated data.
  return b[1]>=tt::CENTER_BLACK_MIN && b[2]>=tt::CENTER_BLACK_MIN &&
      std::min(b[1],b[2])-0.5*(b[0]+b[3])>=tt::CENTER_OUTER_CONTRAST &&
      std::fabs(b[1]-b[2])<=tt::CENTER_LEVEL_TOLERANCE &&
      std::fabs(in.line.volts[1]-in.line.volts[2])<=tt::CENTER_VOLTAGE_TOLERANCE_V;
}
void TapeAlignmentRoutine::Begin(const TapeRoutineInput& in) {
  if (in.approach!=TapeApproach::FromLeft && in.approach!=TapeApproach::FromRight) {
    status_="Choose a tape approach"; return;
  }
  approach_=in.approach;
  Stop(); heading_.Reset(); state_=State::Reference; stateTime_=in.motion.now;
  scanTravel_=0; settleTime_=-1;
  status_=std::string(ApproachName())+"reading left IR and rear sonar";
}
void TapeAlignmentRoutine::SampleReference(const TapeRoutineInput& in) {
  // Independent sensors update at different rates. Use their latest fresh,
  // valid readings once; no gyro stillness gate, pairing window, sample-count
  // requirement or reference timeout. Motors remain stopped while waiting.
  if (!in.leftValid || !FreshSample(in.motion.now,in.leftTime,localization_config::SENSOR_STALE_S) ||
      !std::isfinite(in.leftRangeMM) || in.leftRangeMM<localization_config::IR_MIN_RANGE_MM ||
      in.leftRangeMM>localization_config::IR_MAX_RANGE_MM) {
    status_="Waiting for left IR"; return;
  }
  if (!in.rearValid || !FreshSample(in.motion.now,in.rearTime,localization_config::SONAR_READING_STALE_S) ||
      !std::isfinite(in.rearRangeMM) || in.rearRangeMM<localization_config::SONAR_MIN_RANGE_MM ||
      in.rearRangeMM>localization_config::SONAR_MAX_RANGE_MM) {
    status_="Waiting for rear sonar"; return;
  }
  if (!PoseFromWalls(in.leftRangeMM,in.rearRangeMM,referencePose_)) {
    status_="Reference position is outside the arena"; return;
  }
  state_=State::ReferenceReady; status_="Starting pose ready";
}
void TapeAlignmentRoutine::ConfirmPoseReset(double now) {
  if (!NeedsPoseReset()) return;
  Stop(); heading_.Reset(); state_=State::DriveToPoint; stateTime_=now;
  point_.Reset(GetTarget(),now,tt::DRIVE_SPEED_MMPS,tt::DRIVE_ACCEL_MMPS2); motionEffortTime_=0;
  progressPose_=referencePose_; settleTime_=-1;
  status_=std::string(ApproachName())+"driving to approach waypoint; holding starting heading";
}
void TapeAlignmentRoutine::LimitTranslation(double oldForward,double oldLeft,double acceleration,double dt) {
  const double df=forward_-oldForward,dl=left_-oldLeft,delta=std::hypot(df,dl),step=acceleration*dt;
  if (delta>step) { forward_=oldForward+df*step/delta; left_=oldLeft+dl*step/delta; }
}
void WaypointDrive::Reset(ArenaPoint target,double now,double speed,double acceleration,
                         double initialForward,double initialLeft) {
  *this=WaypointDrive{}; target_=target; phaseTime_=now;
  maxSpeed_=speed; acceleration_=acceleration;
  forward_=filteredForward_=initialForward; left_=filteredLeft_=initialLeft;
}
void WaypointDrive::LimitTranslation(double oldForward,double oldLeft,double acceleration,double dt) {
  const double df=forward_-oldForward,dl=left_-oldLeft,delta=std::hypot(df,dl),step=acceleration*dt;
  if (delta>step) { forward_=oldForward+df*step/delta; left_=oldLeft+dl*step/delta; }
}
void WaypointDrive::Update(const MotionInput& m,double measuredForward,double measuredLeft,double headingError) {
  const double oldForward=forward_,oldLeft=left_;
  forward_=left_=0;
  if (reached_) return;
  const double velocityGain=m.dt/(tt::VELOCITY_FILTER_S+m.dt);
  filteredForward_+=velocityGain*(measuredForward-filteredForward_);
  filteredLeft_+=velocityGain*(measuredLeft-filteredLeft_);
  const auto target=target_;
  const double dx=target.x-m.pose.x_MM,dy=target.y-m.pose.y_MM,distance=std::hypot(dx,dy);
  const double angle=robot_geometry::ArenaAngle(m.pose.headingRad);
  const double c=std::cos(angle),s=std::sin(angle);
  const double forwardError=c*dx+s*dy,leftError=-s*dx+c*dy;
  const double measuredToward=distance>0?std::max(0.0,
      (filteredForward_*forwardError+filteredLeft_*leftError)/distance):0;
  const double toward=distance>0?std::max(0.0,std::max(
      (filteredForward_*forwardError+filteredLeft_*leftError)/distance,
      (oldForward*forwardError+oldLeft*leftError)/distance)):0;
  const double stoppingDistance=toward*toward/(2*acceleration_)+toward*(tt::BRAKE_RESPONSE_S+m.dt);
  if (waypointPhase_==WaypointPhase::Travel &&
      (distance<=tt::NEAR_TARGET_MM || stoppingDistance>=distance-tt::CAPTURE_RADIUS_MM)) {
    waypointPhase_=WaypointPhase::Brake; settleTime_=-1;
  }
  if (waypointPhase_==WaypointPhase::Correct) {
    const double along=dx*correctionX_+dy*correctionY_;
    if (along<=tt::CAPTURE_RADIUS_MM || distance<=tt::CAPTURE_RADIUS_MM ||
        (measuredToward>heading_control_config::STATIONARY_SPEED_MMPS && stoppingDistance>=along-tt::CAPTURE_RADIUS_MM) ||
        m.now-phaseTime_>=tt::CORRECTION_MAX_S) {
      waypointPhase_=WaypointPhase::Brake; settleTime_=-1;
    }
  }
  if (waypointPhase_==WaypointPhase::Brake) {
    // The chassis request decelerates to a true zero, which bypasses the
    // electrical minimum duty. No reversal is blended through a target.
    LimitTranslation(oldForward,oldLeft,acceleration_,m.dt);
    status_="Braking before waypoint; waiting for the chassis to settle";
    if (std::hypot(forward_,left_)<0.01) {
      forward_=left_=0; waypointPhase_=WaypointPhase::Settle; settleTime_=-1;
    }
    return;
  }
  if (waypointPhase_==WaypointPhase::Settle) {
    status_="Stopped at waypoint: verifying XY and heading";
    // Stop translation before remeasuring/reversing, but do not wait for yaw:
    // XY correction and heading hold may command the wheel mixer together.
    if (std::hypot(measuredForward,measuredLeft)>heading_control_config::STATIONARY_SPEED_MMPS) {
      settleTime_=-1; return;
    }
    if (settleTime_<0) settleTime_=m.now;
    if (distance<=tt::POSITION_TOLERANCE_MM) {
      // Heading gates completion only, never an outstanding XY correction.
      if (std::fabs(headingError)>heading_control_config::HEADING_TOLERANCE_RAD) {
        settleTime_=-1; return;
      }
      if (m.now-settleTime_>=tt::WAYPOINT_SETTLE_S) {
        reached_=true;
      }
      return;
    }
    if (m.now-settleTime_<tt::CORRECTION_REST_S) return;
    waypointPhase_=WaypointPhase::Correct; phaseTime_=m.now; settleTime_=-1;
    correctionX_=dx/distance; correctionY_=dy/distance;
  }
  if (waypointPhase_==WaypointPhase::Correct) {
    forward_=(c*correctionX_+s*correctionY_)*tt::MIN_APPROACH_MMPS;
    left_=(-s*correctionX_+c*correctionY_)*tt::MIN_APPROACH_MMPS;
    LimitTranslation(oldForward,oldLeft,acceleration_,m.dt);
    status_="Bounded waypoint correction; stop and remeasure before reversing";
    return;
  }
  const double brakingSpeed=std::sqrt(2*acceleration_*
      std::max(0.0,distance-tt::CAPTURE_RADIUS_MM));
  const double speed=std::min(maxSpeed_,std::max(tt::MIN_APPROACH_MMPS,
      std::min(distance*tt::POSITION_KP,brakingSpeed)));
  forward_=forwardError*speed/distance; left_=leftError*speed/distance;
  LimitTranslation(oldForward,oldLeft,acceleration_,m.dt);
  status_="driving to waypoint with heading hold and predictive braking";
}
void TapeAlignmentRoutine::DriveWaypoint(const TapeRoutineInput& in,double headingError) {
  point_.Update(in.motion,in.bodyForwardMMps,in.bodyLeftMMps,headingError);
  forward_=point_.Forward(); left_=point_.Left(); status_=point_.Status();
  if (point_.Reached()) {
    Stop(); heading_.Reset(); scanStart_=progressPose_=in.motion.pose;
    scanTime_=in.motion.now; motionEffortTime_=0; scanTravel_=0; lostTime_=-1; settleTime_=-1;
    state_=State::SearchTape;
    status_=std::string(ApproachName())+"at waypoint; searching inward for tape";
  }
}
void TapeAlignmentRoutine::Update(const TapeRoutineInput& in) {
  const auto& m=in.motion;
  const double oldForward=forward_,oldLeft=left_,oldYaw=yaw_;
  Stop();
  if (m.stopRequested) { Abort(); return; }
  if (!IsRunning() && !m.startRequested) return;
  if (!robot_geometry::Finite(m.pose) || !std::isfinite(m.now) || !std::isfinite(m.dt) ||
      !std::isfinite(m.yawRateRadps) || !std::isfinite(in.bodyForwardMMps) || !std::isfinite(in.bodyLeftMMps) ||
      m.dt<=0 || m.dt>localization_config::MAX_CONTROL_GAP_S || !m.healthy) {
    Fail("Unhealthy IMU/odometry, disconnected Control Station or stale timing - stopped"); return;
  }
  if (!IsRunning()) { Begin(in); return; }
  if (state_==State::Reference) { SampleReference(in); return; }
  if (state_==State::ReferenceReady) return; // wait for the explicit pose-reset handshake
  const auto yawFault=heading_.CheckFeedback(oldYaw,m.yawRateRadps,m.dt);
  if (!yawFault.empty()) { Fail(yawFault); return; }
  const double headingError=robot_geometry::Wrap(referencePose_.headingRad-m.pose.headingRad);
  yaw_=heading_.Request(headingError,oldYaw,m.yawRateRadps,m.dt);
  if (state_==State::DriveToPoint) {
    if (m.now-stateTime_>tt::DRIVE_TIMEOUT_S) { Fail(std::string(ApproachName())+"drive to waypoint timed out - stopped"); return; }
    DriveWaypoint(in,headingError);
  } else {
    if (!CobraHealthy(in)) {
      Stop(); heading_.Reset(); scanTime_+=m.dt; settleTime_=lostTime_=-1;
      status_="Waiting for Cobra reading"; return;
    }
    scanTravel_=std::hypot(m.pose.x_MM-scanStart_.x_MM,m.pose.y_MM-scanStart_.y_MM);
    if (scanTravel_>tt::SCAN_MAX_TRAVEL_MM || m.now-scanTime_>tt::SCAN_TIMEOUT_S) {
      Fail("Tape scan limit reached without a stable CH1/CH2 match - stopped"); return;
    }
    if (Centered(in)) {
      if (state_!=State::VerifyTape) { state_=State::VerifyTape; settleTime_=-1; }
      status_="CH1/CH2 match on tape: stopped, verifying alignment";
      if (std::hypot(in.bodyForwardMMps,in.bodyLeftMMps)<=heading_control_config::STATIONARY_SPEED_MMPS &&
          std::fabs(headingError)<=heading_control_config::HEADING_TOLERANCE_RAD) {
        if (settleTime_<0) settleTime_=m.now;
        if (m.now-settleTime_>=tt::TAPE_SETTLE_S) {
          Stop(); state_=State::Complete;
          status_=std::string(ApproachName())+"complete: reference -> waypoint -> tape centred on CH1/CH2";
        }
      } else settleTime_=-1;
      return;
    }
    if (state_==State::VerifyTape) {
      state_=State::CenterTape; settleTime_=-1;
    }
    const double peak=std::max(in.blackLevel[1],in.blackLevel[2]);
    if (peak>=tt::TAPE_ACQUIRE_LEVEL) {
      state_=State::CenterTape; lostTime_=-1;
    } else if (state_==State::CenterTape) {
      if (lostTime_<0) lostTime_=m.now;
      if (m.now-lostTime_>=tt::TAPE_LOST_S) { state_=State::SearchTape; lostTime_=-1; }
    }
    if (state_==State::SearchTape) {
      // At +Y heading, robot-right increases arena X; robot-left decreases X.
      left_=(approach_==TapeApproach::FromLeft?-1:1)*tt::SCAN_SPEED_MMPS;
      status_=std::string(ApproachName())+"scanning inward; equal background channels are ignored";
    } else {
      const double error=in.blackLevel[1]-in.blackLevel[2];
      // CH0 is leftmost: more black on CH1 means tape is left of centre.
      const double magnitude=std::min(tt::SCAN_SPEED_MMPS,
          std::max(tt::CENTER_MIN_SPEED_MMPS,std::fabs(error)*tt::CENTER_KP_MMPS));
      left_=error>0?magnitude:-magnitude;
      status_="Centring on tape using CH1/CH2; holding starting heading";
    }
    LimitTranslation(oldForward,oldLeft,tt::SCAN_ACCEL_MMPS2,m.dt);
  }
  if (std::hypot(m.pose.x_MM-progressPose_.x_MM,m.pose.y_MM-progressPose_.y_MM)>=tt::PROGRESS_MM) {
    progressPose_=m.pose; motionEffortTime_=0;
  } else if (std::hypot(forward_,left_)>20) {
    // Count only time actually requesting movement. Settling/braking pauses
    // must not make the next small correction fail immediately.
    motionEffortTime_+=m.dt;
    if (motionEffortTime_>tt::NO_PROGRESS_TIMEOUT_S)
      Fail(state_==State::DriveToPoint?"Waypoint correction stalled: check wheel/encoder response":
          "Tape scan stalled: check wheel/encoder response or configured scan speed");
  }
}
