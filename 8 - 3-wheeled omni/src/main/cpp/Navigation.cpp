#include "Autonomous.h"
#include "TeleopControl.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace {
bool Fresh(double now,double stamp,double stale) {
  return std::isfinite(stamp) && stamp>=0 && now>=stamp && now-stamp<=stale;
}
}
void FrontClearanceControl::Reset() { *this=FrontClearanceControl{}; }
NavigationMap FrontClearanceControl::ObservedMap(const NavigationMap& map) const {
  auto observed=map;
  observed.forbidden.insert(observed.forbidden.end(),hits_.begin(),hits_.end());
  return observed;
}
NavigationMap FrontClearanceControl::WithHits(const NavigationMap& map) const {
  auto observed=ObservedMap(map);
  if (pendingCount_>0 && Fresh(lastNow_,pendingStamp_,localization_config::SONAR_READING_STALE_S))
    observed.forbidden.push_back(pending_);
  return observed;
}
void FrontClearanceControl::ReanchorPose(RobotPose pose,double now) {
  poses_.clear(); poses_.push_back({now,pose}); pendingCount_=0;
  for (auto& stamp:lastStamp_) stamp=now; // discard samples acquired before the localization correction
}
void FrontClearanceControl::Observe(const NavigationInput& in,const NavigationMap& map,bool mapReadings) {
  const auto& m=in.motion;
  lastNow_=m.now;
  if (poses_.empty() || m.now>poses_.back().now) poses_.push_back({m.now,m.pose});
  while (poses_.size()>1 && poses_[1].now<m.now-1.0) poses_.erase(poses_.begin());
  if (!mapReadings) return;
  for (int sensor=0;sensor<4;++sensor) ObserveSensor(in,map,sensor);
}
void FrontClearanceControl::ObserveSensor(const NavigationInput& in,const NavigationMap& map,int sensor) {
  const auto& m=in.motion; const bool ir=sensor==0 || sensor==2;
  const double stamp=in.stamp[sensor],range=in.rangeMM[sensor];
  if (!in.valid[sensor] || !std::isfinite(range) ||
      range<(ir?localization_config::IR_MIN_RANGE_MM:localization_config::SONAR_MIN_RANGE_MM) ||
      range>navigation_config::OBSTACLE_EVIDENCE_MAX_RANGE_MM || stamp<=lastStamp_[sensor] ||
      !Fresh(m.now,stamp,ir?localization_config::SENSOR_STALE_S:localization_config::SONAR_READING_STALE_S) ||
      stamp<poses_.front().now) return;
  lastStamp_[sensor]=stamp;
  RobotPose acquired=poses_.back().pose;
  for (unsigned i=1;i<poses_.size();++i) if (poses_[i].now>=stamp) {
    const auto a=poses_[i-1],b=poses_[i]; const double t=(stamp-a.now)/(b.now-a.now);
    acquired={a.pose.x_MM+t*(b.pose.x_MM-a.pose.x_MM),a.pose.y_MM+t*(b.pose.y_MM-a.pose.y_MM),
        robot_geometry::Wrap(a.pose.headingRad+t*robot_geometry::Wrap(b.pose.headingRad-a.pose.headingRad))};
    break;
  }
  const auto mount=localization_config::RANGE_SENSOR_MOUNTS[sensor];
  const double body=robot_geometry::ArenaAngle(acquired.headingRad);
  const double angle=body+robot_geometry::Radians(mount.beamAngleDeg);
  const ArenaPoint hit={acquired.x_MM+std::cos(body)*mount.mountX_MM-std::sin(body)*mount.mountY_MM+std::cos(angle)*range,
      acquired.y_MM+std::sin(body)*mount.mountX_MM+std::cos(body)*mount.mountY_MM+std::sin(angle)*range};
  if (hit.x<0 || hit.y<0 || hit.x>map.widthMM || hit.y>map.heightMM) return;
  // Mapped occupied space already explains the echo. Otherwise retain a
  // finite uncertain hit, not an invented infinite wall or an odometry reset.
  for (auto r:map.forbidden)
    if (hit.x>=r.xmin-5 && hit.x<=r.xmax+5 && hit.y>=r.ymin-5 && hit.y<=r.ymax+5) return;
  if (std::min(std::min(hit.x,map.widthMM-hit.x),std::min(hit.y,map.heightMM-hit.y))<10) return;
  const double radius=navigation_config::SONAR_HIT_UNCERTAINTY_MM+0.12*range;
  ForbiddenRegion patch={std::max(0.0,hit.x-radius),std::max(0.0,hit.y-radius),
      std::min(map.widthMM,hit.x+radius),std::min(map.heightMM,hit.y+radius)};
  // A hit just in front of a mapped horizontal face is evidence of that wall
  // being closer than its keep-out boundary. Use its known finite span, padded
  // by half the robot width; do not pretend the echo describes one small point.
  for (auto r:map.forbidden) if (r.ymin>acquired.y_MM && std::fabs(hit.y-r.ymin)<=navigation_config::WALL_ASSOCIATION_RESIDUAL_MM &&
      hit.x>=r.xmin-navigation_config::BODY_RADIUS_MM && hit.x<=r.xmax+navigation_config::BODY_RADIUS_MM &&
      std::sin(angle)>0.25) {
    patch={std::max(0.0,r.xmin-navigation_config::BODY_RADIUS_MM),std::max(0.0,hit.y-navigation_config::WALL_RANGE_ALLOWANCE_MM),
        std::min(map.widthMM,r.xmax+navigation_config::BODY_RADIUS_MM),std::min(map.heightMM,std::max(hit.y+20,r.ymin))};
    break;
  }
  for (auto& r:hits_) if (hit.x>=r.xmin && hit.x<=r.xmax && hit.y>=r.ymin && hit.y<=r.ymax) return;
  const bool consistent=pendingCount_>0 && stamp-pendingStamp_<=0.5 &&
      ((hit.x>=pending_.xmin && hit.x<=pending_.xmax && hit.y>=pending_.ymin && hit.y<=pending_.ymax) ||
      std::hypot(hit.x-(pending_.xmin+pending_.xmax)/2,hit.y-(pending_.ymin+pending_.ymax)/2)<=2*radius);
  pendingCount_=consistent?pendingCount_+1:1; pending_=patch; pendingStamp_=stamp;
  // A first measurement constrains motion, but an isolated outlier expires.
  // Only a second distinct consistent sample becomes a retained static obstacle.
  if (pendingCount_<2) return;
  if (hits_.size()>=navigation_config::MAX_OBSTACLE_PATCHES) {
    auto farthest=hits_.begin(); double distance=-1;
    for (auto it=hits_.begin();it!=hits_.end();++it) {
      const double d=std::hypot((it->xmin+it->xmax)/2-m.pose.x_MM,(it->ymin+it->ymax)/2-m.pose.y_MM);
      if (d>distance) { distance=d; farthest=it; }
    }
    hits_.erase(farthest);
  }
  hits_.push_back(patch);
  ++revision_;
  pendingCount_=0;
}
double FrontClearanceControl::SelectHeading(const NavigationInput& in,const NavigationMap& map,
                                            ArenaPoint preview,double previous,bool finalApproach) {
  namespace c=navigation_config;
  headingBlocked_=false;
  const auto physical=WithHits(map); const auto footprint=DefaultNavigationFootprint();
  const ClearancePlanner safety(physical,footprint);
  const ArenaPoint center={in.motion.pose.x_MM,in.motion.pose.y_MM};
  // Extra room is requested only from obstacles ahead of preferred north.
  // Existing map geometry also protects manipulator corners outside one sonar beam.
  const auto frontMap=[&](double gap) {
    auto guarded=physical; const double extra=std::max(0.0,gap-c::MIN_CLEARANCE_MM);
    for (auto r:physical.forbidden) {
      const bool overlap=r.xmax>std::min(center.x,preview.x)-footprint.leftMM &&
          r.xmin<std::max(center.x,preview.x)+footprint.rightMM;
      if (!overlap || r.ymin<std::min(center.y,preview.y) ||
          r.ymin-std::max(center.y,preview.y)-footprint.frontMM>c::FRONT_TRIGGER_GAP_MM) continue;
      // A centre beam cannot establish the exact end of a wooden wall. Pad
      // its lateral span by half the body width to cover blind front corners.
      guarded.forbidden.push_back({std::max(0.0,r.xmin-std::max(extra,footprint.leftMM)),std::max(0.0,r.ymin-extra),
          std::min(map.widthMM,r.xmax+std::max(extra,footprint.rightMM)),std::min(map.heightMM,r.ymax+extra)});
    }
    if (extra>0 && map.heightMM-std::max(center.y,preview.y)-footprint.frontMM<c::FRONT_TRIGGER_GAP_MM)
      guarded.forbidden.push_back({0,map.heightMM-extra,map.widthMM,map.heightMM});
    return guarded;
  };
  const double desired=finalApproach?c::MIN_CLEARANCE_MM:c::FRONT_TRAVEL_GAP_MM;
  const ClearancePlanner returning(frontMap(desired+(active_&&!finalApproach?c::FRONT_RETURN_HYSTERESIS_MM:0)),footprint);
  const auto allowed=[&](double h,const ClearancePlanner& space) {
    return space.SegmentFree(center,preview,h) &&
        safety.RotationFree(center,in.motion.pose.headingRad,h);
  };
  if (allowed(c::PREFERRED_HEADING_RAD,returning)) { active_=false; return c::PREFERRED_HEADING_RAD; }
  active_=true;
  const double first=preview.x>=center.x?-1:1;
  // Smallest feasible deviation from north wins. Direction follows the route
  // only as a tie-breaker; there is no predetermined -90-degree crossing.
  for (double gap=desired;gap>=c::MIN_CLEARANCE_MM-1e-9;gap-=10) {
    const ClearancePlanner space(frontMap(gap),footprint);
    for (double delta=gap==desired?c::HEADING_SEARCH_STEP_RAD:0;delta<=robot_geometry::PI/2+1e-9;delta+=c::HEADING_SEARCH_STEP_RAD)
      for (double sign:{first,-first}) {
        const double h=robot_geometry::Wrap(c::PREFERRED_HEADING_RAD+sign*delta);
        if (allowed(h,space)) return h;
      }
  }
  headingBlocked_=true;
  return previous; // make room before swinging a wide manipulator
}
void FrontClearanceControl::LimitMotion(const NavigationInput& in,const NavigationMap& map,
                                       double& forward,double& left,double& yaw,double headingTarget) {
  namespace c=navigation_config;
  limited_=false;
  if (std::hypot(forward,left)<0.01 && std::fabs(yaw)<0.0001) return;
  const auto physical=WithHits(map);
  const auto& m=in.motion;
  const ClearancePlanner physicalSpace(physical,DefaultNavigationFootprint(),0);
  const double startingGap=physicalSpace.ExtraClearanceMM({m.pose.x_MM,m.pose.y_MM},m.pose.headingRad);
  const bool escaping=startingGap<c::MIN_CLEARANCE_MM;
  const ClearancePlanner safety(physical,DefaultNavigationFootprint(),
      escaping?std::max(0.0,startingGap-2):c::MIN_CLEARANCE_MM);
  if (!active_ && hits_.empty() && safety.ExtraClearanceMM({m.pose.x_MM,m.pose.y_MM},m.pose.headingRad)>150) return;
  const double angle=robot_geometry::ArenaAngle(m.pose.headingRad),co=std::cos(angle),si=std::sin(angle);
  const double vx=co*forward-si*left,vy=si*forward+co*left;
  const auto safe=[&](double x,double y,double turn,double commandLag) {
    const double f=co*x+si*y,l=-si*x+co*y;
    double wheel[3];
    for (int i=0;i<3;++i) wheel[i]=drive_kinematics::WheelSpeedMMps(i,f,l,turn);
    const double denominator=drive_kinematics::OutputDenominator(wheel,localization_config::TELEOP_MAX_MOTOR_OUTPUT);
    double output[3];
    for (int i=0;i<3;++i) output[i]=std::fabs(wheel[i])<=wheel_drive_config::ZERO_SPEED_MMPS?0:wheel[i]/denominator;
    motor_output::PrepareAutonomousVector(output,localization_config::TELEOP_MAX_MOTOR_OUTPUT);
    for (int i=0;i<3;++i) wheel[i]=std::fabs(wheel[i])<=wheel_drive_config::ZERO_SPEED_MMPS?0:
        motor_output::WheelTitanPercent(i,output[i],localization_config::TELEOP_MAX_MOTOR_OUTPUT)*
        localization_config::WHEEL_MOTOR_SIGN[i]*localization_config::MAX_WHEEL_SPEED_MMPS/100.0;
    const double targetYaw=(wheel[0]+wheel[1]+wheel[2])/
        (localization_config::WHEEL_CENTER_RADIUS_MM[0]+localization_config::WHEEL_CENTER_RADIUS_MM[1]+localization_config::WHEEL_CENTER_RADIUS_MM[2]);
    double targetF=0,targetL=0;
    if (!drive_kinematics::BodyDisplacement(wheel,targetYaw,targetF,targetL)) return false;
    auto pose=m.pose; double measuredF=in.forwardMMps,measuredL=in.leftMMps,rate=m.yawRateRadps;
    const double horizon=std::max(m.dt,c::GUARD_HORIZON_S);
    bool stopYaw=false;
    for (double t=0;t<horizon+3*c::GUARD_BRAKE_LAG_S-1e-9;t+=c::GUARD_STEP_S) {
      if (std::hypot(f,l)<0.01 && std::fabs(turn)>0.0001 && std::isfinite(headingTarget)) {
        const double error=robot_geometry::Wrap(headingTarget-pose.headingRad);
        const double toward=(error<0?-1:1)*rate;
        if (error*turn<=0 || (toward>0 && toward*(heading_control_config::HOLD_BRAKE_RESPONSE_S+c::GUARD_STEP_S)>=
            std::max(0.0,std::fabs(error)-heading_control_config::HOLD_HEADING_DEADBAND_RAD))) stopYaw=true;
      }
      const bool braking=t>=horizon || stopYaw || std::hypot(targetF,targetL)+std::fabs(targetYaw)<0.0001;
      const double lag=braking?c::GUARD_BRAKE_LAG_S:commandLag;
      const double gain=c::GUARD_STEP_S/(lag+c::GUARD_STEP_S);
      measuredF+=gain*((braking?0:targetF)-measuredF);
      measuredL+=gain*((braking?0:targetL)-measuredL); rate+=gain*((braking?0:targetYaw)-rate);
      const double mid=robot_geometry::ArenaAngle(pose.headingRad+rate*c::GUARD_STEP_S/2);
      const RobotPose next={pose.x_MM+(std::cos(mid)*measuredF-std::sin(mid)*measuredL)*c::GUARD_STEP_S,
          pose.y_MM+(std::sin(mid)*measuredF+std::cos(mid)*measuredL)*c::GUARD_STEP_S,
          robot_geometry::Wrap(pose.headingRad+rate*c::GUARD_STEP_S)};
      if (!safety.MotionFree({pose.x_MM,pose.y_MM},{next.x_MM,next.y_MM},pose.headingRad,next.headingRad)) return false;
      pose=next;
    }
    return !escaping || physicalSpace.ExtraClearanceMM({pose.x_MM,pose.y_MM},pose.headingRad)>startingGap+1;
  };
  struct Request { double x,y,w; };
  const bool translating=std::hypot(forward,left)>0.01;
  const Request candidates[]={ {vx,vy,yaw},{vx,0,yaw},{0,vy,yaw},{vx*0.5,vy*0.5,yaw},{vx*0.5,0,yaw},
      {0,0,yaw},{vx,vy,0},{vx,0,0},{0,vy,0},{vx*0.5,vy*0.5,0},
      {0,translating?-c::CLEARANCE_RECOVERY_SPEED_MMPS:0,yaw},{0,translating?-c::CLEARANCE_RECOVERY_SPEED_MMPS:0,0},{0,0,0} };
  for (auto request:candidates) if (safe(request.x,request.y,request.w,c::GUARD_MOTOR_LAG_S) &&
      safe(request.x,request.y,request.w,c::GUARD_BRAKE_LAG_S)) {
    limited_=std::hypot(request.x-vx,request.y-vy)>0.01 || std::fabs(request.w-yaw)>0.0001;
    forward=co*request.x+si*request.y; left=-si*request.x+co*request.y; yaw=request.w; return;
  }
  forward=left=yaw=0; limited_=true;
}
void GoalNavigationRoutine::Enter() { *this=GoalNavigationRoutine{}; }
void GoalNavigationRoutine::Abort() { Stop(); state_=State::Aborted; status_="Navigation stopped"; }
void GoalNavigationRoutine::Fail(const std::string& message) { Stop(); state_=State::Fault; status_=message; }
void GoalNavigationRoutine::ReferenceCorner(double now) {
  Stop(); heading_.Reset(); referenced_=false; state_=State::Reference; phaseStarted_=now;
  clearance_.Reset(); obstacleRevision_=0; obstacleReplanPending_=false;
  planner_=ClearancePlanner(map_,DefaultNavigationFootprint());
  status_="Reading clear left/bottom outer walls; robot must face north (0 deg)";
}
void GoalNavigationRoutine::ConfirmReference() {
  if (!NeedsPoseReset()) return;
  referenced_=true; state_=State::Ready; status_="Corner referenced; set goal and press Go to goal";
}
bool GoalNavigationRoutine::StartGoal(RobotPose goal,const NavigationInput& in) {
  if (IsRunning()) { status_="Navigation already running; Stop before selecting another goal"; return false; }
  if (!referenced_) { status_="Reference corner first"; return false; }
  if (!robot_geometry::Finite(goal)) { status_="Goal must contain finite X/Y/heading values"; return false; }
  goal_=goal; goal_.headingRad=robot_geometry::Wrap(goal_.headingRad);
  started_=in.motion.now; path_.clear(); pathLength_=0; waypoint_=0;
  NeedNewPlan(in); return true;
}
void GoalNavigationRoutine::NeedNewPlan(const NavigationInput& in) {
  Stop(); heading_.Reset(); travelHeading_=in.motion.pose.headingRad;
  state_=State::NeedPlan; status_="Planning route with full footprint and 40 mm clearance";
}
void GoalNavigationRoutine::AcceptPlan(NavigationPlan plan,const NavigationInput& in) {
  if (state_!=State::Planning) return;
  if (!plan.Valid()) { Fail(plan.error); return; }
  if (std::hypot(plan.points.front().x-in.motion.pose.x_MM,plan.points.front().y-in.motion.pose.y_MM)>10) {
    NeedNewPlan(in); return;
  }
  // Sensor evidence may have changed while the worker was planning.
  double checkedHeading=plan.travelHeadingRad; unsigned turn=0;
  for (unsigned i=0;i<plan.points.size();++i) {
    if (!planner_.SegmentFree(i?plan.points[i-1]:plan.points[i],plan.points[i],checkedHeading)) {
      NeedNewPlan(in); return;
    }
    if (turn<plan.turns.size() && plan.turns[turn].pointIndex==i) {
      if (!planner_.RotationFree(plan.points[i],checkedHeading,plan.turns[turn].headingRad)) { NeedNewPlan(in); return; }
      checkedHeading=plan.turns[turn++].headingRad;
    }
  }
  if (turn!=plan.turns.size()) { Fail("Invalid clearance transitions in route"); return; }
  // Keep the planner's collision-checked vertices. Do not insert artificial
  // precision stops along otherwise straight segments.
  path_=std::move(plan.points);
  travelHeading_=plan.travelHeadingRad;
  turns_=std::move(plan.turns); nextTurn_=0;
  legEnd_=turns_.empty()?path_.size()-1:turns_[0].pointIndex;
  pathLength_=plan.lengthMM; waypoint_=1; effort_=0; progressPose_=in.motion.pose;
  preciseApproach_=false;
  const double angle=robot_geometry::ArenaAngle(in.motion.pose.headingRad);
  routeVX_=std::cos(angle)*in.forwardMMps-std::sin(angle)*in.leftMMps;
  routeVY_=std::sin(angle)*in.forwardMMps+std::cos(angle)*in.leftMMps;
  if (!turns_.empty() && legEnd_==0) { BeginClearanceTurn(in); return; }
  if (path_.size()==1) { BeginObservation(in.motion.now); return; }
  state_=State::Following; status_="Following clearance route without intermediate stops";
}
void GoalNavigationRoutine::BeginClearanceTurn(const NavigationInput& in) {
  Stop(); heading_.Reset(); routeVX_=routeVY_=0;
  if (!planner_.RotationFree({in.motion.pose.x_MM,in.motion.pose.y_MM},in.motion.pose.headingRad,turns_[nextTurn_].headingRad)) {
    NeedNewPlan(in); return;
  }
  state_=State::ClearanceTurn;
  status_="Geometry-selected clearance turn to "+std::to_string(std::lround(turns_[nextTurn_].headingRad*180/robot_geometry::PI))+" deg";
}
void GoalNavigationRoutine::BeginObservation(double now) {
  Stop(); state_=State::Observing; phaseStarted_=now;
  observations_[0].clear(); observations_[1].clear();
  for (auto& time:lastStamp_) time=-1;
  status_="Settling at destination before checking outer-wall readings";
}
void GoalNavigationRoutine::ContinueAfterObservation(const NavigationInput& in) {
  const auto& p=in.motion.pose;
  if (std::hypot(goal_.x_MM-p.x_MM,goal_.y_MM-p.y_MM)<=navigation_config::POSITION_TOLERANCE_MM) {
    if (!planner_.RotationFree({p.x_MM,p.y_MM},p.headingRad,goal_.headingRad)) {
      Fail("No clearance for requested final rotation"); return;
    }
    state_=State::FinalHeading; heading_.Reset(); settled_=-1;
    status_="Position reached; settling at goal heading"; return;
  }
  NeedNewPlan(in); // Only a final position correction may require more travel.
}
double GoalNavigationRoutine::TravelHeadingDeadband(ArenaPoint center) const {
  const double relaxed=navigation_config::TRAVEL_HEADING_DEADBAND_RAD;
  if (waypoint_<path_.size()) {
    // Relax yaw only where the full swept footprint can tolerate it. The
    // narrow passage can require closer heading control without an XY stop.
    for (int i=-3;i<=3;++i)
      if (!planner_.SegmentFree(center,path_[waypoint_],travelHeading_+relaxed*i/3))
        return navigation_config::TIGHT_HEADING_DEADBAND_RAD;
  }
  return relaxed;
}
void GoalNavigationRoutine::FollowRoute(const NavigationInput& in) {
  namespace c=navigation_config;
  const auto& m=in.motion;
  const ArenaPoint center={m.pose.x_MM,m.pose.y_MM};
  const auto free=[&](ArenaPoint p) {
    // The local rollout checks motion at the CURRENT heading while it turns.
    // Requiring the whole future segment to fit that heading would force stops.
    return planner_.SegmentFree(center,p,travelHeading_);
  };
  // Passing a guide requires a clear shortcut to the next segment, not exact
  // XY/heading arrival. Otherwise keep approaching the original safe vertex.
  while (waypoint_<legEnd_) {
    const auto a=path_[waypoint_-1],b=path_[waypoint_];
    const bool passed=(center.x-b.x)*(b.x-a.x)+(center.y-b.y)*(b.y-a.y)>=0;
    const bool near=std::hypot(b.x-center.x,b.y-center.y)<=c::PASS_RADIUS_MM;
    if (!passed && !near) break;
    if (!free(path_[waypoint_+1])) {
      // A sensor-driven turn may need a different centreline through a gap.
      // Replan from the live pose instead of chasing a guide already behind us.
      if (passed) { NeedNewPlan(in); return; }
      break;
    }
    ++waypoint_;
  }
  const bool waypointFree=free(path_[waypoint_]);
  const double distance=std::hypot(path_[waypoint_].x-center.x,path_[waypoint_].y-center.y);
  if (!preciseApproach_ && waypoint_==legEnd_ && distance<=c::FINAL_APPROACH_MM) {
    preciseApproach_=true;
    const double angle=robot_geometry::ArenaAngle(m.pose.headingRad);
    const double initialForward=std::cos(angle)*routeVX_+std::sin(angle)*routeVY_;
    const double initialLeft=-std::sin(angle)*routeVX_+std::cos(angle)*routeVY_;
    point_.Reset(path_[legEnd_],m.now,c::FINAL_SPEED_MMPS,c::ACCEL_MMPS2,initialForward,initialLeft,
        nextTurn_<turns_.size()?c::RECOVERY_POSITION_TOLERANCE_MM:c::POSITION_TOLERANCE_MM);
  }
  if (preciseApproach_) {
    if (!waypointFree) { NeedNewPlan(in); return; }
    // Precision is for the final XY position. Heading correction continues in
    // parallel; only FinalHeading enforces the destination's 1-degree angle.
    point_.Update(m,in.forwardMMps,in.leftMMps,0);
    forward_=point_.Forward(); left_=point_.Left();
    status_=(nextTurn_<turns_.size()?"Clearance reposition: ":"Final XY approach: ")+point_.Status();
    if (point_.Reached()) {
      if (nextTurn_<turns_.size()) BeginClearanceTurn(in);
      else BeginObservation(m.now);
    }
    return;
  }
  const auto a=path_[waypoint_-1],b=path_[waypoint_];
  const double dx=b.x-a.x,dy=b.y-a.y,length2=dx*dx+dy*dy;
  const double t=length2>0?std::max(0.0,std::min(1.0,((center.x-a.x)*dx+(center.y-a.y)*dy)/length2)):0;
  ArenaPoint along={a.x+t*dx,a.y+t*dy},target=b;
  bool localTargetFree=waypointFree;
  double remaining=c::LOOKAHEAD_MM;
  // Blend through a corner only when the actual footprint has line of sight
  // to the carrot. An unsafe cut falls back to the original planner vertex.
  for (unsigned i=waypoint_;i<=legEnd_;++i) {
    const auto end=path_[i]; const double length=std::hypot(end.x-along.x,end.y-along.y);
    const double step=length>0?std::min(1.0,remaining/length):1;
    const ArenaPoint candidate={along.x+step*(end.x-along.x),along.y+step*(end.y-along.y)};
    if (!free(candidate)) break;
    target=candidate; localTargetFree=true;
    if (remaining<=length) break;
    remaining-=length; along=end;
  }
  if (!localTargetFree) { NeedNewPlan(in); return; }
  const double targetDistance=std::hypot(target.x-center.x,target.y-center.y);
  if (targetDistance<0.01) { NeedNewPlan(in); return; }
  double speed=c::SPEED_MMPS;
  if (std::min(planner_.ExtraClearanceMM(center,m.pose.headingRad),
      planner_.ExtraClearanceMM(target,m.pose.headingRad))<c::SLOW_CLEARANCE_MM)
    speed=std::min(speed,c::NEAR_OBSTACLE_SPEED_MMPS);
  if (waypoint_<legEnd_) {
    const auto next=path_[waypoint_+1];
    const double incoming=std::hypot(dx,dy),outgoing=std::hypot(next.x-b.x,next.y-b.y);
    const double cosine=incoming*outgoing>0?
        std::max(-1.0,std::min(1.0,(dx*(next.x-b.x)+dy*(next.y-b.y))/(incoming*outgoing))):1;
    const double corner=c::CORNER_SPEED_MMPS+(c::SPEED_MMPS-c::CORNER_SPEED_MMPS)*std::max(0.0,cosine);
    speed=std::min(speed,std::sqrt(corner*corner+2*c::ACCEL_MMPS2*std::max(0.0,distance-c::LOOKAHEAD_MM)));
  }
  const double wantedX=(target.x-center.x)*speed/targetDistance,wantedY=(target.y-center.y)*speed/targetDistance;
  const double change=std::hypot(wantedX-routeVX_,wantedY-routeVY_);
  const double blend=change>0?std::min(1.0,c::ACCEL_MMPS2*m.dt/change):1;
  routeVX_+=blend*(wantedX-routeVX_); routeVY_+=blend*(wantedY-routeVY_);
  const double angle=robot_geometry::ArenaAngle(m.pose.headingRad);
  forward_=std::cos(angle)*routeVX_+std::sin(angle)*routeVY_;
  left_=-std::sin(angle)*routeVX_+std::cos(angle)*routeVY_;
  status_="Passing guide "+std::to_string(waypoint_)+"/"+std::to_string(path_.size()-1)+"; continuous XY and heading control";
}
void GoalNavigationRoutine::ConfirmPositionCorrection(const NavigationInput& in) {
  if (!NeedsPositionCorrection()) return;
  clearance_.ReanchorPose(in.motion.pose,in.motion.now);
  // A position reference never resets the gyro/encoders. Replan from the new
  // arena position rather than carrying an obsolete path origin into motion.
  if (std::hypot(goal_.x_MM-in.motion.pose.x_MM,goal_.y_MM-in.motion.pose.y_MM)<=navigation_config::POSITION_TOLERANCE_MM)
    ContinueAfterObservation(in);
  else NeedNewPlan(in);
}
void GoalNavigationRoutine::Update(const NavigationInput& in) {
  const auto& m=in.motion;
  const double previousYaw=yaw_; Stop();
  if (m.stopRequested) { Abort(); return; }
  if (!IsRunning()) return;
  if (!m.healthy || !robot_geometry::Finite(m.pose) || !std::isfinite(m.now) ||
      !std::isfinite(m.dt) || m.dt<=0 || m.dt>localization_config::MAX_CONTROL_GAP_S ||
      !std::isfinite(m.yawRateRadps) || !std::isfinite(in.forwardMMps) || !std::isfinite(in.leftMMps)) {
    Fail("Unhealthy odometry/IMU, disconnected Control Station or invalid timing"); return;
  }
  if (state_==State::Reference) {
    // Operator confirmed clear wall views at this corner. This is the ONLY
    // exception to excluding rear beams through the delivery band.
    if (m.now-phaseStarted_<0.25) return;
    if (!in.valid[0] || !Fresh(m.now,in.stamp[0],localization_config::SENSOR_STALE_S)) {
      status_="Waiting for valid left IR corner reading"; return;
    }
    if (!in.valid[1] || !Fresh(m.now,in.stamp[1],localization_config::SONAR_READING_STALE_S)) {
      status_="Waiting for valid rear sonar corner reading"; return;
    }
    if (!TapeAlignmentRoutine::PoseFromWalls(in.rangeMM[0],in.rangeMM[1],requestedPose_)) {
      status_="Corner readings do not define an arena position"; return;
    }
    const ArenaPoint p={requestedPose_.x_MM,requestedPose_.y_MM};
    if (!planner_.SegmentFree(p,p,0)) { status_="Reference intersects keep-out/clearance margin; reposition robot"; return; }
    state_=State::ReferenceReady; status_="Corner pose ready"; return;
  }
  clearance_.Observe(in,map_,state_!=State::Observing && state_!=State::PositionReady);
  if (clearance_.Revision()!=obstacleRevision_) {
    obstacleRevision_=clearance_.Revision();
    planner_=ClearancePlanner(clearance_.ObservedMap(map_),DefaultNavigationFootprint());
    if (state_==State::Following || state_==State::Planning || state_==State::NeedPlan || state_==State::ClearanceTurn) {
      obstacleReplanPending_=true; state_=State::Following;
    }
  }
  if (obstacleReplanPending_) {
    const ArenaPoint p={m.pose.x_MM,m.pose.y_MM};
    if (planner_.SegmentFree(p,p,m.pose.headingRad)) {
      if (std::hypot(in.forwardMMps,in.leftMMps)>heading_control_config::STATIONARY_SPEED_MMPS) {
        status_="Braking before updating route from confirmed obstacle"; return;
      }
      obstacleReplanPending_=false; NeedNewPlan(in); return;
    }
    const ClearancePlanner physical(clearance_.ObservedMap(map_),DefaultNavigationFootprint(),0);
    if (physical.SegmentFree(p,p,m.pose.headingRad)) {
      // An observed wall can be closer than the old map. Escape its planning
      // margin before asking A* for a route; never start a blind in-place turn.
      const double angle=robot_geometry::ArenaAngle(m.pose.headingRad);
      forward_=-navigation_config::CLEARANCE_RECOVERY_SPEED_MMPS*std::sin(angle);
      left_=-navigation_config::CLEARANCE_RECOVERY_SPEED_MMPS*std::cos(angle);
      clearance_.LimitMotion(in,map_,forward_,left_,yaw_,m.pose.headingRad);
      status_="Restoring measured clearance before replanning"; return;
    }
    Fail("Observed obstacle overlaps current footprint; stopped"); return;
  }
  if (state_==State::ReferenceReady || state_==State::NeedPlan || state_==State::Planning || state_==State::PositionReady) return;
  if (m.now-started_>navigation_config::MISSION_TIMEOUT_S) { Fail("Navigation time limit reached"); return; }
  const ArenaPoint center={m.pose.x_MM,m.pose.y_MM};
  if (!planner_.SegmentFree(center,center,m.pose.headingRad)) { Fail("Robot footprint reached clearance boundary"); return; }
  const auto fault=heading_.CheckFeedback(previousYaw,m.yawRateRadps,m.dt);
  if (!fault.empty()) { Fail(fault); return; }
  if (state_==State::Following && nextTurn_>=turns_.size() && waypoint_<path_.size()) {
    const auto next=path_[waypoint_]; const double distance=std::hypot(next.x-center.x,next.y-center.y);
    const double t=distance>0?std::min(1.0,navigation_config::LOOKAHEAD_MM/distance):0;
    const ArenaPoint preview={center.x+t*(next.x-center.x),center.y+t*(next.y-center.y)};
    const bool final=std::hypot(goal_.x_MM-center.x,goal_.y_MM-center.y)<=navigation_config::FINAL_APPROACH_MM;
    travelHeading_=clearance_.SelectHeading(in,map_,preview,travelHeading_,final);
  }
  const double targetHeading=state_==State::FinalHeading?goal_.headingRad:
      (state_==State::ClearanceTurn?turns_[nextTurn_].headingRad:travelHeading_);
  const double error=robot_geometry::Wrap(targetHeading-m.pose.headingRad);
  if (state_==State::Following)
    yaw_=heading_.Request(error,previousYaw,m.yawRateRadps,m.dt,TravelHeadingDeadband(center),navigation_config::TRAVEL_HEADING_KP);
  else if (state_==State::FinalHeading)
    yaw_=heading_.Request(error,previousYaw,m.yawRateRadps,m.dt,heading_control_config::HEADING_TOLERANCE_RAD);
  else if (state_==State::ClearanceTurn)
    yaw_=heading_.Request(error,previousYaw,m.yawRateRadps,m.dt);
  if (state_==State::Following) {
    if (nextTurn_>=turns_.size() && clearance_.HeadingBlocked()) {
      // A turn can initially increase front reach. Make space instead of
      // demanding an impossible pivot in a tight corridor.
      const double angle=robot_geometry::ArenaAngle(m.pose.headingRad);
      forward_=-navigation_config::CLEARANCE_RECOVERY_SPEED_MMPS*std::sin(angle);
      left_=-navigation_config::CLEARANCE_RECOVERY_SPEED_MMPS*std::cos(angle); yaw_=0;
      clearance_.LimitMotion(in,map_,forward_,left_,yaw_,m.pose.headingRad);
      routeVX_=routeVY_=0;
      status_="Making room for a sensor-driven clearance turn"; return;
    }
    FollowRoute(in);
    if (state_!=State::Following) return;
    clearance_.LimitMotion(in,map_,forward_,left_,yaw_,travelHeading_);
    const double angle=robot_geometry::ArenaAngle(m.pose.headingRad);
    routeVX_=std::cos(angle)*forward_-std::sin(angle)*left_;
    routeVY_=std::sin(angle)*forward_+std::cos(angle)*left_;
    if (clearance_.Active() || clearance_.Limited())
      status_="Front clearance: heading "+std::to_string(std::lround(travelHeading_*180/robot_geometry::PI))+
          " deg"+(clearance_.Limited()?"; limiting swept motion":"; XY and yaw together");
    if (std::hypot(m.pose.x_MM-progressPose_.x_MM,m.pose.y_MM-progressPose_.y_MM)>=2) {
      progressPose_=m.pose; effort_=0;
    } else if (std::hypot(forward_,left_)>20) {
      effort_+=m.dt; if (effort_>2.5) Fail("No motion progress on planned path");
    }
  } else if (state_==State::ClearanceTurn) {
    if (std::hypot(in.forwardMMps,in.leftMMps)>heading_control_config::STATIONARY_SPEED_MMPS) { yaw_=0; return; }
    clearance_.LimitMotion(in,map_,forward_,left_,yaw_,targetHeading);
    if (std::fabs(error)<=heading_control_config::HEADING_TOLERANCE_RAD &&
        std::fabs(m.yawRateRadps)<=robot_geometry::Radians(heading_control_config::STATIONARY_YAW_DPS)) {
      Stop(); heading_.Reset(); travelHeading_=targetHeading; waypoint_=legEnd_+1; ++nextTurn_;
      legEnd_=nextTurn_<turns_.size()?turns_[nextTurn_].pointIndex:path_.size()-1;
      preciseApproach_=false; routeVX_=routeVY_=0; progressPose_=m.pose; effort_=0;
      if (waypoint_>=path_.size()) BeginObservation(m.now);
      else { state_=State::Following; status_="Continuing route after measured-clearance maneuver"; }
    }
  } else if (state_==State::Observing) {
    // This is a final-position observation only. Use the measured heading for
    // wall association; do not align to an intermediate yaw just to read walls.
    if (std::hypot(in.forwardMMps,in.leftMMps)>heading_control_config::STATIONARY_SPEED_MMPS ||
        std::fabs(m.yawRateRadps)>robot_geometry::Radians(heading_control_config::STATIONARY_YAW_DPS)) { BeginObservation(m.now); return; }
    status_="Stopped at destination; checking unobstructed outer-wall readings";
    const auto referenceMap=clearance_.ObservedMap(map_);
    for (int i=0;i<4;++i) {
      const double stale=(i==0 || i==2)?localization_config::SENSOR_STALE_S:localization_config::SONAR_READING_STALE_S;
      if (in.stamp[i]<=lastStamp_[i] || in.stamp[i]<phaseStarted_+0.05 || !Fresh(m.now,in.stamp[i],stale)) continue;
      lastStamp_[i]=in.stamp[i];
      const auto estimate=EstimateFromOuterWall(referenceMap,m.pose,i,in.rangeMM[i],in.valid[i]);
      if (estimate.valid && observations_[estimate.axis].size()<128) observations_[estimate.axis].push_back(estimate.coordinateMM);
    }
    if (m.now-phaseStarted_<navigation_config::OBSERVATION_S) return;
    requestedPose_=m.pose;
    for (int axis=0;axis<2;++axis) {
      auto samples=observations_[axis];
      if (samples.size()<2) continue;
      std::sort(samples.begin(),samples.end());
      if (samples.back()-samples.front()>20) continue;
      if (axis==0) requestedPose_.x_MM=samples[samples.size()/2]; else requestedPose_.y_MM=samples[samples.size()/2];
    }
    const ArenaPoint corrected={requestedPose_.x_MM,requestedPose_.y_MM};
    if (std::hypot(corrected.x-m.pose.x_MM,corrected.y-m.pose.y_MM)>0.5 &&
        planner_.SegmentFree(corrected,corrected,m.pose.headingRad)) {
      state_=State::PositionReady; status_="Applying stable outer-wall position reference"; return;
    }
    ContinueAfterObservation(in);
  } else if (state_==State::FinalHeading) {
    clearance_.LimitMotion(in,map_,forward_,left_,yaw_,goal_.headingRad);
    if (std::hypot(goal_.x_MM-m.pose.x_MM,goal_.y_MM-m.pose.y_MM)>navigation_config::POSITION_TOLERANCE_MM) { NeedNewPlan(in); return; }
    if (std::fabs(error)<=heading_control_config::HEADING_TOLERANCE_RAD &&
        std::fabs(m.yawRateRadps)<=robot_geometry::Radians(heading_control_config::STATIONARY_YAW_DPS) &&
        std::hypot(in.forwardMMps,in.leftMMps)<=heading_control_config::STATIONARY_SPEED_MMPS) {
      if (settled_<0) settled_=m.now;
      if (m.now-settled_>=0.30) { Stop(); state_=State::Complete; status_="Goal reached: position <=10 mm, heading <=1 deg"; }
    } else settled_=-1;
  }
}
