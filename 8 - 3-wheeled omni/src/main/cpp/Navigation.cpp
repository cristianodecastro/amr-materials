#include "Autonomous.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace {
bool Fresh(double now,double stamp,double stale) {
  return std::isfinite(stamp) && stamp>=0 && now>=stamp && now-stamp<=stale;
}
}
void GoalNavigationRoutine::Enter() { *this=GoalNavigationRoutine{}; }
void GoalNavigationRoutine::Abort() { Stop(); state_=State::Aborted; status_="Navigation stopped"; }
void GoalNavigationRoutine::Fail(const std::string& message) { Stop(); state_=State::Fault; status_=message; }
void GoalNavigationRoutine::ReferenceCorner(double now) {
  Stop(); heading_.Reset(); referenced_=false; state_=State::Reference; phaseStarted_=now;
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
  // Keep the planner's collision-checked vertices. Do not insert artificial
  // precision stops along otherwise straight segments.
  path_=std::move(plan.points);
  pathLength_=plan.lengthMM; waypoint_=1; effort_=0; progressPose_=in.motion.pose;
  preciseApproach_=false;
  const double angle=robot_geometry::ArenaAngle(in.motion.pose.headingRad);
  routeVX_=std::cos(angle)*in.forwardMMps-std::sin(angle)*in.leftMMps;
  routeVY_=std::sin(angle)*in.forwardMMps+std::cos(angle)*in.leftMMps;
  if (path_.size()==1) { BeginObservation(in.motion.now); return; }
  state_=State::Following; status_="Following clearance route without intermediate stops";
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
    return planner_.SegmentFree(center,p,m.pose.headingRad) &&
        planner_.SegmentFree(center,p,travelHeading_);
  };
  // Passing a guide requires a clear shortcut to the next segment, not exact
  // XY/heading arrival. Otherwise keep approaching the original safe vertex.
  while (waypoint_+1<path_.size() &&
      std::hypot(path_[waypoint_].x-center.x,path_[waypoint_].y-center.y)<=c::PASS_RADIUS_MM &&
      free(path_[waypoint_+1])) ++waypoint_;
  if (!free(path_[waypoint_])) { NeedNewPlan(in); return; }
  const double distance=std::hypot(path_[waypoint_].x-center.x,path_[waypoint_].y-center.y);
  if (!preciseApproach_ && waypoint_+1==path_.size() && distance<=c::FINAL_APPROACH_MM) {
    preciseApproach_=true;
    const double angle=robot_geometry::ArenaAngle(m.pose.headingRad);
    const double initialForward=std::cos(angle)*routeVX_+std::sin(angle)*routeVY_;
    const double initialLeft=-std::sin(angle)*routeVX_+std::cos(angle)*routeVY_;
    point_.Reset(path_.back(),m.now,c::SPEED_MMPS,c::ACCEL_MMPS2,initialForward,initialLeft);
  }
  if (preciseApproach_) {
    // Precision is for the final XY position. Heading correction continues in
    // parallel; only FinalHeading enforces the destination's 1-degree angle.
    point_.Update(m,in.forwardMMps,in.leftMMps,0);
    forward_=point_.Forward(); left_=point_.Left();
    status_="Final XY approach: "+point_.Status();
    if (point_.Reached()) BeginObservation(m.now);
    return;
  }
  const auto a=path_[waypoint_-1],b=path_[waypoint_];
  const double dx=b.x-a.x,dy=b.y-a.y,length2=dx*dx+dy*dy;
  const double t=length2>0?std::max(0.0,std::min(1.0,((center.x-a.x)*dx+(center.y-a.y)*dy)/length2)):0;
  ArenaPoint along={a.x+t*dx,a.y+t*dy},target=b;
  double remaining=c::LOOKAHEAD_MM;
  // Blend through a corner only when the actual footprint has line of sight
  // to the carrot. An unsafe cut falls back to the original planner vertex.
  for (unsigned i=waypoint_;i<path_.size();++i) {
    const auto end=path_[i]; const double length=std::hypot(end.x-along.x,end.y-along.y);
    const double step=length>0?std::min(1.0,remaining/length):1;
    const ArenaPoint candidate={along.x+step*(end.x-along.x),along.y+step*(end.y-along.y)};
    if (!free(candidate)) break;
    target=candidate;
    if (remaining<=length) break;
    remaining-=length; along=end;
  }
  const double targetDistance=std::hypot(target.x-center.x,target.y-center.y);
  if (targetDistance<0.01) { NeedNewPlan(in); return; }
  double speed=c::SPEED_MMPS;
  if (waypoint_+1<path_.size()) {
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
  if (state_==State::ReferenceReady || state_==State::NeedPlan || state_==State::Planning || state_==State::PositionReady) return;
  if (m.now-started_>navigation_config::MISSION_TIMEOUT_S) { Fail("Navigation time limit reached"); return; }
  const ArenaPoint center={m.pose.x_MM,m.pose.y_MM};
  if (!planner_.SegmentFree(center,center,m.pose.headingRad)) { Fail("Robot footprint reached clearance boundary"); return; }
  const auto fault=heading_.CheckFeedback(previousYaw,m.yawRateRadps,m.dt);
  if (!fault.empty()) { Fail(fault); return; }
  const double targetHeading=state_==State::FinalHeading?goal_.headingRad:travelHeading_;
  const double error=robot_geometry::Wrap(targetHeading-m.pose.headingRad);
  if (state_==State::Following)
    yaw_=heading_.Request(error,previousYaw,m.yawRateRadps,m.dt,TravelHeadingDeadband(center),navigation_config::TRAVEL_HEADING_KP);
  else if (state_==State::FinalHeading)
    yaw_=heading_.Request(error,previousYaw,m.yawRateRadps,m.dt,heading_control_config::HEADING_TOLERANCE_RAD);
  if (state_==State::Following) {
    FollowRoute(in);
    if (state_!=State::Following) return;
    if (std::hypot(m.pose.x_MM-progressPose_.x_MM,m.pose.y_MM-progressPose_.y_MM)>=2) {
      progressPose_=m.pose; effort_=0;
    } else if (std::hypot(forward_,left_)>20) {
      effort_+=m.dt; if (effort_>2.5) Fail("No motion progress on planned path");
    }
  } else if (state_==State::Observing) {
    // This is a final-position observation only. Use the measured heading for
    // wall association; do not align to an intermediate yaw just to read walls.
    if (std::hypot(in.forwardMMps,in.leftMMps)>heading_control_config::STATIONARY_SPEED_MMPS ||
        std::fabs(m.yawRateRadps)>robot_geometry::Radians(heading_control_config::STATIONARY_YAW_DPS)) { BeginObservation(m.now); return; }
    status_="Stopped at destination; checking unobstructed outer-wall readings";
    for (int i=0;i<4;++i) {
      const double stale=(i==0 || i==2)?localization_config::SENSOR_STALE_S:localization_config::SONAR_READING_STALE_S;
      if (in.stamp[i]<=lastStamp_[i] || in.stamp[i]<phaseStarted_+0.05 || !Fresh(m.now,in.stamp[i],stale)) continue;
      lastStamp_[i]=in.stamp[i];
      const auto estimate=EstimateFromOuterWall(map_,m.pose,i,in.rangeMM[i],in.valid[i]);
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
    if (std::hypot(goal_.x_MM-m.pose.x_MM,goal_.y_MM-m.pose.y_MM)>navigation_config::POSITION_TOLERANCE_MM) { NeedNewPlan(in); return; }
    if (std::fabs(error)<=heading_control_config::HEADING_TOLERANCE_RAD &&
        std::fabs(m.yawRateRadps)<=robot_geometry::Radians(heading_control_config::STATIONARY_YAW_DPS) &&
        std::hypot(in.forwardMMps,in.leftMMps)<=heading_control_config::STATIONARY_SPEED_MMPS) {
      if (settled_<0) settled_=m.now;
      if (m.now-settled_>=0.30) { Stop(); state_=State::Complete; status_="Goal reached: position <=10 mm, heading <=1 deg"; }
    } else settled_=-1;
  }
}
