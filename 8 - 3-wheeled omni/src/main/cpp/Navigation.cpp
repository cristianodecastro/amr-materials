#include "Autonomous.h"
#include <algorithm>
#include <cmath>

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
  path_.clear(); path_.push_back(plan.points.front());
  for (unsigned i=1;i<plan.points.size();++i) {
    const auto a=plan.points[i-1],b=plan.points[i];
    const int count=std::max(1,static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/navigation_config::OBSERVATION_SPACING_MM)));
    for (int j=1;j<=count;++j) {
      const double t=static_cast<double>(j)/count;
      path_.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)});
    }
  }
  pathLength_=plan.lengthMM; waypoint_=1; effort_=0; progressPose_=in.motion.pose;
  if (path_.size()==1) { BeginObservation(in.motion.now); return; }
  point_.Reset(path_[waypoint_],in.motion.now,navigation_config::SPEED_MMPS,navigation_config::ACCEL_MMPS2);
  state_=State::Following; status_="Following clearance route; holding travel heading";
}
void GoalNavigationRoutine::BeginObservation(double now) {
  forward_=left_=0; state_=State::Observing; phaseStarted_=now;
  observations_[0].clear(); observations_[1].clear();
  for (auto& time:lastStamp_) time=-1;
  status_="Stopped at waypoint; checking unobstructed outer-wall readings";
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
  if (++waypoint_>=path_.size()) { NeedNewPlan(in); return; }
  point_.Reset(path_[waypoint_],in.motion.now,navigation_config::SPEED_MMPS,navigation_config::ACCEL_MMPS2);
  progressPose_=p; effort_=0; state_=State::Following;
  status_="Continuing route from checked waypoint";
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
  yaw_=heading_.Request(error,previousYaw,m.yawRateRadps,m.dt);
  if (state_==State::Following) {
    if (!planner_.SegmentFree(center,path_[waypoint_],travelHeading_)) { Fail("Position drift left free path corridor"); return; }
    point_.Update(m,in.forwardMMps,in.leftMMps,error);
    forward_=point_.Forward(); left_=point_.Left();
    status_="Waypoint "+std::to_string(waypoint_)+"/"+std::to_string(path_.size()-1)+": "+point_.Status();
    if (point_.Reached()) { BeginObservation(m.now); return; }
    if (std::hypot(m.pose.x_MM-progressPose_.x_MM,m.pose.y_MM-progressPose_.y_MM)>=2) {
      progressPose_=m.pose; effort_=0;
    } else if (std::hypot(forward_,left_)>20) {
      effort_+=m.dt; if (effort_>2.5) Fail("No motion progress on planned path");
    }
  } else if (state_==State::Observing) {
    if (std::hypot(in.forwardMMps,in.leftMMps)>heading_control_config::STATIONARY_SPEED_MMPS ||
        std::fabs(error)>heading_control_config::HEADING_TOLERANCE_RAD) { BeginObservation(m.now); return; }
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
        std::hypot(in.forwardMMps,in.leftMMps)<=heading_control_config::STATIONARY_SPEED_MMPS) {
      if (settled_<0) settled_=m.now;
      if (m.now-settled_>=0.30) { Stop(); state_=State::Complete; status_="Goal reached: position <=10 mm, heading <=1 deg"; }
    } else settled_=-1;
  }
}
