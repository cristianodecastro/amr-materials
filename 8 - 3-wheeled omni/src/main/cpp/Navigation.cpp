#include "Autonomous.h"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace {
bool Fresh(double now,double stamp,double stale) {
  return std::isfinite(stamp) && stamp>=0 && now>=stamp && now-stamp<=stale;
}
bool HealthyMotion(const NavigationInput& in) {
  const auto& m=in.motion;
  return m.healthy && robot_geometry::Finite(m.pose) && std::isfinite(m.now) &&
      std::isfinite(m.dt) && m.dt>0 && m.dt<=localization_config::MAX_CONTROL_GAP_S &&
      std::isfinite(m.yawRateRadps) && std::isfinite(in.forwardMMps) && std::isfinite(in.leftMMps);
}
bool ReadIntermediateXY(const std::string& text,std::vector<ArenaPoint>& points) {
  std::istringstream list(text); std::string item;
  while (std::getline(list,item,';')) {
    if (item.find_first_not_of(" \t\r\n")==std::string::npos) continue;
    std::istringstream values(item); ArenaPoint p; char separator;
    if (!(values>>p.x>>separator>>p.y) || separator!=',' || !std::isfinite(p.x) || !std::isfinite(p.y) ||
        p.x<0 || p.y<0 || p.x>localization_config::ARENA_WIDTH_MM || p.y>localization_config::ARENA_HEIGHT_MM) return false;
    values>>std::ws;
    if (!values.eof() || points.size()>=navigation_config::MAX_INTERMEDIATE_POINTS) return false;
    points.push_back(p);
  }
  return true;
}
}
void GoalNavigationRoutine::Enter() { *this=GoalNavigationRoutine{}; }
void GoalNavigationRoutine::Abort() { Stop(); state_=State::Aborted; status_="Stopped"; }
void GoalNavigationRoutine::Fail(const std::string& message) { Stop(); state_=State::Fault; status_=message; }
void GoalNavigationRoutine::ReferenceCorner(double) {
  Stop(); heading_.Reset(); referenced_=false; state_=State::Reference;
  status_="Reading left IR and rear sonar; face north (0 deg)";
}
void GoalNavigationRoutine::ConfirmReference() {
  if (!NeedsPoseReset()) return;
  referenced_=true; state_=State::Ready; status_="Referenced; select a goal, edit its points, then Go";
}
bool GoalNavigationRoutine::StartGoal(RobotPose goal,const NavigationInput& in,const std::string& intermediateXY) {
  if (IsRunning()) return false;
  if (!referenced_) { status_="Reference corner first"; return false; }
  if (!HealthyMotion(in)) { status_="Cannot start: IMU/odometry unavailable"; return false; }
  if (!robot_geometry::Finite(goal) || goal.x_MM<0 || goal.y_MM<0 ||
      goal.x_MM>localization_config::ARENA_WIDTH_MM || goal.y_MM>localization_config::ARENA_HEIGHT_MM) {
    status_="Goal must be inside the arena with finite coordinates"; return false;
  }
  std::vector<ArenaPoint> route;
  if (!ReadIntermediateXY(intermediateXY,route)) {
    status_="Intermediate XY: use x,y; x,y in arena mm (up to "+
        std::to_string(navigation_config::MAX_INTERMEDIATE_POINTS)+" points)"; return false;
  }
  route.push_back({goal.x_MM,goal.y_MM}); route_=std::move(route); leg_=0;
  legStart_={in.motion.pose.x_MM,in.motion.pose.y_MM}; worldVX_=worldVY_=0;
  finalControllerReady_=route_.size()==1;
  goal_=goal; goal_.headingRad=robot_geometry::Wrap(goal_.headingRad);
  started_=in.motion.now; settled_=-1; travelHeading_=in.motion.pose.headingRad;
  remaining_=0; auto previous=legStart_;
  for (auto p:route_) { remaining_+=std::hypot(p.x-previous.x,p.y-previous.y); previous=p; }
  point_.Reset({goal_.x_MM,goal_.y_MM},started_,navigation_config::SPEED_MMPS,navigation_config::ACCEL_MMPS2);
  heading_.Reset(); Stop(); state_=State::Driving;
  status_=route_.size()==1?"Direct XY travel; holding starting heading":"Manual route started; holding starting heading";
  return true;
}
void GoalNavigationRoutine::DriveRoute(const NavigationInput& in) {
  const auto& m=in.motion; const ArenaPoint center={m.pose.x_MM,m.pose.y_MM};
  while (leg_+1<route_.size()) {
    const auto p=route_[leg_];
    const bool passed=(center.x-p.x)*(p.x-legStart_.x)+(center.y-p.y)*(p.y-legStart_.y)>=0;
    if (!passed && std::hypot(p.x-center.x,p.y-center.y)>navigation_config::PASS_RADIUS_MM) break;
    legStart_=p; ++leg_;
  }
  remaining_=std::hypot(route_[leg_].x-center.x,route_[leg_].y-center.y);
  for (unsigned i=leg_+1;i<route_.size();++i)
    remaining_+=std::hypot(route_[i].x-route_[i-1].x,route_[i].y-route_[i-1].y);
  const double angle=robot_geometry::ArenaAngle(m.pose.headingRad),co=std::cos(angle),si=std::sin(angle);
  if (leg_+1==route_.size()) {
    if (!finalControllerReady_) {
      point_.Reset(route_.back(),m.now,navigation_config::SPEED_MMPS,navigation_config::ACCEL_MMPS2,
          co*worldVX_+si*worldVY_,-si*worldVX_+co*worldVY_);
      finalControllerReady_=true;
    }
    point_.Update(m,in.forwardMMps,in.leftMMps,0);
    forward_=point_.Forward(); left_=point_.Left(); status_="Final XY: "+point_.Status();
    if (point_.Reached()) {
      Stop(); heading_.Reset(); settled_=-1; state_=State::FinalHeading;
      status_="XY reached; setting final heading";
    }
    return;
  }
  const auto p=route_[leg_]; const double distance=std::hypot(p.x-center.x,p.y-center.y);
  const double speed=std::min(navigation_config::SPEED_MMPS,
      std::max(navigation_config::MIN_PASS_SPEED_MMPS,1.5*distance));
  const double wantedX=(p.x-center.x)*speed/distance,wantedY=(p.y-center.y)*speed/distance;
  const double change=std::hypot(wantedX-worldVX_,wantedY-worldVY_);
  const double gain=change>0?std::min(1.0,navigation_config::ACCEL_MMPS2*m.dt/change):1;
  worldVX_+=gain*(wantedX-worldVX_); worldVY_+=gain*(wantedY-worldVY_);
  forward_=co*worldVX_+si*worldVY_; left_=-si*worldVX_+co*worldVY_;
  status_="Intermediate "+std::to_string(leg_+1)+"/"+std::to_string(IntermediateCount())+"; holding heading";
}
void GoalNavigationRoutine::Update(const NavigationInput& in) {
  const auto& m=in.motion; const double previousYaw=yaw_; Stop();
  if (m.stopRequested) { Abort(); return; }
  if (!IsRunning()) return;
  if (state_==State::Reference) {
    if (!HealthyMotion(in)) { status_="Waiting for IMU/odometry"; return; }
    if (!in.leftValid || !Fresh(m.now,in.leftTime,localization_config::SENSOR_STALE_S)) {
      status_="Waiting for valid left IR"; return;
    }
    if (!in.rearValid || !Fresh(m.now,in.rearTime,localization_config::SONAR_READING_STALE_S)) {
      status_="Waiting for valid rear sonar"; return;
    }
    if (!TapeAlignmentRoutine::PoseFromWalls(in.leftRangeMM,in.rearRangeMM,requestedPose_)) {
      status_="Corner ranges do not define an arena position"; return;
    }
    state_=State::ReferenceReady; status_="Starting pose ready"; return;
  }
  if (state_==State::ReferenceReady) return;
  if (!HealthyMotion(in)) { Fail("IMU/odometry or Control Station unavailable - stopped"); return; }
  if (m.now-started_>navigation_config::MISSION_TIMEOUT_S) { Fail("Task timeout - stopped"); return; }
  const auto fault=heading_.CheckFeedback(previousYaw,m.yawRateRadps,m.dt);
  if (!fault.empty()) { Fail(fault); return; }
  remaining_=std::hypot(goal_.x_MM-m.pose.x_MM,goal_.y_MM-m.pose.y_MM);
  if (state_==State::Driving) {
    const double error=robot_geometry::Wrap(travelHeading_-m.pose.headingRad);
    yaw_=heading_.Request(error,previousYaw,m.yawRateRadps,m.dt,
        navigation_config::TRAVEL_HEADING_DEADBAND_RAD,navigation_config::TRAVEL_HEADING_KP);
    DriveRoute(in);
  } else if (state_==State::FinalHeading) {
    if (remaining_>navigation_config::POSITION_TOLERANCE_MM) {
      travelHeading_=goal_.headingRad; heading_.Reset();
      point_.Reset({goal_.x_MM,goal_.y_MM},m.now,navigation_config::SPEED_MMPS,
          navigation_config::ACCEL_MMPS2,in.forwardMMps,in.leftMMps);
      state_=State::Driving; settled_=-1; status_="Correcting final XY"; return;
    }
    const double error=robot_geometry::Wrap(goal_.headingRad-m.pose.headingRad);
    yaw_=heading_.Request(error,previousYaw,m.yawRateRadps,m.dt,heading_control_config::HEADING_TOLERANCE_RAD);
    if (std::fabs(error)<=heading_control_config::HEADING_TOLERANCE_RAD &&
        std::fabs(m.yawRateRadps)<=robot_geometry::Radians(heading_control_config::STATIONARY_YAW_DPS) &&
        std::hypot(in.forwardMMps,in.leftMMps)<=heading_control_config::STATIONARY_SPEED_MMPS) {
      yaw_=0;
      if (settled_<0) settled_=m.now;
      if (m.now-settled_>=0.30) { Stop(); state_=State::Complete; status_="Goal reached: within 10 mm / 1 deg"; }
    } else settled_=-1;
  }
}
