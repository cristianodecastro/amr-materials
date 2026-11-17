#include "Autonomous.h"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace {
bool Fresh(double now,double stamp,double stale) {
  return std::isfinite(stamp) && stamp>=0 && now>=stamp && now-stamp<=stale;
}
std::string MotionIssue(const NavigationInput& in) {
  const auto& m=in.motion;
  if (!std::isfinite(m.now) || !std::isfinite(m.dt) || m.dt<=0) return "Invalid control clock/interval";
  if (m.dt>localization_config::AUTO_MAX_SAMPLE_GAP_S) return "Control loop gap "+std::to_string(static_cast<int>(m.dt*1000))+" ms exceeds 500 ms";
  if (!m.healthy) return in.healthIssue.empty()?"Odometry/IMU health check failed":in.healthIssue;
  if (!robot_geometry::Finite(m.pose)) return "Odometry pose is not finite";
  if (!std::isfinite(m.yawRateRadps)) return "IMU yaw-rate sample is not finite";
  if (!std::isfinite(in.forwardMMps) || !std::isfinite(in.leftMMps)) return "Odometry velocity is not finite";
  return {};
}
bool HealthyMotion(const NavigationInput& in) { return MotionIssue(in).empty(); }
bool UsableRange(const NavigationInput& in,int sensor) {
  const bool ir=sensor==0 || sensor==2;
  const double range=in.rangeMM[sensor];
  return in.valid[sensor] && Fresh(in.motion.now,in.stamp[sensor],ir?localization_config::SENSOR_STALE_S:
      localization_config::SONAR_READING_STALE_S) && std::isfinite(range) &&
      range>=(ir?localization_config::IR_MIN_RANGE_MM:localization_config::SONAR_MIN_RANGE_MM) &&
      range<=(ir?localization_config::IR_MAX_RANGE_MM:localization_config::SONAR_MAX_RANGE_MM);
}
std::string Trim(const std::string& text) {
  const auto first=text.find_first_not_of(" \t\r\n");
  return first==std::string::npos?"":text.substr(first,text.find_last_not_of(" \t\r\n")-first+1);
}
bool OptionalCoordinate(const std::string& text,bool& known,double& value) {
  const auto clean=Trim(text); known=!clean.empty() && clean!="-";
  if (!known) return true;
  std::istringstream input(clean);
  if (!(input>>value) || !std::isfinite(value)) return false;
  input>>std::ws; return input.eof();
}
bool ReadIntermediatePoses(const std::string& text,double heading,std::vector<RobotPose>& points) {
  std::istringstream list(text); std::string item;
  while (std::getline(list,item,';')) {
    if (item.find_first_not_of(" \t\r\n")==std::string::npos) continue;
    std::istringstream values(item); RobotPose p; char separator;
    if (!(values>>p.x_MM>>separator>>p.y_MM) || separator!=',' || !std::isfinite(p.x_MM) || !std::isfinite(p.y_MM) ||
        p.x_MM<0 || p.y_MM<0 || p.x_MM>localization_config::ARENA_WIDTH_MM || p.y_MM>localization_config::ARENA_HEIGHT_MM) return false;
    values>>std::ws;
    if (!values.eof()) {
      double degrees;
      if (!(values>>separator>>degrees) || separator!=',' || !std::isfinite(degrees)) return false;
      heading=robot_geometry::Wrap(robot_geometry::Radians(degrees)); values>>std::ws;
    }
    if (!values.eof() || points.size()>=navigation_config::MAX_INTERMEDIATE_POINTS) return false;
    p.headingRad=heading; // legacy x,y entries inherit the preceding heading
    points.push_back(p);
  }
  return true;
}
}
GoalNavigationRoutine::~GoalNavigationRoutine() { CancelPlanning(); }
void GoalNavigationRoutine::Enter() {
  auto worker=planner_; const bool background=backgroundPlanning_,court=obstacles_.UsesCourt();
  CancelPlanning(); *this=GoalNavigationRoutine{court,background}; planner_=std::move(worker);
}
void GoalNavigationRoutine::Abort() { CancelPlanning(); Stop(); state_=State::Aborted; status_="Stopped"; }
void GoalNavigationRoutine::Fail(const std::string& message) { CancelPlanning(); Stop(); state_=State::Fault; status_=message; }
bool GoalNavigationRoutine::InitializePose(const PoseInitialization& settings) {
  if (IsRunning() && state_!=State::Reference && state_!=State::ReferenceReady) return false;
  CancelPlanning(); Stop(); positionCorrection_={0,0}; referenced_=false; state_=State::Unreferenced;
  const auto reject=[this](const std::string& message) { status_=initializationStatus_=message; return false; };
  bool known[2]; double coordinate[2]={}; WallReference references[4];
  if (!OptionalCoordinate(settings.xMM,known[0],coordinate[0]) ||
      !OptionalCoordinate(settings.yMM,known[1],coordinate[1]) || !std::isfinite(settings.headingDeg))
    return reject("Start X/Y: number or '-'; heading must be a finite number");
  for (int axis=0;axis<2;++axis) if (known[axis] &&
      (coordinate[axis]<0 || coordinate[axis]>(axis==0?localization_config::ARENA_WIDTH_MM:localization_config::ARENA_HEIGHT_MM)))
    return reject("Supplied start coordinate is outside the arena");
  for (int i=0;i<4;++i) {
    const auto text=Trim(settings.wall[i]);
    if (text.empty() || text=="-") continue;
    std::istringstream value(text); char axis,equals; double wall;
    if (!(value>>axis>>equals>>wall) || equals!='=' || !std::isfinite(wall) ||
        (axis!='X' && axis!='x' && axis!='Y' && axis!='y')) return reject("Sensor walls: X=coordinate, Y=coordinate or '-'");
    value>>std::ws;
    if (!value.eof()) return reject("Sensor walls: X=coordinate, Y=coordinate or '-'");
    references[i]={axis=='X'||axis=='x'?0:1,wall};
  }
  for (int axis=0;axis<2;++axis) { initialKnown_[axis]=known[axis]; initialCoordinate_[axis]=coordinate[axis]; }
  for (int i=0;i<4;++i) walls_[i]=references[i];
  initialHeading_=robot_geometry::Wrap(robot_geometry::Radians(settings.headingDeg));
  heading_.Reset(); state_=State::Reference; status_=initializationStatus_="Reading selected initialization references";
  return true;
}
void GoalNavigationRoutine::ConfirmReference() {
  if (!NeedsPoseReset()) return;
  referenced_=true; state_=State::Ready; status_="Referenced; select a goal, edit its points, then Go";
}
bool GoalNavigationRoutine::StartGoal(RobotPose goal,const NavigationInput& in,const std::string& intermediatePoses) {
  if (IsRunning()) return false;
  if (!referenced_) { status_="Initialize pose first"; return false; }
  if (!HealthyMotion(in)) { status_="Cannot start: "+MotionIssue(in); return false; }
  if (!robot_geometry::Finite(goal) || goal.x_MM<0 || goal.y_MM<0 ||
      goal.x_MM>localization_config::ARENA_WIDTH_MM || goal.y_MM>localization_config::ARENA_HEIGHT_MM) {
    status_="Goal must be inside the arena with finite coordinates"; return false;
  }
  std::vector<RobotPose> route;
  if (!ReadIntermediatePoses(intermediatePoses,in.motion.pose.headingRad,route)) {
    status_="Intermediates: x,y,headingDeg; ... in arena mm (up to "+
        std::to_string(navigation_config::MAX_INTERMEDIATE_POINTS)+" points)"; return false;
  }
  goal.headingRad=robot_geometry::Wrap(goal.headingRad);
  if (obstacles_.UsesCourt()) {
    const LocalObstacleMap fixedCourt; // old-goal sensor patches must not reject a new goal
    for (const auto& p:route) if (fixedCourt.Clearance({p.x_MM,p.y_MM},p.headingRad)<navigation_config::OBSTACLE_CLEARANCE_MM) {
      status_="Intermediate pose footprint overlaps a court barrier or outer wall"; return false;
    }
    if (fixedCourt.Clearance({goal.x_MM,goal.y_MM},goal.headingRad)<navigation_config::OBSTACLE_CLEARANCE_MM) {
      status_="Goal pose footprint overlaps a court barrier or outer wall"; return false;
    }
  }
  CancelPlanning(); route.push_back(goal); route_=std::move(route); leg_=0; worldVX_=worldVY_=0;
  legStart_={in.motion.pose.x_MM,in.motion.pose.y_MM}; carryHeading_=clearanceHeading_=false;
  detourActive_=recoveryActive_=haveRetreatOrigin_=false; preciseTurnLeg_=-1; obstacles_.ClearHits(); for (auto& stamp:observedStamp_) stamp=-1;
  finalControllerReady_=route_.size()==1;
  goal_=goal; goal_.headingRad=robot_geometry::Wrap(goal_.headingRad);
  started_=in.motion.now; settled_=-1; travelHeading_=in.motion.pose.headingRad;
  remaining_=0; auto previous=in.motion.pose;
  for (auto p:route_) { remaining_+=std::hypot(p.x_MM-previous.x_MM,p.y_MM-previous.y_MM); previous=p; }
  point_.Reset({goal_.x_MM,goal_.y_MM},started_,navigation_config::SPEED_MMPS,navigation_config::ACCEL_MMPS2);
  heading_.Reset(); Stop(); state_=State::Driving;
  history_.clear(); positionCorrection_={0,0}; wallCorrectionPending_=false;
  for (auto& stamp:consumedStamp_) stamp=-1;
  for (auto& time:wallTime_) time=-1;
  status_=route_.size()==1?"Direct XY travel; holding starting heading":"Manual pose route started";
  return true;
}
void GoalNavigationRoutine::CancelPlanning() {
  if (cancelPlan_) cancelPlan_->store(true,std::memory_order_relaxed);
  pendingPlan_.reset(); cancelPlan_.reset();
}
bool GoalNavigationRoutine::RequestPlan(const NavigationInput& in,bool turn,double heading) {
  if (!planner_) planner_=std::make_shared<NavigationPlanWorker>();
  if (pendingPlan_) return false;
  const double a=robot_geometry::ArenaAngle(in.motion.pose.headingRad),co=std::cos(a),si=std::sin(a);
  planningVelocity_={co*in.forwardMMps-si*in.leftMMps,si*in.forwardMMps+co*in.leftMMps};
  planningHeading_=turn?heading:(leg_+1<route_.size()?route_[leg_].headingRad:travelHeading_);
  if (planner_->Busy()) return false; // an explicitly cancelled older job is winding down
  auto plan=std::make_shared<GoalNavigationRoutine>(*this);
  plan->backgroundPlanning_=false; plan->planner_.reset(); plan->pendingPlan_.reset();
  auto cancelled=std::make_shared<std::atomic<bool>>(false);
  plan->cancelPlan_=cancelled; plan->obstacles_.SetCancellation(cancelled);
  pendingMapRevision_=obstacles_.Revision(); cancelPlan_=cancelled; pendingPlan_=plan;
  planner_->Submit([plan,in,turn,heading] {
    try {
      if (turn) plan->PrepareTurnDetour(in,heading); else plan->EnsureClearRoute(in);
    } catch (const std::exception& e) { plan->Fail(std::string("Route planning error: ")+e.what()); }
      catch (...) { plan->Fail("Route planning error"); }
  });
  return false;
}
bool GoalNavigationRoutine::PollPlan() {
  if (!pendingPlan_) return false;
  if (!planner_->Ready()) return true;
  auto plan=pendingPlan_; pendingPlan_.reset(); cancelPlan_.reset(); lastPlanMS_=planner_->ElapsedMS();
  if (plan->state_==State::Fault) {
    if (pendingMapRevision_==obstacles_.Revision()) Fail(plan->status_);
    return false; // changed evidence must be replanned, not rejected by a stale result
  }
  // Commit only the route. Live sensor history, corrected pose, current
  // velocities, elapsed task time and heading feedback remain on this thread.
  route_=std::move(plan->route_); leg_=plan->leg_; legStart_=plan->legStart_;
  detourActive_=plan->detourActive_; detourEnd_=plan->detourEnd_; preciseTurnLeg_=plan->preciseTurnLeg_;
  recoveryActive_=plan->recoveryActive_; haveRetreatOrigin_=plan->haveRetreatOrigin_; retreatOrigin_=plan->retreatOrigin_;
  clearanceHeading_=plan->clearanceHeading_; clearanceHeadingTarget_=plan->clearanceHeadingTarget_;
  carryHeading_=plan->carryHeading_; carriedHeading_=plan->carriedHeading_; travelHeading_=plan->travelHeading_;
  state_=State::Driving; finalControllerReady_=false; settled_=-1;
  return false;
}
void GoalNavigationRoutine::ContinueWhilePlanning(const NavigationInput& in) {
  const double a=robot_geometry::ArenaAngle(in.motion.pose.headingRad),co=std::cos(a),si=std::sin(a);
  forward_=co*planningVelocity_.x+si*planningVelocity_.y; left_=-si*planningVelocity_.x+co*planningVelocity_.y;
  const double speed=std::hypot(forward_,left_);
  if (speed>navigation_config::SPEED_MMPS) { forward_*=navigation_config::SPEED_MMPS/speed; left_*=navigation_config::SPEED_MMPS/speed; }
  yaw_=obstacles_.TurnClear({in.motion.pose.x_MM,in.motion.pose.y_MM},in.motion.pose.headingRad,planningHeading_)?
      heading_.Request(robot_geometry::Wrap(planningHeading_-in.motion.pose.headingRad),heading_.FeedbackRequest(),in.motion.yawRateRadps,in.motion.dt,
          navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD,navigation_config::TRAVEL_HEADING_KP):0;
  ConstrainMotion(in); // never blindly replay a previous command into new evidence
  status_="Calculating detour; live odometry and clearance control";
}
double GoalNavigationRoutine::GuideRadius(unsigned index) const {
  const auto& p=route_[index];
  return std::min(navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM,
      std::max(5.0,obstacles_.Clearance({p.x_MM,p.y_MM},p.headingRad)-navigation_config::OBSTACLE_CLEARANCE_MM));
}
void GoalNavigationRoutine::AdvanceRoute(const NavigationInput& in) {
  const auto& m=in.motion; const ArenaPoint center={m.pose.x_MM,m.pose.y_MM};
  if (carryHeading_ && std::fabs(robot_geometry::Wrap(carriedHeading_-m.pose.headingRad))<=navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD)
    carryHeading_=false;
  while (leg_+1<route_.size()) {
    const auto p=route_[leg_];
    const double dx=p.x_MM-legStart_.x,dy=p.y_MM-legStart_.y,length=std::hypot(dx,dy);
    const bool inPlace=length<1;
    const double radius=GuideRadius(leg_);
    const double distance=std::hypot(p.x_MM-center.x,p.y_MM-center.y);
    const bool crossed=!inPlace && (center.x-p.x_MM)*dx+(center.y-p.y_MM)*dy>=0 &&
        std::fabs((center.x-p.x_MM)*dy-(center.y-p.y_MM)*dx)/length<=radius;
    if (distance>radius && !crossed) break;
    const bool tightTurn=static_cast<int>(leg_)==preciseTurnLeg_ &&
        !obstacles_.SegmentClear(center,{route_[leg_+1].x_MM,route_[leg_+1].y_MM},m.pose.headingRad);
    const double headingError=std::fabs(robot_geometry::Wrap(p.headingRad-m.pose.headingRad));
    const double tolerance=tightTurn?heading_control_config::HEADING_TOLERANCE_RAD:navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD;
    if ((inPlace || tightTurn) && headingError>tolerance) break;
    if (!carryHeading_ && headingError>navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD) {
      carryHeading_=true; carriedHeading_=p.headingRad;
    }
    if (static_cast<int>(leg_)==preciseTurnLeg_) preciseTurnLeg_=-1;
    travelHeading_=p.headingRad; legStart_={p.x_MM,p.y_MM}; ++leg_;
    if (detourActive_ && leg_>=detourEnd_) detourActive_=recoveryActive_=false;
  }
}
void GoalNavigationRoutine::DriveRoute(const NavigationInput& in,double previousYaw) {
  const auto& m=in.motion; const ArenaPoint center={m.pose.x_MM,m.pose.y_MM};
  AdvanceRoute(in);
  remaining_=std::hypot(route_[leg_].x_MM-center.x,route_[leg_].y_MM-center.y);
  for (unsigned i=leg_+1;i<route_.size();++i)
    remaining_+=std::hypot(route_[i].x_MM-route_[i-1].x_MM,route_[i].y_MM-route_[i-1].y_MM);
  const double angle=robot_geometry::ArenaAngle(m.pose.headingRad),co=std::cos(angle),si=std::sin(angle);
  const bool intermediate=leg_+1<route_.size();
  const double target=clearanceHeading_?clearanceHeadingTarget_:
      (carryHeading_?carriedHeading_:(intermediate?route_[leg_].headingRad:travelHeading_));
  const bool turnClear=obstacles_.TurnClear(center,m.pose.headingRad,target);
  if (intermediate && !turnClear && std::hypot(route_[leg_].x_MM-center.x,route_[leg_].y_MM-center.y)<=GuideRadius(leg_)) {
    if (PrepareTurnDetour(in,target)) DriveRoute(in,previousYaw);
    return;
  }
  const bool tightTurn=intermediate && static_cast<int>(leg_)==preciseTurnLeg_ &&
      !obstacles_.SegmentClear(center,{route_[leg_+1].x_MM,route_[leg_+1].y_MM},m.pose.headingRad);
  const double deadband=clearanceHeading_ || tightTurn?heading_control_config::HEADING_TOLERANCE_RAD:
      (intermediate?navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD:navigation_config::TRAVEL_HEADING_DEADBAND_RAD);
  yaw_=heading_.Request(robot_geometry::Wrap(target-m.pose.headingRad),previousYaw,m.yawRateRadps,m.dt,
      deadband,navigation_config::TRAVEL_HEADING_KP);
  if (!turnClear) yaw_=0; // continue translating to clear space before swinging the manipulator
  if (leg_+1==route_.size()) {
    if (!finalControllerReady_) {
      point_.Reset({goal_.x_MM,goal_.y_MM},m.now,navigation_config::SPEED_MMPS,navigation_config::ACCEL_MMPS2,
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
  const auto p=route_[leg_]; const double distance=std::hypot(p.x_MM-center.x,p.y_MM-center.y);
  if (distance<=GuideRadius(leg_)) {
    worldVX_=worldVY_=0; status_=(recoveryActive_?"Recovery turn ":"Intermediate ")+std::to_string(leg_+1)+": setting heading"; return;
  }
  ArenaPoint carrot={p.x_MM,p.y_MM};
  const auto next=route_[leg_+1]; const double ox=next.x_MM-p.x_MM,oy=next.y_MM-p.y_MM,outgoing=std::hypot(ox,oy);
  const double cap=recoveryActive_?navigation_config::RECOVERY_SPEED_MMPS:
      (detourActive_?navigation_config::DETOUR_SPEED_MMPS:navigation_config::SPEED_MMPS);
  double cornerSpeed=navigation_config::MIN_PASS_SPEED_MMPS;
  if (outgoing>1) {
    const double ix=p.x_MM-legStart_.x,iy=p.y_MM-legStart_.y,incoming=std::hypot(ix,iy);
    const double cosine=incoming>1?std::max(-1.0,std::min(1.0,(ix*ox+iy*oy)/(incoming*outgoing))):1;
    cornerSpeed=std::max(cornerSpeed,cap*std::sqrt((1+cosine)/2));
    double blend=std::min(std::min(navigation_config::CORNER_BLEND_MM,GuideRadius(leg_)),std::min(outgoing/2,
        std::max(0.0,navigation_config::LOOKAHEAD_MM-distance)));
    // Blend only through geometry that is clear for the present footprint.
    while (blend>1) {
      const ArenaPoint candidate={p.x_MM+ox*blend/outgoing,p.y_MM+oy*blend/outgoing};
      if (obstacles_.SegmentClear(center,candidate,m.pose.headingRad)) { carrot=candidate; break; }
      blend/=2;
    }
  }
  double speed=std::min(cap,std::sqrt(cornerSpeed*cornerSpeed+2*navigation_config::ACCEL_MMPS2*
      std::max(0.0,distance-navigation_config::LOOKAHEAD_MM)));
  // Anticipate final braking across the last transit pose, rather than handing
  // the final controller a high-speed command only a few millimeters out.
  speed=std::min(speed,std::max(navigation_config::MIN_PASS_SPEED_MMPS,
      std::sqrt(2*navigation_config::ACCEL_MMPS2*std::max(0.0,remaining_-cap*navigation_config::CLEARANCE_BRAKE_HORIZON_S))));
  if (std::hypot(carrot.x-center.x,carrot.y-center.y)<1) carrot={p.x_MM,p.y_MM};
  const double carrotDistance=std::hypot(carrot.x-center.x,carrot.y-center.y);
  const double wantedX=(carrot.x-center.x)*speed/carrotDistance,wantedY=(carrot.y-center.y)*speed/carrotDistance;
  const double change=std::hypot(wantedX-worldVX_,wantedY-worldVY_);
  const double gain=change>0?std::min(1.0,navigation_config::ACCEL_MMPS2*m.dt/change):1;
  worldVX_+=gain*(wantedX-worldVX_); worldVY_+=gain*(wantedY-worldVY_);
  forward_=co*worldVX_+si*worldVY_; left_=-si*worldVX_+co*worldVY_;
  status_=(recoveryActive_?"Recovery route ":(detourActive_?"Detour ":"Intermediate "))+
      std::to_string(leg_+1)+"/"+std::to_string(IntermediateCount())+"; XY and heading";
}
RobotPose GoalNavigationRoutine::SamplePose(double stamp) const {
  for (unsigned i=1;i<history_.size();++i) if (stamp<=history_[i].time) {
    const auto& a=history_[i-1]; const auto& b=history_[i];
    const double fraction=(stamp-a.time)/(b.time-a.time);
    return {a.pose.x_MM+fraction*(b.pose.x_MM-a.pose.x_MM),a.pose.y_MM+fraction*(b.pose.y_MM-a.pose.y_MM),
        robot_geometry::Wrap(a.pose.headingRad+fraction*robot_geometry::Wrap(b.pose.headingRad-a.pose.headingRad))};
  }
  return history_.back().pose;
}
void GoalNavigationRoutine::CorrectFromWalls(NavigationInput& in) {
  const auto& m=in.motion;
  if (!history_.empty() && m.now<history_.back().time) history_.clear();
  if (!history_.empty() && m.now==history_.back().time) history_.back().pose=m.pose;
  else history_.push_back({m.now,m.pose});
  while (history_.size()>1 && history_[1].time<m.now-navigation_config::WALL_HISTORY_S) history_.erase(history_.begin());
  double sums[2]={},weights[2]={};
  for (int sensor=0;sensor<4;++sensor) {
    if (!UsableRange(in,sensor) || in.stamp[sensor]<=consumedStamp_[sensor]) continue;
    consumedStamp_[sensor]=in.stamp[sensor];
    if (std::fabs(m.yawRateRadps)>navigation_config::WALL_MAX_YAW_RATE_RADPS || in.stamp[sensor]<history_.front().time) continue;
    const auto sample=SamplePose(in.stamp[sensor]);
    const auto mount=localization_config::RANGE_SENSOR_MOUNTS[sensor];
    const double angle=robot_geometry::ArenaAngle(sample.headingRad),co=std::cos(angle),si=std::sin(angle);
    const double origin[]={sample.x_MM+co*mount.mountX_MM-si*mount.mountY_MM,
        sample.y_MM+si*mount.mountX_MM+co*mount.mountY_MM};
    const double beam=angle+robot_geometry::Radians(mount.beamAngleDeg),component[]={std::cos(beam),std::sin(beam)};
    // Reuse known surfaces with ANY sensor after a turn. The nearest positive
    // ray/plane intersection is the expected surface; it must be near-normal.
    int axis=-1; double predicted=1e9;
    const auto consider=[&](int a,double coordinate) {
      if (a<0 || std::fabs(component[a])<=1e-6) return;
      const double distance=(coordinate-origin[a])/component[a];
      if (distance>0 && distance<predicted) { predicted=distance; axis=a; }
    };
    for (const auto& wall:walls_) consider(wall.axis,wall.coordinate);
    if (obstacles_.UsesCourt()) { consider(0,0); consider(0,localization_config::ARENA_WIDTH_MM);
      consider(1,0); consider(1,localization_config::ARENA_HEIGHT_MM); }
    if (axis<0 || std::fabs(component[axis])<navigation_config::WALL_NORMAL_COS ||
        std::fabs(predicted-in.rangeMM[sensor])>navigation_config::WALL_MAX_INNOVATION_MM) continue;
    const double weight=component[axis]*component[axis];
    sums[axis]+=weight*(predicted-in.rangeMM[sensor])*component[axis]; weights[axis]+=weight;
  }
  double step[2]={};
  for (int axis=0;axis<2;++axis) if (weights[axis]>0) {
    wallResidual_[axis]=sums[axis]/weights[axis]; wallTime_[axis]=m.now;
    step[axis]=std::max(-navigation_config::WALL_MAX_STEP_MM,std::min(navigation_config::WALL_MAX_STEP_MM,
        navigation_config::WALL_BLEND*wallResidual_[axis]));
    wallResidual_[axis]-=step[axis];
  }
  positionCorrection_={step[0],step[1]}; in.motion.pose.x_MM+=step[0]; in.motion.pose.y_MM+=step[1];
  obstacles_.ShiftHits(positionCorrection_);
  // A correction changes the estimator's frame, not its gyro/encoder baselines.
  for (auto& sample:history_) { sample.pose.x_MM+=step[0]; sample.pose.y_MM+=step[1]; }
  const bool x=Fresh(m.now,wallTime_[0],localization_config::SONAR_READING_STALE_S);
  const bool y=Fresh(m.now,wallTime_[1],localization_config::SONAR_READING_STALE_S);
  wallCorrectionPending_=(x && std::fabs(wallResidual_[0])>navigation_config::WALL_FINAL_RESIDUAL_MM) ||
      (y && std::fabs(wallResidual_[1])>navigation_config::WALL_FINAL_RESIDUAL_MM);
  wallStatus_=std::string("Wall support: X ")+(x?"active":"unavailable")+", Y "+(y?"active":"unavailable")+
      "; other motion uses encoder + gyro";
}
void GoalNavigationRoutine::ObserveObstacles(const NavigationInput& in) {
  for (int sensor=0;sensor<4;++sensor) {
    if (!UsableRange(in,sensor) || in.stamp[sensor]<=observedStamp_[sensor] || in.stamp[sensor]<history_.front().time) continue;
    observedStamp_[sensor]=in.stamp[sensor]; const auto sample=SamplePose(in.stamp[sensor]);
    const auto mount=localization_config::RANGE_SENSOR_MOUNTS[sensor];
    const double angle=robot_geometry::ArenaAngle(sample.headingRad),co=std::cos(angle),si=std::sin(angle);
    const double beam=angle+robot_geometry::Radians(mount.beamAngleDeg);
    const ArenaPoint origin={sample.x_MM+co*mount.mountX_MM-si*mount.mountY_MM,sample.y_MM+si*mount.mountX_MM+co*mount.mountY_MM};
    const ArenaPoint hit={origin.x+in.rangeMM[sensor]*std::cos(beam),origin.y+in.rangeMM[sensor]*std::sin(beam)};
    bool knownWall=false;
    for (const auto& wall:walls_) if (wall.axis>=0 && std::fabs((wall.axis==0?hit.x:hit.y)-wall.coordinate)<40) knownWall=true;
    const double bodyBeam=robot_geometry::Radians(mount.beamAngleDeg);
    const double closing=std::max(0.0,in.forwardMMps*std::cos(bodyBeam)+in.leftMMps*std::sin(bodyBeam));
    const double reach=sensor==3?navigation_config::FRONT_REACH_MM:navigation_config::BODY_RADIUS_MM;
    const double free=in.rangeMM[sensor]+mount.mountX_MM*std::cos(bodyBeam)+mount.mountY_MM*std::sin(bodyBeam)-reach;
    const bool urgent=free<=navigation_config::OBSTACLE_CLEARANCE_MM+closing*navigation_config::CLEARANCE_BRAKE_HORIZON_S+
        closing*closing/(2*navigation_config::ACCEL_MMPS2)+closing*(in.motion.now-in.stamp[sensor]);
    obstacles_.ObserveHit(origin,hit,beam,in.stamp[sensor],sensor,urgent,
        !knownWall && in.rangeMM[sensor]<=navigation_config::OBSTACLE_OBSERVE_RANGE_MM);
  }
}
bool GoalNavigationRoutine::EnsureClearRoute(const NavigationInput& in) {
  clearanceHeading_=false;
  const ArenaPoint start={in.motion.pose.x_MM,in.motion.pose.y_MM};
  const auto target=route_[leg_];
  if (obstacles_.SegmentClear(start,{target.x_MM,target.y_MM},in.motion.pose.headingRad,navigation_config::OBSTACLE_CLEARANCE_MM,true)) return true;
  const double wantedHeading=leg_+1<route_.size()?target.headingRad:travelHeading_;
  if (obstacles_.SegmentClear(start,{target.x_MM,target.y_MM},wantedHeading) &&
      obstacles_.TurnClear(start,in.motion.pose.headingRad,wantedHeading)) {
    // Rotate while translating when the outgoing orientation is safe. A small
    // heading error must not create an extra stop-and-align waypoint.
    clearanceHeading_=true; clearanceHeadingTarget_=wantedHeading; return true;
  }
  // Repair only the current manual leg; preserve every subsequent requested pose.
  if (backgroundPlanning_) return RequestPlan(in,false);
  unsigned original=detourActive_?detourEnd_:leg_;
  const auto rejoin=route_[original]; std::vector<ArenaPoint> bypass;
  if (!obstacles_.Detour(start,{rejoin.x_MM,rejoin.y_MM},in.motion.pose.headingRad,bypass)) {
    return RecoverRoute(in,original);
  }
  if (!bypass.empty() && std::hypot(bypass.back().x-rejoin.x_MM,bypass.back().y-rejoin.y_MM)<1) bypass.pop_back();
  if (bypass.empty()) return RecoverRoute(in,original);
  if (detourActive_) {
    if (preciseTurnLeg_>=static_cast<int>(leg_) && preciseTurnLeg_<static_cast<int>(detourEnd_)) preciseTurnLeg_=-1;
    else if (preciseTurnLeg_>=static_cast<int>(detourEnd_)) preciseTurnLeg_-=detourEnd_-leg_;
    route_.erase(route_.begin()+leg_,route_.begin()+detourEnd_);
  }
  std::vector<RobotPose> inserted;
  for (const auto& p:bypass) inserted.push_back({p.x,p.y,in.motion.pose.headingRad});
  if (preciseTurnLeg_>=static_cast<int>(leg_)) preciseTurnLeg_+=inserted.size();
  route_.insert(route_.begin()+leg_,inserted.begin(),inserted.end());
  legStart_=start;
  detourActive_=true; detourEnd_=leg_+inserted.size(); finalControllerReady_=false;
  return true;
}
bool GoalNavigationRoutine::PrepareTurnDetour(const NavigationInput& in,double heading) {
  if (backgroundPlanning_) return RequestPlan(in,true,heading);
  const ArenaPoint start={in.motion.pose.x_MM,in.motion.pose.y_MM};
  std::vector<ArenaPoint> best; ArenaPoint staging={0,0}; double bestLength=1e9;
  const double step=navigation_config::DETOUR_GRID_MM;
  if (obstacles_.TurnClear(start,in.motion.pose.headingRad,heading,navigation_config::OBSTACLE_CLEARANCE_MM+
      navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM)) { staging=start; best.push_back(start); }
  for (int ring=1;ring<=6 && best.empty();++ring) {
    for (int dx=-ring;dx<=ring;++dx) for (int dy=-ring;dy<=ring;++dy) {
      if (std::abs(dx)!=ring && std::abs(dy)!=ring) continue;
      const ArenaPoint p={start.x+dx*step,start.y+dy*step};
      if (p.x<0 || p.y<0 || p.x>localization_config::ARENA_WIDTH_MM || p.y>localization_config::ARENA_HEIGHT_MM ||
          !obstacles_.TurnClear(p,in.motion.pose.headingRad,heading,navigation_config::OBSTACLE_CLEARANCE_MM+
              navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM)) continue;
      std::vector<ArenaPoint> route;
      if (!obstacles_.Detour(start,p,in.motion.pose.headingRad,route)) continue;
      double length=0; auto previous=start;
      for (const auto& point:route) { length+=std::hypot(point.x-previous.x,point.y-previous.y); previous=point; }
      if (length<bestLength) { bestLength=length; staging=p; best=std::move(route); }
    }
  }
  if (best.empty()) return RecoverRoute(in,detourActive_?detourEnd_:leg_);
  std::vector<RobotPose> inserted;
  for (const auto& p:best) inserted.push_back({p.x,p.y,in.motion.pose.headingRad});
  inserted.push_back({staging.x,staging.y,heading});
  // Keep all requested poses; return to the current target after the clear-space turn.
  route_.insert(route_.begin()+leg_,inserted.begin(),inserted.end());
  legStart_=start; clearanceHeading_=false;
  preciseTurnLeg_=leg_+inserted.size()-1;
  detourActive_=true; detourEnd_=leg_+inserted.size(); finalControllerReady_=false;
  state_=State::Driving; settled_=-1; return true;
}
void GoalNavigationRoutine::InstallRecovery(const NavigationInput& in,unsigned rejoin,const std::vector<RobotPose>& points,int precise) {
  // Remove only superseded temporary poses / an unreachable transit guide.
  // The chosen rejoin pose, remaining requested route, and final goal survive.
  const unsigned removed=rejoin-leg_;
  if (preciseTurnLeg_>=static_cast<int>(leg_) && preciseTurnLeg_<static_cast<int>(rejoin)) preciseTurnLeg_=-1;
  else if (preciseTurnLeg_>=static_cast<int>(rejoin)) preciseTurnLeg_-=removed;
  route_.erase(route_.begin()+leg_,route_.begin()+rejoin);
  if (preciseTurnLeg_>=static_cast<int>(leg_)) preciseTurnLeg_+=points.size();
  route_.insert(route_.begin()+leg_,points.begin(),points.end());
  if (precise>=0) preciseTurnLeg_=leg_+precise;
  legStart_={in.motion.pose.x_MM,in.motion.pose.y_MM};
  detourActive_=recoveryActive_=!points.empty(); detourEnd_=leg_+points.size();
  carryHeading_=clearanceHeading_=false; finalControllerReady_=false;
  state_=State::Driving; settled_=-1;
}
bool GoalNavigationRoutine::RecoverRoute(const NavigationInput& in,unsigned original) {
  const ArenaPoint start={in.motion.pose.x_MM,in.motion.pose.y_MM};
  const double current=in.motion.pose.headingRad,margin=navigation_config::OBSTACLE_CLEARANCE_MM;
  unsigned first=original;
  // Transit guides are optional path guides. Do not insist on entering one
  // whose requested footprint is now occupied by confirmed sensor evidence.
  if (original+1<route_.size() && obstacles_.Clearance({route_[original].x_MM,route_[original].y_MM},route_[original].headingRad)<margin) ++first;
  unsigned searches=0;
  for (unsigned destination=first;destination<=std::min(original+1,static_cast<unsigned>(route_.size()-1)) && searches<8;++destination) {
    const auto target=route_[destination]; const ArenaPoint goal={target.x_MM,target.y_MM};
    if (obstacles_.Clearance(goal,target.headingRad)<margin) continue;
    std::vector<double> headings;
    for (double value:{target.headingRad,current+robot_geometry::PI/2,current-robot_geometry::PI/2,
        current+robot_geometry::PI,current-robot_geometry::PI,0.0,robot_geometry::PI/2,-robot_geometry::PI/2}) {
      value=robot_geometry::Wrap(value); bool duplicate=false;
      for (double old:headings) if (std::fabs(old-value)<robot_geometry::Radians(1)) duplicate=true;
      if (!duplicate) headings.push_back(value);
    }
    for (double heading:headings) {
      if (obstacles_.Cancelled()) return false;
      if (searches>=8) break;
      if (obstacles_.Clearance(goal,heading)<margin) continue;
      struct Stage { ArenaPoint point; double cost; };
      std::vector<Stage> stages;
      if (obstacles_.TurnClear(start,current,heading)) stages.push_back({start,-1});
      else for (double radius=50;radius<=navigation_config::RECOVERY_SEARCH_RADIUS_MM;radius+=50)
        for (int direction=0;direction<8;++direction) {
          const double a=direction*robot_geometry::PI/4;
          const ArenaPoint p={start.x+radius*std::cos(a),start.y+radius*std::sin(a)};
          if (p.x<0 || p.y<0 || p.x>localization_config::ARENA_WIDTH_MM || p.y>localization_config::ARENA_HEIGHT_MM ||
              !obstacles_.TurnClear(p,current,heading,margin+navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM) ||
              !obstacles_.SegmentClear(start,p,current,margin,true)) continue;
          stages.push_back({p,radius+std::hypot(goal.x-p.x,goal.y-p.y)});
        }
      std::sort(stages.begin(),stages.end(),[](const Stage& a,const Stage& b) { return a.cost<b.cost; });
      for (unsigned i=0;i<std::min(2u,static_cast<unsigned>(stages.size())) && searches<8;++i) {
        ++searches; const auto stage=stages[i].point; std::vector<ArenaPoint> path;
        if (!obstacles_.Detour(stage,goal,heading,path)) continue;
        std::vector<RobotPose> inserted; int precise=-1; bool movingTurn=false;
        if (std::hypot(stage.x-start.x,stage.y-start.y)<1 && !path.empty() &&
            std::fabs(robot_geometry::Wrap(heading-current))>navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD) {
          // If the whole turning sweep fits along a short clear prefix, use
          // a moving heading guide instead of an artificial in-place turn.
          const double length=std::hypot(path.front().x-start.x,path.front().y-start.y);
          if (length>50) {
            const double t=std::min(120.0,length/2)/length;
            const ArenaPoint moving={start.x+t*(path.front().x-start.x),start.y+t*(path.front().y-start.y)};
            movingTurn=true;
            for (int sample=0;sample<=6;++sample) {
              const ArenaPoint p={start.x+(moving.x-start.x)*sample/6,start.y+(moving.y-start.y)*sample/6};
              if (!obstacles_.TurnClear(p,current,heading,margin+navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM)) movingTurn=false;
            }
            if (movingTurn) inserted.push_back({moving.x,moving.y,heading});
          }
        }
        if (std::hypot(stage.x-start.x,stage.y-start.y)>1) inserted.push_back({stage.x,stage.y,current});
        if (!movingTurn && (std::fabs(robot_geometry::Wrap(heading-current))>navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD ||
            !obstacles_.SegmentClear(stage,goal,current))) {
          precise=inserted.size(); inserted.push_back({stage.x,stage.y,heading});
        }
        for (const auto& p:path) if (std::hypot(p.x-goal.x,p.y-goal.y)>1) inserted.push_back({p.x,p.y,heading});
        InstallRecovery(in,destination,inserted,precise);
        if (inserted.empty()) { clearanceHeading_=true; clearanceHeadingTarget_=heading; }
        return true;
      }
    }
  }
  // If a full connection is not available yet, move to a verified clearer
  // nearby pose and retry from there as fresh sensor evidence arrives.
  ArenaPoint retreat={0,0}; double best=-1e9; bool found=false;
  const double angle=robot_geometry::ArenaAngle(current),co=std::cos(angle),si=std::sin(angle);
  const double initial=obstacles_.Clearance(start,current);
  for (int direction=0;direction<8;++direction) for (double distance:{100.0,150.0,200.0}) {
    const double a=direction*robot_geometry::PI/4,x=std::cos(a),y=std::sin(a);
    const ArenaPoint p={start.x+distance*x,start.y+distance*y};
    if (p.x<0 || p.y<0 || p.x>localization_config::ARENA_WIDTH_MM || p.y>localization_config::ARENA_HEIGHT_MM ||
        (haveRetreatOrigin_ && std::hypot(p.x-retreatOrigin_.x,p.y-retreatOrigin_.y)<75) ||
        !obstacles_.SegmentClear(start,p,current,margin,true)) continue;
    const double gap=obstacles_.Clearance(p,current); if (gap<initial+10) continue;
    const double f=co*x+si*y,l=-si*x+co*y;
    bool measuredClear=false,blocked=false;
    for (int sensor=0;sensor<4;++sensor) {
      const auto mount=localization_config::RANGE_SENSOR_MOUNTS[sensor];
      const double beam=robot_geometry::Radians(mount.beamAngleDeg),c=std::cos(beam),s=std::sin(beam),toward=f*c+l*s;
      if (toward<=0.7) continue;
      if (!UsableRange(in,sensor)) { blocked=true; continue; }
      const double reach=sensor==3?navigation_config::FRONT_REACH_MM:navigation_config::BODY_RADIUS_MM;
      const double free=in.rangeMM[sensor]+mount.mountX_MM*c+mount.mountY_MM*s-reach;
      const double speed=navigation_config::RECOVERY_SPEED_MMPS;
      const double braking=speed*(navigation_config::CLEARANCE_BRAKE_HORIZON_S+in.motion.now-in.stamp[sensor])+
          speed*speed/(2*navigation_config::ACCEL_MMPS2);
      if (free>=distance*toward+margin+braking) measuredClear=true; else blocked=true;
    }
    const double score=gap-0.05*std::hypot(goal_.x_MM-p.x,goal_.y_MM-p.y);
    if (measuredClear && !blocked && score>best) { best=score; retreat=p; found=true; }
  }
  if (found) {
    InstallRecovery(in,original,{{retreat.x,retreat.y,current}});
    retreatOrigin_=start; haveRetreatOrigin_=true; return true;
  }
  Fail("Recovery blocked: no map/sensor-verified collision-free movement"); return false;
}
void GoalNavigationRoutine::ConstrainMotion(const NavigationInput& in) {
  // Brake the component that would consume clearance during motor coasting;
  // keep tangential/away motion. This is not a wait-for-obstacle state.
  const double horizon=navigation_config::CLEARANCE_BRAKE_HORIZON_S;
  const double a=robot_geometry::ArenaAngle(in.motion.pose.headingRad),co=std::cos(a),si=std::sin(a);
  const ArenaPoint predicted={in.motion.pose.x_MM+horizon*(co*in.forwardMMps-si*in.leftMMps),
      in.motion.pose.y_MM+horizon*(si*in.forwardMMps+co*in.leftMMps)};
  const double heading=in.motion.pose.headingRad+horizon*in.motion.yawRateRadps;
  const double gap=obstacles_.Clearance(predicted,heading);
  if (gap>=navigation_config::OBSTACLE_CLEARANCE_MM) return;
  const double gx=obstacles_.Clearance({predicted.x+2,predicted.y},heading)-obstacles_.Clearance({predicted.x-2,predicted.y},heading);
  const double gy=obstacles_.Clearance({predicted.x,predicted.y+2},heading)-obstacles_.Clearance({predicted.x,predicted.y-2},heading);
  double x=co*forward_-si*left_,y=si*forward_+co*left_;
  const double norm=gx*gx+gy*gy,toward=x*gx+y*gy;
  if (toward<0 && norm>1e-9) { x-=toward*gx/norm; y-=toward*gy/norm; }
  forward_=co*x+si*y; left_=-si*x+co*y; worldVX_=x; worldVY_=y;
  if (yaw_*in.motion.yawRateRadps>0 && obstacles_.Clearance(predicted,in.motion.pose.headingRad)>gap) yaw_=0;
}
void GoalNavigationRoutine::Update(const NavigationInput& input) {
  NavigationInput in=input;
  positionCorrection_={0,0};
  const auto& m=in.motion; const double previousYaw=yaw_; Stop();
  if (m.stopRequested) { Abort(); return; }
  if (!IsRunning()) return;
  if (state_==State::Reference) {
    if (!HealthyMotion(in)) { status_=initializationStatus_="Waiting for IMU/odometry"; return; }
    double sums[2]={},weights[2]={}; unsigned count[2]={};
    const double angle=robot_geometry::ArenaAngle(initialHeading_),co=std::cos(angle),si=std::sin(angle);
    for (int sensor=0;sensor<4;++sensor) {
      const int axis=walls_[sensor].axis;
      if (axis<0 || initialKnown_[axis]) continue;
      if (!UsableRange(in,sensor)) continue;
      const double range=in.rangeMM[sensor];
      const auto mount=localization_config::RANGE_SENSOR_MOUNTS[sensor];
      const double beam=angle+robot_geometry::Radians(mount.beamAngleDeg);
      const double component=axis==0?std::cos(beam):std::sin(beam);
      if (std::fabs(component)<0.5) continue; // nearly parallel beams cannot locate this axis
      const double offset=axis==0?co*mount.mountX_MM-si*mount.mountY_MM:si*mount.mountX_MM+co*mount.mountY_MM;
      const double estimate=walls_[sensor].coordinate-offset-range*component;
      const double limit=axis==0?localization_config::ARENA_WIDTH_MM:localization_config::ARENA_HEIGHT_MM;
      if (!std::isfinite(estimate) || estimate<0 || estimate>limit) continue;
      const double weight=component*component; // geometric weighting, no covariance model
      sums[axis]+=weight*estimate; weights[axis]+=weight; ++count[axis];
    }
    for (int axis=0;axis<2;++axis) if (!initialKnown_[axis] && weights[axis]==0) {
      status_=initializationStatus_=std::string("Need start ")+(axis==0?"X":"Y")+" or a valid sensor reference for that axis"; return;
    }
    requestedPose_={initialKnown_[0]?initialCoordinate_[0]:sums[0]/weights[0],
        initialKnown_[1]?initialCoordinate_[1]:sums[1]/weights[1],initialHeading_};
    initializationStatus_=std::string("X: ")+(initialKnown_[0]?"given":std::to_string(count[0])+" sensor(s)")+
        "; Y: "+(initialKnown_[1]?"given":std::to_string(count[1])+" sensor(s)");
    state_=State::ReferenceReady; status_="Starting pose ready"; return;
  }
  if (state_==State::ReferenceReady) return;
  const auto issue=MotionIssue(in);
  if (!issue.empty()) { Fail(issue+" - stopped"); return; }
  in.motion.dt=std::min(in.motion.dt,localization_config::AUTO_MAX_CONTROL_STEP_S);
  CorrectFromWalls(in);
  ObserveObstacles(in);
  if (m.now-started_>navigation_config::MISSION_TIMEOUT_S) { Fail("Task timeout - stopped"); return; }
  const auto fault=heading_.CheckFeedback(previousYaw,m.yawRateRadps,m.dt);
  if (!fault.empty()) { Fail(fault); return; }
  if (PollPlan()) { ContinueWhilePlanning(in); return; }
  if (!IsRunning()) return;
  remaining_=std::hypot(goal_.x_MM-m.pose.x_MM,goal_.y_MM-m.pose.y_MM);
  if (state_==State::Driving) {
    AdvanceRoute(in); // leave accepted guides before checking/replanning the next segment
    if (!EnsureClearRoute(in)) { if (IsPlanning()) ContinueWhilePlanning(in); return; }
    DriveRoute(in,previousYaw);
  } else if (state_==State::FinalHeading) {
    if (!obstacles_.TurnClear({m.pose.x_MM,m.pose.y_MM},m.pose.headingRad,goal_.headingRad)) {
      if (PrepareTurnDetour(in,goal_.headingRad)) { DriveRoute(in,previousYaw); ConstrainMotion(in); }
      else if (IsPlanning()) ContinueWhilePlanning(in);
      return;
    }
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
        std::hypot(in.forwardMMps,in.leftMMps)<=heading_control_config::STATIONARY_SPEED_MMPS && !wallCorrectionPending_) {
      yaw_=0;
      if (settled_<0) settled_=m.now;
      if (m.now-settled_>=0.30) { Stop(); state_=State::Complete; status_="Goal reached: within 10 mm / 1 deg"; }
    } else settled_=-1;
  }
  if (IsPlanning()) ContinueWhilePlanning(in);
  else if (IsRunning()) ConstrainMotion(in);
}
