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
  if (m.dt>localization_config::AUTO_MAX_SAMPLE_GAP_S) return "Control loop gap "+std::to_string(static_cast<int>(m.dt*1000))+
      " ms exceeds "+std::to_string(static_cast<int>(localization_config::AUTO_MAX_SAMPLE_GAP_S*1000))+" ms";
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
struct RangeRay { double origin[2],direction[2]; };
RangeRay SensorRay(const RobotPose& pose,int sensor) {
  const auto mount=localization_config::RANGE_SENSOR_MOUNTS[sensor];
  const double angle=robot_geometry::ArenaAngle(pose.headingRad),co=std::cos(angle),si=std::sin(angle);
  const double beam=angle+robot_geometry::Radians(mount.beamAngleDeg);
  return {{pose.x_MM+co*mount.mountX_MM-si*mount.mountY_MM,pose.y_MM+si*mount.mountX_MM+co*mount.mountY_MM},
      {std::cos(beam),std::sin(beam)}};
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
void GoalNavigationRoutine::Enter() { *this=GoalNavigationRoutine{}; }
std::string GoalNavigationRoutine::DefaultIntermediates(int source,int destination) {
  if (source<0 || source>=navigation_config::GOAL_COUNT || destination<0 || destination>=navigation_config::GOAL_COUNT) return {};
  std::ostringstream poses;
  bool first=true;
  for (const int guide:navigation_config::ROUTES[source][destination].guide) {
    if (guide==navigation_config::NO_GUIDE) continue;
    const auto& pose=navigation_config::INTERMEDIATES[guide];
    if (!first) poses<<';';
    poses<<pose.xMM<<','<<pose.yMM<<','<<pose.headingDeg; first=false;
  }
  return poses.str();
}
void GoalNavigationRoutine::Abort() { Stop(); state_=State::Aborted; status_="Stopped"; }
void GoalNavigationRoutine::Fail(const std::string& message) { Stop(); state_=State::Fault; status_=message; }
bool GoalNavigationRoutine::InitializePose(const PoseInitialization& settings) {
  if (IsRunning() && state_!=State::Reference && state_!=State::ReferenceReady) return false;
  Stop(); positionCorrection_={0,0}; referenced_=false; state_=State::Unreferenced;
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
  controlPose_=requestedPose_;
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
  // Consecutive goals retain a valid driving offset if that axis could not
  // receive a stationary global update at the preceding goal.
  const bool continuing=state_==State::Complete;
  if (!continuing) for (auto& offset:rangeOffset_) offset=0;
  const RobotPose start={in.motion.pose.x_MM+rangeOffset_[0],in.motion.pose.y_MM+rangeOffset_[1],in.motion.pose.headingRad};
  route.push_back(goal); route_=std::move(route); leg_=0; worldVX_=worldVY_=0;
  legStart_={start.x_MM,start.y_MM}; carryHeading_=transitHeadingActive_=false;
  transitHeadingTarget_=in.motion.pose.headingRad;
  finalControllerReady_=route_.size()==1;
  goal_=goal; goal_.headingRad=robot_geometry::Wrap(goal_.headingRad);
  settled_=-1; travelHeading_=in.motion.pose.headingRad;
  remaining_=0; auto previous=start;
  for (auto p:route_) { remaining_+=std::hypot(p.x_MM-previous.x_MM,p.y_MM-previous.y_MM); previous=p; }
  point_.Reset({goal_.x_MM,goal_.y_MM},in.motion.now,navigation_config::SPEED_MMPS,navigation_config::ACCEL_MMPS2);
  heading_.Reset(); Stop(); state_=State::Driving;
  history_.clear(); positionCorrection_={0,0};
  controlPose_=start;
  for (int axis=0;axis<2;++axis) { rangeTarget_[axis]=0; rangeStamp_[axis]=-1; rangeSources_[axis]=0; }
  for (auto& stamp:consumedStamp_) stamp=-1;
  for (auto& time:wallTime_) time=-1;
  status_=route_.size()==1?"Direct XY travel; holding starting heading":"Manual pose route started";
  return true;
}
void GoalNavigationRoutine::AdvanceRoute(const NavigationInput& in) {
  const auto& m=in.motion; const ArenaPoint center={m.pose.x_MM,m.pose.y_MM};
  if (carryHeading_ && std::fabs(robot_geometry::Wrap(carriedHeading_-m.pose.headingRad))<=navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD)
    carryHeading_=false;
  while (leg_+1<route_.size()) {
    const auto p=route_[leg_];
    const double dx=p.x_MM-legStart_.x,dy=p.y_MM-legStart_.y,length=std::hypot(dx,dy);
    const bool inPlace=length<navigation_config::MIN_SEGMENT_MM;
    const double radius=navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM;
    const double distance=std::hypot(p.x_MM-center.x,p.y_MM-center.y);
    const bool crossed=!inPlace && (center.x-p.x_MM)*dx+(center.y-p.y_MM)*dy>=0 &&
        std::fabs((center.x-p.x_MM)*dy-(center.y-p.y_MM)*dx)/length<=radius;
    if (distance>radius && !crossed) break;
    const double headingError=std::fabs(robot_geometry::Wrap(p.headingRad-m.pose.headingRad));
    if (inPlace && headingError>navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD) break;
    if (!carryHeading_ && headingError>navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD) {
      carryHeading_=true; carriedHeading_=p.headingRad;
    }
    travelHeading_=p.headingRad; legStart_={p.x_MM,p.y_MM}; ++leg_;
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
  const double target=carryHeading_?carriedHeading_:(intermediate?route_[leg_].headingRad:travelHeading_);
  const double headingError=robot_geometry::Wrap(target-m.pose.headingRad);
  const bool inPlace=intermediate && std::hypot(route_[leg_].x_MM-legStart_.x,route_[leg_].y_MM-legStart_.y)<navigation_config::MIN_SEGMENT_MM;
  const bool approach=remaining_<=navigation_config::FINAL_APPROACH_DISTANCE_MM;
  if (approach || inPlace) {
    transitHeadingActive_=false;
    yaw_=heading_.Request(headingError,previousYaw,m.yawRateRadps,m.dt,
        inPlace?navigation_config::INTERMEDIATE_HEADING_TOLERANCE_RAD:navigation_config::APPROACH_HEADING_DEADBAND_RAD,
        navigation_config::APPROACH_HEADING_KP);
  } else {
    if (target!=transitHeadingTarget_) { transitHeadingTarget_=target; transitHeadingActive_=false; }
    if (!transitHeadingActive_ && std::fabs(headingError)>=navigation_config::TRANSIT_HEADING_START_RAD) transitHeadingActive_=true;
    if (std::fabs(headingError)<=navigation_config::TRANSIT_HEADING_RELEASE_RAD) transitHeadingActive_=false;
    yaw_=transitHeadingActive_?heading_.Request(headingError,previousYaw,m.yawRateRadps,m.dt,
        navigation_config::TRANSIT_HEADING_RELEASE_RAD,navigation_config::TRANSIT_HEADING_KP):0;
    // A coasting/braking release completes this correction. Do not restart it
    // at the same threshold on the next frame while XY travel continues.
    if (std::fabs(yaw_)<=wheel_drive_config::COMMAND_STOP_RADPS) transitHeadingActive_=false;
  }
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
  if (distance<=navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM) {
    worldVX_=worldVY_=0; status_="In-place turn "+std::to_string(leg_+1); return;
  }
  ArenaPoint carrot={p.x_MM,p.y_MM};
  const auto next=route_[leg_+1]; const double ox=next.x_MM-p.x_MM,oy=next.y_MM-p.y_MM,outgoing=std::hypot(ox,oy);
  const double cap=navigation_config::SPEED_MMPS;
  double cornerSpeed=navigation_config::MIN_PASS_SPEED_MMPS;
  if (outgoing>navigation_config::MIN_SEGMENT_MM) {
    const double ix=p.x_MM-legStart_.x,iy=p.y_MM-legStart_.y,incoming=std::hypot(ix,iy);
    const double cosine=incoming>navigation_config::MIN_SEGMENT_MM?std::max(-1.0,std::min(1.0,(ix*ox+iy*oy)/(incoming*outgoing))):1;
    cornerSpeed=std::max(cornerSpeed,cap*std::sqrt((1+cosine)/2));
    const double blend=std::min(std::min(navigation_config::CORNER_BLEND_MM,navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM),std::min(outgoing/2,
        std::max(0.0,navigation_config::LOOKAHEAD_MM-distance)));
    carrot={p.x_MM+ox*blend/outgoing,p.y_MM+oy*blend/outgoing};
  }
  double speed=std::min(cap,std::sqrt(cornerSpeed*cornerSpeed+2*navigation_config::ACCEL_MMPS2*
      std::max(0.0,distance-navigation_config::LOOKAHEAD_MM)));
  // Anticipate final braking across the last transit pose, rather than handing
  // the final controller a high-speed command only a few millimeters out.
  speed=std::min(speed,std::max(navigation_config::MIN_PASS_SPEED_MMPS,
      std::sqrt(2*navigation_config::ACCEL_MMPS2*std::max(0.0,remaining_-cap*navigation_config::BRAKE_PREVIEW_S))));
  if (std::hypot(carrot.x-center.x,carrot.y-center.y)<navigation_config::MIN_SEGMENT_MM) carrot={p.x_MM,p.y_MM};
  const double carrotDistance=std::hypot(carrot.x-center.x,carrot.y-center.y);
  const double wantedX=(carrot.x-center.x)*speed/carrotDistance,wantedY=(carrot.y-center.y)*speed/carrotDistance;
  const double change=std::hypot(wantedX-worldVX_,wantedY-worldVY_);
  const double gain=change>0?std::min(1.0,navigation_config::ACCEL_MMPS2*m.dt/change):1;
  worldVX_+=gain*(wantedX-worldVX_); worldVY_+=gain*(wantedY-worldVY_);
  forward_=co*worldVX_+si*worldVY_; left_=-si*worldVX_+co*worldVY_;
  status_="Intermediate "+std::to_string(leg_+1)+"/"+std::to_string(IntermediateCount())+"; XY and heading";
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
  double sums[2]={},weights[2]={},stamp[2]={m.now,m.now};
  unsigned sources[2]={};
  for (int sensor=0;sensor<4;++sensor) {
    if (!UsableRange(in,sensor) || in.stamp[sensor]<=consumedStamp_[sensor]) continue;
    consumedStamp_[sensor]=in.stamp[sensor];
    if (in.stamp[sensor]<history_.front().time) continue;
    const auto sample=SamplePose(in.stamp[sensor]);
    const auto ray=SensorRay(sample,sensor);
    // Beams rotate with the chassis: use heading at acquisition, not a fixed
    // world-facing direction. The nearest known forward plane is the reference.
    int axis=-1; double predicted=1e9;
    const auto consider=[&](int a,double coordinate) {
      if (a<0 || std::fabs(ray.direction[a])<=1e-6) return;
      const double distance=(coordinate-ray.origin[a])/ray.direction[a];
      if (distance>0 && distance<predicted) { predicted=distance; axis=a; }
    };
    for (const auto& wall:walls_) consider(wall.axis,wall.coordinate);
    consider(0,0); consider(0,localization_config::ARENA_WIDTH_MM);
    consider(1,0); consider(1,localization_config::ARENA_HEIGHT_MM);
    if (arena_config::HORIZONTAL_WALL_REFERENCE_ENABLED) {
      // A finite wall, not an infinite Y plane: a beam alongside its ends
      // must still reference the outer wall, with mount/heading at acquisition.
      for (const double face:{arena_config::HORIZONTAL_WALL_SOUTH_Y_MM,arena_config::HORIZONTAL_WALL_NORTH_Y_MM}) {
        if (std::fabs(ray.direction[1])<=1e-6) continue;
        const double distance=(face-ray.origin[1])/ray.direction[1];
        const double hitX=ray.origin[0]+distance*ray.direction[0];
        if (hitX>=arena_config::HORIZONTAL_WALL_MIN_X_MM && hitX<=arena_config::HORIZONTAL_WALL_MAX_X_MM)
          consider(1,face);
      }
    }
    if (axis<0 || std::fabs(predicted-in.rangeMM[sensor])>navigation_config::WALL_MAX_INNOVATION_MM) continue;
    const double weight=ray.direction[axis]*ray.direction[axis];
    sums[axis]+=weight*(predicted-in.rangeMM[sensor])*ray.direction[axis]; weights[axis]+=weight;
    stamp[axis]=std::min(stamp[axis],in.stamp[sensor]);
    sources[axis]|=1u<<sensor;
  }
  for (int axis=0;axis<2;++axis) if (weights[axis]>0) {
    wallTime_[axis]=m.now; rangeStamp_[axis]=stamp[axis]; rangeTarget_[axis]=sums[axis]/weights[axis]; rangeSources_[axis]=sources[axis];
    rangeOffset_[axis]+=std::max(-navigation_config::WALL_MAX_STEP_MM,std::min(navigation_config::WALL_MAX_STEP_MM,
        navigation_config::WALL_BLEND*(rangeTarget_[axis]-rangeOffset_[axis])));
  }
  // Guide driving with the range/encoder difference. Global encoder odometry
  // and its acquisition history stay untouched while the chassis moves.
  in.motion.pose.x_MM+=rangeOffset_[0]; in.motion.pose.y_MM+=rangeOffset_[1];
  const bool x=Fresh(m.now,wallTime_[0],localization_config::SONAR_READING_STALE_S);
  const bool y=Fresh(m.now,wallTime_[1],localization_config::SONAR_READING_STALE_S);
  wallStatus_=std::string("Wall support: X ")+(x?"active":"unavailable")+", Y "+(y?"active":"unavailable")+
      "; other motion uses encoder + gyro";
}
void GoalNavigationRoutine::Update(const NavigationInput& input) {
  NavigationInput in=input;
  positionCorrection_={0,0};
  const auto& m=in.motion; const double previousYaw=yaw_; Stop();
  if (m.stopRequested) { Abort(); return; }
  if (!IsRunning()) return;
  if (state_==State::Reference) {
    if (!HealthyMotion(in)) { status_=initializationStatus_="Reference waiting: "+MotionIssue(in); return; }
    double sums[2]={},weights[2]={};
    for (int sensor=0;sensor<4;++sensor) {
      const int axis=walls_[sensor].axis;
      if (axis<0 || initialKnown_[axis]) continue;
      if (!UsableRange(in,sensor)) continue;
      const double range=in.rangeMM[sensor];
      const auto ray=SensorRay({0,0,initialHeading_},sensor); // origin is the rotated mount offset
      const double component=ray.direction[axis];
      if (std::fabs(component)<1e-6) continue; // a parallel beam cannot observe this axis
      const double estimate=walls_[sensor].coordinate-ray.origin[axis]-range*component;
      const double limit=axis==0?localization_config::ARENA_WIDTH_MM:localization_config::ARENA_HEIGHT_MM;
      if (!std::isfinite(estimate) || estimate<0 || estimate>limit) continue;
      const double weight=component*component; // geometric weighting, no covariance model
      sums[axis]+=weight*estimate; weights[axis]+=weight;
    }
    for (int axis=0;axis<2;++axis) if (!initialKnown_[axis] && weights[axis]==0) {
      status_=initializationStatus_=std::string("Need start ")+(axis==0?"X":"Y")+" or a valid sensor reference for that axis"; return;
    }
    requestedPose_={initialKnown_[0]?initialCoordinate_[0]:sums[0]/weights[0],
        initialKnown_[1]?initialCoordinate_[1]:sums[1]/weights[1],initialHeading_};
    initializationStatus_="X="+std::to_string(static_cast<int>(std::round(requestedPose_.x_MM)))+
        (initialKnown_[0]?" given":" sensed")+", Y="+std::to_string(static_cast<int>(std::round(requestedPose_.y_MM)))+
        (initialKnown_[1]?" given":" sensed");
    state_=State::ReferenceReady; status_="Starting pose ready"; return;
  }
  if (state_==State::ReferenceReady) return;
  const auto issue=MotionIssue(in);
  if (!issue.empty()) { Fail(issue+" - stopped"); return; }
  in.motion.dt=std::min(in.motion.dt,localization_config::AUTO_MAX_CONTROL_STEP_S);
  // Fresh ranges retarget this frame's remaining segment through the driving
  // pose. Keep the global waypoints fixed and preserve the velocity ramp.
  CorrectFromWalls(in);
  controlPose_=m.pose;
  remaining_=std::hypot(goal_.x_MM-m.pose.x_MM,goal_.y_MM-m.pose.y_MM);
  if (state_==State::Driving) {
    DriveRoute(in,previousYaw);
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
      if (m.now-settled_>=navigation_config::FINAL_SETTLE_S) {
        // Only readings acquired during this stationary goal settle may be
        // committed to global odometry. Missing/invalid ranges do not gate it.
        double correction[2]={},pose[2]={m.pose.x_MM,m.pose.y_MM};
        for (int axis=0;axis<2;++axis) {
          bool usable=rangeSources_[axis]!=0;
          for (int sensor=0;sensor<4;++sensor) if ((rangeSources_[axis]&(1u<<sensor)) && !UsableRange(input,sensor)) usable=false;
          if (!usable || rangeStamp_[axis]<settled_ || !Fresh(m.now,rangeStamp_[axis],localization_config::SONAR_READING_STALE_S)) continue;
          correction[axis]=rangeTarget_[axis];
          pose[axis]+=correction[axis]-rangeOffset_[axis];
          rangeOffset_[axis]=rangeTarget_[axis]=0;
        }
        positionCorrection_={correction[0],correction[1]};
        for (auto& sample:history_) { sample.pose.x_MM+=correction[0]; sample.pose.y_MM+=correction[1]; }
        controlPose_={pose[0],pose[1],m.pose.headingRad};
        remaining_=std::hypot(goal_.x_MM-pose[0],goal_.y_MM-pose[1]);
        if (remaining_>navigation_config::POSITION_TOLERANCE_MM) {
          point_.Reset({goal_.x_MM,goal_.y_MM},m.now,navigation_config::SPEED_MMPS,navigation_config::ACCEL_MMPS2);
          state_=State::Driving; settled_=-1; status_="Final position correction"; return;
        }
        Stop(); state_=State::Complete; status_="Goal reached: within 10 mm / 1 deg";
      }
    } else settled_=-1;
  }
}
