#include "Autonomous.h"
#include "TeleopControl.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#ifdef ARENA_TEST_STANDALONE
#include <iostream>
#else
#include <gtest/gtest.h>
#endif

namespace {
#define REQUIRE_ARENA(expr) do { if (!(expr)) throw std::runtime_error( \
    std::string("Line ")+std::to_string(__LINE__)+": "+#expr); } while (false)
using State=ArenaTestController::State;
namespace at=arena_test_config;
bool Near(double a,double b,double tolerance=1e-6) { return std::fabs(a-b)<=tolerance; }
bool Stopped(const ArenaTestController& c) {
  return c.GetForwardMMps()==0 && c.GetLeftMMps()==0 && c.GetYawRadps()==0;
}
ArenaTestInput Input() {
  return {{at::SP.x_MM,at::SP.y_MM,robot_geometry::Radians(at::SP.headingDeg)},
      0,0.02,true,true,false,false,{-1,-1,-1,-1}};
}
void GeometryAndWallAssociation() {
  ArenaMap map(false);
  REQUIRE_ARENA(map.SegmentFree({at::SP.x_MM,at::SP.y_MM},{at::MW.x_MM,at::MW.y_MM}));
  REQUIRE_ARENA(map.SegmentFree({at::MW.x_MM,at::MW.y_MM},{at::RDW.x_MM,at::RDW.y_MM}));
  REQUIRE_ARENA(!map.SegmentFree({800,1500},{1200,1500}));
  const RobotPose mw={at::MW.x_MM,at::MW.y_MM,robot_geometry::Radians(at::MW.headingDeg)};
  const auto right=map.SonarHit(2,mw);
  REQUIRE_ARENA(right.trustedWall && right.surface==0); // west wall
  REQUIRE_ARENA(Near(right.range+localization_config::RIGHT_SENSOR_OFFSET_MM,mw.x_MM));
  const RobotPose rdw={at::RDW.x_MM,at::RDW.y_MM,robot_geometry::Radians(at::RDW.headingDeg)};
  REQUIRE_ARENA(map.SonarHit(2,rdw).surface==0); // right sensor -> west wall
  REQUIRE_ARENA(map.SonarHit(1,rdw).surface==3); // rear sensor -> north wall
  // Turn to -90 at MW; RDW keeps that heading. Signed-angle wrapping still
  // selects the short turn when a future heading crosses the -180/180 boundary.
  REQUIRE_ARENA(Near(robot_geometry::Wrap(robot_geometry::Radians(at::MW.headingDeg-at::SP.headingDeg)),
      -robot_geometry::PI/2));
  REQUIRE_ARENA(Near(robot_geometry::Wrap(robot_geometry::Radians(at::RDW.headingDeg-at::MW.headingDeg)),0));
  REQUIRE_ARENA(Near(robot_geometry::Wrap(robot_geometry::Radians(-270)),robot_geometry::PI/2));
  // A changed heading must see the real divider instead of the far west wall.
  const auto divider=map.SonarHit(0,rdw);
  REQUIRE_ARENA(divider.trustedWall && divider.surface==5);
  REQUIRE_ARENA(Near(divider.range,autonomous_config::DIVIDER_X_MIN_MM-
      at::RDW.x_MM-localization_config::LEFT_SENSOR_OFFSET_MM));
  ArenaTestController controller;
  REQUIRE_ARENA(controller.ValidateGeometry().empty());
}
void StartStopAndHealth() {
  ArenaTestController c;
  auto in=Input();
  c.Enter(in.now);
  for (int i=0;i<25;++i) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::WaitingStart && Stopped(c));
  in.startRequested=true; in.healthy=false; c.Update(in);
  REQUIRE_ARENA(c.GetState()==State::WaitingStart && Stopped(c));
  in.healthy=true; c.Update(in); in.startRequested=false;
  REQUIRE_ARENA(c.GetState()==State::TravelMW);
  in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(std::hypot(c.GetForwardMMps(),c.GetLeftMMps())>0 && c.GetYawRadps()==0);
  in.stopRequested=true; c.Update(in);
  REQUIRE_ARENA(c.GetState()==State::Aborted && Stopped(c));
  in.stopRequested=false; in.startRequested=true; c.Update(in);
  REQUIRE_ARENA(c.GetState()==State::Aborted && Stopped(c));
  c.Enter(in.now); c.Update(in); in.startRequested=false;
  in.healthy=false; in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetState()==State::Fault && Stopped(c));
}
void TimingAndBounds() {
  for (double gap:{0.0,-0.02,0.2,std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::quiet_NaN()}) {
    ArenaTestController c;
    auto in=Input(); c.Enter(0); in.startRequested=true; c.Update(in);
    in.dt=gap; c.Update(in);
    REQUIRE_ARENA(c.GetState()==State::Fault && Stopped(c));
  }
  ArenaTestController c;
  auto in=Input(); c.Enter(0); in.startRequested=true; c.Update(in);
  in.startRequested=false; in.pose={1000,400,0}; in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetState()==State::Fault && Stopped(c));
  c.Enter(0); in=Input(); in.startRequested=true; c.Update(in);
  in.yawRateRadps=std::numeric_limits<double>::quiet_NaN();
  c.Update(in);
  REQUIRE_ARENA(c.GetState()==State::Fault && Stopped(c));
}
void HeadingHoldCorrectsDriftDuringXY() {
  ArenaTestController c;
  auto in=Input(); c.Enter(0); in.startRequested=true; c.Update(in);
  in.startRequested=false;
  in.pose.headingRad=robot_geometry::Radians(10);
  for (int tick=0;tick<30;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::TravelMW);
  REQUIRE_ARENA(std::hypot(c.GetForwardMMps(),c.GetLeftMMps())>0);
  REQUIRE_ARENA(c.GetYawRadps()<0);
  REQUIRE_ARENA(std::fabs(c.GetYawRadps())<=at::HOLD_HEADING_MAX_YAW_RADPS);
  REQUIRE_ARENA(Near(c.GetHeadingTargetDeg(),at::SP.headingDeg));
  in.pose.headingRad=robot_geometry::Radians(-10);
  for (int tick=0;tick<60;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::TravelMW && c.GetYawRadps()>0);
  REQUIRE_ARENA(Near(c.GetHeadingTargetDeg(),at::SP.headingDeg));
}
void ApproachRequestsRemainUsable() {
  ArenaTestController c;
  auto in=Input(); c.Enter(0); in.startRequested=true; c.Update(in); in.startRequested=false;
  in.pose.x_MM=at::MW.x_MM+at::POSITION_TOLERANCE_MM+1;
  in.pose.y_MM=at::MW.y_MM;
  for (int tick=0;tick<100;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::TravelMW);
  REQUIRE_ARENA(Near(std::hypot(c.GetForwardMMps(),c.GetLeftMMps()),at::MIN_APPROACH_MMPS));
  REQUIRE_ARENA(c.GetForwardMMps()<0);
  const double vx=drive_kinematics::LayoutVx(c.GetForwardMMps(),c.GetLeftMMps());
  const double vy=drive_kinematics::LayoutVy(c.GetForwardMMps(),c.GetLeftMMps());
  REQUIRE_ARENA(drive_kinematics::WheelSpeedMMps(0,vx,vy,0)<0);
  REQUIRE_ARENA(drive_kinematics::WheelSpeedMMps(1,vx,vy,0)>0);
  in.pose.x_MM=at::MW.x_MM;
  for (int tick=0;tick<25 && c.GetState()==State::TravelMW;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::TurnMW);
  in.pose.headingRad=robot_geometry::Radians(at::MW.headingDeg+2);
  for (int tick=0;tick<100;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::TurnMW);
  REQUIRE_ARENA(Near(c.GetYawRadps(),-at::MIN_TURN_YAW_RADPS));
  in.pose.headingRad=robot_geometry::Radians(at::MW.headingDeg); in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(Stopped(c));
}
void TurnDriftIsReacquiredBeforeReference() {
  ArenaTestController c;
  auto in=Input(); c.Enter(0); in.startRequested=true; c.Update(in); in.startRequested=false;
  in.pose.x_MM=at::MW.x_MM; in.pose.y_MM=at::MW.y_MM;
  for (int tick=0;tick<25 && c.GetState()==State::TravelMW;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::TurnMW);
  in.pose.headingRad=robot_geometry::Radians(at::MW.headingDeg);
  in.pose.x_MM+=25;
  for (int tick=0;tick<25 && c.GetState()==State::TurnMW;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::TravelMW && c.ReferenceMask()==0);
  REQUIRE_ARENA(Near(c.GetHeadingTargetDeg(),at::MW.headingDeg));
  in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetYawRadps()==0 && std::hypot(c.GetForwardMMps(),c.GetLeftMMps())>0);
  in.pose.x_MM=at::MW.x_MM;
  for (int tick=0;tick<60 && c.GetState()!=State::ReferenceMW;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::ReferenceMW);
}
void TurnBrakesBeforeTargetAndStopsBeforeReversing() {
  ArenaTestController c;
  auto in=Input(); c.Enter(0); in.startRequested=true; c.Update(in); in.startRequested=false;
  in.pose.x_MM=at::MW.x_MM; in.pose.y_MM=at::MW.y_MM;
  for (int tick=0;tick<25 && c.GetState()==State::TravelMW;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetState()==State::TurnMW);
  REQUIRE_ARENA(Near(c.GetHeadingTargetDeg(),-90));
  // Build up a clockwise request, then approach -90 with real angular speed.
  in.stationary=false;
  for (int tick=0;tick<45;++tick) { in.now+=in.dt; c.Update(in); }
  REQUIRE_ARENA(c.GetYawRadps()< -robot_geometry::Radians(30));
  in.pose.headingRad=robot_geometry::Radians(-82);
  in.yawRateRadps=robot_geometry::Radians(-30);
  in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetYawRadps()==0); // stop BEFORE crossing -90
  in.pose.headingRad=robot_geometry::Radians(-87);
  in.yawRateRadps=robot_geometry::Radians(-15);
  in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetYawRadps()==0); // stay off while coasting
  in.pose.headingRad=robot_geometry::Radians(-92);
  in.yawRateRadps=robot_geometry::Radians(-3);
  in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetYawRadps()==0); // do not reverse against ongoing motion
  in.yawRateRadps=0;
  in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetYawRadps()>0); // correct overshoot CCW once stopped
  // If the pose crosses the target between samples, an old clockwise request
  // must disappear immediately instead of slewing clockwise beyond the goal.
  in.pose.headingRad=robot_geometry::Radians(-88);
  in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetYawRadps()==0);
  in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetYawRadps()<0);
  in.pose.headingRad=robot_geometry::Radians(-92);
  in.now+=in.dt; c.Update(in);
  REQUIRE_ARENA(c.GetYawRadps()==0);
  REQUIRE_ARENA(Near(c.GetHeadingTargetDeg(),-90)); // absolute, never another -90 turn
}
void InstalledYawPolarityMatchesGyroFrame() {
  for (double command:{-0.5,0.5}) {
    double wheels[3];
    for (int i=0;i<3;++i) wheels[i]=drive_kinematics::WheelSpeedMMps(i,0,0,command);
    // Independent installation observation: positive motor/encoder rotation
    // decreases the already calibrated CCW-positive gyro heading.
    const double measured=-(wheels[0]+wheels[1]+wheels[2])/(3*168.0);
    REQUIRE_ARENA(Near(command,measured));
  }
}
void ReversedFeedbackStopsBeforeTimeout(bool duringTravel) {
  ArenaTestController c;
  auto in=Input(); c.Enter(0); in.startRequested=true; c.Update(in); in.startRequested=false;
  if (duringTravel) {
    in.pose.headingRad=robot_geometry::Radians(5);
  } else {
    in.pose.x_MM=at::MW.x_MM; in.pose.y_MM=at::MW.y_MM;
    for (int tick=0;tick<25 && c.GetState()==State::TravelMW;++tick) { in.now+=in.dt; c.Update(in); }
    REQUIRE_ARENA(c.GetState()==State::TurnMW);
  }
  const double started=in.now;
  double rate=0;
  for (int tick=0;tick<1900 && c.GetState()!=State::Fault;++tick) {
    // Deliberately reverse the motor response. Include lag and neutral braking
    // intervals so a succession of short wrong-way attempts also gets caught.
    rate+=in.dt/(0.08+in.dt)*(-c.GetYawRadps()-rate);
    in.pose.headingRad=robot_geometry::Wrap(in.pose.headingRad+rate*in.dt);
    in.yawRateRadps=rate;
    in.stationary=std::fabs(rate)<=robot_geometry::Radians(at::STATIONARY_YAW_DPS);
    in.now+=in.dt; c.Update(in);
  }
  REQUIRE_ARENA(c.GetState()==State::Fault && Stopped(c));
  REQUIRE_ARENA(c.GetStatus().find("Yaw direction mismatch: commanded CW")!=std::string::npos);
  REQUIRE_ARENA(Near(c.GetHeadingTargetDeg(),duringTravel?at::SP.headingDeg:at::MW.headingDeg));
  REQUIRE_ARENA(in.now-started<at::LEG_TIMEOUT_S);
  if (in.pose.headingRad>=robot_geometry::Radians(duringTravel?13:8))
    throw std::runtime_error("Reversed yaw exceeded excursion bound: heading="+
        std::to_string(in.pose.headingRad*180/robot_geometry::PI)+
        ", duringTravel="+std::to_string(duringTravel));
}
void Simulate(bool supplyReferences,bool repeatOldTimestamp=false,bool motorDeadZone=false,bool motorLag=false,
              double responseTime=0.08) {
  ArenaTestController c;
  auto in=Input(); c.Enter(0);
  in.startRequested=true; c.Update(in); in.startRequested=false;
  bool sawTurnMW=false,sawTurnRDW=false,sawReferenceMW=false,sawReferenceRDW=false;
  bool sawNegativeMWYaw=false;
  bool disturbedFinalPose=false;
  bool disturbedTravelHeading=false;
  bool disturbedMiddlePose=false,disturbedFinalTurn=false;
  bool leftMiddle=false;
  double actualForward=0,actualLeft=0,actualYaw=0;
  for (int tick=1;tick<6000 && c.GetState()!=State::Complete;++tick) {
    const double commandForward=c.GetForwardMMps(),commandLeft=c.GetLeftMMps(),commandYaw=c.GetYawRadps();
    const double vx=drive_kinematics::LayoutVx(commandForward,commandLeft);
    const double vy=drive_kinematics::LayoutVy(commandForward,commandLeft);
    double wheel[3];
    for (int i=0;i<3;++i) wheel[i]=drive_kinematics::WheelSpeedMMps(i,vx,vy,commandYaw);
    // Independent plant from the reported installation: positive layout
    // translation moves opposite the physical front/left axes, and positive
    // wheel yaw gives decreasing gyro heading. Do not use production inverse
    // calibration helpers here or a matching sign bug could hide.
    double forward=-(wheel[1]-wheel[0])/std::sqrt(3.0);
    double left=-(2*wheel[2]-wheel[0]-wheel[1])/3.0;
    double yaw=-(wheel[0]+wheel[1]+wheel[2])/(3*localization_config::WHEEL_CENTER_RADIUS_MM[0]);
    if (motorDeadZone && std::hypot(forward,left)<100) {
      forward=left=0;
      if (std::fabs(yaw)<25*robot_geometry::PI/180) yaw=0;
    }
    if (motorLag) {
      const double response=in.dt/(responseTime+in.dt);
      actualForward+=response*(forward-actualForward);
      actualLeft+=response*(left-actualLeft);
      actualYaw+=response*(yaw-actualYaw);
      forward=actualForward; left=actualLeft; yaw=actualYaw;
    }
    const double cosine=std::cos(in.pose.headingRad),sine=std::sin(in.pose.headingRad);
    in.pose.x_MM+=(cosine*forward-sine*left)*in.dt;
    in.pose.y_MM+=(sine*forward+cosine*left)*in.dt;
    in.pose.headingRad=robot_geometry::Wrap(in.pose.headingRad+yaw*in.dt);
    in.yawRateRadps=yaw;
    in.now=tick*in.dt;
    in.stationary=std::hypot(forward,left)<=at::STATIONARY_SPEED_MMPS &&
        std::fabs(yaw)*180/robot_geometry::PI<=at::STATIONARY_YAW_DPS;
    if (!disturbedTravelHeading && tick>30 && c.GetState()==State::TravelMW) {
      in.pose.headingRad=robot_geometry::Wrap(in.pose.headingRad+robot_geometry::Radians(5));
      disturbedTravelHeading=true;
    }
    const auto state=c.GetState();
    if (state==State::TurnMW) {
      sawTurnMW=true;
      REQUIRE_ARENA(std::hypot(in.pose.x_MM-at::MW.x_MM,in.pose.y_MM-at::MW.y_MM)<=at::POSITION_TOLERANCE_MM+1e-5);
    }
    if (sawTurnMW)
      REQUIRE_ARENA(in.pose.headingRad>=robot_geometry::Radians(at::MW.headingDeg-1));
    if (state==State::TurnRDW) sawTurnRDW=true;
    if (state==State::ReferenceMW) {
      sawReferenceMW=true; REQUIRE_ARENA(c.ReferenceMask()==at::MW_REFERENCE_MASK);
      if (!disturbedMiddlePose && supplyReferences && !repeatOldTimestamp) {
        in.pose.x_MM+=24; in.pose.y_MM-=18; disturbedMiddlePose=true;
      }
    }
    if (state==State::TravelRDW && !leftMiddle) {
      REQUIRE_ARENA(std::hypot(in.pose.x_MM-at::MW.x_MM,in.pose.y_MM-at::MW.y_MM)<=at::POSITION_TOLERANCE_MM+1e-5);
      REQUIRE_ARENA(std::fabs(robot_geometry::Wrap(in.pose.headingRad-robot_geometry::Radians(at::MW.headingDeg)))<=at::HEADING_TOLERANCE_RAD+1e-5);
      leftMiddle=true;
    }
    if (state==State::ReferenceRDW) {
      sawReferenceRDW=true; REQUIRE_ARENA(c.ReferenceMask()==at::RDW_REFERENCE_MASK);
      if (!disturbedFinalPose && supplyReferences && !repeatOldTimestamp) {
        // A wall correction changes the estimated position: finish at RDW
        // after that correction, rather than declaring success away from it.
        in.pose.x_MM-=20; in.pose.y_MM-=15; disturbedFinalPose=true;
      }
    }
    if (motorLag && state==State::FinishHeadingRDW && !disturbedFinalTurn) {
      in.pose.x_MM+=24; disturbedFinalTurn=true;
    }
    if (supplyReferences && c.ReferenceMask()!=0 && tick%20==0)
      for (int sensor=0;sensor<4;++sensor) if (c.ReferenceMask()&(1u<<sensor)) {
        REQUIRE_ARENA(c.ReferenceSurface(sensor)>=0);
        if (!repeatOldTimestamp || in.correctionTime[sensor]<0) in.correctionTime[sensor]=in.now;
      }
    c.Update(in);
    if (c.GetState()==State::TurnMW && c.GetYawRadps()<0) sawNegativeMWYaw=true;
    REQUIRE_ARENA(c.GetState()!=State::Fault);
    REQUIRE_ARENA(std::hypot(c.GetForwardMMps(),c.GetLeftMMps())<=at::SPEED_MMPS+1e-6);
    REQUIRE_ARENA(std::fabs(c.GetYawRadps())<=at::MAX_YAW_RADPS+1e-6);
    if (c.GetState()==State::TravelMW || c.GetState()==State::FinishMW || c.GetState()==State::TravelRDW || c.GetState()==State::FinishRDW)
      REQUIRE_ARENA(std::fabs(c.GetYawRadps())<=at::HOLD_HEADING_MAX_YAW_RADPS+1e-6);
    if (c.GetState()==State::TurnMW || c.GetState()==State::FinishHeadingMW || c.GetState()==State::TurnRDW || c.GetState()==State::FinishHeadingRDW)
      REQUIRE_ARENA(c.GetForwardMMps()==0 && c.GetLeftMMps()==0);
  }
  REQUIRE_ARENA(c.GetState()==State::Complete && Stopped(c));
  REQUIRE_ARENA(sawTurnMW && sawTurnRDW && sawReferenceMW && sawReferenceRDW);
  REQUIRE_ARENA(sawNegativeMWYaw);
  REQUIRE_ARENA(leftMiddle);
  REQUIRE_ARENA(std::hypot(in.pose.x_MM-at::RDW.x_MM,in.pose.y_MM-at::RDW.y_MM)<=at::POSITION_TOLERANCE_MM+1e-5);
  REQUIRE_ARENA(std::fabs(robot_geometry::Wrap(in.pose.headingRad-robot_geometry::Radians(at::RDW.headingDeg)))<=at::HEADING_TOLERANCE_RAD+1e-5);
  const bool skipped=c.GetStatus().find("unavailable")!=std::string::npos;
  REQUIRE_ARENA(skipped==(!supplyReferences || repeatOldTimestamp));
  REQUIRE_ARENA(c.GetLastReferenceReport().find("MW:")!=std::string::npos);
  REQUIRE_ARENA(c.GetLastReferenceReport().find("RDW:")!=std::string::npos);
}
void SimulateSimpleTask(SimpleTask task,double startDegrees,bool disturbHeading=false) {
  SimpleTaskController c;
  auto in=Input();
  in.pose={42,-300,robot_geometry::Radians(startDegrees)}; // no arena placement dependency
  const auto start=in.pose;
  REQUIRE_ARENA(c.Start(task,in));
  REQUIRE_ARENA(!c.Start(SimpleTask::Forward1000,in)); // cannot replace a running task
  const bool translation=task==SimpleTask::Forward1000 || task==SimpleTask::Backward1000;
  double actualForward=0,actualLeft=0,actualYaw=0,totalTurn=0;
  for (int tick=1;tick<2000 && c.IsRunning();++tick) {
    const double vx=drive_kinematics::LayoutVx(c.GetForwardMMps(),c.GetLeftMMps());
    const double vy=drive_kinematics::LayoutVy(c.GetForwardMMps(),c.GetLeftMMps());
    double wheel[3];
    for (int i=0;i<3;++i) wheel[i]=drive_kinematics::WheelSpeedMMps(i,vx,vy,c.GetYawRadps());
    double forward=-(wheel[1]-wheel[0])/std::sqrt(3.0),left=-(2*wheel[2]-wheel[0]-wheel[1])/3;
    double yaw=-(wheel[0]+wheel[1]+wheel[2])/(3*168.0);
    if (std::hypot(forward,left)<100) {
      forward=left=0;
      if (std::fabs(yaw)<robot_geometry::Radians(25)) yaw=0;
    }
    const double response=in.dt/(0.08+in.dt);
    actualForward+=response*(forward-actualForward); actualLeft+=response*(left-actualLeft);
    actualYaw+=response*(yaw-actualYaw);
    const double cosine=std::cos(in.pose.headingRad),sine=std::sin(in.pose.headingRad);
    in.pose.x_MM+=(cosine*actualForward-sine*actualLeft)*in.dt;
    in.pose.y_MM+=(sine*actualForward+cosine*actualLeft)*in.dt;
    totalTurn+=actualYaw*in.dt;
    in.pose.headingRad=robot_geometry::Wrap(in.pose.headingRad+actualYaw*in.dt);
    if (disturbHeading && tick==60 && translation) {
      in.pose.headingRad=robot_geometry::Wrap(in.pose.headingRad+robot_geometry::Radians(3));
      totalTurn+=robot_geometry::Radians(3);
    }
    in.yawRateRadps=actualYaw; in.now=tick*in.dt;
    in.stationary=std::hypot(actualForward,actualLeft)<=at::STATIONARY_SPEED_MMPS &&
        std::fabs(actualYaw)<=robot_geometry::Radians(at::STATIONARY_YAW_DPS);
    c.Update(in);
    REQUIRE_ARENA(std::hypot(c.GetForwardMMps(),c.GetLeftMMps())<=at::SPEED_MMPS+1e-6);
    REQUIRE_ARENA(std::fabs(c.GetYawRadps())<=at::MAX_YAW_RADPS+1e-6);
    if (!translation) REQUIRE_ARENA(c.GetForwardMMps()==0 && c.GetLeftMMps()==0);
  }
  if (c.GetState()!=SimpleTaskController::State::Complete)
    throw std::runtime_error(std::string(SimpleTaskName(task))+": "+c.GetStatus());
  const double wanted=task==SimpleTask::Forward1000?1000:(task==SimpleTask::Backward1000?-1000:0);
  REQUIRE_ARENA(std::hypot(in.pose.x_MM-start.x_MM-wanted*std::cos(start.headingRad),
      in.pose.y_MM-start.y_MM-wanted*std::sin(start.headingRad))<=at::POSITION_TOLERANCE_MM);
  const double wantedTurn=task==SimpleTask::Ccw90?robot_geometry::PI/2:(task==SimpleTask::Cw90?-robot_geometry::PI/2:
      (task==SimpleTask::Ccw180?robot_geometry::PI:(task==SimpleTask::Cw180?-robot_geometry::PI:0)));
  REQUIRE_ARENA(std::fabs(totalTurn-wantedTurn)<=at::HEADING_TOLERANCE_RAD);
  c.Update(in); REQUIRE_ARENA(!c.IsRunning() && c.GetYawRadps()==0 && c.GetForwardMMps()==0);
}
void SimpleTaskStopHealthAndPhoto() {
  SimpleTaskController c;
  auto in=Input();
  REQUIRE_ARENA(!c.Start(SimpleTask::None,in));
  in.stationary=false; REQUIRE_ARENA(!c.Start(SimpleTask::Forward1000,in));
  in.stationary=true; REQUIRE_ARENA(c.Start(SimpleTask::Forward1000,in));
  in.now+=in.dt; c.Update(in); REQUIRE_ARENA(c.GetForwardMMps()>0);
  in.stopRequested=true; c.Update(in); REQUIRE_ARENA(!c.IsRunning() && c.GetForwardMMps()==0);
  in.stopRequested=false; REQUIRE_ARENA(c.Start(SimpleTask::Backward1000,in));
  in.healthy=false; c.Update(in); REQUIRE_ARENA(c.GetState()==SimpleTaskController::State::Fault && c.GetForwardMMps()==0);
  REQUIRE_ARENA(c.Start(SimpleTask::Colour,in)); // vision needs no IMU
  c.Update(in); REQUIRE_ARENA(c.GetState()==SimpleTaskController::State::Photograph);
  c.CompletePhoto(true,"Detected Red"); REQUIRE_ARENA(c.GetState()==SimpleTaskController::State::Complete);
  REQUIRE_ARENA(c.Start(SimpleTask::Colour,in)); c.Abort();
  c.CompletePhoto(true,"Old photo"); REQUIRE_ARENA(c.GetState()==SimpleTaskController::State::Aborted);
  REQUIRE_ARENA(c.Start(SimpleTask::Colour,in)); c.CompletePhoto(false,"Camera timeout");
  REQUIRE_ARENA(c.GetState()==SimpleTaskController::State::Fault);
  in.healthy=true; REQUIRE_ARENA(c.Start(SimpleTask::Ccw90,in));
  in.dt=0.2; c.Update(in);
  REQUIRE_ARENA(c.GetState()==SimpleTaskController::State::Fault && c.GetYawRadps()==0);
}
void ColourVotingRejectsBackgroundAndAmbiguity() {
  for (int h:{0,179,30,60,120}) {
    ColourVotes votes;
    for (int i=0;i<100;++i) votes.AddHsv(h,i<70?0:220,200,80,50);
    const auto result=votes.Estimate(0.1,0.65);
    const auto expected=h==0 || h==179?PieceColour::Red:(h==30?PieceColour::Yellow:
        (h==60?PieceColour::Green:PieceColour::Blue));
    REQUIRE_ARENA(result.colour==expected && Near(result.coverage,0.3) && Near(result.dominance,1));
  }
  ColourVotes white,dark,mixed,purple,speck;
  for (int i=0;i<100;++i) {
    white.AddHsv(0,0,255,80,50); dark.AddHsv(0,255,10,80,50);
    mixed.AddHsv(i%2?0:120,220,200,80,50);
    purple.AddHsv(i<10?0:150,220,200,80,50);
    speck.AddHsv(0,i<5?220:0,200,80,50);
  }
  for (const auto& votes:{white,dark,mixed,purple,speck})
    REQUIRE_ARENA(votes.Estimate(0.1,0.65).colour==PieceColour::Unknown);
}
void SimpleTasksAndColourChecks() {
  for (auto task:{SimpleTask::Forward1000,SimpleTask::Backward1000,SimpleTask::Ccw90,
      SimpleTask::Cw90,SimpleTask::Ccw180,SimpleTask::Cw180})
    for (double heading:{0.0,170.0,-170.0,90.0}) SimulateSimpleTask(task,heading);
  SimulateSimpleTask(SimpleTask::Forward1000,170,true);
  SimpleTaskStopHealthAndPhoto(); ColourVotingRejectsBackgroundAndAmbiguity();
}
} // namespace

#ifdef ARENA_TEST_STANDALONE
int main() {
  try {
    GeometryAndWallAssociation(); StartStopAndHealth(); TimingAndBounds(); HeadingHoldCorrectsDriftDuringXY();
    ApproachRequestsRemainUsable(); TurnDriftIsReacquiredBeforeReference();
    TurnBrakesBeforeTargetAndStopsBeforeReversing();
    InstalledYawPolarityMatchesGyroFrame();
    ReversedFeedbackStopsBeforeTimeout(false); ReversedFeedbackStopsBeforeTimeout(true);
    Simulate(true); Simulate(false); Simulate(true,true); Simulate(true,false,true); Simulate(true,false,true,true);
    Simulate(true,false,true,true,0.25);
    SimpleTasksAndColourChecks();
    std::cout<<"PASS: relative forward/backward, CW/CCW 90/180 across signed yaw wrap, task cancellation/health/photo results, HSV background/ambiguity rejection; arena route and gyro braking regressions\n";
  } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
#else
TEST(ArenaRoutine, GeometryAndWallAssociation) { EXPECT_NO_THROW(GeometryAndWallAssociation()); }
TEST(ArenaRoutine, StartStopAndHealth) { EXPECT_NO_THROW(StartStopAndHealth()); }
TEST(ArenaRoutine, TimingAndBounds) { EXPECT_NO_THROW(TimingAndBounds()); }
TEST(ArenaRoutine, HeadingHoldCorrectsDriftDuringXY) { EXPECT_NO_THROW(HeadingHoldCorrectsDriftDuringXY()); }
TEST(ArenaRoutine, ApproachRequestsRemainUsable) { EXPECT_NO_THROW(ApproachRequestsRemainUsable()); }
TEST(ArenaRoutine, TurnDriftIsReacquiredBeforeReference) { EXPECT_NO_THROW(TurnDriftIsReacquiredBeforeReference()); }
TEST(ArenaRoutine, TurnBrakesBeforeTargetAndStopsBeforeReversing) { EXPECT_NO_THROW(TurnBrakesBeforeTargetAndStopsBeforeReversing()); }
TEST(ArenaRoutine, InstalledYawPolarityMatchesGyroFrame) { EXPECT_NO_THROW(InstalledYawPolarityMatchesGyroFrame()); }
TEST(ArenaRoutine, ReversedTurnStopsBeforeTimeout) { EXPECT_NO_THROW(ReversedFeedbackStopsBeforeTimeout(false)); }
TEST(ArenaRoutine, ReversedHeadingHoldStopsBeforeTimeout) { EXPECT_NO_THROW(ReversedFeedbackStopsBeforeTimeout(true)); }
TEST(ArenaRoutine, ValidReferenceSimulation) { EXPECT_NO_THROW(Simulate(true)); }
TEST(ArenaRoutine, MissingReferenceSimulation) { EXPECT_NO_THROW(Simulate(false)); }
TEST(ArenaRoutine, StaleReferenceSimulation) { EXPECT_NO_THROW(Simulate(true,true)); }
TEST(ArenaRoutine, MotorDeadZoneSimulation) { EXPECT_NO_THROW(Simulate(true,false,true)); }
TEST(ArenaRoutine, MotorLagAndPoseCorrectionSimulation) { EXPECT_NO_THROW(Simulate(true,false,true,true)); }
TEST(ArenaRoutine, SlowerMotorResponseSimulation) { EXPECT_NO_THROW(Simulate(true,false,true,true,0.25)); }
TEST(SimpleTasks, MotionAndColour) { EXPECT_NO_THROW(SimpleTasksAndColourChecks()); }
#endif
