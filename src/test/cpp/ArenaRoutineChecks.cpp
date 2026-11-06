#include "Autonomous.h"
#include "TeleopControl.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>

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
      0,0.02,true,true,false,false,0};
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
  REQUIRE_ARENA(c.GetState()==State::TravelMW);
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
void Simulate(bool disturbPose,bool motorDeadZone=false,bool motorLag=false,
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
      sawReferenceMW=true;
      if (!disturbedMiddlePose && disturbPose) {
        in.pose.x_MM+=24; in.pose.y_MM-=18; disturbedMiddlePose=true;
      }
    }
    if (state==State::TravelRDW && !leftMiddle) {
      REQUIRE_ARENA(std::hypot(in.pose.x_MM-at::MW.x_MM,in.pose.y_MM-at::MW.y_MM)<=at::POSITION_TOLERANCE_MM+1e-5);
      REQUIRE_ARENA(std::fabs(robot_geometry::Wrap(in.pose.headingRad-robot_geometry::Radians(at::MW.headingDeg)))<=at::HEADING_TOLERANCE_RAD+1e-5);
      leftMiddle=true;
    }
    if (state==State::ReferenceRDW) {
      sawReferenceRDW=true;
      if (!disturbedFinalPose && disturbPose) {
        // An external disturbance changes pose: finish at RDW before success.
        in.pose.x_MM-=20; in.pose.y_MM-=15; disturbedFinalPose=true;
      }
    }
    if (motorLag && state==State::FinishHeadingRDW && !disturbedFinalTurn) {
      in.pose.x_MM+=24; disturbedFinalTurn=true;
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
  REQUIRE_ARENA(c.GetStatus().find("encoder distance and gyro heading only")!=std::string::npos);
  REQUIRE_ARENA(c.GetLastReferenceReport().find("Position referencing disabled")!=std::string::npos);
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
void AverageColourAndSurpriseReferences() {
  auto calibration=DefaultColourCalibration();
  auto& references=calibration.references;
  auto classify=[&references](RgbColour rgb,double spread=6) {
    return ClassifyAverageColour(rgb,spread,references.data(),static_cast<int>(references.size()),
        camera_config::COLOUR_MAX_DISTANCE,camera_config::COLOUR_MIN_MARGIN,camera_config::COLOUR_MAX_STDDEV);
  };
  // Independent averages measured from supplied PNGs at (260,200,100,50).
  const RgbColour samples[3]={{166.1108,136.0396,33.9726},{17.7432,73.8264,35.2924},{105.0694,33.7314,7.8134}};
  const char* names[3]={"Yellow","Green","Red"};
  for (int i=0;i<3;++i) {
    REQUIRE_ARENA(classify(samples[i]).name==names[i]);
    const auto s=samples[i];
    REQUIRE_ARENA(classify({s.red+12,s.green+12,s.blue+12}).name==names[i]);
    REQUIRE_ARENA(classify(s,40).reference<0); // reject floor/gripper/edge contamination
  }
  for (auto rgb:{RgbColour{255,255,255},RgbColour{0,0,0},RgbColour{50,50,50},RgbColour{10,15,200},
      RgbColour{std::numeric_limits<double>::quiet_NaN(),0,0}})
    REQUIRE_ARENA(classify(rgb).reference<0);
  references[4]={"Competition violet",{110,60,180},true};
  REQUIRE_ARENA(classify({110,60,180}).name=="Competition violet"); // no enum/hue edits
  references[5]={"White",{220,220,220},true};
  REQUIRE_ARENA(classify({220,220,220}).name=="White"); // neutral surprise colours also supported
  references[4].enabled=false;
  REQUIRE_ARENA(classify({110,60,180}).reference<0);
  references[4]={"Almost Red",{106,34,8},true};
  REQUIRE_ARENA(classify(samples[2]).reference<0); // competing references too close
  const auto roi=ScaleColourRegion(calibration.roi,640,480),half=ScaleColourRegion(calibration.roi,320,240);
  REQUIRE_ARENA(roi.x==260 && roi.y==200 && roi.width==100 && roi.height==50);
  REQUIRE_ARENA(half.x==130 && half.y==100 && half.width==50 && half.height==25);
  REQUIRE_ARENA(!ValidColourRegion({630,200,100,50}));
  REQUIRE_ARENA(ScaleColourRegion({0,0,0,50},640,480).width==0);
}
void ColourCalibrationPersistsAndRejectsCorruption() {
  auto calibration=DefaultColourCalibration();
  calibration.references[4]={"Surprise gold",{160.5,115.25,40.125},true};
  std::string error;
  REQUIRE_ARENA(ValidateColourCalibration(calibration,error));
  const char* temporary=std::getenv("TEMP");
  const std::string path=std::string(temporary?temporary:"/tmp")+"/SkillsChallenge-colour-check-"+
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".txt";
  REQUIRE_ARENA(SaveColourCalibration(path,calibration,error));
  auto loaded=DefaultColourCalibration();
  REQUIRE_ARENA(LoadColourCalibration(path,loaded,error));
  REQUIRE_ARENA(loaded.references[4].enabled && loaded.references[4].name=="Surprise gold");
  REQUIRE_ARENA(Near(loaded.references[4].mean.green,115.25));
  auto invalid=calibration;
  invalid.references[4].name="rEd";
  REQUIRE_ARENA(!SaveColourCalibration(path,invalid,error));
  invalid=calibration; invalid.references[4].mean.red=256;
  REQUIRE_ARENA(!SaveColourCalibration(path,invalid,error));
  REQUIRE_ARENA(LoadColourCalibration(path,loaded,error) && loaded.references[4].name=="Surprise gold");
  const std::string corrupt=path+".bad";
  { std::ofstream file(corrupt); file<<"SkillsChallengeColourCalibration 1\n260 200 100 50\n8\n"; }
  REQUIRE_ARENA(!LoadColourCalibration(corrupt,loaded,error));
  REQUIRE_ARENA(loaded.references[4].name=="Surprise gold"); // malformed file cannot partially replace profiles
  std::remove(path.c_str()); std::remove(corrupt.c_str());
}
void SimpleTasksAndColourChecks() {
  for (auto task:{SimpleTask::Forward1000,SimpleTask::Backward1000,SimpleTask::Ccw90,
      SimpleTask::Cw90,SimpleTask::Ccw180,SimpleTask::Cw180})
    for (double heading:{0.0,170.0,-170.0,90.0}) SimulateSimpleTask(task,heading);
  SimulateSimpleTask(SimpleTask::Forward1000,170,true);
  SimpleTaskStopHealthAndPhoto(); AverageColourAndSurpriseReferences(); ColourCalibrationPersistsAndRejectsCorruption();
}
// Independent installed-wheel model for odometry tests. These expressions do
// not reuse production projections/inverse helpers, so a matching bug cannot hide.
void AddPhysicalCounts(double (&counts)[3],double forward,double left,double turn) {
  const double mmPerCount=102*robot_geometry::PI/1464;
  const double wheel[3]={std::sqrt(3.0)/2*forward+0.5*left-168*turn,
      -std::sqrt(3.0)/2*forward+0.5*left-168*turn,-left-168*turn};
  for (int i=0;i<3;++i) counts[i]+=wheel[i]/mmPerCount;
}
void KinematicsUsesWheelProjectionAndSingleCountConversion() {
  REQUIRE_ARENA(Near(localization_config::ENCODER_MM_PER_COUNT,0.218881455373059,1e-9));
  for (double forward:{-1000.0,0.0,1000.0}) for (double left:{-1000.0,0.0,1000.0}) {
    const double turn=0.3;
    double counts[3]={},wheel[3]; AddPhysicalCounts(counts,forward,left,turn);
    for (int i=0;i<3;++i) wheel[i]=counts[i]*localization_config::ENCODER_MM_PER_COUNT;
    double actualForward=0,actualLeft=0;
    REQUIRE_ARENA(drive_kinematics::BodyDisplacement(wheel,turn,actualForward,actualLeft));
    REQUIRE_ARENA(Near(actualForward,forward) && Near(actualLeft,left));
    for (int i=0;i<3;++i)
      REQUIRE_ARENA(Near(wheel[i],drive_kinematics::WheelSpeedMMps(i,
          drive_kinematics::LayoutVx(forward,left),drive_kinematics::LayoutVy(forward,left),turn)));
  }
}
void EncoderDistanceDoesNotDependOnTickRate() {
  for (double dt:{0.01,0.02,0.05}) for (double direction:{-1.0,1.0}) {
    drive_kinematics::EncoderGyroOdometry odometry;
    double counts[3]={}; odometry.Reset({0,0,0},counts,137);
    for (int tick=0;tick<int(std::round(5/dt));++tick) {
      AddPhysicalCounts(counts,direction*200*dt,0,0);
      REQUIRE_ARENA(odometry.Update(counts,137,0,dt));
    }
    REQUIRE_ARENA(Near(odometry.Pose().x_MM,direction*1000*autonomous_odometry_config::DISTANCE_SCALE));
    REQUIRE_ARENA(Near(odometry.Pose().y_MM,0) && Near(odometry.Pose().headingRad,0));
  }
  drive_kinematics::EncoderGyroOdometry odometry;
  double counts[3]={}; odometry.Reset({0,0,0},counts,-172);
  const double periods[4]={0.01,0.03,0.02,0.04};
  for (int tick=0;tick<200;++tick) {
    const double dt=periods[tick%4]; AddPhysicalCounts(counts,200*dt,0,0);
    REQUIRE_ARENA(odometry.Update(counts,-172,0,dt));
  }
  REQUIRE_ARENA(Near(odometry.Pose().x_MM,1000*autonomous_odometry_config::DISTANCE_SCALE));
  // CAN counts can arrive in bursts. Repeated cached values cannot create or
  // erase distance; all increments are consumed once on their arrival.
  counts[0]=counts[1]=counts[2]=0; odometry.Reset({0,0,0},counts,0);
  double truth[3]={};
  for (int tick=1;tick<=300;++tick) {
    AddPhysicalCounts(truth,200*0.02,0,0);
    if (tick%3==0) for (int i=0;i<3;++i) counts[i]=std::round(truth[i]);
    REQUIRE_ARENA(odometry.Update(counts,0,0,0.02));
  }
  REQUIRE_ARENA(Near(odometry.Pose().x_MM,1200*autonomous_odometry_config::DISTANCE_SCALE,0.3));
  // Even very small valid increments survive; no stationary velocity filter
  // discards low-speed motion from the encoder distance.
  counts[0]=counts[1]=counts[2]=0; odometry.Reset({0,0,0},counts,0);
  for (int tick=0;tick<100;++tick) {
    ++counts[0]; --counts[1]; REQUIRE_ARENA(odometry.Update(counts,0,0,0.1));
  }
  REQUIRE_ARENA(Near(odometry.Pose().x_MM,autonomous_odometry_config::DISTANCE_SCALE*
      200*localization_config::ENCODER_MM_PER_COUNT/std::sqrt(3.0)));
}
void GyroRelativeHeadingAndCurvedTravel() {
  for (double dt:{0.01,0.02,0.05}) {
    drive_kinematics::EncoderGyroOdometry odometry;
    double counts[3]={}; odometry.Reset({0,0,0},counts,170);
    double elapsed=0;
    for (int tick=0;tick<int(std::round(4/dt));++tick) {
      elapsed+=dt; AddPhysicalCounts(counts,200*dt,0,0.3*dt);
      const double yaw=robot_geometry::Wrap(robot_geometry::Radians(170)+0.3*elapsed)*180/robot_geometry::PI;
      REQUIRE_ARENA(odometry.Update(counts,yaw,0.3*180/robot_geometry::PI,dt));
    }
    REQUIRE_ARENA(Near(odometry.Pose().x_MM,autonomous_odometry_config::DISTANCE_SCALE*200/0.3*std::sin(1.2)));
    REQUIRE_ARENA(Near(odometry.Pose().y_MM,autonomous_odometry_config::DISTANCE_SCALE*200/0.3*(1-std::cos(1.2))));
    REQUIRE_ARENA(Near(odometry.Pose().headingRad,1.2));
  }
  drive_kinematics::EncoderGyroOdometry odometry;
  double counts[3]={}; odometry.Reset({950,900,0},counts,137);
  for (int tick=1;tick<=50;++tick) {
    AddPhysicalCounts(counts,0,0,robot_geometry::PI/100);
    REQUIRE_ARENA(odometry.Update(counts,137+90*tick/50.0,90,0.02));
  }
  REQUIRE_ARENA(Near(odometry.Pose().x_MM,950) && Near(odometry.Pose().y_MM,900));
  REQUIRE_ARENA(Near(odometry.Pose().headingRad,robot_geometry::PI/2));
  odometry.Reset({0,0,0},counts,137);
  REQUIRE_ARENA(odometry.Update(counts,137,30,0.02));
  REQUIRE_ARENA(Near(odometry.Pose().headingRad,0)); // no extra gyro-rate integration
  REQUIRE_ARENA(!odometry.Update(counts,137,0,0.2) && !odometry.Healthy());
  REQUIRE_ARENA(Near(odometry.Pose().x_MM,0));
  odometry.Reset({0,0,0},counts,137); counts[0]=std::numeric_limits<double>::quiet_NaN();
  REQUIRE_ARENA(!odometry.Update(counts,137,0,0.02));
}
void EncoderGyroOdometryChecks() {
  KinematicsUsesWheelProjectionAndSingleCountConversion();
  EncoderDistanceDoesNotDependOnTickRate(); GyroRelativeHeadingAndCurvedTravel();
  drive_kinematics::EncoderGyroOdometry autonomous;
  drive_kinematics::TeleopPoseTelemetry manual;
  drive_kinematics::TeleopPoseTelemetry manualWithoutAcceleration;
  double counts[3]={};
  autonomous.Reset({0,0,0},counts,0); manual.Reset({0,0,0},counts,0,0,0);
  manualWithoutAcceleration.Reset({0,0,0},counts,0,0,0);
  AddPhysicalCounts(counts,100*0.02,0,0);
  REQUIRE_ARENA(autonomous.Update(counts,0,0,0.02));
  REQUIRE_ARENA(manual.Update(counts,0,0,0.1,0,0.02));
  REQUIRE_ARENA(manualWithoutAcceleration.Update(counts,0,0,0,0,0.02));
  REQUIRE_ARENA(manual.Pose().x_MM>manualWithoutAcceleration.Pose().x_MM); // legacy display still uses acceleration
  REQUIRE_ARENA(Near(autonomous.Pose().x_MM,2*autonomous_odometry_config::DISTANCE_SCALE));
  const auto before=manual.Pose();
  REQUIRE_ARENA(autonomous.Update(counts,0,30,0.02));
  REQUIRE_ARENA(Near(manual.Pose().x_MM,before.x_MM));
  REQUIRE_ARENA(Near(autonomous.Pose().headingRad,0));
}
void AutonomousDistanceScaleDoesNotChangeTeleopOrHeading() {
  // This fixture represents nominal wheel travel BEFORE empirical calibration.
  // 1000 nominal mm must report 955.5 after the latest measured calibration;
  // the same scalar applies in both directions and on either body axis.
  for (double direction:{-1.0,1.0}) for (bool sideways:{false,true}) {
    drive_kinematics::EncoderGyroOdometry autonomous;
    drive_kinematics::TeleopPoseTelemetry manual;
    double counts[3]={};
    autonomous.Reset({0,0,0},counts,0); manual.Reset({0,0,0},counts,0,0,0);
    for (int tick=0;tick<250;++tick) {
      const double step=direction*4;
      AddPhysicalCounts(counts,sideways?0:step,sideways?step:0,0);
      REQUIRE_ARENA(autonomous.Update(counts,0,0,0.02));
      REQUIRE_ARENA(manual.Update(counts,0,0,0,0,0.02));
    }
    REQUIRE_ARENA(Near(sideways?autonomous.Pose().y_MM:autonomous.Pose().x_MM,direction*955.5));
    REQUIRE_ARENA(Near(sideways?manual.Pose().y_MM:manual.Pose().x_MM,direction*1000,1));
    REQUIRE_ARENA(Near(autonomous.Pose().headingRad,0));
    autonomous.Reset({0,0,0},counts,0);
    for (int tick=0;tick<250;++tick) {
      const double step=direction*4/0.9555;
      AddPhysicalCounts(counts,sideways?0:step,sideways?step:0,0);
      REQUIRE_ARENA(autonomous.Update(counts,0,0,0.02));
    }
    REQUIRE_ARENA(Near(sideways?autonomous.Pose().y_MM:autonomous.Pose().x_MM,direction*1000));
  }
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
    Simulate(true); Simulate(false); Simulate(true,true); Simulate(true,true,true);
    Simulate(true,true,true,0.25);
    SimpleTasksAndColourChecks();
    EncoderGyroOdometryChecks();
    AutonomousDistanceScaleDoesNotChangeTeleopOrHeading();
    std::cout<<"PASS: direct encoder/gyro odometry, forward/inverse wheel projections, timing jitter and sparse counts, exact turn integration; motion tasks, arena route and colour calibration regressions\n";
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
TEST(ArenaRoutine, DisturbedPoseSimulation) { EXPECT_NO_THROW(Simulate(true)); }
TEST(ArenaRoutine, NominalPoseSimulation) { EXPECT_NO_THROW(Simulate(false)); }
TEST(ArenaRoutine, MotorDeadZoneSimulation) { EXPECT_NO_THROW(Simulate(true,true)); }
TEST(ArenaRoutine, MotorLagAndPoseDriftSimulation) { EXPECT_NO_THROW(Simulate(true,true,true)); }
TEST(ArenaRoutine, SlowerMotorResponseSimulation) { EXPECT_NO_THROW(Simulate(true,true,true,0.25)); }
TEST(SimpleTasks, MotionAndColour) { EXPECT_NO_THROW(SimpleTasksAndColourChecks()); }
TEST(Odometry, EncoderGyroAndTiming) { EXPECT_NO_THROW(EncoderGyroOdometryChecks()); }
TEST(Odometry, AutonomousOnlyDistanceScale) { EXPECT_NO_THROW(AutonomousDistanceScaleDoesNotChangeTeleopOrHeading()); }
#endif
