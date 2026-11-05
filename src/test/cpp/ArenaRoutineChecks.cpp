#include "Autonomous.h"

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
void Simulate(bool supplyReferences,bool repeatOldTimestamp=false) {
  ArenaTestController c;
  auto in=Input(); c.Enter(0);
  in.startRequested=true; c.Update(in); in.startRequested=false;
  bool sawTurnMW=false,sawTurnRDW=false,sawReferenceMW=false,sawReferenceRDW=false;
  bool sawNegativeMWYaw=false;
  bool disturbedFinalPose=false;
  bool disturbedTravelHeading=false;
  for (int tick=1;tick<6000 && c.GetState()!=State::Complete;++tick) {
    const double forward=c.GetForwardMMps(),left=c.GetLeftMMps(),yaw=c.GetYawRadps();
    const double cosine=std::cos(in.pose.headingRad),sine=std::sin(in.pose.headingRad);
    in.pose.x_MM+=(cosine*forward-sine*left)*in.dt;
    in.pose.y_MM+=(sine*forward+cosine*left)*in.dt;
    in.pose.headingRad=robot_geometry::Wrap(in.pose.headingRad+yaw*in.dt);
    in.now=tick*in.dt;
    in.stationary=std::hypot(forward,left)<1e-6 && std::fabs(yaw)<1e-6;
    if (!disturbedTravelHeading && tick>30 && c.GetState()==State::TravelMW) {
      in.pose.headingRad=robot_geometry::Wrap(in.pose.headingRad+robot_geometry::Radians(5));
      disturbedTravelHeading=true;
    }
    const auto state=c.GetState();
    if (state==State::TurnMW) {
      sawTurnMW=true;
      REQUIRE_ARENA(std::hypot(in.pose.x_MM-at::MW.x_MM,in.pose.y_MM-at::MW.y_MM)<=at::POSITION_TOLERANCE_MM+1e-5);
    }
    if (state==State::TurnRDW) sawTurnRDW=true;
    if (state==State::ReferenceMW) { sawReferenceMW=true; REQUIRE_ARENA(c.ReferenceMask()==at::MW_REFERENCE_MASK); }
    if (state==State::ReferenceRDW) {
      sawReferenceRDW=true; REQUIRE_ARENA(c.ReferenceMask()==at::RDW_REFERENCE_MASK);
      if (!disturbedFinalPose && supplyReferences && !repeatOldTimestamp) {
        // A wall correction changes the estimated position: finish at RDW
        // after that correction, rather than declaring success away from it.
        in.pose.x_MM-=20; in.pose.y_MM-=15; disturbedFinalPose=true;
      }
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
    if (c.GetState()==State::TravelMW || c.GetState()==State::TravelRDW || c.GetState()==State::FinishRDW)
      REQUIRE_ARENA(std::fabs(c.GetYawRadps())<=at::HOLD_HEADING_MAX_YAW_RADPS+1e-6);
    if (c.GetState()==State::TurnMW || c.GetState()==State::TurnRDW || c.GetState()==State::FinishHeadingRDW)
      REQUIRE_ARENA(c.GetForwardMMps()==0 && c.GetLeftMMps()==0);
  }
  REQUIRE_ARENA(c.GetState()==State::Complete && Stopped(c));
  REQUIRE_ARENA(sawTurnMW && sawTurnRDW && sawReferenceMW && sawReferenceRDW);
  REQUIRE_ARENA(sawNegativeMWYaw);
  REQUIRE_ARENA(std::hypot(in.pose.x_MM-at::RDW.x_MM,in.pose.y_MM-at::RDW.y_MM)<=at::POSITION_TOLERANCE_MM+1e-5);
  REQUIRE_ARENA(std::fabs(robot_geometry::Wrap(in.pose.headingRad-robot_geometry::Radians(at::RDW.headingDeg)))<=at::HEADING_TOLERANCE_RAD+1e-5);
  const bool skipped=c.GetStatus().find("unavailable")!=std::string::npos;
  REQUIRE_ARENA(skipped==(!supplyReferences || repeatOldTimestamp));
  REQUIRE_ARENA(c.GetLastReferenceReport().find("MW:")!=std::string::npos);
  REQUIRE_ARENA(c.GetLastReferenceReport().find("RDW:")!=std::string::npos);
}
} // namespace

#ifdef ARENA_TEST_STANDALONE
int main() {
  try {
    GeometryAndWallAssociation(); StartStopAndHealth(); TimingAndBounds(); HeadingHoldCorrectsDriftDuringXY();
    Simulate(true); Simulate(false); Simulate(true,true);
    std::cout<<"PASS: west-route geometry, start/stop, health/timing, gyro heading hold, planned turn after XY, valid/missing/stale references, 15 mm/1 degree completion\n";
  } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
#else
TEST(ArenaRoutine, GeometryAndWallAssociation) { EXPECT_NO_THROW(GeometryAndWallAssociation()); }
TEST(ArenaRoutine, StartStopAndHealth) { EXPECT_NO_THROW(StartStopAndHealth()); }
TEST(ArenaRoutine, TimingAndBounds) { EXPECT_NO_THROW(TimingAndBounds()); }
TEST(ArenaRoutine, HeadingHoldCorrectsDriftDuringXY) { EXPECT_NO_THROW(HeadingHoldCorrectsDriftDuringXY()); }
TEST(ArenaRoutine, ValidReferenceSimulation) { EXPECT_NO_THROW(Simulate(true)); }
TEST(ArenaRoutine, MissingReferenceSimulation) { EXPECT_NO_THROW(Simulate(false)); }
TEST(ArenaRoutine, StaleReferenceSimulation) { EXPECT_NO_THROW(Simulate(true,true)); }
#endif
