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
  REQUIRE_ARENA(map.SegmentFree({at::SP.x_MM,at::SP.y_MM},{at::ME.x_MM,at::ME.y_MM}));
  REQUIRE_ARENA(map.SegmentFree({at::ME.x_MM,at::ME.y_MM},{at::RDE.x_MM,at::RDE.y_MM}));
  REQUIRE_ARENA(!map.SegmentFree({800,1500},{1200,1500}));
  const RobotPose me={at::ME.x_MM,at::ME.y_MM,robot_geometry::Radians(at::ME.headingDeg)};
  const auto left=map.SonarHit(0,me);
  REQUIRE_ARENA(left.trustedWall && left.surface==2); // south wall
  REQUIRE_ARENA(Near(left.range+localization_config::LEFT_SENSOR_OFFSET_MM,me.y_MM));
  const RobotPose rde={at::RDE.x_MM,at::RDE.y_MM,robot_geometry::Radians(at::RDE.headingDeg)};
  REQUIRE_ARENA(map.SonarHit(0,rde).surface==2);
  REQUIRE_ARENA(map.SonarHit(1,rde).surface==1); // east wall
  // A changed heading must see the real divider instead of the far west wall.
  const auto divider=map.SonarHit(1,{at::RDE.x_MM,at::RDE.y_MM,0});
  REQUIRE_ARENA(divider.trustedWall && divider.surface==5);
  REQUIRE_ARENA(Near(divider.range,at::RDE.x_MM-localization_config::REAR_SENSOR_OFFSET_MM-
      autonomous_config::DIVIDER_X_MAX_MM));
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
  REQUIRE_ARENA(c.GetState()==State::TravelME);
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
void Simulate(bool supplyReferences,bool repeatOldTimestamp=false) {
  ArenaTestController c;
  auto in=Input(); c.Enter(0);
  in.startRequested=true; c.Update(in); in.startRequested=false;
  bool sawTurnME=false,sawTurnRDE=false,sawReferenceME=false,sawReferenceRDE=false;
  bool disturbedFinalPose=false;
  for (int tick=1;tick<6000 && c.GetState()!=State::Complete;++tick) {
    const double forward=c.GetForwardMMps(),left=c.GetLeftMMps(),yaw=c.GetYawRadps();
    const double cosine=std::cos(in.pose.headingRad),sine=std::sin(in.pose.headingRad);
    in.pose.x_MM+=(cosine*forward-sine*left)*in.dt;
    in.pose.y_MM+=(sine*forward+cosine*left)*in.dt;
    in.pose.headingRad=robot_geometry::Wrap(in.pose.headingRad+yaw*in.dt);
    in.now=tick*in.dt;
    in.stationary=std::hypot(forward,left)<1e-6 && std::fabs(yaw)<1e-6;
    const auto state=c.GetState();
    if (state==State::TurnME) {
      sawTurnME=true;
      REQUIRE_ARENA(std::hypot(in.pose.x_MM-at::ME.x_MM,in.pose.y_MM-at::ME.y_MM)<=at::POSITION_TOLERANCE_MM+1e-5);
    }
    if (state==State::TurnRDE) sawTurnRDE=true;
    if (state==State::ReferenceME) { sawReferenceME=true; REQUIRE_ARENA(c.ReferenceMask()==at::ME_REFERENCE_MASK); }
    if (state==State::ReferenceRDE) {
      sawReferenceRDE=true; REQUIRE_ARENA(c.ReferenceMask()==at::RDE_REFERENCE_MASK);
      if (!disturbedFinalPose && supplyReferences && !repeatOldTimestamp) {
        // A wall correction changes the estimated position: finish at RDE
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
    REQUIRE_ARENA(c.GetState()!=State::Fault);
    REQUIRE_ARENA(!(std::hypot(c.GetForwardMMps(),c.GetLeftMMps())>1e-9 && std::fabs(c.GetYawRadps())>1e-9));
    REQUIRE_ARENA(std::hypot(c.GetForwardMMps(),c.GetLeftMMps())<=at::SPEED_MMPS+1e-6);
    REQUIRE_ARENA(std::fabs(c.GetYawRadps())<=at::MAX_YAW_RADPS+1e-6);
    if (c.GetState()==State::TravelME || c.GetState()==State::TravelRDE || c.GetState()==State::FinishRDE)
      REQUIRE_ARENA(c.GetYawRadps()==0);
    if (c.GetState()==State::TurnME || c.GetState()==State::TurnRDE || c.GetState()==State::FinishHeadingRDE)
      REQUIRE_ARENA(c.GetForwardMMps()==0 && c.GetLeftMMps()==0);
  }
  REQUIRE_ARENA(c.GetState()==State::Complete && Stopped(c));
  REQUIRE_ARENA(sawTurnME && sawTurnRDE && sawReferenceME && sawReferenceRDE);
  REQUIRE_ARENA(std::hypot(in.pose.x_MM-at::RDE.x_MM,in.pose.y_MM-at::RDE.y_MM)<=at::POSITION_TOLERANCE_MM+1e-5);
  REQUIRE_ARENA(std::fabs(robot_geometry::Wrap(in.pose.headingRad-robot_geometry::Radians(at::RDE.headingDeg)))<=at::HEADING_TOLERANCE_RAD+1e-5);
  const bool skipped=c.GetStatus().find("unavailable")!=std::string::npos;
  REQUIRE_ARENA(skipped==(!supplyReferences || repeatOldTimestamp));
  REQUIRE_ARENA(c.GetLastReferenceReport().find("ME:")!=std::string::npos);
  REQUIRE_ARENA(c.GetLastReferenceReport().find("RDE:")!=std::string::npos);
}
} // namespace

#ifdef ARENA_TEST_STANDALONE
int main() {
  try {
    GeometryAndWallAssociation(); StartStopAndHealth(); TimingAndBounds();
    Simulate(true); Simulate(false); Simulate(true,true);
    std::cout<<"PASS: geometry, start/stop, health/timing, XY-before-turn, wall references, missing/stale samples, final corrected pose\n";
  } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
#else
TEST(ArenaRoutine, GeometryAndWallAssociation) { EXPECT_NO_THROW(GeometryAndWallAssociation()); }
TEST(ArenaRoutine, StartStopAndHealth) { EXPECT_NO_THROW(StartStopAndHealth()); }
TEST(ArenaRoutine, TimingAndBounds) { EXPECT_NO_THROW(TimingAndBounds()); }
TEST(ArenaRoutine, ValidReferenceSimulation) { EXPECT_NO_THROW(Simulate(true)); }
TEST(ArenaRoutine, MissingReferenceSimulation) { EXPECT_NO_THROW(Simulate(false)); }
TEST(ArenaRoutine, StaleReferenceSimulation) { EXPECT_NO_THROW(Simulate(true,true)); }
#endif
