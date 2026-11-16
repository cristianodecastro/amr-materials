#include "TeleopControl.h"
#include <limits>

namespace {
constexpr double Abs(double x) { return x<0?-x:x; }
constexpr bool Near(double a,double b) { return Abs(a-b)<1e-7; }
constexpr bool Stopped(const TeleopDriveController& c) {
  return c.GetForwardMMps()==0 && c.GetLeftMMps()==0 && c.GetYawRateRadps()==0;
}
constexpr TeleopDriveController Armed() {
  TeleopDriveController c; c.Update(true,true,0,0,0,0.02); return c;
}
constexpr TeleopDriveController Settled(double x,double y,double turn=0) {
  auto c=Armed(); for (int i=0;i<120;++i) c.Update(true,true,x,y,turn,0.02); return c;
}
static_assert(localization_config::WHEEL_TITAN_PORT[0]==0 &&
              localization_config::WHEEL_TITAN_PORT[1]==1 &&
              localization_config::WHEEL_TITAN_PORT[2]==2,"Only M0/M1/M2 are drive motors");
constexpr bool ControlsUseCorrectBodyDirections() {
  return Settled(0,-1).GetForwardMMps()>0 && Settled(0,1).GetForwardMMps()<0 &&
      Settled(-1,0).GetLeftMMps()>0 && Settled(1,0).GetLeftMMps()<0 &&
      Settled(0,0,-1).GetYawRateRadps()>0 && Settled(0,0,1).GetYawRateRadps()<0;
}
static_assert(ControlsUseCorrectBodyDirections(),"Up/left move forward/left; right stick left turns CCW");
constexpr bool MappingMatchesIndependentNewGeometry() {
  for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int r=-1;r<=1;++r) {
    const auto c=Settled(x,y,r);
    const double f=c.GetForwardMMps(),l=c.GetLeftMMps(),w=c.GetYawRateRadps();
    const double expected[3]={-0.8660254037844386*f+0.5*l+159.74*w,
        -l+159.74*w,0.8660254037844386*f+0.5*l+159.74*w};
    for (int i=0;i<3;++i) if (!Near(drive_kinematics::WheelSpeedMMps(i,f,l,w),expected[i])) return false;
  }
  return true;
}
static_assert(MappingMatchesIndependentNewGeometry(),"M0 front-left, M1 rear, M2 front-right; positive yaw CCW");
constexpr bool TranslationAndRotationRemainIndependent() {
  auto c=Settled(-1,-1,1);
  if (!(c.GetForwardMMps()>0 && c.GetLeftMMps()>0 && c.GetYawRateRadps()<0)) return false;
  c.Update(true,true,0,0,1,0.02);
  if (c.GetForwardMMps()!=0 || c.GetLeftMMps()!=0 || c.GetYawRateRadps()>=0) return false;
  c=Settled(-1,-1,1); c.Update(true,true,-1,-1,0,0.02);
  return c.GetForwardMMps()>0 && c.GetLeftMMps()>0 && c.GetYawRateRadps()==0;
}
static_assert(TranslationAndRotationRemainIndependent(),"Translation and yaw run together and release independently");
constexpr bool NeutralEnableDisableAndDisconnect() {
  TeleopDriveController c;
  c.Update(true,true,0,-1,0,0.02);
  if (c.IsArmed() || !Stopped(c)) return false;
  c.Update(true,true,0,0,0,0.02); c.Update(true,true,0,-1,1,0.02);
  if (!c.IsArmed() || c.GetForwardMMps()<=0 || c.GetYawRateRadps()>=0) return false;
  c.Update(false,true,0,-1,1,0.02);
  if (c.IsArmed() || !Stopped(c)) return false;
  c.Update(true,true,0,0,0,0.02); c.Update(true,true,0,-1,1,0.02);
  c.Update(true,false,0,-1,1,0.02); return !c.IsArmed() && Stopped(c);
}
static_assert(NeutralEnableDisableAndDisconnect(),"Enable requires neutral sticks; Disable/disconnect stops and disarms");
constexpr bool InvalidInputsStop() {
  const double bad[3]={std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),1.02};
  for (double v:bad) {
    auto c=Settled(0,-1,1); c.Update(true,true,v,0,0,0.02);
    if (c.IsArmed() || !Stopped(c)) return false;
    c=Settled(0,-1,1); c.Update(true,true,0,v,0,0.02);
    if (c.IsArmed() || !Stopped(c)) return false;
    c=Settled(0,-1,1); c.Update(true,true,0,0,v,0.02);
    if (c.IsArmed() || !Stopped(c)) return false;
  }
  return true;
}
static_assert(InvalidInputsStop(),"Invalid joystick values stop all drive requests");
constexpr bool ChassisSmoothingAndLimits() {
  auto c=Armed(); c.Update(true,true,-1,-1,1,0.02);
  if (!Near(std::hypot(c.GetForwardMMps(),c.GetLeftMMps()),8)) return false;
  if (!Near(c.GetYawRateRadps()*180/robot_geometry::PI,-0.9)) return false;
  const auto full=Settled(-1,-1,1);
  if (!Near(std::hypot(full.GetForwardMMps(),full.GetLeftMMps()),localization_config::TELEOP_SPEED_MMPS)) return false;
  c=Armed(); c.Update(true,true,0,-1,1,10);
  return Near(c.GetForwardMMps(),20) && Near(c.GetYawRateRadps()*180/robot_geometry::PI,-2.25);
}
static_assert(ChassisSmoothingAndLimits(),"Only chassis-level smoothing and operator speed limits remain");
constexpr bool WheelOutputIsProportionalAndNotEncoderCompensated() {
  WheelDriveController a,b;
  const double desired[3]={-120,-120,240}; double ca[3]={},cb[3]={};
  for (int tick=0;tick<35;++tick) {
    for (int i=0;i<3;++i) {
      ca[i]+=desired[i]*0.02/localization_config::ENCODER_MM_PER_COUNT;
      cb[i]+=desired[i]*0.03/localization_config::ENCODER_MM_PER_COUNT;
    }
    a.Update(desired,ca,0.02,0.9); b.Update(desired,cb,0.02,0.9);
    if (a.HasFault() || b.HasFault()) return false;
    for (int i=0;i<3;++i) if (!Near(a.Output(i),desired[i]/localization_config::MAX_WHEEL_SPEED_MMPS) || !Near(a.Output(i),b.Output(i))) return false;
  }
  return Abs(b.MeasuredMMps(0))>Abs(a.MeasuredMMps(0));
}
static_assert(WheelOutputIsProportionalAndNotEncoderCompensated(),"No wheel boost/trim/hold, even with different encoder speeds");
constexpr bool CommonSaturationAndFaultStops() {
  WheelDriveController c;
  const double desired[3]={2000,-1000,500},counts[3]={},zero[3]={};
  c.Update(desired,counts,0.02,0.25);
  if (!Near(c.Output(0),0.25) || !Near(c.Output(1),-0.125) || !Near(c.Output(2),0.0625)) return false;
  c.Update(zero,counts,0.02,0.25);
  if (c.Output(0)!=0 || c.Output(1)!=0 || c.Output(2)!=0) return false;
  const double request[3]={-120,-120,240};
  for (int tick=0;tick<75;++tick) c.Update(request,counts,0.02,0.9);
  return c.HasFault() && c.Output(0)==0 && c.Output(1)==0 && c.Output(2)==0;
}
static_assert(CommonSaturationAndFaultStops(),"Common electrical cap preserves ratios; neutral/fault stops all motors");
}
