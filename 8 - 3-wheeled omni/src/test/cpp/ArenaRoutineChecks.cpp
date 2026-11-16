#include "Autonomous.h"
#include "TeleopControl.h"
#include "Sensors.h"
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
#define REQUIRE_ARENA(expr) do { if (!(expr)) throw std::runtime_error(std::string("Line ")+std::to_string(__LINE__)+": "+#expr); } while (false)
bool Near(double a,double b,double tolerance=1e-6) { return std::fabs(a-b)<=tolerance; }
void SharpAnalogConversionAndValidity() {
  const auto oneVolt=SharpIrFromVoltage(1),twoVolts=SharpIrFromVoltage(2);
  REQUIRE_ARENA(oneVolt.valid && Near(oneVolt.distanceMM,554.52));
  REQUIRE_ARENA(twoVolts.valid && twoVolts.distanceMM>240 && twoVolts.distanceMM<242);
  REQUIRE_ARENA(oneVolt.distanceMM>twoVolts.distanceMM);
  for (double target:{101.0,200.0,400.0,600.0,799.0}) {
    const double voltage=std::pow(target/554.52,-1/1.2045);
    const auto reading=SharpIrFromVoltage(voltage);
    REQUIRE_ARENA(reading.valid && Near(reading.distanceMM,target));
    REQUIRE_ARENA(Near(reading.voltage,voltage));
  }
  for (double voltage:{-1.0,0.0,0.01,0.4,0.5,5.0,5.1,1e-300,
      std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    const auto reading=SharpIrFromVoltage(voltage);
    REQUIRE_ARENA(!reading.valid && reading.distanceMM==-1);
    REQUIRE_ARENA(std::isfinite(reading.voltage));
  }
  REQUIRE_ARENA(Near(SharpIrFromVoltage(0.4).voltage,0.4));
  // Sensor mounting offsets must not be doubled along with the measured gap.
  const double rawFor200MM=std::pow(200/277.26,-1/1.2045);
  RobotPose pose;
  REQUIRE_ARENA(TapeAlignmentRoutine::PoseFromWalls(SharpIrFromVoltage(rawFor200MM).distanceMM,100,pose));
  REQUIRE_ARENA(Near(pose.x_MM,460) && Near(pose.y_MM,273));
  // Nominal 60 mm is invalid before doubling but 120 mm is a valid final gap.
  const auto doubledIntoRange=SharpIrFromVoltage(std::pow(60/277.26,-1/1.2045));
  REQUIRE_ARENA(doubledIntoRange.valid && Near(doubledIntoRange.distanceMM,120));
  // Nominal 450 mm must instead reject after doubling to 900 mm.
  const auto doubledOutOfRange=SharpIrFromVoltage(std::pow(450/277.26,-1/1.2045));
  REQUIRE_ARENA(!doubledOutOfRange.valid && doubledOutOfRange.distanceMM==-1);
}
void YawRateUsesSensorSampleTime() {
  for (int sampleMS:{5,20,40}) for (int sign:{-1,1}) {
    YawRateEstimator rate;
    REQUIRE_ARENA(rate.Update(0,1000)==0);
    for (int sample=1;sample<=20;++sample) {
      const double yaw=sign*60.0*sample*sampleMS/1000;
      REQUIRE_ARENA(Near(rate.Update(yaw,1000+sample*sampleMS),sign*60));
      // Repeated robot-loop reads must retain the last measured rate.
      REQUIRE_ARENA(Near(rate.Update(yaw,1000+sample*sampleMS),sign*60));
    }
  }
  YawRateEstimator rate;
  rate.Update(179.4,1000);
  REQUIRE_ARENA(Near(rate.Update(-179.4,1020),60));
  REQUIRE_ARENA(Near(rate.Update(179.4,1040),-60));
  rate=YawRateEstimator{};
  rate.Update(0,std::numeric_limits<std::uint32_t>::max()-9);
  REQUIRE_ARENA(Near(rate.Update(1.2,10),60)); // timestamp rollover
  REQUIRE_ARENA(rate.Update(0,0)==0); // sensor restart, no spurious huge turn
  REQUIRE_ARENA(Near(rate.Update(-1.2,20),-60));
  REQUIRE_ARENA(rate.Update(-1.2,500)==0); // interrupted stream re-baselines
  REQUIRE_ARENA(Near(rate.Update(0,520),60));
  REQUIRE_ARENA(rate.Update(std::numeric_limits<double>::quiet_NaN(),540)==0);
  REQUIRE_ARENA(rate.Update(0,560)==0);
  // A 1.2 degree increment over 20 ms is 60 deg/s, NOT the legacy 1.2.
  rate=YawRateEstimator{}; rate.Update(0,1000);
  const double actualDps=rate.Update(1.2,1020);
  HeadingMotionControl corrected,legacyUnits;
  const double error=robot_geometry::Radians(5),old=robot_geometry::Radians(10);
  REQUIRE_ARENA(corrected.Request(error,old,robot_geometry::Radians(actualDps),0.02)==0);
  REQUIRE_ARENA(legacyUnits.Request(error,old,robot_geometry::Radians(1.2),0.02)>0);
}
void FixedCobraReferencesNeedNoCalibration() {
  const double white[4]={2.172852,1.005859,1.257324,2.868652};
  for (int boot=0;boot<3;++boot) {
    CobraCheck check; // exactly the active RobotSubsystem construction path
    LineReading reading; reading.communicationValid=true;
    for (int step=0;step<100;++step) {
      const double level=step%3==0?0:(step%3==1?0.6:1);
      for (int i=0;i<4;++i) reading.volts[i]=white[i]+level*(4.997559-white[i]);
      check.Update(reading,step*0.05); // no capture/file overrides
      REQUIRE_ARENA(check.Ready());
      for (int i=0;i<4;++i) {
        REQUIRE_ARENA(check.White(i)==white[i] && check.Black(i)==4.997559);
        REQUIRE_ARENA(Near(check.BlackLevel(i),level));
        REQUIRE_ARENA(check.IsBlack(i)==(level>=0.5));
      }
    }
  }
}
void AddPhysicalCounts(double (&counts)[3],double forward,double left,double turn) {
  const double mmPerCount=102*robot_geometry::PI/1464;
  const double wheel[3]={-std::sqrt(3.0)/2*forward+0.5*left+159.74*turn,
      -left+159.74*turn,std::sqrt(3.0)/2*forward+0.5*left+159.74*turn};
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
          forward,left,turn)));
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
    REQUIRE_ARENA(Near(odometry.Pose().y_MM,direction*1000));
    REQUIRE_ARENA(Near(odometry.Pose().x_MM,0) && Near(odometry.Pose().headingRad,0));
  }
  drive_kinematics::EncoderGyroOdometry odometry;
  double counts[3]={}; odometry.Reset({0,0,0},counts,-172);
  const double periods[4]={0.01,0.03,0.02,0.04};
  for (int tick=0;tick<200;++tick) {
    const double dt=periods[tick%4]; AddPhysicalCounts(counts,200*dt,0,0);
    REQUIRE_ARENA(odometry.Update(counts,-172,0,dt));
  }
  REQUIRE_ARENA(Near(odometry.Pose().y_MM,1000));
  // CAN counts can arrive in bursts. Repeated cached values cannot create or
  // erase distance; all increments are consumed once on their arrival.
  counts[0]=counts[1]=counts[2]=0; odometry.Reset({0,0,0},counts,0);
  double truth[3]={};
  for (int tick=1;tick<=300;++tick) {
    AddPhysicalCounts(truth,200*0.02,0,0);
    if (tick%3==0) for (int i=0;i<3;++i) counts[i]=std::round(truth[i]);
    REQUIRE_ARENA(odometry.Update(counts,0,0,0.02));
  }
  REQUIRE_ARENA(Near(odometry.Pose().y_MM,1200,0.3));
  // Even very small valid increments survive; no stationary velocity filter
  // discards low-speed motion from the encoder distance.
  counts[0]=counts[1]=counts[2]=0; odometry.Reset({0,0,0},counts,0);
  for (int tick=0;tick<100;++tick) {
    --counts[0]; ++counts[2]; REQUIRE_ARENA(odometry.Update(counts,0,0,0.1));
  }
  REQUIRE_ARENA(Near(odometry.Pose().y_MM,
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
    REQUIRE_ARENA(Near(odometry.Pose().y_MM,200/0.3*std::sin(1.2)));
    REQUIRE_ARENA(Near(odometry.Pose().x_MM,-200/0.3*(1-std::cos(1.2))));
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
  REQUIRE_ARENA(manual.Pose().y_MM>manualWithoutAcceleration.Pose().y_MM); // legacy display still uses acceleration
  REQUIRE_ARENA(Near(autonomous.Pose().y_MM,2));
  const auto before=manual.Pose();
  REQUIRE_ARENA(autonomous.Update(counts,0,30,0.02));
  REQUIRE_ARENA(Near(manual.Pose().x_MM,before.x_MM));
  REQUIRE_ARENA(Near(autonomous.Pose().headingRad,0));
}
TapeRoutineInput TapeInput() {
  TapeRoutineInput in;
  in.motion={{950,900,0},0,0.02,true,false,false,0}; // pre-reference pose is deliberately unrelated
  in.leftRangeMM=250; in.rearRangeMM=100;
  in.leftTime=in.rearTime=in.cobraTime=0;
  in.leftValid=in.rearValid=in.cobraCalibrated=true;
  in.line.communicationValid=true;
  for (int i=0;i<4;++i) { in.line.volts[i]=0.5; in.blackLevel[i]=0; }
  return in;
}
void FreshTapeTick(TapeRoutineInput& in) {
  in.motion.now+=in.motion.dt;
  in.leftTime=in.rearTime=in.cobraTime=in.motion.now;
}
void AnchorTapeRoutine(TapeAlignmentRoutine& routine,TapeRoutineInput& in) {
  routine.Enter(); in.motion.startRequested=true; routine.Update(in); in.motion.startRequested=false;
  for (int i=0;i<3;++i) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.NeedsPoseReset());
  in.motion.pose=routine.GetReferencePose();
  routine.ConfirmPoseReset(in.motion.now);
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint);
}
void ReachTapeWaypoint(TapeAlignmentRoutine& routine,TapeRoutineInput& in) {
  AnchorTapeRoutine(routine,in);
  const auto target=routine.GetTarget();
  in.motion.pose={target.x,target.y,0};
  for (int tick=0;tick<30 && routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint;++tick) {
    FreshTapeTick(in); routine.Update(in);
  }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::SearchTape);
}
void TapeReferenceNeedsOnlyFreshValidRanges() {
  RobotPose pose;
  REQUIRE_ARENA(TapeAlignmentRoutine::PoseFromWalls(150,100,pose));
  REQUIRE_ARENA(Near(pose.x_MM,210) && Near(pose.y_MM,273) && Near(pose.headingRad,0));
  REQUIRE_ARENA(!TapeAlignmentRoutine::PoseFromWalls(90,100,pose));
  REQUIRE_ARENA(!TapeAlignmentRoutine::PoseFromWalls(150,-1,pose));
  REQUIRE_ARENA(!TapeAlignmentRoutine::PoseFromWalls(std::numeric_limits<double>::quiet_NaN(),100,pose));
  TapeAlignmentRoutine routine; auto in=TapeInput(); routine.Enter();
   in.motion.yawRateRadps=robot_geometry::Radians(1.5);
  in.motion.startRequested=true; routine.Update(in); in.motion.startRequested=false;
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Reference);
  // A cached but fresh pair suffices; no stillness or post-Start count condition.
  in.motion.now+=in.motion.dt; routine.Update(in);
  REQUIRE_ARENA(routine.NeedsPoseReset());
  REQUIRE_ARENA(Near(routine.GetReferencePose().x_MM,310));
  REQUIRE_ARENA(Near(routine.GetReferencePose().y_MM,273));
  in.motion.pose=routine.GetReferencePose(); routine.ConfirmPoseReset(in.motion.now);
  FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetForwardMMps()>0 && routine.GetLeftMMps()>0);
  routine.Abort();
  REQUIRE_ARENA(!routine.IsRunning() && routine.GetForwardMMps()==0 && routine.GetYawRadps()==0);
  in=TapeInput(); routine.Enter();
  in.motion.startRequested=true; routine.Update(in); in.motion.startRequested=false;
  in.leftValid=false;
  for (int tick=0;tick<700;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Reference);
  REQUIRE_ARENA(routine.GetStatus()=="Waiting for left IR");
  REQUIRE_ARENA(routine.GetForwardMMps()==0 && routine.GetLeftMMps()==0 && routine.GetYawRadps()==0);
  in.leftValid=true; in.rearValid=false;
  for (int tick=0;tick<700;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Reference);
  REQUIRE_ARENA(routine.GetStatus()=="Waiting for rear sonar");
  in.rearValid=true; FreshTapeTick(in); in.rearTime=in.motion.now-1; routine.Update(in);
  REQUIRE_ARENA(!routine.NeedsPoseReset());
  FreshTapeTick(in); in.leftTime=in.motion.now-1; routine.Update(in);
  REQUIRE_ARENA(!routine.NeedsPoseReset());
  FreshTapeTick(in);
  in.rearTime=in.motion.now-0.25; // independently fresh; no pairing restriction
  routine.Update(in);
  REQUIRE_ARENA(routine.NeedsPoseReset());
}
void TapeEqualityRequiresActualTapeAndStableStop() {
  TapeAlignmentRoutine routine; auto in=TapeInput(); ReachTapeWaypoint(routine,in);
  FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetLeftMMps()<0 && routine.GetForwardMMps()==0); // robot-right
  REQUIRE_ARENA(std::fabs(routine.GetLeftMMps())<=tape_test_config::SCAN_ACCEL_MMPS2*in.motion.dt+1e-6);
  REQUIRE_ARENA(routine.GetState()!=TapeAlignmentRoutine::State::VerifyTape); // white == white is not tape
  for (int i=0;i<4;++i) { in.blackLevel[i]=1; in.line.volts[i]=4.5; }
  FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetState()!=TapeAlignmentRoutine::State::VerifyTape); // all-black is not a centred stripe
  in.blackLevel[0]=in.blackLevel[3]=0; in.line.volts[0]=in.line.volts[3]=0.5;
  FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::VerifyTape);
  REQUIRE_ARENA(routine.GetForwardMMps()==0 && routine.GetLeftMMps()==0);
  for (int tick=0;tick<6;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetState()!=TapeAlignmentRoutine::State::Complete);
  in.blackLevel[1]=0.5; in.line.volts[1]=2.5; // disturbed while settling: resume correction
  FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::CenterTape && routine.GetLeftMMps()<0);
  in.blackLevel[1]=1; in.line.volts[1]=4.5;
  for (int tick=0;tick<22;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Complete && routine.GetLeftMMps()==0);
}
void TapeSensorWaitsRecoverAndGyroNoiseDoesNotBlockCompletion() {
  TapeAlignmentRoutine routine; auto in=TapeInput();
  in.cobraCalibrated=false; in.line.communicationValid=false;

  ReachTapeWaypoint(routine,in); // Cobra is not required for reference/XY travel
  for (int tick=0;tick<1100;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::SearchTape);
  REQUIRE_ARENA(routine.GetStatus()=="Waiting for Cobra reading");
  REQUIRE_ARENA(routine.GetForwardMMps()==0 && routine.GetLeftMMps()==0 && routine.GetYawRadps()==0);
  in.cobraCalibrated=true; in.line.communicationValid=true;
  FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::SearchTape && routine.GetLeftMMps()<0);
  in.blackLevel[0]=in.blackLevel[3]=0; in.blackLevel[1]=in.blackLevel[2]=1;
  in.line.volts[0]=in.line.volts[3]=0.5; in.line.volts[1]=in.line.volts[2]=4.5;
  in.motion.yawRateRadps=robot_geometry::Radians(1.5); // noisy stationary indication
  in.bodyForwardMMps=30; // real translation still prevents completion
  for (int tick=0;tick<25;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::VerifyTape);
  in.bodyForwardMMps=0;
  for (int tick=0;tick<25;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Complete);
  in=TapeInput(); routine.Enter(); in.rearValid=false;
  in.motion.startRequested=true; routine.Update(in); in.motion.startRequested=false;
  FreshTapeTick(in); routine.Update(in);
  in.motion.stopRequested=true; FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Aborted);
  REQUIRE_ARENA(routine.GetForwardMMps()==0 && routine.GetLeftMMps()==0);
}
void TapeLimitsAndCenterDirection() {
  TapeAlignmentRoutine routine; auto in=TapeInput(); ReachTapeWaypoint(routine,in);
  in.blackLevel[1]=0.2; in.blackLevel[2]=0.9; in.line.volts[1]=1.3; in.line.volts[2]=4.1;
  FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetLeftMMps()<0); // more tape on CH2 -> right
  in.blackLevel[1]=0.9; in.blackLevel[2]=0.2; in.line.volts[1]=4.1; in.line.volts[2]=1.3;
  for (int tick=0;tick<8;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetLeftMMps()>0); // more tape on CH1 -> left, after ramp reversal
  in.motion.pose.x_MM+=tape_test_config::SCAN_MAX_TRAVEL_MM+1;
  FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Fault && routine.GetLeftMMps()==0);
  in=TapeInput(); ReachTapeWaypoint(routine,in);
  in.motion.now+=tape_test_config::SCAN_TIMEOUT_S+1;
  FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Fault);
  in=TapeInput(); AnchorTapeRoutine(routine,in);
  in.motion.stopRequested=true; FreshTapeTick(in); routine.Update(in);
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Aborted && routine.GetForwardMMps()==0);
}
void TapeApproachSelectionAndSearchDirections() {
  for (auto approach:{TapeApproach::FromLeft,TapeApproach::FromRight}) {
    TapeAlignmentRoutine routine; auto in=TapeInput(); in.approach=approach;
    ReachTapeWaypoint(routine,in);
    const auto selected=routine.GetTarget();
    REQUIRE_ARENA(selected.x==(approach==TapeApproach::FromLeft?220:580) && selected.y==500);
    FreshTapeTick(in); routine.Update(in);
    REQUIRE_ARENA(routine.GetApproach()==approach);
    REQUIRE_ARENA(routine.GetLeftMMps()*(approach==TapeApproach::FromLeft?-1:1)>0);
    // A second start cannot alter an active route's target or scan direction.
    in.approach=approach==TapeApproach::FromLeft?TapeApproach::FromRight:TapeApproach::FromLeft;
    in.motion.startRequested=true; FreshTapeTick(in); routine.Update(in);
    in.motion.startRequested=false;
    REQUIRE_ARENA(routine.GetApproach()==approach && routine.GetTarget().x==selected.x);
    // Centering is a physical array error, so its sign NEVER flips with side.
    in.blackLevel[1]=0.9; in.blackLevel[2]=0.2;
    in.line.volts[1]=4.1; in.line.volts[2]=1.3;
    for (int tick=0;tick<8;++tick) { FreshTapeTick(in); routine.Update(in); }
    REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::CenterTape && routine.GetLeftMMps()>0);
    for (int i=0;i<4;++i) { in.blackLevel[i]=0; in.line.volts[i]=0.5; }
    for (int tick=0;tick<50;++tick) { FreshTapeTick(in); routine.Update(in); }
    REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::SearchTape);
    REQUIRE_ARENA(routine.GetLeftMMps()*(approach==TapeApproach::FromLeft?-1:1)>0);
    // Stop takes priority even when a new start arrives in the same cycle.
    in.motion.startRequested=in.motion.stopRequested=true; FreshTapeTick(in); routine.Update(in);
    REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Aborted);
    REQUIRE_ARENA(routine.GetForwardMMps()==0 && routine.GetLeftMMps()==0 && routine.GetYawRadps()==0);
  }
}
void SimulateCornerToWaypointToTape(TapeApproach approach=TapeApproach::FromLeft) {
  TapeAlignmentRoutine routine; auto in=TapeInput();
  in.approach=approach;
  routine.Enter(); in.motion.startRequested=true; routine.Update(in); in.motion.startRequested=false;
  bool drove=false,scanned=false,matched=false;
  const double tapeX=330;
  double previousYaw=0;
  for (int tick=1;tick<=2500 && routine.GetState()!=TapeAlignmentRoutine::State::Complete;++tick) {
    in.motion.now=tick*0.02;
    // Sensor streams have independent real rates: ~20 Hz I2C/IR and 5 Hz sonar.
    if (tick%3==0) {
      in.leftTime=in.cobraTime=in.motion.now;
      for (int ch=0;ch<4;++ch) {
        const double sensorX=in.motion.pose.x_MM+(ch-1.5)*9;
        const double level=std::max(0.0,std::min(1.0,(12-std::fabs(sensorX-tapeX))/4));
        in.blackLevel[ch]=level; in.line.volts[ch]=0.5+4*level;
      }
    }
    if (tick%10==0) in.rearTime=in.motion.now;

    in.motion.yawRateRadps=previousYaw;
    routine.Update(in);
    if (routine.NeedsPoseReset()) {
      in.motion.pose=routine.GetReferencePose(); routine.ConfirmPoseReset(in.motion.now);
    }
    if (routine.GetState()==TapeAlignmentRoutine::State::Fault)
      throw std::runtime_error("Tape routine simulation: "+routine.GetStatus());
    drove=drove || routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint;
    scanned=scanned || routine.GetState()==TapeAlignmentRoutine::State::SearchTape;
    matched=matched || routine.GetState()==TapeAlignmentRoutine::State::VerifyTape;
    const double forward=routine.GetForwardMMps(),left=routine.GetLeftMMps(),yaw=routine.GetYawRadps();
    const double theta=robot_geometry::ArenaAngle(in.motion.pose.headingRad+yaw*0.01);
    in.motion.pose.x_MM+=(std::cos(theta)*forward-std::sin(theta)*left)*0.02;
    in.motion.pose.y_MM+=(std::sin(theta)*forward+std::cos(theta)*left)*0.02;
    in.motion.pose.headingRad=robot_geometry::Wrap(in.motion.pose.headingRad+yaw*0.02);
    previousYaw=yaw;
  }
  REQUIRE_ARENA(drove && scanned && matched);
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Complete);
  REQUIRE_ARENA(std::fabs(in.motion.pose.x_MM-tapeX)<10 && std::fabs(in.motion.pose.y_MM-500)<=10);
  REQUIRE_ARENA(Near(in.motion.pose.headingRad,0));
  REQUIRE_ARENA(routine.GetForwardMMps()==0 && routine.GetLeftMMps()==0 && routine.GetYawRadps()==0);
}
void TapeTranslationAndHeadingCorrectTogether() {
  for (auto approach:{TapeApproach::FromLeft,TapeApproach::FromRight}) for (int sign:{-1,1}) {
    TapeAlignmentRoutine routine; auto in=TapeInput(); in.approach=approach;
    AnchorTapeRoutine(routine,in);
    in.motion.pose.headingRad+=robot_geometry::Radians(sign*5);
    FreshTapeTick(in); routine.Update(in);
    REQUIRE_ARENA(std::hypot(routine.GetForwardMMps(),routine.GetLeftMMps())>0);
    REQUIRE_ARENA(sign*routine.GetYawRadps()<0);

    // After braking, a remaining XY error must not wait for heading to settle.
    const auto target=routine.GetTarget();
    in.motion.pose={target.x+20,target.y,robot_geometry::Radians(sign*5)};
    bool correctedTogether=false;
    for (int tick=0;tick<20;++tick) {
      FreshTapeTick(in); routine.Update(in);
      if (std::hypot(routine.GetForwardMMps(),routine.GetLeftMMps())>0 &&
          sign*routine.GetYawRadps()<0) correctedTogether=true;
    }
    REQUIRE_ARENA(correctedTogether);

    // Reaching XY alone does not complete the waypoint with an incorrect yaw.
    in.motion.pose.x_MM=target.x; in.motion.pose.y_MM=target.y;
    for (int tick=0;tick<30;++tick) { FreshTapeTick(in); routine.Update(in); }
    REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint);
    REQUIRE_ARENA(routine.GetForwardMMps()==0 && routine.GetLeftMMps()==0 && sign*routine.GetYawRadps()<0);
    in.motion.pose.headingRad=0;
    for (int tick=0;tick<30 && routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint;++tick) {
      FreshTapeTick(in); routine.Update(in);
    }
    REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::SearchTape);

    in.motion.pose.headingRad+=robot_geometry::Radians(sign*5);
    FreshTapeTick(in); routine.Update(in);
    REQUIRE_ARENA(routine.GetLeftMMps()*(approach==TapeApproach::FromLeft?-1:1)>0);
    REQUIRE_ARENA(sign*routine.GetYawRadps()<0);
    in.blackLevel[1]=0.9; in.blackLevel[2]=0.2;
    in.line.volts[1]=4.1; in.line.volts[2]=1.3;
    for (int tick=0;tick<10;++tick) { FreshTapeTick(in); routine.Update(in); }
    REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::CenterTape);
    REQUIRE_ARENA(routine.GetLeftMMps()>0 && sign*routine.GetYawRadps()<0);
  }
}
void TapeRoutineChecks() {
  TapeReferenceNeedsOnlyFreshValidRanges(); TapeEqualityRequiresActualTapeAndStableStop();
  TapeSensorWaitsRecoverAndGyroNoiseDoesNotBlockCompletion();
  TapeApproachSelectionAndSearchDirections();
  TapeTranslationAndHeadingCorrectTogether();
  TapeLimitsAndCenterDirection(); SimulateCornerToWaypointToTape();
  SimulateCornerToWaypointToTape(TapeApproach::FromRight);
}
void WaypointWithCoastingAndMotorDeadband(double lag,double deadband,bool screenshotStart) {
  TapeAlignmentRoutine routine; auto in=TapeInput(); AnchorTapeRoutine(routine,in);
  RobotPose physical=in.motion.pose;
  if (screenshotStart) physical={226.037002,486.136416,0};
  in.motion.pose=physical;
  const double startDistance=std::hypot(220-physical.x_MM,500-physical.y_MM);
  const double travelX=(220-physical.x_MM)/startDistance,travelY=(500-physical.y_MM)/startDistance;
  double maximumOvershoot=0;
  double forward=0,left=0;
  int tick=0;
  for (;tick<1500 && routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint;++tick) {
    FreshTapeTick(in);
    if (tick%3==0) { in.motion.pose=physical; in.bodyForwardMMps=forward; in.bodyLeftMMps=left; }

    routine.Update(in);
    if (routine.GetState()==TapeAlignmentRoutine::State::Fault)
      throw std::runtime_error("Coast/deadband waypoint failure: "+routine.GetStatus()+" x="+
          std::to_string(physical.x_MM)+" y="+std::to_string(physical.y_MM));
    double targetForward=routine.GetForwardMMps(),targetLeft=routine.GetLeftMMps();
    if (std::hypot(targetForward,targetLeft)<deadband) targetForward=targetLeft=0;
    const double gain=lag==0?1:0.02/(lag+0.02);
    forward+=gain*(targetForward-forward); left+=gain*(targetLeft-left);
    physical.x_MM-=left*0.02; physical.y_MM+=forward*0.02;
    maximumOvershoot=std::max(maximumOvershoot,(physical.x_MM-220)*travelX+(physical.y_MM-500)*travelY);
  }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::SearchTape);
  REQUIRE_ARENA(std::hypot(physical.x_MM-220,physical.y_MM-500)<=10);
  REQUIRE_ARENA(tick<1000); // finite completion, rather than repeated full-speed crossings
  REQUIRE_ARENA(maximumOvershoot<=15); // stop prediction also bounds intermediate overshoot in this model
}
void WaypointSettlingDoesNotExpireProgressWatchdog() {
  TapeAlignmentRoutine routine; auto in=TapeInput(); AnchorTapeRoutine(routine,in);
  in.motion.pose={220,500,0};
   in.bodyForwardMMps=30; // inside tolerance but still coasting
  for (int tick=0;tick<130;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint);
  in.motion.pose={226.037002,486.136416,0}; // drifts outside the 10 mm tolerance
   in.bodyForwardMMps=0;
  for (int tick=0;tick<18;++tick) { FreshTapeTick(in); routine.Update(in); }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint);
  REQUIRE_ARENA(routine.GetForwardMMps()>0); // correction was not killed by time spent stopped
  in=TapeInput(); AnchorTapeRoutine(routine,in);
  in.motion.pose={226.037002,486.136416,0};
  for (int tick=0;tick<600 && routine.GetState()!=TapeAlignmentRoutine::State::Fault;++tick) {
    FreshTapeTick(in); routine.Update(in); // genuine stall: commands have no encoder response
  }
  REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::Fault && routine.GetForwardMMps()==0);
  REQUIRE_ARENA(routine.GetStatus().find("Waypoint correction stalled")!=std::string::npos);
}

void ProportionalTitanOutputAndQuantization() {
  for (double cap:{0.01,0.15,0.159,0.25,0.9,1.0}) for (int n=-2000;n<=2000;++n) {
    const double demand=n/2000.0;
    const int percent=motor_output::TitanPercent(demand,cap);
    const double argument=motor_output::TitanSetArgument(percent);
    REQUIRE_ARENA(static_cast<int>(std::fabs(argument*100))==std::abs(percent));
    REQUIRE_ARENA(std::abs(percent)<=static_cast<int>(std::floor(cap*100+1e-9)));
    REQUIRE_ARENA(std::fabs(percent/100.0-std::max(-cap,std::min(cap,demand)))<=0.010001);
  }
  REQUIRE_ARENA(motor_output::TitanPercent(0.04,0.9)==4); // no 15% boost
  REQUIRE_ARENA(motor_output::TitanPercent(0,0.9)==0);
  REQUIRE_ARENA(motor_output::TitanPercent(std::numeric_limits<double>::quiet_NaN(),0.9)==0);
  REQUIRE_ARENA(motor_output::TitanPercent(0.3,0)==0);
  WheelDriveController drive;
  const double demand[3]={-120,-120,240},counts[3]={};
  drive.Update(demand,counts,0.02,0.9);
  for (int i=0;i<3;++i) REQUIRE_ARENA(Near(drive.Output(i),demand[i]/localization_config::MAX_WHEEL_SPEED_MMPS));
  REQUIRE_ARENA(localization_config::WHEEL_TITAN_PORT[0]==0 && localization_config::WHEEL_TITAN_PORT[1]==1 && localization_config::WHEEL_TITAN_PORT[2]==2);
  // Verify actual vectors independently of production's trigonometry/inverse.
  for (double f:{-100.0,0.0,100.0}) for (double l:{-100.0,0.0,100.0}) for (double yaw:{-0.5,0.0,0.5}) {
    const double expected[3]={-std::sqrt(3.0)/2*f+0.5*l+159.74*yaw,-l+159.74*yaw,
        std::sqrt(3.0)/2*f+0.5*l+159.74*yaw};
    for (int i=0;i<3;++i) REQUIRE_ARENA(Near(drive_kinematics::WheelSpeedMMps(i,f,l,yaw),expected[i]));
    const double physicalForward=(expected[2]-expected[0])/std::sqrt(3.0);
    const double physicalLeft=(expected[0]+expected[2]-2*expected[1])/3;
    const double physicalYaw=(expected[0]+expected[1]+expected[2])/(3*159.74);
    REQUIRE_ARENA(Near(physicalForward,f) && Near(physicalLeft,l) && Near(physicalYaw,yaw));
  }
}
void FixedCobraRejectsDisconnectedAndInvalidReadings() {
  CobraCheck check; LineReading reading;
  check.Update(reading,0); REQUIRE_ARENA(check.Ready() && !check.VoltagesValid());
  REQUIRE_ARENA(check.BlackLevel(1)==-1);
  reading.communicationValid=true;
  for (int i=0;i<4;++i) reading.volts[i]=check.White(i);
  check.Update(reading,0.05); REQUIRE_ARENA(check.VoltagesValid() && check.BlackLevel(1)==0);
  reading.volts[2]=std::numeric_limits<double>::quiet_NaN(); check.Update(reading,0.10);
  REQUIRE_ARENA(!check.VoltagesValid() && !check.ChannelReadingValid(2) && check.BlackLevel(1)==-1);
}
// Exercise the real mixer and whole-percent CAN conversion with independent
// chassis equations. Motor lag is simulated; no fitted friction compensation.
void WaypointWithDirectMotorOutput(double lag,bool disturbed,TapeApproach approach) {
  TapeAlignmentRoutine routine; auto in=TapeInput(); in.approach=approach; AnchorTapeRoutine(routine,in);
  const auto target=routine.GetTarget(); RobotPose physical=in.motion.pose;
  WheelDriveController drive; double counts[3]={},wheelSpeed[3]={},forward=0,left=0,yaw=0;
  for (int tick=0;tick<1500 && routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint;++tick) {
    if (disturbed && tick==40) physical.headingRad+=robot_geometry::Radians(5);
    FreshTapeTick(in); in.motion.yawRateRadps=yaw;
    in.motion.pose=physical; in.bodyForwardMMps=forward; in.bodyLeftMMps=left;
    routine.Update(in);
    if (routine.GetState()==TapeAlignmentRoutine::State::SearchTape) break;
    REQUIRE_ARENA(routine.GetState()==TapeAlignmentRoutine::State::DriveToPoint);
    double desired[3];
    for (int i=0;i<3;++i) desired[i]=drive_kinematics::WheelSpeedMMps(i,routine.GetForwardMMps(),routine.GetLeftMMps(),routine.GetYawRadps());
    drive.Update(desired,counts,0.02,0.9); REQUIRE_ARENA(!drive.HasFault());
    for (int i=0;i<3;++i) {
      const double duty=motor_output::TitanPercent(drive.Output(i),0.9)/100.0;
      const double targetSpeed=duty*localization_config::MAX_WHEEL_SPEED_MMPS;
      wheelSpeed[i]+=(lag==0?1:0.02/(lag+0.02))*(targetSpeed-wheelSpeed[i]);
      counts[i]+=wheelSpeed[i]*0.02/localization_config::ENCODER_MM_PER_COUNT;
    }
    forward=(wheelSpeed[2]-wheelSpeed[0])/std::sqrt(3.0);
    left=(wheelSpeed[0]+wheelSpeed[2]-2*wheelSpeed[1])/3;
    yaw=(wheelSpeed[0]+wheelSpeed[1]+wheelSpeed[2])/(3*159.74);
    const double mid=robot_geometry::ArenaAngle(physical.headingRad+yaw*0.01);
    physical.x_MM+=(std::cos(mid)*forward-std::sin(mid)*left)*0.02;
    physical.y_MM+=(std::sin(mid)*forward+std::cos(mid)*left)*0.02;
    physical.headingRad=robot_geometry::Wrap(physical.headingRad+yaw*0.02);
  }
  if (routine.GetState()!=TapeAlignmentRoutine::State::SearchTape)
    throw std::runtime_error("Direct motor waypoint: "+routine.GetStatus());
  REQUIRE_ARENA(std::hypot(physical.x_MM-target.x,physical.y_MM-target.y)<=10);
  REQUIRE_ARENA(std::fabs(robot_geometry::Wrap(physical.headingRad))<=robot_geometry::Radians(1));
}
void ClearancePlanningAndWallAssociation() {
  const auto map=DefaultNavigationMap(); const auto footprint=DefaultNavigationFootprint();
  REQUIRE_ARENA(Near(footprint.frontMM,291.74) && Near(footprint.rearMM,173));
  ClearancePlanner planner(map,footprint);
  REQUIRE_ARENA(planner.SegmentFree({950,850},{950,850},0)); // central square is floor tape
  REQUIRE_ARENA(!planner.SegmentFree({1000,500},{1000,500},0)); // lower divider
  REQUIRE_ARENA(!planner.SegmentFree({1600,1750},{1600,1750},0)); // manipulator reaches too far north
  for (auto start:{RobotPose{250,450,0},RobotPose{247.2,457.9,0}}) {
    const RobotPose goal={1600,1600,0};
    const auto plan=planner.Plan(start,goal);
    if (!plan.Valid()) throw std::runtime_error("Arena planner: "+plan.error);
    REQUIRE_ARENA(Near(plan.points.front().x,start.x_MM) && Near(plan.points.front().y,start.y_MM));
    REQUIRE_ARENA(Near(plan.points.back().x,goal.x_MM) && Near(plan.points.back().y,goal.y_MM));
    bool crossed=false;
    for (unsigned i=1;i<plan.points.size();++i) {
      const auto a=plan.points[i-1],b=plan.points[i];
      REQUIRE_ARENA(planner.SegmentFree(a,b,0));
      const int steps=std::max(1,static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y))));
      for (int j=0;j<=steps;++j) {
        const double t=static_cast<double>(j)/steps,x=a.x+t*(b.x-a.x),y=a.y+t*(b.y-a.y);
        // Independent axis-aligned check for a NORTH-facing rectangular robot.
        const double x0=x-171.74-40,x1=x+171.74+40,y0=y-173-40,y1=y+291.74+40;
        REQUIRE_ARENA(x0>=-1e-6 && y0>=-1e-6 && x1<=2000+1e-6 && y1<=2000+1e-6);
        for (auto r:map.forbidden) REQUIRE_ARENA(x1<r.xmin || x0>r.xmax || y1<r.ymin || y0>r.ymax);
      }
      if (a.x<1000 && b.x>=1000) {
        const double t=(1000-a.x)/(b.x-a.x),y=a.y+t*(b.y-a.y);
        REQUIRE_ARENA(y>=813 && y<=888.26); crossed=true;
      }
    }
    REQUIRE_ARENA(crossed && plan.lengthMM>std::hypot(goal.x_MM-start.x_MM,goal.y_MM-start.y_MM));
  }
  REQUIRE_ARENA(planner.SegmentFree({1750,1600},{1750,1600},0));
  REQUIRE_ARENA(planner.SegmentFree({1750,1600},{1750,1600},robot_geometry::PI/2));
  REQUIRE_ARENA(!planner.RotationFree({1750,1600},0,robot_geometry::PI/2)); // intermediate corner hits wall
  REQUIRE_ARENA(!planner.Plan({250,450,0},{1750,1600,robot_geometry::PI/2}).Valid());
  NavigationMap thin; thin.forbidden={{900,0,900.1,2000}};
  ClearancePlanner barrier(thin,footprint);
  REQUIRE_ARENA(!barrier.SegmentFree({300,800},{1600,800},0));
  REQUIRE_ARENA(!barrier.Plan({300,800,0},{1600,800,0}).Valid());
  NavigationMap invalid; invalid.forbidden={{500,500,400,600}};
  REQUIRE_ARENA(!ClearancePlanner(invalid,footprint).Plan({250,450,0},{1600,1600,0}).Valid());
  REQUIRE_ARENA(!ClearancePlanner(map,{}).Plan({250,450,0},{1600,1600,0}).Valid());
  REQUIRE_ARENA(!planner.Plan({250,450,0},{1000,1700,0}).Valid());

  const RobotPose predicted={1640,1640,0};
  const auto right=EstimateFromOuterWall(map,predicted,2,340,true);
  const auto front=EstimateFromOuterWall(map,predicted,3,228.4,true);
  REQUIRE_ARENA(right.valid && right.axis==0 && Near(right.coordinateMM,1600));
  REQUIRE_ARENA(front.valid && front.axis==1 && Near(front.coordinateMM,1600));
  REQUIRE_ARENA(!EstimateFromOuterWall(map,predicted,2,340,false).valid);
  REQUIRE_ARENA(!EstimateFromOuterWall(map,predicted,2,-1,true).valid);
  REQUIRE_ARENA(!EstimateFromOuterWall(map,predicted,2,500,true).valid); // residual too large
  REQUIRE_ARENA(!EstimateFromOuterWall(map,{1600,1600,0},0,1540,true).valid); // toward storages
  REQUIRE_ARENA(!EstimateFromOuterWall(map,{250,450,0},1,277,true).valid); // delivery band blocks trust
  const auto west=EstimateFromOuterWall(map,{250,450,0},0,190,true);
  REQUIRE_ARENA(west.valid && west.axis==0 && Near(west.coordinateMM,250));
}
void SampleNavigationRanges(NavigationInput& in,const RobotPose& truth,int tick) {
  const double angle=robot_geometry::ArenaAngle(truth.headingRad),c=std::cos(angle),s=std::sin(angle);
  for (int i=0;i<4;++i) {
    const bool ir=i==0 || i==2;
    if (ir?tick%3!=0:tick%10!=(i==1?0:5)) continue;
    const auto& mount=localization_config::RANGE_SENSOR_MOUNTS[i];
    const double x=truth.x_MM+c*mount.mountX_MM-s*mount.mountY_MM;
    const double y=truth.y_MM+s*mount.mountX_MM+c*mount.mountY_MM;
    const double a=angle+robot_geometry::Radians(mount.beamAngleDeg),dx=std::cos(a),dy=std::sin(a);
    const double inf=std::numeric_limits<double>::infinity();
    double range=inf;
    if (dx>1e-8) range=std::min(range,(2000-x)/dx);
    if (dx< -1e-8) range=std::min(range,-x/dx);
    if (dy>1e-8) range=std::min(range,(2000-y)/dy);
    if (dy< -1e-8) range=std::min(range,-y/dy);
    in.rangeMM[i]=range; in.stamp[i]=in.motion.now;
    in.valid[i]=range>=(ir?100:20) && range<=(ir?800:4000);
  }
}
void NavigationGoalAndCorrectionSimulation(double finalHeading,bool invalidSensors=false) {
  GoalNavigationRoutine nav; nav.Enter();
  NavigationInput in; in.motion={{950,900,0},0,0.02,true,false,false,0};
  RobotPose physical={250,450,0};
  nav.ReferenceCorner(0);
  for (int tick=0;tick<30 && !nav.NeedsPoseReset();++tick) {
    in.motion.now=tick*0.02; SampleNavigationRanges(in,physical,tick); nav.Update(in);
  }
  REQUIRE_ARENA(nav.NeedsPoseReset()); in.motion.pose=nav.RequestedPose(); nav.ConfirmReference();
  REQUIRE_ARENA(Near(in.motion.pose.x_MM,250) && Near(in.motion.pose.y_MM,450) && Near(in.motion.pose.headingRad,0));
  const double goalX=finalHeading==0?1600:1610; // center of the final rotation pocket
  REQUIRE_ARENA(nav.StartGoal({goalX,1600,finalHeading},in));
  WheelDriveController drive; ClearancePlanner planner(nav.Map(),DefaultNavigationFootprint());
  double counts[3]={},speeds[3]={},forward=0,left=0,yaw=0,biasX=0,biasY=0;
  bool injected=false,sawCorrection=false,sawObservation=false;
  for (int tick=30;tick<8000 && nav.GetState()!=GoalNavigationRoutine::State::Complete;++tick) {
    in.motion.now=tick*0.02;
    in.motion.pose={physical.x_MM+biasX,physical.y_MM+biasY,physical.headingRad};
    in.forwardMMps=forward; in.leftMMps=left; in.motion.yawRateRadps=yaw;
    if (!injected && nav.GetState()==GoalNavigationRoutine::State::Observing) {
      injected=true;
      if (!invalidSensors) { biasX=20; in.motion.pose.x_MM+=20; }
    }
    SampleNavigationRanges(in,physical,tick);
    if (invalidSensors) for (auto& v:in.valid) v=false;
    nav.Update(in);
    if (nav.NeedsPositionCorrection()) {
      sawCorrection=true;
      biasX=nav.RequestedPose().x_MM-physical.x_MM; biasY=nav.RequestedPose().y_MM-physical.y_MM;
      in.motion.pose=nav.RequestedPose(); nav.ConfirmPositionCorrection(in);
    }
    if (nav.NeedsPlan()) {
      REQUIRE_ARENA(nav.Forward()==0 && nav.Left()==0 && nav.Yaw()==0);
      const auto plan=planner.Plan(nav.PlanStart(in),nav.Goal());
      nav.PlanningStarted(); nav.AcceptPlan(plan,in);
    }
    sawObservation=sawObservation || nav.GetState()==GoalNavigationRoutine::State::Observing;
    if (nav.GetState()==GoalNavigationRoutine::State::Fault)
      throw std::runtime_error("Goal navigation: "+nav.Status()+" heading="+std::to_string(finalHeading)+" x="+std::to_string(physical.x_MM)+" y="+std::to_string(physical.y_MM));
    double request[3];
    for (int i=0;i<3;++i) request[i]=drive_kinematics::WheelSpeedMMps(i,nav.Forward(),nav.Left(),nav.Yaw());
    drive.Update(request,counts,0.02,0.9); REQUIRE_ARENA(!drive.HasFault());
    for (int i=0;i<3;++i) {
      const double target=motor_output::TitanPercent(drive.Output(i),0.9)/100.0*localization_config::MAX_WHEEL_SPEED_MMPS;
      speeds[i]+=0.02/(0.12+0.02)*(target-speeds[i]);
      counts[i]+=speeds[i]*0.02/localization_config::ENCODER_MM_PER_COUNT;
    }
    forward=(speeds[2]-speeds[0])/std::sqrt(3.0); left=(speeds[0]+speeds[2]-2*speeds[1])/3;
    yaw=(speeds[0]+speeds[1]+speeds[2])/(3*159.74);
    const double angle=robot_geometry::ArenaAngle(physical.headingRad+yaw*0.01);
    physical.x_MM+=(std::cos(angle)*forward-std::sin(angle)*left)*0.02;
    physical.y_MM+=(std::sin(angle)*forward+std::cos(angle)*left)*0.02;
    physical.headingRad=robot_geometry::Wrap(physical.headingRad+yaw*0.02);
    REQUIRE_ARENA(planner.SegmentFree({physical.x_MM,physical.y_MM},{physical.x_MM,physical.y_MM},physical.headingRad));
  }
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Complete && sawObservation);
  REQUIRE_ARENA(invalidSensors || sawCorrection);
  REQUIRE_ARENA(std::hypot(physical.x_MM-goalX,physical.y_MM-1600)<=10);
  REQUIRE_ARENA(std::fabs(robot_geometry::Wrap(physical.headingRad-finalHeading))<=robot_geometry::Radians(1));
  REQUIRE_ARENA(nav.Forward()==0 && nav.Left()==0 && nav.Yaw()==0);
}
void NavigationStopAndBadReference() {
  GoalNavigationRoutine nav; NavigationInput in; in.motion={{0,0,0},0,0.02,true,false,false,0};
  REQUIRE_ARENA(!nav.StartGoal({1600,1600,0},in));
  nav.ReferenceCorner(0);
  for (int tick=0;tick<700;++tick) { in.motion.now=tick*0.02; nav.Update(in); }
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Reference);
  in.motion.stopRequested=true; nav.Update(in);
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Aborted && nav.Forward()==0);
  nav.AcceptPlan(ClearancePlanner(DefaultNavigationMap(),DefaultNavigationFootprint()).Plan({250,450,0},{1600,1600,0}),in);
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Aborted); // old background result cannot restart
  drive_kinematics::EncoderGyroOdometry odometry; double counts[3]={};
  odometry.Reset({250,450,0},counts,137);
  REQUIRE_ARENA(odometry.CorrectPosition(20,-10));
  AddPhysicalCounts(counts,0,0,0.1);
  REQUIRE_ARENA(odometry.Update(counts,137+0.1*180/robot_geometry::PI,0,0.02));
  REQUIRE_ARENA(Near(odometry.Pose().x_MM,270) && Near(odometry.Pose().y_MM,440));
  REQUIRE_ARENA(Near(odometry.Pose().headingRad,0.1)); // correcting XY never changes gyro reference
  HeadingMotionControl feedback;
  for (int tick=0;tick<1000;++tick) REQUIRE_ARENA(feedback.CheckFeedback(0,robot_geometry::Radians(-2),0.02).empty());
  bool rejected=false;
  for (int tick=0;tick<20;++tick) if (!feedback.CheckFeedback(0.2,-0.5,0.02).empty()) rejected=true;
  REQUIRE_ARENA(rejected); // powered yaw genuinely going backwards still stops
}
void RunActiveChecks() {
  SharpAnalogConversionAndValidity(); YawRateUsesSensorSampleTime();
  FixedCobraReferencesNeedNoCalibration(); FixedCobraRejectsDisconnectedAndInvalidReadings();
  EncoderGyroOdometryChecks(); ProportionalTitanOutputAndQuantization();
  ClearancePlanningAndWallAssociation();
  NavigationStopAndBadReference();
  NavigationGoalAndCorrectionSimulation(0);
  NavigationGoalAndCorrectionSimulation(robot_geometry::PI/2);
  NavigationGoalAndCorrectionSimulation(-robot_geometry::PI/2);
  NavigationGoalAndCorrectionSimulation(0,true);
  TapeRoutineChecks(); WaypointSettlingDoesNotExpireProgressWatchdog();
  for (double lag:{0.0,0.12,0.25}) for (double deadband:{0.0,70.0,100.0})
    for (bool near:{false,true}) WaypointWithCoastingAndMotorDeadband(lag,deadband,near);
  for (double lag:{0.0,0.12,0.25}) for (bool disturbance:{false,true})
    for (auto side:{TapeApproach::FromLeft,TapeApproach::FromRight}) WaypointWithDirectMotorOutput(lag,disturbance,side);
}
}
#ifdef ARENA_TEST_STANDALONE
int main() {
  try { RunActiveChecks(); std::cout<<"PASS: north-zero odometry, clearance A*, swept footprint/rotation, wall association, four goal simulations, 10 mm arrival, tape, teleop geometry and sensor validity\n"; }
  catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
#else
TEST(Project8, ActiveMotionAndSensing) { EXPECT_NO_THROW(RunActiveChecks()); }
#endif
