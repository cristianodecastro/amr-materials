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
  // Installed raw counts increase for CW rolling, opposite canonical travel.
  for (int i=0;i<3;++i) counts[i]-=wheel[i]/mmPerCount;
}
void ClockwiseRawCountsMatchOdometryAndWheelMonitor() {
  for (double rawDirection:{-1.0,1.0}) {
    // Positive raw increments are the observed CLOCKWISE chassis rotation.
    // Deliberately do not derive this input sign from production configuration.
    drive_kinematics::EncoderGyroOdometry odometry;
    WheelDriveController drive;
    double counts[3]={};
    const double speed=-rawDirection*150,rate=speed/159.74;
    const double requested[3]={speed,speed,speed};
    odometry.Reset({500,500,0},counts,0);
    for (int tick=1;tick<=50;++tick) {
      for (int i=0;i<3;++i) counts[i]+=rawDirection*150*0.02/(102*robot_geometry::PI/1464);
      drive.Update(requested,counts,0.02,0.9);
      REQUIRE_ARENA(!drive.HasFault());
      REQUIRE_ARENA(odometry.Update(counts,rate*tick*0.02*180/robot_geometry::PI,
          rate*180/robot_geometry::PI,0.02));
    }
    for (int i=0;i<3;++i) REQUIRE_ARENA(Near(drive.MeasuredMMps(i),speed,0.01));
    REQUIRE_ARENA(Near(odometry.Pose().x_MM,500) && Near(odometry.Pose().y_MM,500));
    REQUIRE_ARENA(Near(odometry.Pose().headingRad,rate));
  }
}
void KinematicsUsesWheelProjectionAndSingleCountConversion() {
  REQUIRE_ARENA(Near(localization_config::ENCODER_MM_PER_COUNT,0.218881455373059,1e-9));
  for (double forward:{-1000.0,0.0,1000.0}) for (double left:{-1000.0,0.0,1000.0}) {
    const double turn=0.3;
    double counts[3]={},wheel[3]; AddPhysicalCounts(counts,forward,left,turn);
    for (int i=0;i<3;++i) wheel[i]=counts[i]*localization_config::WHEEL_ENCODER_SIGN[i]*
        localization_config::ENCODER_MM_PER_COUNT;
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
    ++counts[0]; --counts[2]; REQUIRE_ARENA(odometry.Update(counts,0,0,0.1));
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

void TitanOutputFloorPolarityAndQuantization() {
  for (double cap:{0.01,0.15,0.159,0.25,0.9,1.0}) for (int n=-2000;n<=2000;++n) {
    const double demand=n/2000.0;
    const int percent=motor_output::TitanPercent(demand,cap);
    const double argument=motor_output::TitanSetArgument(percent);
    REQUIRE_ARENA(static_cast<int>(std::fabs(argument*100))==std::abs(percent));
    REQUIRE_ARENA(std::abs(percent)<=static_cast<int>(std::floor(cap*100+1e-9)));
    REQUIRE_ARENA(std::fabs(percent/100.0-std::max(-cap,std::min(cap,demand)))<=0.010001);
  }
  REQUIRE_ARENA(motor_output::TitanPercent(0.04,0.9)==4); // raw quantizer; wheel boundary applies the floor
  REQUIRE_ARENA(motor_output::TitanPercent(0,0.9)==0);
  REQUIRE_ARENA(motor_output::TitanPercent(std::numeric_limits<double>::quiet_NaN(),0.9)==0);
  REQUIRE_ARENA(motor_output::TitanPercent(0.3,0)==0);
  REQUIRE_ARENA(motor_output::WheelTitanPercent(-1,0.4,0.9)==0);
  REQUIRE_ARENA(motor_output::WheelTitanPercent(3,0.4,0.9)==0);
  for (int i=0;i<3;++i) {
    REQUIRE_ARENA(motor_output::WheelTitanPercent(i,0,0.9)==0);
    REQUIRE_ARENA(motor_output::WheelTitanPercent(i,std::numeric_limits<double>::quiet_NaN(),0.9)==0);
    REQUIRE_ARENA(motor_output::WheelTitanPercent(i,0.4,0)==0);
    REQUIRE_ARENA(motor_output::WheelTitanPercent(i,1,0.25)==-25);
    REQUIRE_ARENA(motor_output::WheelTitanPercent(i,0.000001,0.9)==-15);
    REQUIRE_ARENA(motor_output::WheelTitanPercent(i,-0.04,0.9)==15);
    REQUIRE_ARENA(motor_output::WheelTitanPercent(i,1,0.14999)==0);
    for (double cap:{0.01,0.14999,0.15,0.159,0.25,0.9}) for (int n=-2000;n<=2000;++n) {
      const int percent=motor_output::WheelTitanPercent(i,n/2000.0,cap);
      REQUIRE_ARENA(percent==0 || std::abs(percent)>=15);
      REQUIRE_ARENA(std::abs(percent)<=static_cast<int>(std::floor(cap*100+1e-9)));
      REQUIRE_ARENA(static_cast<int>(std::fabs(motor_output::TitanSetArgument(percent)*100))==std::abs(percent));
      if (n==0 || cap<0.15) REQUIRE_ARENA(percent==0);
      else REQUIRE_ARENA(n*percent<0); // installed polarity, including tiny commands
    }
  }
  WheelDriveController drive;
  const double demand[3]={-120,-120,240},counts[3]={};
  drive.Update(demand,counts,0.02,0.9);
  for (int i=0;i<3;++i) REQUIRE_ARENA(Near(drive.Output(i),demand[i]/localization_config::MAX_WHEEL_SPEED_MMPS));
  REQUIRE_ARENA(localization_config::WHEEL_TITAN_PORT[0]==0 && localization_config::WHEEL_TITAN_PORT[1]==1 && localization_config::WHEEL_TITAN_PORT[2]==2);
  double sideways[3]={-0.01,0.02,-0.01};
  motor_output::PrepareAutonomousVector(sideways,0.9);
  REQUIRE_ARENA(Near(sideways[0],-0.15) && Near(sideways[1],0.30) && Near(sideways[2],-0.15));
  REQUIRE_ARENA(Near(sideways[0]+sideways[1]+sideways[2],0));
  double cancellation[3]={0.3,0.0003,-0.2997};
  motor_output::PrepareAutonomousVector(cancellation,0.9);
  REQUIRE_ARENA(Near(cancellation[0],0.3)); // never amplify a tiny residual into a 90% peak
  // Verify actual vectors independently of production's trigonometry/inverse.
  for (double f:{-100.0,0.0,100.0}) for (double l:{-100.0,0.0,100.0}) for (double yaw:{-0.5,0.0,0.5}) {
    const double expected[3]={-std::sqrt(3.0)/2*f+0.5*l+159.74*yaw,-l+159.74*yaw,
        std::sqrt(3.0)/2*f+0.5*l+159.74*yaw};
    for (int i=0;i<3;++i) REQUIRE_ARENA(Near(drive_kinematics::WheelSpeedMMps(i,f,l,yaw),expected[i]));
    const double physicalForward=(expected[2]-expected[0])/std::sqrt(3.0);
    const double physicalLeft=(expected[0]+expected[2]-2*expected[1])/3;
    const double physicalYaw=(expected[0]+expected[1]+expected[2])/(3*159.74);
    REQUIRE_ARENA(Near(physicalForward,f) && Near(physicalLeft,l) && Near(physicalYaw,yaw));
    // Exercise the same final electrical mapping used by Hardware. Installed
    // motors rotate opposite the CCW tangent for positive electrical duty,
    // independently established by the user's all-directions Teleop test.
    WheelDriveController mapped;
    double requested[3],actual[3],floored[3];
    for (int i=0;i<3;++i) requested[i]=drive_kinematics::WheelSpeedMMps(i,f,l,yaw);
    mapped.Update(requested,counts,0.02,0.9);
    REQUIRE_ARENA(!mapped.HasFault());
    for (int i=0;i<3;++i) {
      const double electrical=motor_output::WheelTitanPercent(i,mapped.Output(i),0.9)/100.0;
      actual[i]=-electrical*localization_config::MAX_WHEEL_SPEED_MMPS;
      const double runningSpeed=0.15*localization_config::MAX_WHEEL_SPEED_MMPS;
      floored[i]=std::fabs(expected[i])<=0.5?0:
          (expected[i]<0?-1:1)*std::max(runningSpeed,std::fabs(expected[i]));
    }
    // Below the running floor, individual wheel ratios change. Independently
    // check the resulting chassis twist rather than assume ideal requested speed.
    REQUIRE_ARENA(Near((actual[2]-actual[0])/std::sqrt(3.0),(floored[2]-floored[0])/std::sqrt(3.0),4.1));
    REQUIRE_ARENA(Near((actual[0]+actual[2]-2*actual[1])/3,(floored[0]+floored[2]-2*floored[1])/3,4.1));
    REQUIRE_ARENA(Near((actual[0]+actual[1]+actual[2])/(3*159.74),(floored[0]+floored[1]+floored[2])/(3*159.74),0.019));
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
// chassis equations. Motor lag and the real 15% electrical floor are included.
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
    double output[3]; for (int i=0;i<3;++i) output[i]=drive.Output(i);
    motor_output::PrepareAutonomousVector(output,0.9);
    for (int i=0;i<3;++i) {
      const double duty=motor_output::WheelTitanPercent(i,output[i],0.9)/100.0;
      const double targetSpeed=-duty*localization_config::MAX_WHEEL_SPEED_MMPS;
      wheelSpeed[i]+=(lag==0?1:0.02/(lag+0.02))*(targetSpeed-wheelSpeed[i]);
      counts[i]-=wheelSpeed[i]*0.02/localization_config::ENCODER_MM_PER_COUNT;
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
NavigationInput MvpReference(GoalNavigationRoutine& nav) {
  NavigationInput in;
  in.motion={{950,900,0},0.1,0.02,true,false,false,0};
  in.rangeMM[0]=190; in.rangeMM[1]=277; in.valid[0]=in.valid[1]=true;
  in.stamp[0]=in.stamp[1]=0.1;
  REQUIRE_ARENA(nav.InitializePose(PoseInitialization{})); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset());
  in.motion.pose=nav.RequestedPose(); nav.ConfirmReference();
  REQUIRE_ARENA(Near(in.motion.pose.x_MM,250) && Near(in.motion.pose.y_MM,450));
  return in;
}
void MvpStartReferenceAndStop() {
  GoalNavigationRoutine nav(false); NavigationInput in;
  REQUIRE_ARENA(!nav.StartGoal({250,700,0},in));
  REQUIRE_ARENA(nav.InitializePose(PoseInitialization{})); in.motion={{0,0,0},0.1,0.02,true,false,false,0};
  for (int tick=0;tick<100;++tick) {in.motion.now+=0.02;nav.Update(in);}
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Reference && nav.Forward()==0);
  in=MvpReference(nav);
  REQUIRE_ARENA(!nav.StartGoal({-1,700,0},in));
  REQUIRE_ARENA(!nav.StartGoal({250,700,std::numeric_limits<double>::quiet_NaN()},in));
  REQUIRE_ARENA(nav.StartGoal({250,700,0},in));
  REQUIRE_ARENA(!nav.StartGoal({1500,1500,1},in) && nav.Goal().x_MM==250 && nav.Goal().y_MM==700);
  in.motion.stopRequested=true; nav.Update(in);
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Aborted && nav.Forward()==0 && nav.Left()==0 && nav.Yaw()==0);
  in.motion.stopRequested=false; nav.Update(in);
  REQUIRE_ARENA(nav.Forward()==0 && !nav.IsRunning());
  REQUIRE_ARENA(nav.StartGoal({250,700,0},in));
  in.motion.healthy=false; nav.Update(in);
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Fault && nav.Forward()==0 && nav.Yaw()==0);
  in.motion.healthy=true;
  REQUIRE_ARENA(nav.StartGoal({250,700,0},in));
  in.motion.now+=navigation_config::MISSION_TIMEOUT_S+1; nav.Update(in);
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Fault && nav.Forward()==0);
}
// Clear-floor tests of the real odometry, controller, electrical mapping and
// motor lag. An operator, not this MVP, chooses obstacle-free arena segments.
void MvpDirectGoal(RobotPose goal,double lag,bool disturb=false,const std::string& via="",bool slip=false,bool court=false,bool sensorObstacle=false) {
  GoalNavigationRoutine nav(court); auto in=MvpReference(nav);
  LocalObstacleMap courtGeometry;
  LocalObstacleMap detectedGeometry(false);
  if (sensorObstacle) detectedGeometry.AddHit({250,1021.6},robot_geometry::PI/2);
  RobotPose physical=in.motion.pose;
  double counts[3]={},wheel[3]={},forward=0,left=0,yaw=0;
  drive_kinematics::EncoderGyroOdometry odometry;
  odometry.Reset(physical,counts,137); // raw gyro is not the arena reference
  REQUIRE_ARENA(nav.StartGoal(goal,in,via));
  for (auto& valid:in.valid) valid=false; // ranges are for initial reference only
  WheelDriveController drive; bool sawConcurrent=false,sawPassing=false,sawWallCorrection=false;
  for (int tick=1;tick<3000 && nav.GetState()!=GoalNavigationRoutine::State::Complete;++tick) {
    if (disturb && tick==40) {
      physical.headingRad+=robot_geometry::Radians(5);
      AddPhysicalCounts(counts,0,0,robot_geometry::Radians(5));
    }
    REQUIRE_ARENA(odometry.Update(counts,robot_geometry::Wrap(robot_geometry::Radians(137)+physical.headingRad)*180/robot_geometry::PI,
        yaw*180/robot_geometry::PI,0.02));
    in.motion.pose=odometry.Pose(); in.motion.now=0.1+tick*0.02; in.motion.yawRateRadps=yaw;
    in.forwardMMps=odometry.ForwardSpeed(); in.leftMMps=odometry.LeftSpeed();
    if (slip && tick%5==0) {
      // Independent simulated wall measurements, while encoders over-report
      // true chassis travel because the driven wheels slip by eight percent.
      in.valid[0]=in.valid[1]=true;
      in.rangeMM[0]=physical.x_MM-60; in.rangeMM[1]=physical.y_MM-173;
      in.stamp[0]=in.stamp[1]=in.motion.now;
    }
    if (sensorObstacle && tick==5) {
      in.valid[3]=true; in.rangeMM[3]=1021.6-physical.y_MM-171.6; in.stamp[3]=in.motion.now;
    }
    const unsigned previousLeg=nav.LegIndex();
    nav.Update(in);
    if (nav.LegIndex()>previousLeg) {
      sawPassing=true;
      REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Driving);
    }
    if (nav.GetState()==GoalNavigationRoutine::State::Fault) throw std::runtime_error("MVP goal "+std::to_string(goal.x_MM)+","+std::to_string(goal.y_MM)+
        " at "+std::to_string(physical.x_MM)+","+std::to_string(physical.y_MM)+" leg "+std::to_string(nav.LegIndex())+": "+nav.Status());
    const auto correction=nav.PositionCorrection();
    odometry.CorrectPosition(correction.x,correction.y);
    sawWallCorrection=sawWallCorrection || std::hypot(correction.x,correction.y)>0.01;
    if (nav.GetState()==GoalNavigationRoutine::State::Driving && std::hypot(nav.Forward(),nav.Left())>1 && std::fabs(nav.Yaw())>0.001)
      sawConcurrent=true;
    double request[3];
    for (int i=0;i<3;++i) request[i]=drive_kinematics::WheelSpeedMMps(i,nav.Forward(),nav.Left(),nav.Yaw());
    drive.Update(request,counts,0.02,0.9); REQUIRE_ARENA(!drive.HasFault());
    double output[3]; for (int i=0;i<3;++i) output[i]=drive.Output(i);
    motor_output::PrepareAutonomousVector(output,0.9);
    for (int i=0;i<3;++i) {
      const int percent=motor_output::WheelTitanPercent(i,output[i],0.9);
      REQUIRE_ARENA(percent==0 || std::abs(percent)>=15);
      const double target=-percent/100.0*localization_config::MAX_WHEEL_SPEED_MMPS;
      wheel[i]+=0.02/(lag+0.02)*(target-wheel[i]);
      counts[i]-=wheel[i]*0.02/localization_config::ENCODER_MM_PER_COUNT;
    }
    forward=(wheel[2]-wheel[0])/std::sqrt(3.0); left=(wheel[0]+wheel[2]-2*wheel[1])/3;
    yaw=(wheel[0]+wheel[1]+wheel[2])/(3*159.74);
    const double angle=robot_geometry::ArenaAngle(physical.headingRad+yaw*0.01);
    physical.x_MM+=(std::cos(angle)*forward-std::sin(angle)*left)*0.02*(slip?0.92:1);
    physical.y_MM+=(std::sin(angle)*forward+std::cos(angle)*left)*0.02*(slip?0.92:1);
    physical.headingRad=robot_geometry::Wrap(physical.headingRad+yaw*0.02);
    if (court && courtGeometry.Clearance({physical.x_MM,physical.y_MM},physical.headingRad)<0)
      throw std::runtime_error("Court collision toward "+std::to_string(goal.x_MM)+","+std::to_string(goal.y_MM)+" during detour at "+std::to_string(physical.x_MM)+","+std::to_string(physical.y_MM)+" heading "+std::to_string(physical.headingRad*180/robot_geometry::PI));
    REQUIRE_ARENA(!sensorObstacle || detectedGeometry.Clearance({physical.x_MM,physical.y_MM},physical.headingRad)>=0);
  }
  if (nav.GetState()!=GoalNavigationRoutine::State::Complete) throw std::runtime_error("MVP did not finish: "+nav.Status());
  REQUIRE_ARENA(std::hypot(physical.x_MM-goal.x_MM,physical.y_MM-goal.y_MM)<=10);
  REQUIRE_ARENA(std::fabs(robot_geometry::Wrap(physical.headingRad-goal.headingRad))<=robot_geometry::Radians(1));
  REQUIRE_ARENA(nav.Forward()==0 && nav.Left()==0 && nav.Yaw()==0);
  REQUIRE_ARENA(!disturb || sawConcurrent);
  REQUIRE_ARENA(nav.IntermediateCount()==0 || sawPassing);
  REQUIRE_ARENA(!slip || sawWallCorrection);
}
void FlexibleInitializationChecks() {
  NavigationInput in; in.motion={{100,100,0},1,0.02,true,false,false,0};
  GoalNavigationRoutine nav(false); PoseInitialization settings;
  settings.xMM="500"; settings.yMM="700"; settings.headingDeg=-90;
  for (auto& wall:settings.wall) wall="-";
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset() && Near(nav.RequestedPose().x_MM,500) && Near(nav.RequestedPose().y_MM,700));
  REQUIRE_ARENA(Near(nav.RequestedPose().headingRad,-robot_geometry::PI/2)); nav.ConfirmReference();
  settings=PoseInitialization{}; settings.xMM="500"; // only Y comes from the rear sonar
  in.valid[1]=true; in.rangeMM[1]=527; in.stamp[1]=1;
  REQUIRE_ARENA(nav.InitializePose(settings)); settings.xMM="20"; nav.Update(in); // snapshot
  REQUIRE_ARENA(nav.NeedsPoseReset() && Near(nav.RequestedPose().x_MM,500) && Near(nav.RequestedPose().y_MM,700));
  settings=PoseInitialization{}; settings.yMM="700"; settings.wall[0]="-"; settings.wall[2]="X=1100";
  in.valid[2]=true; in.rangeMM[2]=540; in.stamp[2]=1;
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset() && Near(nav.RequestedPose().x_MM,500) && Near(nav.RequestedPose().y_MM,700));
  settings=PoseInitialization{}; settings.wall[2]="X=1100"; settings.wall[3]="Y=1200";
  for (int i=0;i<4;++i) {in.valid[i]=true;in.stamp[i]=1;}
  in.rangeMM[0]=430; in.rangeMM[1]=517; in.rangeMM[2]=530; in.rangeMM[3]=318.4;
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset() && Near(nav.RequestedPose().x_MM,500) && Near(nav.RequestedPose().y_MM,700));
  // Invalid/stale redundant readings never contaminate the remaining sources.
  in.valid[0]=false; in.stamp[1]=0;
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset() && Near(nav.RequestedPose().x_MM,510) && Near(nav.RequestedPose().y_MM,710));
  // Heading +90 faces west: left points south, rear points east.
  settings=PoseInitialization{}; settings.headingDeg=90; settings.wall[0]="Y=0"; settings.wall[1]="X=1000";
  for (auto& valid:in.valid) valid=false;
  in.valid[0]=in.valid[1]=true; in.stamp[0]=in.stamp[1]=1; in.rangeMM[0]=640; in.rangeMM[1]=327;
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset() && Near(nav.RequestedPose().x_MM,500) && Near(nav.RequestedPose().y_MM,700));
  settings=PoseInitialization{}; settings.wall[0]="-"; // no X source: never guess zero
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(!nav.NeedsPoseReset() && nav.GetState()==GoalNavigationRoutine::State::Reference && nav.Forward()==0);
  settings.wall[3]="X=0"; in.valid[3]=true; in.rangeMM[3]=300; in.stamp[3]=1; // parallel beam
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in); REQUIRE_ARENA(!nav.NeedsPoseReset());
  settings.xMM="bad"; REQUIRE_ARENA(!nav.InitializePose(settings));
  settings.xMM="500"; settings.yMM="700"; settings.wall[3]="Z=0"; REQUIRE_ARENA(!nav.InitializePose(settings));
  settings.wall[3]="Y=100 extra"; REQUIRE_ARENA(!nav.InitializePose(settings));
  settings.wall[3]="-"; settings.xMM="2001"; REQUIRE_ARENA(!nav.InitializePose(settings));
  REQUIRE_ARENA(!nav.HasReference() && !nav.IsRunning());
}
void IntermediatePoseToleranceChecks() {
  GoalNavigationRoutine nav(false); auto in=MvpReference(nav);
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in,"250,700,-90"));
  in.motion.pose={250,700,0}; nav.Update(in);
  REQUIRE_ARENA(nav.LegIndex()==0 && nav.Forward()==0 && nav.Left()==0 && nav.Yaw()<0);
  in.motion.pose={250,700,robot_geometry::Radians(-79)}; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==0);
  in.motion.pose={276,700,robot_geometry::Radians(-90)}; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==0);
  in.motion.pose={274,700,robot_geometry::Radians(-81)}; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==1);
  nav.Abort(); in.motion.pose={250,450,robot_geometry::Radians(179)};
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in,"250,450,-179")); nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==1);
  nav.Abort(); in.motion.pose={250,450,0};
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in,"250,450,90;250,700"));
  nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==0 && nav.Yaw()>0); // in-place intermediate heading
  in.motion.pose.headingRad=robot_geometry::PI/2; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==1);
  in.motion.pose={250,700,0}; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==1 && nav.Yaw()>0); // inherited +90
  in.motion.pose.headingRad=robot_geometry::PI/2; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==2);
}
void TravelWallSupportChecks() {
  GoalNavigationRoutine nav(false); auto in=MvpReference(nav);
  in.motion.pose={280,410,0}; REQUIRE_ARENA(nav.StartGoal({250,950,0},in)); nav.Update(in);
  auto correction=nav.PositionCorrection(); REQUIRE_ARENA(Near(correction.x,-5) && Near(correction.y,5));
  in.motion.pose.x_MM+=correction.x; in.motion.pose.y_MM+=correction.y;
  in.motion.now+=0.02; nav.Update(in); // cached readings are NOT fused again
  REQUIRE_ARENA(Near(nav.PositionCorrection().x,0) && Near(nav.PositionCorrection().y,0));
  for (int tick=0;tick<30;++tick) {
    in.motion.now+=0.05; in.stamp[0]=in.stamp[1]=in.motion.now; nav.Update(in);
    correction=nav.PositionCorrection(); in.motion.pose.x_MM+=correction.x; in.motion.pose.y_MM+=correction.y;
  }
  REQUIRE_ARENA(Near(in.motion.pose.x_MM,250,0.1) && Near(in.motion.pose.y_MM,450,0.1));
  nav.Abort(); in=MvpReference(nav); in.motion.pose={280,410,-robot_geometry::PI/2};
  in.valid[0]=false; in.valid[2]=true; in.rangeMM[1]=77; in.rangeMM[2]=390; in.stamp[2]=in.motion.now;
  REQUIRE_ARENA(nav.StartGoal({500,450,0},in)); nav.Update(in);
  REQUIRE_ARENA(Near(nav.PositionCorrection().x,-5) && Near(nav.PositionCorrection().y,5)); // sensors reassociated after turn
  nav.Abort(); in=MvpReference(nav); in.motion.pose.headingRad=robot_geometry::PI/4;
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in)); nav.Update(in);
  REQUIRE_ARENA(Near(nav.PositionCorrection().x,0) && Near(nav.PositionCorrection().y,0)); // oblique
  nav.Abort(); in=MvpReference(nav); in.rangeMM[1]=100; in.valid[0]=false;
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in)); nav.Update(in);
  REQUIRE_ARENA(Near(nav.PositionCorrection().y,0)); // short echo from an object is not the wall
  nav.Abort(); in=MvpReference(nav); for (auto& valid:in.valid) valid=false;
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in)); nav.Update(in);
  in.motion.now+=0.1; in.motion.pose.y_MM+=30; in.forwardMMps=300;
  in.valid[1]=true; in.rangeMM[1]=277; in.stamp[1]=0.1; nav.Update(in);
  REQUIRE_ARENA(Near(nav.PositionCorrection().y,0)); // ping-time pose, not delayed read-time pose
  in.motion.now+=0.1; in.motion.pose.y_MM+=30; in.rangeMM[1]=307; in.stamp[1]=0.2; nav.Update(in);
  REQUIRE_ARENA(Near(nav.PositionCorrection().y,0));
  in.motion.now+=1; in.motion.pose.y_MM+=50; nav.Update(in);
  REQUIRE_ARENA(Near(nav.PositionCorrection().y,0)); // stale
}
void SensorDetourChecks() {
  for (int sensor=0;sensor<4;++sensor) {
    GoalNavigationRoutine nav(false); auto in=MvpReference(nav);
    in.motion.pose={1000,1000,0};
    for (auto& valid:in.valid) valid=false;
    in.valid[sensor]=true; in.rangeMM[sensor]=sensor==1?200:(sensor==3?250:350);
    in.stamp[sensor]=in.motion.now;
    const RobotPose goals[]={{100,1000,0},{1000,100,0},{1900,1000,0},{1000,1900,0}};
    REQUIRE_ARENA(nav.StartGoal(goals[sensor],in)); nav.Update(in);
    if (!nav.IsRunning()) throw std::runtime_error("Sensor detour "+std::to_string(sensor)+": "+nav.Status());
    REQUIRE_ARENA(nav.IsRunning() && std::hypot(nav.Forward(),nav.Left())>0 && nav.IntermediateCount()>0);
    REQUIRE_ARENA(nav.Status().find("Detour")!=std::string::npos);
    REQUIRE_ARENA(Near(nav.Goal().x_MM,goals[sensor].x_MM) && Near(nav.Goal().y_MM,goals[sensor].y_MM));
    nav.Abort(); in.valid[sensor]=false;
    const auto opposite=goals[(sensor+2)%4]; REQUIRE_ARENA(nav.StartGoal(opposite,in)); nav.Update(in);
    REQUIRE_ARENA(std::hypot(nav.Forward(),nav.Left())>0 && nav.IntermediateCount()==0); // invalid readings never invent obstacles
  }
}
void CourtDetourGeometryChecks() {
  LocalObstacleMap map; std::vector<ArenaPoint> path;
  REQUIRE_ARENA(!map.SegmentClear({250,450},{1500,450},0)); // divider blocks the direct leg
  REQUIRE_ARENA(map.Detour({250,450},{1500,450},0,path));
  REQUIRE_ARENA(path.size()>1); ArenaPoint previous={250,450}; bool passedAboveDivider=false;
  for (const auto& point:path) {
    REQUIRE_ARENA(map.SegmentClear(previous,point,0));
    passedAboveDivider=passedAboveDivider || point.y>800; previous=point;
  }
  REQUIRE_ARENA(passedAboveDivider && Near(path.back().x,1500) && Near(path.back().y,450));
  REQUIRE_ARENA(map.Detour({250,900},{1700,1600},0,path));
  REQUIRE_ARENA(!map.Detour({250,450},{1000,1700},0,path)); // inside storage block
  REQUIRE_ARENA(map.Clearance({250,700},robot_geometry::PI/2)<0); // west-facing front reaches outer wall
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
  REQUIRE_ARENA(nav.StartGoal({1500,450,0},in)); nav.Update(in);
  REQUIRE_ARENA(nav.IsRunning() && nav.IntermediateCount()>0 && std::hypot(nav.Forward(),nav.Left())>0);
}
void MvpManualRouteInput() {
  GoalNavigationRoutine nav(false); auto in=MvpReference(nav);
  for (const std::string bad:{"wrong","250,700,90,1","250,700,nan","250,","-1,700","250,2500","250,700; nope"}) {
    REQUIRE_ARENA(!nav.StartGoal({250,950,0},in,bad));
    REQUIRE_ARENA(!nav.IsRunning() && nav.Forward()==0);
  }
  std::string points=" 250,700; 250,950; ";
  REQUIRE_ARENA(nav.StartGoal({250,1200,0},in,points));
  REQUIRE_ARENA(nav.IntermediateCount()==2);
  points.clear(); REQUIRE_ARENA(nav.IntermediateCount()==2); // snapshot is independent of edits
  REQUIRE_ARENA(!nav.StartGoal({500,500,1},in,""));
  REQUIRE_ARENA(nav.IntermediateCount()==2 && nav.Goal().y_MM==1200);
  nav.Abort();
  REQUIRE_ARENA(nav.StartGoal({250,700,0},in,""));
  REQUIRE_ARENA(nav.IntermediateCount()==0);
  nav.Abort();
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in,"250,450;250,700"));
  nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==1); // a point at the start is skipped without stopping
  for (const auto& preset:navigation_config::GOALS) {
    nav.Abort();
    REQUIRE_ARENA(nav.StartGoal({preset.xMM,preset.yMM,robot_geometry::Radians(preset.headingDeg)},in,preset.intermediatePoses));
  }
}
void YawDirectionCheckAllowsDelayedReversalButRejectsSustainedOpposition() {
  HeadingMotionControl feedback;
  // A newly requested CCW correction may initially brake a CW turn. The
  // previous monitor failed after 5 degrees even before the motors reversed.
  for (int tick=0;tick<15;++tick)
    REQUIRE_ARENA(feedback.CheckFeedback(0.2,robot_geometry::Radians(-40),0.02).empty());
  for (int tick=0;tick<20;++tick)
    REQUIRE_ARENA(feedback.CheckFeedback(0.2,robot_geometry::Radians(20),0.02).empty());
  // Every reversal establishes its own settling interval, not only startup.
  for (int tick=0;tick<15;++tick)
    REQUIRE_ARENA(feedback.CheckFeedback(-0.2,robot_geometry::Radians(40),0.02).empty());
  for (int tick=0;tick<20;++tick)
    REQUIRE_ARENA(feedback.CheckFeedback(-0.2,robot_geometry::Radians(-20),0.02).empty());
  feedback.Reset();
  int faultTick=-1;
  for (int tick=0;tick<40;++tick) {
    const auto fault=feedback.CheckFeedback(0.2,robot_geometry::Radians(-20),0.02);
    if (!fault.empty()) { faultTick=tick; REQUIRE_ARENA(fault.find("commanded CCW")!=std::string::npos); break; }
  }
  REQUIRE_ARENA(faultTick>=27 && faultTick<=30); // 300 ms response + five degrees opposite travel
  REQUIRE_ARENA(Near(feedback.FeedbackRequest(),0.2));
  REQUIRE_ARENA(Near(feedback.FeedbackMeasured(),robot_geometry::Radians(-20)));
  feedback.CheckFeedback(0,0,0.02);
  for (int tick=0;tick<15;++tick)
    REQUIRE_ARENA(feedback.CheckFeedback(0.2,robot_geometry::Radians(-40),0.02).empty());
}
void RunActiveChecks() {
  SharpAnalogConversionAndValidity(); YawRateUsesSensorSampleTime();
  FixedCobraReferencesNeedNoCalibration(); FixedCobraRejectsDisconnectedAndInvalidReadings();
  ClockwiseRawCountsMatchOdometryAndWheelMonitor(); EncoderGyroOdometryChecks(); TitanOutputFloorPolarityAndQuantization();
  YawDirectionCheckAllowsDelayedReversalButRejectsSustainedOpposition();
  MvpStartReferenceAndStop();
  FlexibleInitializationChecks(); IntermediatePoseToleranceChecks(); TravelWallSupportChecks(); SensorDetourChecks(); CourtDetourGeometryChecks();
  MvpManualRouteInput();
  for (double lag:{0.12,0.18}) {
    MvpDirectGoal({250,1450,0},lag);
    MvpDirectGoal({1250,450,0},lag); // empty simulation, not an arena route recommendation
    MvpDirectGoal({250,700,robot_geometry::PI/2},lag);
    MvpDirectGoal({250,700,-robot_geometry::PI/2},lag);
  }
  MvpDirectGoal({250,1450,0},0.12,true);
  MvpDirectGoal({250,1200,0},0.12,false,"250,700;250,950");
  MvpDirectGoal({500,900,robot_geometry::PI/2},0.18,false,"250,700;500,700");
  MvpDirectGoal({250,950,0},0.12,false,"250,700");
  MvpDirectGoal({500,950,0},0.12,false,"250,700,-90;500,700,0");
  MvpDirectGoal({250,950,0},0.18,false,"250,450,90;250,700,-90");
  MvpDirectGoal({250,1200,0},0.18,false,"",true);
  MvpDirectGoal({1500,450,0},0.18,false,"",false,true);
  MvpDirectGoal({1700,1600,0},0.18,false,"",false,true);
  MvpDirectGoal({250,1450,0},0.18,false,"",false,false,true);
  MvpDirectGoal({330,900,robot_geometry::PI/2},0.18,false,"",false,true); // clear-space turning detour
  MvpDirectGoal({500,1100,0},0.18,false,"330,700,90",false,true);
  TapeRoutineChecks(); WaypointSettlingDoesNotExpireProgressWatchdog();
  for (double lag:{0.0,0.12,0.25}) for (double deadband:{0.0,70.0,100.0})
    for (bool near:{false,true}) WaypointWithCoastingAndMotorDeadband(lag,deadband,near);
  for (double lag:{0.0,0.12,0.25}) for (bool disturbance:{false,true})
    for (auto side:{TapeApproach::FromLeft,TapeApproach::FromRight}) WaypointWithDirectMotorOutput(lag,disturbance,side);
}
}
#ifdef ARENA_TEST_STANDALONE
int main() {
  try { RunActiveChecks(); std::cout<<"PASS: intermediate poses/tolerances, optional XY and four-sensor initialization, live wall fusion/slip, timestamp compensation, sensor/court detours, gyro/encoders, 15% floor, tape and teleop\n"; }
  catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
#else
TEST(Project8, ActiveMotionAndSensing) { EXPECT_NO_THROW(RunActiveChecks()); }
#endif
