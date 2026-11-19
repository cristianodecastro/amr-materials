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
  const double fixedOffset=odometry.HeadingOffsetRad();
  REQUIRE_ARENA(odometry.Update(counts,137,4.0/60,0.02)); // reported 4 deg/min rate is not integrated twice
  REQUIRE_ARENA(Near(odometry.Pose().headingRad,0) && odometry.HeadingOffsetRad()==fixedOffset);
  REQUIRE_ARENA(odometry.Update(counts,138,0,0.02)); // yaw angle itself changes: preserve that measured change
  REQUIRE_ARENA(Near(odometry.Pose().headingRad,robot_geometry::Radians(1)) && odometry.HeadingOffsetRad()==fixedOffset);
  REQUIRE_ARENA(odometry.Update(counts,137,0,0.2) && odometry.Healthy());
  REQUIRE_ARENA(!odometry.Update(counts,137,0,0.6) && !odometry.Healthy());
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
void TapeRoutineChecks() {
  TapeReferenceNeedsOnlyFreshValidRanges(); TapeEqualityRequiresActualTapeAndStableStop();
  TapeSensorWaitsRecoverAndGyroNoiseDoesNotBlockCompletion();
  TapeApproachSelectionAndSearchDirections();
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
NavigationInput MvpReference(GoalNavigationRoutine& nav) {
  NavigationInput in;
  in.motion={{950,900,0},0.1,0.02,true,false,false,0};
  in.rangeMM[0]=190; in.rangeMM[1]=277; in.valid[0]=in.valid[1]=true;
  in.stamp[0]=in.stamp[1]=0.1;
  PoseInitialization settings; settings.xMM="0"; settings.yMM="0"; settings.headingDeg="0";
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset());
  in.motion.pose=nav.RequestedPose(); nav.ConfirmReference();
  REQUIRE_ARENA(Near(in.motion.pose.x_MM,250) && Near(in.motion.pose.y_MM,450));
  return in;
}
void MvpStartReferenceAndStop() {
  GoalNavigationRoutine nav; NavigationInput in;
  REQUIRE_ARENA(!nav.StartGoal({250,700,0},in));
  PoseInitialization settings; settings.xMM="0"; settings.yMM="0"; settings.headingDeg="0";
  REQUIRE_ARENA(nav.InitializePose(settings)); in.motion={{0,0,0},0.1,0.02,true,false,false,0};
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
  in.motion.now+=120; nav.Update(in);
  REQUIRE_ARENA(nav.IsRunning()); // no arbitrary task deadline
  nav.Abort();
}
// Clear-floor tests of the real odometry, controller, electrical mapping and
// motor lag. An operator, not this MVP, chooses obstacle-free arena segments.
void FlexibleInitializationChecks() {
  NavigationInput in; in.motion={{333,444,0.3},1,0.02,true,false,false,0};
  GoalNavigationRoutine nav; PoseInitialization settings;
  settings.headingDeg="-90";
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset() && nav.NeedsHeadingReset());
  REQUIRE_ARENA(Near(nav.RequestedPose().x_MM,333) && Near(nav.RequestedPose().y_MM,444));
  REQUIRE_ARENA(Near(nav.RequestedPose().headingRad,-robot_geometry::PI/2));
  in.motion.pose=nav.RequestedPose(); nav.ConfirmReference();
  // X-only uses the west-facing rear sonar at heading -90; Y stays EXACTLY 444.
  settings=PoseInitialization{}; settings.xMM="0";
  in.valid[1]=true; in.rangeMM[1]=127; in.stamp[1]=1;
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset() && !nav.NeedsHeadingReset());
  REQUIRE_ARENA(Near(nav.RequestedPose().x_MM,300) && nav.RequestedPose().y_MM==444);
  in.motion.pose=nav.RequestedPose(); nav.ConfirmReference();
  // Symmetric Y-only: south-facing RIGHT IR, preserving X and the gyro frame.
  settings=PoseInitialization{}; settings.yMM="0";
  in.valid[2]=true; in.rangeMM[2]=340; in.stamp[2]=1;
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset() && !nav.NeedsHeadingReset());
  REQUIRE_ARENA(nav.RequestedPose().x_MM==300 && Near(nav.RequestedPose().y_MM,400));
  REQUIRE_ARENA(nav.RequestedPose().headingRad==in.motion.pose.headingRad);
  in.motion.pose=nav.RequestedPose(); nav.ConfirmReference();
  // Changing a named area's default heading must select different sensors.
  in.motion.pose={950,900,0}; settings.xMM="0"; settings.headingDeg="-90";
  settings.useArea=true; settings.areaPose={300,400,0};
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(Near(nav.RequestedPose().x_MM,300) && Near(nav.RequestedPose().y_MM,400));
  in.motion.pose=nav.RequestedPose(); nav.ConfirmReference();
  // All dashes: no pose component changes, even with valid sensor returns.
  settings=PoseInitialization{};
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(!nav.NeedsHeadingReset() && nav.RequestedPose().x_MM==300 && nav.RequestedPose().y_MM==400);
  REQUIRE_ARENA(nav.RequestedPose().headingRad==in.motion.pose.headingRad); nav.ConfirmReference();
  // A requested numeric heading applies even while the selected X reading is invalid.
  settings.xMM="2000"; settings.headingDeg="90";
  for (auto& valid:in.valid) valid=false;
  REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in);
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Reference && nav.NeedsHeadingReset());
  REQUIRE_ARENA(nav.NeedsPoseReset() && Near(nav.RequestedPose().headingRad,robot_geometry::PI/2));
  REQUIRE_ARENA(nav.RequestedPose().x_MM==300 && nav.RequestedPose().y_MM==400);
  in.motion.pose=nav.RequestedPose(); nav.ConfirmReference();
  REQUIRE_ARENA(!nav.HasReference() && !nav.NeedsHeadingReset());
  in.valid[1]=true; in.rangeMM[1]=1127; in.stamp[1]=1;
  nav.Update(in);
  REQUIRE_ARENA(nav.NeedsPoseReset() && !nav.NeedsHeadingReset());
  REQUIRE_ARENA(Near(nav.RequestedPose().x_MM,700) && nav.RequestedPose().y_MM==400); nav.ConfirmReference();
  settings.xMM="bad"; REQUIRE_ARENA(!nav.InitializePose(settings));
  settings.xMM="2001"; REQUIRE_ARENA(!nav.InitializePose(settings));
  settings.xMM="0"; settings.headingDeg="NaN"; REQUIRE_ARENA(!nav.InitializePose(settings));
}
void IntermediatePoseToleranceChecks() {
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in,"250,700,-90"));
  in.motion.pose={250,700,0}; nav.Update(in);
  REQUIRE_ARENA(nav.LegIndex()==1 && std::hypot(nav.Forward(),nav.Left())>0 && nav.Yaw()<0); // keep translating with a late heading
  nav.Abort(); in.motion.pose={250,450,0}; REQUIRE_ARENA(nav.StartGoal({250,950,0},in,"250,700,-90"));
  in.motion.pose={276,700,robot_geometry::Radians(-90)}; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==0);
  in.motion.pose={274,700,robot_geometry::Radians(-81)}; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==1);
  nav.Abort(); in.motion.pose={250,450,0};
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in,"250,700,0"));
  in.motion.pose={250,735,0}; nav.Update(in);
  REQUIRE_ARENA(nav.LegIndex()==1 && nav.Forward()>0); // a sample can cross the passing circle without chasing the old point
  nav.Abort(); in.motion.pose={250,450,robot_geometry::Radians(179)};
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in,"250,450,-179")); nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==1);
  nav.Abort(); in.motion.pose={250,450,0};
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in,"250,450,90;250,700"));
  nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==0 && nav.Yaw()>0); // in-place intermediate heading
  in.motion.pose.headingRad=robot_geometry::PI/2; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==1);
  in.motion.pose={250,700,0}; nav.Update(in); REQUIRE_ARENA(nav.LegIndex()==2 && Near(nav.Yaw(),0)); // final leg now uses goal heading 0
}
void ContinuousGoalPoseChecks() {
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
  for (auto& valid:in.valid) valid=false;
  REQUIRE_ARENA(nav.StartGoal({250,1200,robot_geometry::PI/2},in)); nav.Update(in);
  REQUIRE_ARENA(nav.Forward()>0 && nav.Yaw()>0); // final heading begins DURING translation
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Driving);
  nav.Abort(); in.motion.pose={250,1050,robot_geometry::Radians(-3)};
  REQUIRE_ARENA(nav.StartGoal({250,1200,0},in)); nav.Update(in);
  REQUIRE_ARENA(nav.Forward()>0 && nav.Yaw()>0);
  REQUIRE_ARENA(Near(nav.Yaw(),navigation_config::MIN_YAW_RATE_RADPS));
  in.motion.pose.y_MM=990; nav.Update(in); // beyond 200 mm again: final tolerance must not loosen
  REQUIRE_ARENA(nav.Yaw()>0 && nav.Forward()>0);
  nav.Abort(); in.motion.pose={250,1200,robot_geometry::Radians(-3)};
  REQUIRE_ARENA(nav.StartGoal({250,1200,0},in)); nav.Update(in);
  REQUIRE_ARENA(Near(nav.Forward(),0) && Near(nav.Left(),0) && nav.Yaw()>0);
  REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Driving); // no stop/turn state handoff
}
void MinimumYawAndStationaryCorrectionChecks() {
  HeadingMotionControl yaw;
  const double minimum=navigation_config::MIN_YAW_RATE_RADPS,maximum=navigation_config::MAX_YAW_RATE_RADPS;
  const double tolerance=heading_control_config::HEADING_TOLERANCE_RAD;
  REQUIRE_ARENA(Near(yaw.Request(robot_geometry::Radians(2),0,0,0.02,tolerance,1,maximum,minimum),minimum));
  yaw.Reset();
  REQUIRE_ARENA(yaw.Request(robot_geometry::Radians(0.5),minimum,0,0.02,tolerance,1,maximum,minimum)==0);
  yaw.Reset();
  REQUIRE_ARENA(yaw.Request(robot_geometry::Radians(2),minimum,robot_geometry::Radians(40),0.02,tolerance,1,maximum,minimum)==0);
  StationaryYawCorrection correction;
  double counts[3]={};
  correction.Update(0,0,counts,true,true);
  correction.Update(0.03,0.5,counts,true,true);
  correction.Update(0.06,1,counts,true,true);
  const double held=correction.Update(0.09,1.5,counts,true,true);
  REQUIRE_ARENA(correction.Holding());
  REQUIRE_ARENA(Near(correction.Update(0.12,2,counts,true,true),held));
  const double bias=correction.CorrectionDegrees();
  // Even very slow commanded motion passes through immediately.
  const double turning=correction.Update(0.121,2.02,counts,false,true);
  REQUIRE_ARENA(!correction.Holding() && turning>held && Near(correction.CorrectionDegrees(),bias));
  counts[0]+=1;
  REQUIRE_ARENA(correction.Update(0.122,2.04,counts,true,true)>turning && !correction.Holding());
  REQUIRE_ARENA(correction.Update(1.122,2.06,counts,true,true)>turning+0.9 && !correction.Holding());
  REQUIRE_ARENA(Near(correction.CorrectionDegrees(),bias));
  correction.Clear(); REQUIRE_ARENA(!correction.Holding() && correction.CorrectionDegrees()==0);
}

void ContinuousTransitChecks() {
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
  for (auto& valid:in.valid) valid=false;
  REQUIRE_ARENA(nav.StartGoal({250,1200,0},in,"250,700,0;250,950,0"));
  double oldForward=0,oldLeft=0; unsigned transitions=0;
  for (int tick=1;tick<=110;++tick) {
    in.motion.now=0.1+tick*0.02; in.motion.pose.y_MM=450+tick*5;
    in.forwardMMps=250;
    const auto oldLeg=nav.LegIndex(); nav.Update(in);
    REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Driving);
    const double change=std::hypot(nav.Forward()-oldForward,nav.Left()-oldLeft);
    REQUIRE_ARENA(change<=navigation_config::ACCEL_MMPS2*0.02+1e-6);
    if (nav.LegIndex()>oldLeg) {
      ++transitions;
      REQUIRE_ARENA(nav.Forward()>200 && nav.Forward()>=oldForward-6.000001);
    }
    oldForward=nav.Forward(); oldLeft=nav.Left();
  }
  REQUIRE_ARENA(transitions==2);
}
void TravelWallSupportChecks() {
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
  in.motion.pose={280,410,0}; REQUIRE_ARENA(nav.StartGoal({250,950,0},in)); nav.Update(in);
  REQUIRE_ARENA(Near(nav.PositionCorrection().x,0) && Near(nav.PositionCorrection().y,0));
  REQUIRE_ARENA(Near(nav.ControlPose().x_MM,275) && Near(nav.ControlPose().y_MM,415));
  in.motion.now+=0.02; nav.Update(in); // cached readings are NOT fused again
  REQUIRE_ARENA(Near(nav.PositionCorrection().x,0) && Near(nav.PositionCorrection().y,0));
  for (int tick=0;tick<30;++tick) {
    in.motion.now+=0.05; in.stamp[0]=in.stamp[1]=in.motion.now; nav.Update(in);
    REQUIRE_ARENA(Near(nav.PositionCorrection().x,0) && Near(nav.PositionCorrection().y,0));
  }
  REQUIRE_ARENA(Near(nav.ControlPose().x_MM,250,0.1) && Near(nav.ControlPose().y_MM,450,0.1));
  nav.Abort(); in=MvpReference(nav); in.motion.pose={280,410,-robot_geometry::PI/2};
  in.valid[0]=false; in.valid[2]=true; in.rangeMM[1]=77; in.rangeMM[2]=390; in.stamp[2]=in.motion.now;
  REQUIRE_ARENA(nav.StartGoal({500,450,0},in)); nav.Update(in);
  REQUIRE_ARENA(Near(nav.ControlPose().x_MM,275) && Near(nav.ControlPose().y_MM,415)); // sensors reassociated after turn
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
void BoundedLateSamplesKeepOdometryAndAutoDriveHealthy() {
  const RobotPose initial={547,828,robot_geometry::Radians(34.9)};
  drive_kinematics::EncoderGyroOdometry odometry;
  double counts[3]={}; odometry.Reset(initial,counts,137);
  double elapsed=0;
  for (double dt:{0.02,0.18,0.02,0.12,0.02}) {
    elapsed+=dt; AddPhysicalCounts(counts,200*dt,40*dt,0.1*dt);
    REQUIRE_ARENA(odometry.Update(counts,137+0.1*elapsed*180/robot_geometry::PI,0.1*180/robot_geometry::PI,dt));
    REQUIRE_ARENA(odometry.Healthy() && Near(odometry.ForwardSpeed(),200) && Near(odometry.LeftSpeed(),40));
  }
  const double a=robot_geometry::ArenaAngle(initial.headingRad),vx=200*std::cos(a)-40*std::sin(a),vy=200*std::sin(a)+40*std::cos(a);
  REQUIRE_ARENA(Near(odometry.Pose().x_MM,initial.x_MM+(vx*std::sin(0.1*elapsed)+vy*(std::cos(0.1*elapsed)-1))/0.1));
  REQUIRE_ARENA(Near(odometry.Pose().y_MM,initial.y_MM+(vx*(1-std::cos(0.1*elapsed))+vy*std::sin(0.1*elapsed))/0.1));
  WheelDriveController automatic,manual;
  double desired[3]={-200,0,200},baseline[3]={};
  automatic.UpdateAutonomous(desired,baseline,0.02,0.9);
  automatic.UpdateAutonomous(desired,counts,0.18,0.9);
  REQUIRE_ARENA(!automatic.HasFault() && automatic.Output(0)<0 && automatic.Output(2)>0);
  automatic.UpdateAutonomous(desired,counts,0.02,0.9); REQUIRE_ARENA(!automatic.HasFault());
  manual.Update(desired,counts,0.18,0.9); REQUIRE_ARENA(manual.HasFault());
  automatic.UpdateAutonomous(desired,counts,0.6,0.9); REQUIRE_ARENA(automatic.HasFault());
  automatic.UpdateAutonomous(desired,counts,0.18,0.9); REQUIRE_ARENA(automatic.HasFault()); // never clear a genuine latched fault
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
  in.motion.pose=initial; for (auto& valid:in.valid) valid=false;
  REQUIRE_ARENA(nav.StartGoal({900,900,0},in));
  in.motion.now+=0.18; in.motion.dt=0.18; in.motion.pose=odometry.Pose(); nav.Update(in);
  REQUIRE_ARENA(nav.IsRunning());
  in.motion.dt=0.02; in.motion.now+=0.02; nav.Update(in); REQUIRE_ARENA(nav.IsRunning());
}
void HealthFailuresIdentifyTheirActualSource() {
  for (const std::string issue:{"IMU disconnected","IMU is calibrating","Control Station disconnected","Odometry: Encoder sample is not finite"}) {
    GoalNavigationRoutine nav; auto in=MvpReference(nav);
    REQUIRE_ARENA(nav.StartGoal({250,950,0},in));
    in.motion.healthy=false; in.healthIssue=issue; nav.Update(in);
    REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Fault && nav.Status().find(issue)!=std::string::npos);
    REQUIRE_ARENA(nav.Forward()==0 && nav.Left()==0 && nav.Yaw()==0);
  }
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in)); in.motion.dt=0.6; nav.Update(in);
  REQUIRE_ARENA(nav.Status().find("Control loop gap 600 ms")!=std::string::npos);
}
void RangeCorrectionsUseEachBeamHeadingAndMount() {
  const double c=std::sqrt(3.0)/2,s=std::sqrt(0.5);
  const double degrees[]={0,90,-90,180,30,-45};
  // Independent face-to-wall distances at true arena center (500,700).
  // Left/rear/right/front beams always remain normal to their robot side.
  const double ranges[][4]={
      {440,527,1440,1128.4}, {640,1327,1240,328.4},
      {1240,327,640,1328.4}, {1440,1127,440,528.4},
      {500/c-60,700/c-173,1500/c-60,500/0.5-171.6},
      {500/s-60,500/s-173,700/s-60,1300/s-171.6}};
  for (int row=0;row<6;++row) {
    GoalNavigationRoutine nav; NavigationInput in;
    const double h=robot_geometry::Radians(degrees[row]);
    in.motion={{500,700,h},1,0.02,true,false,false,robot_geometry::Radians(20)};
    PoseInitialization settings; settings.headingDeg=std::to_string(degrees[row]);
    REQUIRE_ARENA(nav.InitializePose(settings)); nav.Update(in); REQUIRE_ARENA(nav.NeedsPoseReset()); nav.ConfirmReference();
    in.motion.pose={530,660,h};
    for (int sensor=0;sensor<4;++sensor) { in.valid[sensor]=true; in.rangeMM[sensor]=ranges[row][sensor]; in.stamp[sensor]=1; }
    REQUIRE_ARENA(nav.StartGoal({800,1100,h},in)); nav.Update(in);
    REQUIRE_ARENA(Near(nav.ControlPose().x_MM,525) && Near(nav.ControlPose().y_MM,665));
    REQUIRE_ARENA(Near(nav.PositionCorrection().x,0) && Near(nav.PositionCorrection().y,0));
    REQUIRE_ARENA(nav.IsRunning() && std::hypot(nav.Forward(),nav.Left())>0); // correction supports travel even while yaw is changing
  }
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
  for (auto& valid:in.valid) valid=false;
  in.motion.pose={250,300,0}; in.motion.now=1;
  REQUIRE_ARENA(nav.StartGoal({250,900,0},in)); nav.Update(in);
  in.motion.now=1.1; in.motion.pose.headingRad=robot_geometry::PI/2; in.motion.yawRateRadps=robot_geometry::Radians(20);
  in.valid[0]=true; in.rangeMM[0]=190; in.stamp[0]=1; nav.Update(in);
  REQUIRE_ARENA(Near(nav.PositionCorrection().x,0) && Near(nav.PositionCorrection().y,0)); // measurement used the earlier north-facing beam
}
void UnusableAndCloseRangesNeverVetoMotion() {
  for (int sensor=0;sensor<4;++sensor) for (int scenario=0;scenario<7;++scenario) {
    GoalNavigationRoutine nav; auto in=MvpReference(nav);
    for (auto& valid:in.valid) valid=false;
    REQUIRE_ARENA(nav.StartGoal({250,950,0},in));
    in.valid[sensor]=scenario!=0; in.rangeMM[sensor]=scenario==1?-1:(scenario==2?0:(scenario==3?
        std::numeric_limits<double>::quiet_NaN():(scenario==4?5000:200)));
    in.stamp[sensor]=scenario==5?-1:(scenario==6?in.motion.now+1:in.motion.now);
    nav.Update(in);
    REQUIRE_ARENA(nav.IsRunning() && std::hypot(nav.Forward(),nav.Left())>0 && nav.IntermediateCount()==0);
    REQUIRE_ARENA(Near(nav.PositionCorrection().x,0) && Near(nav.PositionCorrection().y,0));
  }
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
  REQUIRE_ARENA(nav.StartGoal({250,950,0},in));
  for (int sensor=0;sensor<4;++sensor) { in.valid[sensor]=true; in.rangeMM[sensor]=(sensor==0 || sensor==2)?100:25; in.stamp[sensor]=in.motion.now; }
  nav.Update(in); REQUIRE_ARENA(nav.IsRunning() && nav.Forward()>0 && nav.IntermediateCount()==0);
  REQUIRE_ARENA(Near(nav.Goal().x_MM,250) && Near(nav.Goal().y_MM,950)); // no obstacle map, temporary poses or clearance veto
}
void StoppedGoalCorrectionRequiresFreshStationaryRange() {
  for (bool keepValid:{true,false}) {
    GoalNavigationRoutine nav; auto in=MvpReference(nav);
    for (auto& valid:in.valid) valid=false;
    in.motion.pose={500,700,0}; REQUIRE_ARENA(nav.StartGoal({500,700,0},in));
    for (int tick=0;tick<60 && nav.GetState()!=GoalNavigationRoutine::State::Complete;++tick) {
      in.motion.now+=0.02; in.motion.pose={500,700,0};
      in.valid[0]=keepValid || tick<12; in.rangeMM[0]=448; in.stamp[0]=in.motion.now; nav.Update(in);
      if (nav.GetState()!=GoalNavigationRoutine::State::Complete)
        REQUIRE_ARENA(Near(nav.PositionCorrection().x,0) && Near(nav.PositionCorrection().y,0));
    }
    REQUIRE_ARENA(nav.GetState()==GoalNavigationRoutine::State::Complete);
    REQUIRE_ARENA(Near(nav.PositionCorrection().x,keepValid?-8:0));
  }
}
void MvpManualRouteInput() {
  GoalNavigationRoutine nav; auto in=MvpReference(nav);
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
  for (int source=0;source<navigation_config::GOAL_COUNT;++source)
    for (int destination=0;destination<navigation_config::GOAL_COUNT;++destination) {
      if (source==destination) continue;
      nav.Abort();
      const auto& start=navigation_config::GOALS[source];
      const auto& goal=navigation_config::GOALS[destination];
      in.motion.pose={start.xMM,start.yMM,robot_geometry::Radians(start.headingDeg)};
      REQUIRE_ARENA(nav.StartGoal({goal.xMM,goal.yMM,robot_geometry::Radians(goal.headingDeg)},in,
          GoalNavigationRoutine::DefaultIntermediates(source,destination)));
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
  MvpStartReferenceAndStop(); FlexibleInitializationChecks(); IntermediatePoseToleranceChecks(); ContinuousGoalPoseChecks(); MinimumYawAndStationaryCorrectionChecks();
  ContinuousTransitChecks(); TravelWallSupportChecks();
  BoundedLateSamplesKeepOdometryAndAutoDriveHealthy(); HealthFailuresIdentifyTheirActualSource();
  RangeCorrectionsUseEachBeamHeadingAndMount(); UnusableAndCloseRangesNeverVetoMotion();
  StoppedGoalCorrectionRequiresFreshStationaryRange(); MvpManualRouteInput();
  TapeRoutineChecks(); WaypointSettlingDoesNotExpireProgressWatchdog();
}
}
#ifdef ARENA_TEST_STANDALONE
int main() {
  try { RunActiveChecks(); std::cout<<"PASS: component checks for sensor-supported navigation, valid measurements, late samples, motor output and Teleop isolation\n"; }
  catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
#else
TEST(Project8, ActiveMotionAndSensing) { EXPECT_NO_THROW(RunActiveChecks()); }
#endif
