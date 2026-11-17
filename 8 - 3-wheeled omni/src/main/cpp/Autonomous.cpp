#include "Autonomous.h"
#include <algorithm>
#include <cmath>

namespace at=heading_control_config;
double HeadingMotionControl::Request(double error,double oldYaw,double measuredYaw,double dt,
                                     double deadband,double gain) {
  holdRecheck_=std::max(0.0,holdRecheck_-dt);
  if (holdRecheck_>1e-9 && std::fabs(error)<at::HOLD_RECHECK_WINDOW_RAD) return 0;
  const double toward=(error<0?-1:1)*measuredYaw;
  // Stop applying turning power BEFORE coasting carries us through the
  // reference. A zero correction stays exactly zero through the output path.
  const bool coastingToward=toward>robot_geometry::Radians(at::STATIONARY_YAW_DPS) &&
      toward*(at::HOLD_BRAKE_RESPONSE_S+dt)>=std::fabs(error)-deadband;
  const double limit=at::HOLD_HEADING_MAX_YAW_RADPS;
  const double request=std::max(-limit,std::min(limit,
      error*gain-measuredYaw*at::HOLD_YAW_RATE_DAMPING));
  // Rate damping releases power; it must not command an opposite turn while
  // still on the same side of the reference. Once the error crosses zero,
  // discard the old turn ramp so it cannot push farther past the reference.
  if (std::fabs(error)<=deadband || coastingToward || request*error<=0) {
    // Allow a fresh heading sample before applying another correction.
    if (std::fabs(oldYaw)>0.0001) holdRecheck_=at::HOLD_RECHECK_S;
    return 0;
  }
  if (oldYaw*error<0) oldYaw=0;
  const double step=at::HOLD_YAW_ACCEL_RADPS2*dt;
  return oldYaw+std::max(-step,std::min(step,request-oldYaw));
}
std::string HeadingMotionControl::CheckFeedback(double previousRequest,double measuredYaw,double dt) {
  lastRequest_=previousRequest; lastMeasured_=measuredYaw;
  if (std::fabs(previousRequest)<=0.0001) { direction_=0; opposed_=commandTime_=0; return {}; }
  const int requestedDirection=previousRequest<0?-1:1;
  if (requestedDirection!=direction_) opposed_=commandTime_=0;
  direction_=requestedDirection;
  commandTime_+=dt;
  // Allow motor reversal and IMU acquisition latency before diagnosing a
  // sustained wrong-sign response. Do not count coasting into the fault angle.
  if (commandTime_<=at::YAW_RESPONSE_SETTLE_S+1e-9) return {};
  if (direction_!=0 && std::fabs(measuredYaw)>=at::YAW_DIRECTION_CHECK_MIN_RADPS) {
    if (direction_*measuredYaw<0) opposed_+=std::fabs(measuredYaw)*dt;
    else opposed_=0;
    if (opposed_>=at::YAW_DIRECTION_ERROR_RAD)
      return direction_<0?"Yaw direction mismatch: commanded CW, gyro reported CCW - stopped":
          "Yaw direction mismatch: commanded CCW, gyro reported CW - stopped";
  }
  return {};
}

namespace tt=tape_test_config;
namespace {
bool FreshSample(double now,double stamp,double maximumAge) {
  return std::isfinite(stamp) && stamp>=0 && now>=stamp && now-stamp<=maximumAge;
}
}
void TapeAlignmentRoutine::Enter() {
  *this=TapeAlignmentRoutine{};
}
void TapeAlignmentRoutine::Abort() {
  Stop();
  if (IsRunning() || state_==State::Waiting) {
    state_=State::Aborted; status_="Stopped; return to the reference corner before starting again";
  }
}
void TapeAlignmentRoutine::Fail(const std::string& reason) {
  Stop(); state_=State::Fault; status_=reason;
}
bool TapeAlignmentRoutine::PoseFromWalls(double leftGap,double rearGap,RobotPose& pose) {
  if (!std::isfinite(leftGap) || !std::isfinite(rearGap) ||
      leftGap<localization_config::IR_MIN_RANGE_MM || leftGap>localization_config::IR_MAX_RANGE_MM ||
      rearGap<localization_config::SONAR_MIN_RANGE_MM || rearGap>localization_config::SONAR_MAX_RANGE_MM) return false;
  const double heading=robot_geometry::Radians(tt::START_HEADING_DEG);
  const double theta=robot_geometry::ArenaAngle(heading),c=std::cos(theta),s=std::sin(theta);
  const auto& left=localization_config::RANGE_SENSOR_MOUNTS[0];
  const auto& rear=localization_config::RANGE_SENSOR_MOUNTS[1];
  const double leftRay=std::cos(theta+robot_geometry::Radians(left.beamAngleDeg));
  const double rearRay=std::sin(theta+robot_geometry::Radians(rear.beamAngleDeg));
  // These particular walls require left looking west and rear looking south.
  if (leftRay> -0.98 || rearRay> -0.98) return false;
  pose={tt::LEFT_WALL_X_MM-(c*left.mountX_MM-s*left.mountY_MM)-leftRay*leftGap,
      tt::REAR_WALL_Y_MM-(s*rear.mountX_MM+c*rear.mountY_MM)-rearRay*rearGap,
      robot_geometry::Wrap(heading)};
  return robot_geometry::Finite(pose) && pose.x_MM>=0 && pose.x_MM<=localization_config::ARENA_WIDTH_MM &&
      pose.y_MM>=0 && pose.y_MM<=localization_config::ARENA_HEIGHT_MM;
}
bool TapeAlignmentRoutine::CobraHealthy(const TapeRoutineInput& in) const {
  if (!in.cobraCalibrated || !in.line.communicationValid ||
      !FreshSample(in.motion.now,in.cobraTime,localization_config::SENSOR_STALE_S)) return false;
  for (int i=0;i<4;++i) {
    if (!std::isfinite(in.line.volts[i]) || in.line.volts[i]<0 ||
        in.line.volts[i]>cobra_config::COBRA_REFERENCE_V+0.05 ||
        !std::isfinite(in.blackLevel[i]) || in.blackLevel[i]<0 || in.blackLevel[i]>1) return false;
  }
  return true;
}
bool TapeAlignmentRoutine::Centered(const TapeRoutineInput& in) const {
  const auto& b=in.blackLevel;
  // Equality alone also occurs on all-white/all-black background. Require a
  // dark centre, lighter outside, and agreement of BOTH raw and calibrated data.
  return b[1]>=tt::CENTER_BLACK_MIN && b[2]>=tt::CENTER_BLACK_MIN &&
      std::min(b[1],b[2])-0.5*(b[0]+b[3])>=tt::CENTER_OUTER_CONTRAST &&
      std::fabs(b[1]-b[2])<=tt::CENTER_LEVEL_TOLERANCE &&
      std::fabs(in.line.volts[1]-in.line.volts[2])<=tt::CENTER_VOLTAGE_TOLERANCE_V;
}
void TapeAlignmentRoutine::Begin(const TapeRoutineInput& in) {
  if (in.approach!=TapeApproach::FromLeft && in.approach!=TapeApproach::FromRight) {
    status_="Choose a tape approach"; return;
  }
  approach_=in.approach;
  Stop(); heading_.Reset(); state_=State::Reference; stateTime_=in.motion.now;
  scanTravel_=0; settleTime_=-1;
  status_=std::string(ApproachName())+"reading left IR and rear sonar";
}
void TapeAlignmentRoutine::SampleReference(const TapeRoutineInput& in) {
  // Independent sensors update at different rates. Use their latest fresh,
  // valid readings once; no gyro stillness gate, pairing window, sample-count
  // requirement or reference timeout. Motors remain stopped while waiting.
  if (!in.leftValid || !FreshSample(in.motion.now,in.leftTime,localization_config::SENSOR_STALE_S) ||
      !std::isfinite(in.leftRangeMM) || in.leftRangeMM<localization_config::IR_MIN_RANGE_MM ||
      in.leftRangeMM>localization_config::IR_MAX_RANGE_MM) {
    status_="Waiting for left IR"; return;
  }
  if (!in.rearValid || !FreshSample(in.motion.now,in.rearTime,localization_config::SONAR_READING_STALE_S) ||
      !std::isfinite(in.rearRangeMM) || in.rearRangeMM<localization_config::SONAR_MIN_RANGE_MM ||
      in.rearRangeMM>localization_config::SONAR_MAX_RANGE_MM) {
    status_="Waiting for rear sonar"; return;
  }
  if (!PoseFromWalls(in.leftRangeMM,in.rearRangeMM,referencePose_)) {
    status_="Reference position is outside the arena"; return;
  }
  state_=State::ReferenceReady; status_="Starting pose ready";
}
void TapeAlignmentRoutine::ConfirmPoseReset(double now) {
  if (!NeedsPoseReset()) return;
  Stop(); heading_.Reset(); state_=State::DriveToPoint; stateTime_=now;
  point_.Reset(GetTarget(),now,tt::DRIVE_SPEED_MMPS,tt::DRIVE_ACCEL_MMPS2); motionEffortTime_=0;
  progressPose_=referencePose_; settleTime_=-1;
  status_=std::string(ApproachName())+"driving to approach waypoint; holding starting heading";
}
void TapeAlignmentRoutine::LimitTranslation(double oldForward,double oldLeft,double acceleration,double dt) {
  const double df=forward_-oldForward,dl=left_-oldLeft,delta=std::hypot(df,dl),step=acceleration*dt;
  if (delta>step) { forward_=oldForward+df*step/delta; left_=oldLeft+dl*step/delta; }
}
void WaypointDrive::Reset(ArenaPoint target,double now,double speed,double acceleration,
                         double initialForward,double initialLeft) {
  *this=WaypointDrive{}; target_=target; phaseTime_=now;
  maxSpeed_=speed; acceleration_=acceleration;
  forward_=filteredForward_=initialForward; left_=filteredLeft_=initialLeft;
}
void WaypointDrive::LimitTranslation(double oldForward,double oldLeft,double acceleration,double dt) {
  const double df=forward_-oldForward,dl=left_-oldLeft,delta=std::hypot(df,dl),step=acceleration*dt;
  if (delta>step) { forward_=oldForward+df*step/delta; left_=oldLeft+dl*step/delta; }
}
void WaypointDrive::Update(const MotionInput& m,double measuredForward,double measuredLeft,double headingError) {
  const double oldForward=forward_,oldLeft=left_;
  forward_=left_=0;
  if (reached_) return;
  const double velocityGain=m.dt/(tt::VELOCITY_FILTER_S+m.dt);
  filteredForward_+=velocityGain*(measuredForward-filteredForward_);
  filteredLeft_+=velocityGain*(measuredLeft-filteredLeft_);
  const auto target=target_;
  const double captureRadius=0.7*tt::POSITION_TOLERANCE_MM;
  const double dx=target.x-m.pose.x_MM,dy=target.y-m.pose.y_MM,distance=std::hypot(dx,dy);
  const double angle=robot_geometry::ArenaAngle(m.pose.headingRad);
  const double c=std::cos(angle),s=std::sin(angle);
  const double forwardError=c*dx+s*dy,leftError=-s*dx+c*dy;
  const double measuredToward=distance>0?std::max(0.0,
      (filteredForward_*forwardError+filteredLeft_*leftError)/distance):0;
  const double toward=distance>0?std::max(0.0,std::max(
      (filteredForward_*forwardError+filteredLeft_*leftError)/distance,
      (oldForward*forwardError+oldLeft*leftError)/distance)):0;
  const double stoppingDistance=toward*toward/(2*acceleration_)+toward*(tt::BRAKE_RESPONSE_S+m.dt);
  if (waypointPhase_==WaypointPhase::Travel &&
      (distance<=tt::NEAR_TARGET_MM || stoppingDistance>=distance-captureRadius)) {
    waypointPhase_=WaypointPhase::Brake; settleTime_=-1;
  }
  if (waypointPhase_==WaypointPhase::Correct) {
    const double along=dx*correctionX_+dy*correctionY_;
    if (along<=captureRadius || distance<=captureRadius ||
        (measuredToward>heading_control_config::STATIONARY_SPEED_MMPS && stoppingDistance>=along-captureRadius) ||
        m.now-phaseTime_>=tt::CORRECTION_MAX_S) {
      waypointPhase_=WaypointPhase::Brake; settleTime_=-1;
    }
  }
  if (waypointPhase_==WaypointPhase::Brake) {
    // The chassis request decelerates to a true zero, which bypasses the
    // electrical minimum duty. No reversal is blended through a target.
    LimitTranslation(oldForward,oldLeft,acceleration_,m.dt);
    status_="Braking before waypoint; waiting for the chassis to settle";
    if (std::hypot(forward_,left_)<0.01) {
      forward_=left_=0; waypointPhase_=WaypointPhase::Settle; settleTime_=-1;
    }
    return;
  }
  if (waypointPhase_==WaypointPhase::Settle) {
    status_="Stopped at waypoint: verifying XY and heading";
    // Stop translation before remeasuring/reversing, but do not wait for yaw:
    // XY correction and heading hold may command the wheel mixer together.
    if (std::hypot(measuredForward,measuredLeft)>heading_control_config::STATIONARY_SPEED_MMPS) {
      settleTime_=-1; return;
    }
    if (settleTime_<0) settleTime_=m.now;
    if (distance<=tt::POSITION_TOLERANCE_MM) {
      // Heading gates completion only, never an outstanding XY correction.
      if (std::fabs(headingError)>heading_control_config::HEADING_TOLERANCE_RAD) {
        settleTime_=-1; return;
      }
      if (m.now-settleTime_>=tt::WAYPOINT_SETTLE_S) {
        reached_=true;
      }
      return;
    }
    if (m.now-settleTime_<tt::CORRECTION_REST_S) return;
    waypointPhase_=WaypointPhase::Correct; phaseTime_=m.now; settleTime_=-1;
    correctionX_=dx/distance; correctionY_=dy/distance;
  }
  if (waypointPhase_==WaypointPhase::Correct) {
    forward_=(c*correctionX_+s*correctionY_)*tt::MIN_APPROACH_MMPS;
    left_=(-s*correctionX_+c*correctionY_)*tt::MIN_APPROACH_MMPS;
    LimitTranslation(oldForward,oldLeft,acceleration_,m.dt);
    status_="Bounded waypoint correction; stop and remeasure before reversing";
    return;
  }
  const double brakingSpeed=std::sqrt(2*acceleration_*
      std::max(0.0,distance-captureRadius));
  const double speed=std::min(maxSpeed_,std::max(tt::MIN_APPROACH_MMPS,
      std::min(distance*tt::POSITION_KP,brakingSpeed)));
  forward_=forwardError*speed/distance; left_=leftError*speed/distance;
  LimitTranslation(oldForward,oldLeft,acceleration_,m.dt);
  status_="driving to waypoint with heading hold and predictive braking";
}
void TapeAlignmentRoutine::DriveWaypoint(const TapeRoutineInput& in,double headingError) {
  point_.Update(in.motion,in.bodyForwardMMps,in.bodyLeftMMps,headingError);
  forward_=point_.Forward(); left_=point_.Left(); status_=point_.Status();
  if (point_.Reached()) {
    Stop(); heading_.Reset(); scanStart_=progressPose_=in.motion.pose;
    scanTime_=in.motion.now; motionEffortTime_=0; scanTravel_=0; lostTime_=-1; settleTime_=-1;
    state_=State::SearchTape;
    status_=std::string(ApproachName())+"at waypoint; searching inward for tape";
  }
}
void TapeAlignmentRoutine::Update(const TapeRoutineInput& in) {
  const auto& m=in.motion;
  const double oldForward=forward_,oldLeft=left_,oldYaw=yaw_;
  Stop();
  if (m.stopRequested) { Abort(); return; }
  if (!IsRunning() && !m.startRequested) return;
  if (!robot_geometry::Finite(m.pose) || !std::isfinite(m.now) || !std::isfinite(m.dt) ||
      !std::isfinite(m.yawRateRadps) || !std::isfinite(in.bodyForwardMMps) || !std::isfinite(in.bodyLeftMMps) ||
      m.dt<=0 || m.dt>localization_config::MAX_CONTROL_GAP_S || !m.healthy) {
    Fail("Unhealthy IMU/odometry, disconnected Control Station or stale timing - stopped"); return;
  }
  if (!IsRunning()) { Begin(in); return; }
  if (state_==State::Reference) { SampleReference(in); return; }
  if (state_==State::ReferenceReady) return; // wait for the explicit pose-reset handshake
  const auto yawFault=heading_.CheckFeedback(oldYaw,m.yawRateRadps,m.dt);
  if (!yawFault.empty()) { Fail(yawFault); return; }
  const double headingError=robot_geometry::Wrap(referencePose_.headingRad-m.pose.headingRad);
  yaw_=heading_.Request(headingError,oldYaw,m.yawRateRadps,m.dt);
  if (state_==State::DriveToPoint) {
    if (m.now-stateTime_>tt::DRIVE_TIMEOUT_S) { Fail(std::string(ApproachName())+"drive to waypoint timed out - stopped"); return; }
    DriveWaypoint(in,headingError);
  } else {
    if (!CobraHealthy(in)) {
      Stop(); heading_.Reset(); scanTime_+=m.dt; settleTime_=lostTime_=-1;
      status_="Waiting for Cobra reading"; return;
    }
    scanTravel_=std::hypot(m.pose.x_MM-scanStart_.x_MM,m.pose.y_MM-scanStart_.y_MM);
    if (scanTravel_>tt::SCAN_MAX_TRAVEL_MM || m.now-scanTime_>tt::SCAN_TIMEOUT_S) {
      Fail("Tape scan limit reached without a stable CH1/CH2 match - stopped"); return;
    }
    if (Centered(in)) {
      if (state_!=State::VerifyTape) { state_=State::VerifyTape; settleTime_=-1; }
      status_="CH1/CH2 match on tape: stopped, verifying alignment";
      if (std::hypot(in.bodyForwardMMps,in.bodyLeftMMps)<=heading_control_config::STATIONARY_SPEED_MMPS &&
          std::fabs(headingError)<=heading_control_config::HEADING_TOLERANCE_RAD) {
        if (settleTime_<0) settleTime_=m.now;
        if (m.now-settleTime_>=tt::TAPE_SETTLE_S) {
          Stop(); state_=State::Complete;
          status_=std::string(ApproachName())+"complete: reference -> waypoint -> tape centred on CH1/CH2";
        }
      } else settleTime_=-1;
      return;
    }
    if (state_==State::VerifyTape) {
      state_=State::CenterTape; settleTime_=-1;
    }
    const double peak=std::max(in.blackLevel[1],in.blackLevel[2]);
    if (peak>=tt::TAPE_ACQUIRE_LEVEL) {
      state_=State::CenterTape; lostTime_=-1;
    } else if (state_==State::CenterTape) {
      if (lostTime_<0) lostTime_=m.now;
      if (m.now-lostTime_>=tt::TAPE_LOST_S) { state_=State::SearchTape; lostTime_=-1; }
    }
    if (state_==State::SearchTape) {
      // At +Y heading, robot-right increases arena X; robot-left decreases X.
      left_=(approach_==TapeApproach::FromLeft?-1:1)*tt::SCAN_SPEED_MMPS;
      status_=std::string(ApproachName())+"scanning inward; equal background channels are ignored";
    } else {
      const double error=in.blackLevel[1]-in.blackLevel[2];
      // CH0 is leftmost: more black on CH1 means tape is left of centre.
      const double magnitude=std::min(tt::SCAN_SPEED_MMPS,
          std::max(tt::CENTER_MIN_SPEED_MMPS,std::fabs(error)*tt::CENTER_KP_MMPS));
      left_=error>0?magnitude:-magnitude;
      status_="Centring on tape using CH1/CH2; holding starting heading";
    }
    LimitTranslation(oldForward,oldLeft,tt::SCAN_ACCEL_MMPS2,m.dt);
  }
  if (std::hypot(m.pose.x_MM-progressPose_.x_MM,m.pose.y_MM-progressPose_.y_MM)>=tt::PROGRESS_MM) {
    progressPose_=m.pose; motionEffortTime_=0;
  } else if (std::hypot(forward_,left_)>20) {
    // Count only time actually requesting movement. Settling/braking pauses
    // must not make the next small correction fail immediately.
    motionEffortTime_+=m.dt;
    if (motionEffortTime_>tt::NO_PROGRESS_TIMEOUT_S)
      Fail(state_==State::DriveToPoint?"Waypoint correction stalled: check wheel/encoder response":
          "Tape scan stalled: check wheel/encoder response or configured scan speed");
  }
}
