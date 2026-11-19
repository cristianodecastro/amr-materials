#pragma once

#include <cmath>
#include <algorithm>
#include "RobotConfig.h"

namespace motor_output {
// Titan's CAN command is a whole percentage. Raw capped quantization.
inline constexpr int TitanPercent(double output,double cap) {
  if (!std::isfinite(output) || !std::isfinite(cap) || cap<=0 || cap>1) return 0;
  const int maximum=static_cast<int>(std::floor(cap*100+1e-9));
  const int wanted=static_cast<int>(std::round(std::min(cap,std::fabs(output))*100));
  const int percent=std::min(maximum,wanted);
  return output<0?-percent:percent;
}
// One electrical boundary for Teleop and Autonomous: preserve true zero,
// enforce the minimum running duty, then apply wiring polarity. The cap wins:
// if it cannot supply the minimum, stop instead of sending an unusable duty.
inline constexpr int WheelTitanPercent(int wheel,double output,double cap) {
  if (wheel<0 || wheel>=localization_config::WHEEL_COUNT) return 0;
  if (!std::isfinite(output) || output==0 || !std::isfinite(cap) || cap<=0 || cap>1) return 0;
  const int minimum=static_cast<int>(std::ceil(wheel_drive_config::MIN_RUNNING_OUTPUT*100-1e-9));
  const int maximum=static_cast<int>(std::floor(cap*100+1e-9));
  if (maximum<minimum) return 0;
  const double magnitude=std::max(std::fabs(output),minimum/100.0);
  return TitanPercent((output<0?-magnitude:magnitude)*localization_config::WHEEL_MOTOR_SIGN[wheel],cap);
}
inline double TitanSetArgument(int percent) {
  if (percent==0) return 0;
  const double magnitude=std::nextafter(std::fabs(percent)/100.0,1.0);
  return percent<0?-magnitude:magnitude;
}
// Autonomous low-speed vectors must keep their wheel ratios where possible:
// e.g. lateral travel needs 15/30/15, not three independent 15% floors. Bound
// the common increase to 30% peak so a nearly cancelled wheel cannot boost a
// larger command all the way to the electrical cap. Teleop does not call this.
inline void PrepareAutonomousVector(double (&output)[3],double cap) {
  double smallest=1,peak=0;
  for (double x:output) {
    if (!std::isfinite(x)) { for (auto& v:output) v=0; return; }
    if (std::fabs(x)>0) smallest=std::min(smallest,std::fabs(x));
    peak=std::max(peak,std::fabs(x));
  }
  if (peak==0 || !std::isfinite(cap) || cap<=0) return;
  const double floor=wheel_drive_config::MIN_RUNNING_OUTPUT;
  const double allowed=std::min(cap,std::max(peak,wheel_drive_config::VECTOR_PEAK_FLOOR_MULTIPLIER*floor));
  const double scale=std::max(1.0,std::min(floor/smallest,allowed/peak));
  for (auto& x:output) x*=scale;
}
}

namespace drive_kinematics {
// All inputs use body +X forward, +Y left, positive yaw CCW. Angles below
// are wheel CENTRE positions; velocity is projected onto each CCW tangent.
inline constexpr double WheelSpeedMMps(int wheel,double forward,double left,double yawRate) {
  const double a=localization_config::WHEEL_ANGLE_DEG[wheel]*robot_geometry::PI/180;
  return -std::sin(a)*forward+std::cos(a)*left+
      localization_config::WHEEL_CENTER_RADIUS_MM[wheel]*yawRate;
}
// Subtract gyro rotation before solving the same tangent projections for XY.
inline bool BodyDisplacement(const double (&wheelMM)[3],double gyroTurn,double& forward,double& left) {
  double xx=0,xy=0,yy=0,bx=0,by=0;
  for (int i=0;i<3;++i) {
    const double a=localization_config::WHEEL_ANGLE_DEG[i]*robot_geometry::PI/180;
    const double x=-std::sin(a),y=std::cos(a);
    const double d=wheelMM[i]-localization_config::WHEEL_CENTER_RADIUS_MM[i]*gyroTurn;
    xx+=x*x; xy+=x*y; yy+=y*y; bx+=x*d; by+=y*d;
  }
  const double determinant=xx*yy-xy*xy;
  if (!std::isfinite(determinant) || std::fabs(determinant)<1e-8) return false;
  forward=(yy*bx-xy*by)/determinant;
  left=(xx*by-xy*bx)/determinant;
  return std::isfinite(forward) && std::isfinite(left);
}
// Count-increment odometry: displacement never depends on an assumed update
// frequency. Actual dt is used only for velocity display and sample checks.
class EncoderGyroOdometry {
 public:
  double HeadingOffsetRad() const { return offset_; }
  void Reset(RobotPose pose,const double (&counts)[3],double yawDeg) {
    pose_=pose; pose_.headingRad=robot_geometry::Wrap(pose.headingRad);
    offset_=robot_geometry::Wrap(pose_.headingRad-robot_geometry::Radians(yawDeg));
    forwardSpeed_=leftSpeed_=0;
    healthy_=robot_geometry::Finite(pose_) && std::isfinite(yawDeg);
    for (int i=0;i<3;++i) { previous_[i]=counts[i]; healthy_=healthy_ && std::isfinite(counts[i]); }
    faultReason_=healthy_?"OK":"Invalid odometry reset";
  }
  bool Update(const double (&counts)[3],double yawDeg,double gyroRateDps,double dt) {
    healthy_=false;
    forwardSpeed_=leftSpeed_=0;
    if (!std::isfinite(dt) || dt<=0) return Reject("Invalid odometry interval");
    if (dt>localization_config::AUTO_MAX_SAMPLE_GAP_S) return Reject("Odometry sample gap exceeds configured limit");
    if (!std::isfinite(yawDeg) || !std::isfinite(gyroRateDps)) return Reject("IMU yaw/rate sample is not finite");
    double wheel[3];
    for (int i=0;i<3;++i) {
      wheel[i]=(counts[i]-previous_[i])*localization_config::WHEEL_ENCODER_SIGN[i]*
          localization_config::ENCODER_MM_PER_COUNT;
      if (!std::isfinite(wheel[i])) return Reject("Encoder sample is not finite");
      if (std::fabs(wheel[i])/dt>wheel_drive_config::MAX_PLAUSIBLE_SPEED_FACTOR*localization_config::MAX_WHEEL_SPEED_MMPS) return Reject("Encoder jump/out-of-range speed");
    }
    const double heading=robot_geometry::Wrap(robot_geometry::Radians(yawDeg)+offset_);
    const double turn=robot_geometry::Wrap(heading-pose_.headingRad);
    double forward=0,left=0;
    if (!BodyDisplacement(wheel,turn,forward,left)) return Reject("Invalid wheel geometry/displacement");
    const double midpoint=robot_geometry::ArenaAngle(pose_.headingRad+turn/2);
    // Exact constant-twist arc integration; sinc tends to one for straight travel.
    const double arc=std::fabs(turn)<1e-8?1.0:std::sin(turn/2)/(turn/2);
    pose_.x_MM+=arc*(std::cos(midpoint)*forward-std::sin(midpoint)*left);
    pose_.y_MM+=arc*(std::sin(midpoint)*forward+std::cos(midpoint)*left);
    pose_.headingRad=heading;
    for (int i=0;i<3;++i) previous_[i]=counts[i];
    forwardSpeed_=forward/dt; leftSpeed_=left/dt;
    healthy_=robot_geometry::Finite(pose_);
    faultReason_=healthy_?"OK":"Odometry pose is not finite";
    return healthy_;
  }
  void CorrectPosition(double dx,double dy) {
    if (std::isfinite(dx) && std::isfinite(dy)) { pose_.x_MM+=dx; pose_.y_MM+=dy; }
  }
  const RobotPose& Pose() const { return pose_; }
  bool Healthy() const { return healthy_; }
  const char* FaultReason() const { return faultReason_; }
  double ForwardSpeed() const { return forwardSpeed_; }
  double LeftSpeed() const { return leftSpeed_; }
 private:
  bool Reject(const char* reason) { faultReason_=reason; return false; }
  RobotPose pose_{0,0,0};
  double previous_[3]={},offset_=0,forwardSpeed_=0,leftSpeed_=0;
  bool healthy_=false;
  const char* faultReason_="Not initialized";
};

// Preserve the pre-revision manual pose display. This estimate is observational:
// Teleop steering never consumes it, and Autonomous never reads this state.
class TeleopPoseTelemetry {
 public:
  double HeadingOffsetRad() const { return offset_; }
  void SetPosition(double x,double y) { pose_.x_MM=x; pose_.y_MM=y; }
  void Reset(RobotPose pose,const double (&counts)[3],double yawDeg,double ax,double ay) {
    pose_=pose; offset_=robot_geometry::Wrap(pose.headingRad-robot_geometry::Radians(yawDeg));
    worldX_=worldY_=forwardSpeed_=leftSpeed_=0;
    const bool accel=ValidAcceleration(ax,ay);
    biasX_=accel?ax:0; biasY_=accel?ay:0;
    healthy_=robot_geometry::Finite(pose) && std::isfinite(yawDeg);
    for (int i=0;i<3;++i) { previous_[i]=counts[i]; healthy_=healthy_ && std::isfinite(counts[i]); }
  }
  bool Update(const double (&counts)[3],double yawDeg,double rateDps,double ax,double ay,double dt) {
    namespace c=teleop_odometry_config;
    healthy_=false;
    if (!std::isfinite(dt) || dt<=0 || dt>localization_config::MAX_CONTROL_GAP_S ||
        !std::isfinite(yawDeg) || !std::isfinite(rateDps)) return false;
    double wheel[3],maximum=0;
    for (int i=0;i<3;++i) {
      wheel[i]=(counts[i]-previous_[i])*localization_config::WHEEL_ENCODER_SIGN[i]*localization_config::ENCODER_MM_PER_COUNT;
      const double speed=std::fabs(wheel[i])/dt;
      if (!std::isfinite(speed) || speed>wheel_drive_config::MAX_PLAUSIBLE_SPEED_FACTOR*localization_config::MAX_WHEEL_SPEED_MMPS) return false;
      maximum=std::max(maximum,speed);
    }
    double forward=0,left=0;
    if (!BodyDisplacement(wheel,0,forward,left)) return false;
    const double oldHeading=pose_.headingRad;
    const double prediction=robot_geometry::Wrap(oldHeading+robot_geometry::Radians(rateDps)*dt);
    const double yaw=robot_geometry::Wrap(robot_geometry::Radians(yawDeg)+offset_);
    pose_.headingRad=robot_geometry::Wrap(prediction+c::YAW_AHRS_CORRECTION*robot_geometry::Wrap(yaw-prediction));
    const bool stationary=maximum<=c::ENCODER_STATIONARY_MMPS && std::fabs(rateDps)<=c::GYRO_STATIONARY_DPS;
    const bool accel=ValidAcceleration(ax,ay);
    double dx=accel?ax-biasX_:0,dy=accel?ay-biasY_:0;
    if (stationary && accel) {
      const double gain=dt/(c::ACCEL_BIAS_TRACK_TAU_S+dt);
      biasX_+=gain*dx; biasY_+=gain*dy; dx=ax-biasX_; dy=ay-biasY_;
    }
    const double accelX=std::fabs(dx)<c::ACCEL_DEADBAND_G?0:dx*c::GRAVITY_MM_PER_S2;
    const double accelY=std::fabs(dy)<c::ACCEL_DEADBAND_G?0:dy*c::GRAVITY_MM_PER_S2;
    const double accelAngle=robot_geometry::ArenaAngle(offset_);
    const double predictedX=worldX_+(std::cos(accelAngle)*accelX-std::sin(accelAngle)*accelY)*dt;
    const double predictedY=worldY_+(std::sin(accelAngle)*accelX+std::cos(accelAngle)*accelY)*dt;
    const double midpoint=robot_geometry::ArenaAngle(oldHeading+0.5*robot_geometry::Wrap(pose_.headingRad-oldHeading));
    const double encoderX=(std::cos(midpoint)*forward-std::sin(midpoint)*left)/dt;
    const double encoderY=(std::sin(midpoint)*forward+std::cos(midpoint)*left)/dt;
    const double weight=accel?std::pow(c::ACCEL_VELOCITY_BLEND,dt/localization_config::CONTROL_PERIOD_S):0;
    double correctionX=weight*(predictedX-encoderX),correctionY=weight*(predictedY-encoderY);
    const double correction=std::hypot(correctionX,correctionY);
    if (correction>c::ACCEL_MAX_VELOCITY_CORRECTION_MMPS) {
      correctionX*=c::ACCEL_MAX_VELOCITY_CORRECTION_MMPS/correction;
      correctionY*=c::ACCEL_MAX_VELOCITY_CORRECTION_MMPS/correction;
    }
    worldX_=stationary?0:encoderX+correctionX; worldY_=stationary?0:encoderY+correctionY;
    pose_.x_MM+=worldX_*dt; pose_.y_MM+=worldY_*dt;
    const double angle=robot_geometry::ArenaAngle(pose_.headingRad);
    forwardSpeed_=std::cos(angle)*worldX_+std::sin(angle)*worldY_;
    leftSpeed_=-std::sin(angle)*worldX_+std::cos(angle)*worldY_;
    for (int i=0;i<3;++i) previous_[i]=counts[i];
    healthy_=robot_geometry::Finite(pose_); return healthy_;
  }
  const RobotPose& Pose() const { return pose_; }
  bool Healthy() const { return healthy_; }
  double ForwardSpeed() const { return forwardSpeed_; }
  double LeftSpeed() const { return leftSpeed_; }
 private:
  static bool ValidAcceleration(double x,double y) {
    return std::isfinite(x) && std::isfinite(y) && std::hypot(x,y)<=teleop_odometry_config::ACCEL_MAX_PLAUSIBLE_G;
  }
  RobotPose pose_{0,0,0};
  double previous_[3]={},offset_=0,worldX_=0,worldY_=0,biasX_=0,biasY_=0,forwardSpeed_=0,leftSpeed_=0;
  bool healthy_=false;
};

// All wheel requests share one denominator, preserving the combined
// translation/rotation vector when any motor saturates. Zero rejects input.
inline constexpr double OutputDenominator(const double (&wheelSpeed)[3],
                                           double maxOutput) {
  if (!std::isfinite(maxOutput) || maxOutput <= 0.0 ||
      !std::isfinite(localization_config::MAX_WHEEL_SPEED_MMPS) ||
      localization_config::MAX_WHEEL_SPEED_MMPS <= 0.0) return 0.0;
  if (maxOutput > 1.0) maxOutput = 1.0;
  double largest = localization_config::MAX_WHEEL_SPEED_MMPS;
  for (int wheel = 0; wheel < 3; ++wheel) {
    if (!std::isfinite(wheelSpeed[wheel])) return 0.0;
    const double required = std::fabs(wheelSpeed[wheel]) / maxOutput;
    if (required > largest) largest = required;
  }
  return largest;
}

}  // namespace drive_kinematics

// Driver translation and rotation requests.
// Pure manual-drive logic, shared by the robot and compile-time regression
// checks. Inputs are Driver Station values; outputs are speed requests.
class TeleopDriveController {
 public:
  constexpr void Update(bool motionAllowed, bool connected, double rawX,
                        double rawY, double rawRightX, double dt) {
    if (!connected) {
      Stop("Controller inputs missing - check connection/axes", true);
      return;
    }
    if (!Finite(rawX) || !Finite(rawY) || !Finite(rawRightX) ||
        Abs(rawX) > localization_config::DRIVER_MAX_VALID_AXIS || Abs(rawY) > localization_config::DRIVER_MAX_VALID_AXIS || Abs(rawRightX) > localization_config::DRIVER_MAX_VALID_AXIS ||
        !Finite(dt) || dt < 0.0) {
      Stop("Invalid controller input or timing", true);
      return;
    }
    if (!motionAllowed) {
      Stop("Disabled - select and enable Teleoperated mode", true);
      return;
    }
    const double deadband = localization_config::DRIVER_STICK_DEADBAND;
    if (!(deadband >= 0.0 && deadband < 1.0) ||
        !Positive(localization_config::TELEOP_SPEED_MMPS) ||
        !Positive(localization_config::TELEOP_ACCELERATION_MMPS2) ||
        !Positive(localization_config::TELEOP_MAX_MOTOR_OUTPUT) ||
        localization_config::TELEOP_MAX_MOTOR_OUTPUT > 1.0 ||
        !Positive(localization_config::TELEOP_MAX_YAW_RATE_DPS) ||
        !Positive(localization_config::TELEOP_YAW_ACCELERATION_DPS2) ||
        Abs(localization_config::TELEOP_YAW_SIGN) != 1.0 ||
        Abs(localization_config::DRIVER_RIGHT_X_SIGN) != 1.0 ||
        Abs(localization_config::DRIVER_LEFT_X_SIGN) != 1.0 ||
        Abs(localization_config::DRIVER_LEFT_Y_SIGN) != 1.0) {
      Stop("Invalid teleop configuration", true);
      return;
    }
    const double rawMagnitude = Max(Abs(rawX), Abs(rawY));
    if (!armed_) {
      if (rawMagnitude > deadband || Abs(rawRightX) > deadband) {
        Stop("Center both sticks to arm", true);
        return;
      }
      armed_ = true;
    }
    const double x=Abs(rawX)<=deadband?0:rawX;
    const double y=Abs(rawY)<=deadband?0:rawY;
    const double magnitude=std::hypot(x,y);
    const double seconds = Clamp(dt, 0.0, localization_config::TELEOP_MAX_CONTROL_STEP_S);
    if (magnitude <= deadband) {
      // Neutral translation stops immediately; right-stick rotation is independent.
      forward_MMps_ = 0.0;
      left_MMps_ = 0.0;
    } else {
      const double speed = (Min(1.0, magnitude) - deadband) /
          (1.0 - deadband) * localization_config::TELEOP_SPEED_MMPS;
      const double targetForward = y *
          localization_config::DRIVER_LEFT_Y_SIGN / magnitude * speed;
      const double targetLeft = -x *
          localization_config::DRIVER_LEFT_X_SIGN / magnitude * speed;
      // Limit total vector acceleration, so diagonal starts and direction
      // changes have the same acceleration limit as straight movement.
      const double deltaForward=targetForward-forward_MMps_;
      const double deltaLeft=targetLeft-left_MMps_;
      const double delta=std::hypot(deltaForward,deltaLeft);
      const double step = localization_config::TELEOP_ACCELERATION_MMPS2 * seconds;
      const double fraction=delta>step && delta>0.0?step/delta:1.0;
      forward_MMps_+=deltaForward*fraction;
      left_MMps_+=deltaLeft*fraction;
    }
    // Right stick right = CW, left = CCW. Deflection controls yaw speed.
    const double rotation = Abs(rawRightX)<=deadband?0.0:
        -localization_config::DRIVER_RIGHT_X_SIGN*(rawRightX<0?-1.0:1.0)*
        (Min(1.0,Abs(rawRightX))-deadband)/(1.0-deadband);
    if (rotation == 0.0) {
      yawRate_Radps_ = 0.0;
    } else {
      const double targetYaw = rotation * localization_config::TELEOP_YAW_SIGN *
          localization_config::TELEOP_MAX_YAW_RATE_DPS *
          robot_geometry::PI / 180.0;
      const double step = localization_config::TELEOP_YAW_ACCELERATION_DPS2 *
          robot_geometry::PI / 180.0 * seconds;
      yawRate_Radps_=Approach(yawRate_Radps_,targetYaw,step);
    }
    const bool translating = magnitude > deadband;
    status_ = rotation != 0
        ? (translating ? "Driving and rotating" : "Rotating - right stick")
        : (translating ? "Driving - left stick translation"
                       : "Ready - left stick drive / right stick rotate");
  }

  constexpr bool IsArmed() const { return armed_; }
  constexpr double GetForwardMMps() const { return forward_MMps_; }
  constexpr double GetLeftMMps() const { return left_MMps_; }
  constexpr double GetYawRateRadps() const { return yawRate_Radps_; }
  constexpr const char* GetStatus() const { return status_; }

 private:
  static constexpr double Abs(double x) { return x < 0.0 ? -x : x; }
  static constexpr double Min(double a, double b) { return a < b ? a : b; }
  static constexpr double Max(double a, double b) { return a > b ? a : b; }
  static constexpr double Clamp(double x, double low, double high) {
    return Min(high, Max(low, x));
  }
  static constexpr double Approach(double value,double target,double step) {
    return Abs(target-value)<=step?target:value+Clamp(target-value,-step,step);
  }
  static constexpr bool Finite(double x) {
    return std::isfinite(x) && x >= -1.0e15 && x <= 1.0e15;
  }
  static constexpr bool Positive(double x) { return Finite(x) && x > 0.0; }
  constexpr void Stop(const char* status, bool disarm) {
    forward_MMps_ = left_MMps_ = yawRate_Radps_ = 0.0;
    if (disarm) armed_ = false;
    status_ = status;
  }

  bool armed_ = false;
  double forward_MMps_ = 0.0;
  double left_MMps_ = 0.0;
  double yawRate_Radps_ = 0.0;
  const char* status_ = "Waiting for Control Station";
};

enum class WheelDriveContext { Autonomous, Teleop };
// Both modes use the same proportional mixer; only the chassis request is ramped.
class WheelDriveController {
 public:
  constexpr void UpdateAutonomous(const double (&desired)[3],const double (&counts)[3],double dt,double maxOutput) {
    // Position integration already used the real elapsed interval. After a
    // bounded late sample, rebaseline only this speed/direction monitor so
    // accumulated counts are not divided by a fictitious 20 ms interval.
    if (!fault_ && Finite(dt) && dt>wheel_drive_config::MAX_SAMPLE_S && dt<=localization_config::AUTO_MAX_SAMPLE_GAP_S) {
      Reset(); Update(desired,counts,localization_config::CONTROL_PERIOD_S,maxOutput);
    } else Update(desired,counts,dt,maxOutput);
  }
  constexpr void Reset() { *this=WheelDriveController{}; }
  constexpr void Update(const double (&desired)[3],const double (&counts)[3],double dt,double maxOutput) {
    if (fault_) return;
    if (!Finite(dt) || dt<=0 || dt>wheel_drive_config::MAX_SAMPLE_S ||
        !Finite(maxOutput) || maxOutput<=0 || maxOutput>1) {
      Fault("Invalid drive timing/output limit",-1); return;
    }
    double peak=0;
    for (int i=0;i<3;++i) {
      if (!Finite(desired[i]) || !Finite(counts[i])) { Fault("Invalid encoder or wheel request",i); return; }
      if (Abs(desired[i])>peak) peak=Abs(desired[i]);
    }
    if (peak<=wheel_drive_config::ZERO_SPEED_MMPS) { Reset(); return; }
    const double denominator=drive_kinematics::OutputDenominator(desired,maxOutput);
    if (!(denominator>0)) { Fault("Invalid wheel normalization",-1); return; }

    const double scale=localization_config::MAX_WHEEL_SPEED_MMPS/denominator;
    sampleElapsed_+=dt;
    const bool observe=baseline_ && sampleElapsed_+1e-9>=wheel_drive_config::SPEED_WINDOW_S;
    const double alpha=sampleElapsed_/(wheel_drive_config::SPEED_FILTER_S+sampleElapsed_);
    double wanted[3]={};
    for (int i=0;i<3;++i) {
      if (!baseline_) previousCounts_[i]=counts[i];
      if (observe) {
        const double measured=(counts[i]-previousCounts_[i])*localization_config::WHEEL_ENCODER_SIGN[i]*
            localization_config::ENCODER_MM_PER_COUNT/sampleElapsed_;
        if (!Finite(measured) || Abs(measured)>wheel_drive_config::MAX_PLAUSIBLE_SPEED_FACTOR*localization_config::MAX_WHEEL_SPEED_MMPS) {
          Fault("Encoder jump/out-of-range speed",i); return;
        }
        measured_[i]+=alpha*(measured-measured_[i]); previousCounts_[i]=counts[i];
      }
      // Suppress numerical tangent residuals for deliberately stationary wheels.
      target_[i]=Abs(desired[i])<=wheel_drive_config::ZERO_SPEED_MMPS?0:desired[i]*scale;
      wanted[i]=target_[i]==0?0:desired[i]/denominator;
    }
    if (!baseline_ || observe) sampleElapsed_=0;
    baseline_=true;
    for (int i=0;i<3;++i) output_[i]=wanted[i];
    for (int i=0;i<3;++i) {
      const double request=target_[i];
      const int direction=Abs(request)<=wheel_drive_config::ZERO_SPEED_MMPS?0:(request>0?1:-1);
      if (direction!=direction_[i]) {
        stallTime_[i]=wrongDirectionTime_[i]=directionTime_[i]=0; direction_[i]=direction;
      }
      if (direction==0) continue;
      directionTime_[i]=direction*Output(i)>0?directionTime_[i]+dt:0;
      const bool stalled=Abs(request)>=wheel_drive_config::STALL_REQUEST_MMPS &&
          Abs(measured_[i])<wheel_drive_config::STALL_MEASURED_MMPS &&
          direction*Output(i)>=wheel_drive_config::STALL_OUTPUT;
      stallTime_[i]=stalled?stallTime_[i]+dt:0;
      const bool backwards=directionTime_[i]>wheel_drive_config::DIRECTION_GRACE_S &&
          direction*measured_[i]< -wheel_drive_config::STALL_REQUEST_MMPS;
      wrongDirectionTime_[i]=backwards?wrongDirectionTime_[i]+dt:0;
      if (stallTime_[i]>=wheel_drive_config::STALL_TIMEOUT_S) { Fault("Wheel stalled/encoder not changing",i); return; }
      if (wrongDirectionTime_[i]>=wheel_drive_config::WRONG_DIRECTION_TIMEOUT_S) { Fault("Encoder direction mismatch",i); return; }
    }
    status_="Direct proportional mixer; encoders monitored; no compensation";
  }
  constexpr double Output(int i) const {
    return i>=0 && i<3?output_[i]:0;
  }
  constexpr double MeasuredMMps(int i) const { return i>=0 && i<3?measured_[i]:0; }
  constexpr bool HasFault() const { return fault_; }
  constexpr int FaultWheel() const { return faultWheel_; }
  constexpr const char* Status() const { return status_; }
 private:
  static constexpr double Abs(double x) { return x<0?-x:x; }
  static constexpr bool Finite(double x) { return std::isfinite(x) && x>=-1e15 && x<=1e15; }
  constexpr void Fault(const char* message,int wheel) {
    fault_=true; faultWheel_=wheel; status_=message;
    for (int i=0;i<3;++i) output_[i]=0;
  }
  double previousCounts_[3]={},measured_[3]={},target_[3]={},output_[3]={};

  double stallTime_[3]={},wrongDirectionTime_[3]={},directionTime_[3]={};
  int direction_[3]={};
  bool baseline_=false,fault_=false;
  double sampleElapsed_=0;
  int faultWheel_=-1;
  const char* status_="Stopped";
};
