#pragma once

#include <cmath>
#include <algorithm>
#include "RobotConfig.h"

// Wheel geometry and shared output normalization.
namespace drive_kinematics {

inline constexpr double LayoutVx(double forward, double left) {
  const double a=localization_config::DRIVE_FORWARD_DIRECTION_DEG*robot_geometry::PI/180.0;
  return std::cos(a)*forward-std::sin(a)*left;
}
inline constexpr double LayoutVy(double forward, double left) {
  const double a=localization_config::DRIVE_FORWARD_DIRECTION_DEG*robot_geometry::PI/180.0;
  return std::sin(a)*forward+std::cos(a)*left;
}
inline constexpr double BodyForward(double layoutVx, double layoutVy) {
  const double a=localization_config::DRIVE_FORWARD_DIRECTION_DEG*robot_geometry::PI/180.0;
  return std::cos(a)*layoutVx+std::sin(a)*layoutVy;
}
inline constexpr double BodyLeft(double layoutVx, double layoutVy) {
  const double a=localization_config::DRIVE_FORWARD_DIRECTION_DEG*robot_geometry::PI/180.0;
  return -std::sin(a)*layoutVx+std::cos(a)*layoutVy;
}

inline constexpr double WheelSpeedMMps(int wheel, double vx_MMps, double vy_MMps,
                             double yawRate_Radps) {
  const double alpha = localization_config::WHEEL_ANGLE_DEG[wheel] *
                       robot_geometry::PI / 180.0;
  return -std::sin(alpha) * vx_MMps + std::cos(alpha) * vy_MMps +
         localization_config::DRIVE_YAW_DIRECTION_SIGN * yawRate_Radps *
         localization_config::WHEEL_CENTER_RADIUS_MM[wheel];
}

// Invert the SAME tangent projections used by WheelSpeedMMps. Gyro rotation
// removes each wheel's rotational travel; solve the remaining XY least squares.
// Inputs are distances in mm, not wheel RPM or body travel along that wheel.
inline bool BodyDisplacement(const double (&wheelMM)[3],double gyroTurn,
                             double& forward,double& left) {
  double xx=0,xy=0,yy=0,bx=0,by=0;
  for (int i=0;i<3;++i) {
    const double angle=localization_config::WHEEL_ANGLE_DEG[i]*robot_geometry::PI/180;
    const double x=-std::sin(angle),y=std::cos(angle);
    const double d=wheelMM[i]-localization_config::DRIVE_YAW_DIRECTION_SIGN*
        localization_config::WHEEL_CENTER_RADIUS_MM[i]*gyroTurn;
    xx+=x*x; xy+=x*y; yy+=y*y; bx+=x*d; by+=y*d;
  }
  const double determinant=xx*yy-xy*xy;
  if (!std::isfinite(determinant) || std::fabs(determinant)<1e-8) return false;
  const double vx=(yy*bx-xy*by)/determinant,vy=(xx*by-xy*bx)/determinant;
  forward=BodyForward(vx,vy); left=BodyLeft(vx,vy);
  return std::isfinite(forward) && std::isfinite(left);
}

// Count-increment odometry: displacement never depends on an assumed update
// frequency. Actual dt is used only for velocity display and sample checks.
class EncoderGyroOdometry {
 public:
  void Reset(RobotPose pose,const double (&counts)[3],double yawDeg) {
    pose_=pose; pose_.headingRad=robot_geometry::Wrap(pose.headingRad);
    offset_=robot_geometry::Wrap(pose_.headingRad-robot_geometry::Radians(yawDeg));
    forwardSpeed_=leftSpeed_=0;
    healthy_=robot_geometry::Finite(pose_) && std::isfinite(yawDeg);
    for (int i=0;i<3;++i) { previous_[i]=counts[i]; healthy_=healthy_ && std::isfinite(counts[i]); }
  }
  bool Update(const double (&counts)[3],double yawDeg,double gyroRateDps,double dt) {
    healthy_=false;
    forwardSpeed_=leftSpeed_=0;
    if (!std::isfinite(dt) || dt<=0 || dt>autonomous_config::MAX_CONTROL_GAP_S ||
        !std::isfinite(yawDeg) || !std::isfinite(gyroRateDps)) return false;
    double wheel[3];
    for (int i=0;i<3;++i) {
      wheel[i]=(counts[i]-previous_[i])*localization_config::WHEEL_ENCODER_SIGN[i]*
          localization_config::ENCODER_MM_PER_COUNT;
      if (!std::isfinite(wheel[i]) || std::fabs(wheel[i])/dt>3*localization_config::MAX_WHEEL_SPEED_MMPS) return false;
    }
    const double heading=robot_geometry::Wrap(robot_geometry::Radians(yawDeg)+offset_);
    const double turn=robot_geometry::Wrap(heading-pose_.headingRad);
    double forward=0,left=0;
    if (!BodyDisplacement(wheel,turn,forward,left)) return false;
    // Scale translation after separating rotation. Do not alter raw counts,
    // the shared motor mixer, gyro heading, or the Teleop pose observer.
    forward*=autonomous_odometry_config::DISTANCE_SCALE;
    left*=autonomous_odometry_config::DISTANCE_SCALE;
    const double midpoint=pose_.headingRad+turn/2;
    // Exact constant-twist arc integration; sinc tends to one for straight travel.
    const double arc=std::fabs(turn)<1e-8?1.0:std::sin(turn/2)/(turn/2);
    pose_.x_MM+=arc*(std::cos(midpoint)*forward-std::sin(midpoint)*left);
    pose_.y_MM+=arc*(std::sin(midpoint)*forward+std::cos(midpoint)*left);
    pose_.headingRad=heading;
    for (int i=0;i<3;++i) previous_[i]=counts[i];
    forwardSpeed_=forward/dt; leftSpeed_=left/dt;
    healthy_=robot_geometry::Finite(pose_);
    return healthy_;
  }
  const RobotPose& Pose() const { return pose_; }
  bool Healthy() const { return healthy_; }
  double ForwardSpeed() const { return forwardSpeed_; }
  double LeftSpeed() const { return leftSpeed_; }
  double OffsetDegrees() const { return offset_*180/robot_geometry::PI; }
 private:
  RobotPose pose_{0,0,0};
  double previous_[3]={},offset_=0,forwardSpeed_=0,leftSpeed_=0;
  bool healthy_=false;
};

// Preserve the pre-revision manual pose display. This estimate is observational:
// Teleop steering never consumes it, and Autonomous never reads this state.
class TeleopPoseTelemetry {
 public:
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
    if (!std::isfinite(dt) || dt<=0 || dt>autonomous_config::MAX_CONTROL_GAP_S ||
        !std::isfinite(yawDeg) || !std::isfinite(rateDps)) return false;
    double wheel[3],maximum=0;
    for (int i=0;i<3;++i) {
      wheel[i]=(counts[i]-previous_[i])*localization_config::WHEEL_ENCODER_SIGN[i]*localization_config::ENCODER_MM_PER_COUNT;
      const double speed=std::fabs(wheel[i])/dt;
      if (!std::isfinite(speed) || speed>3*localization_config::MAX_WHEEL_SPEED_MMPS) return false;
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
    const double predictedX=worldX_+(std::cos(offset_)*accelX-std::sin(offset_)*accelY)*dt;
    const double predictedY=worldY_+(std::sin(offset_)*accelX+std::cos(offset_)*accelY)*dt;
    const double midpoint=oldHeading+0.5*robot_geometry::Wrap(pose_.headingRad-oldHeading);
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
    forwardSpeed_=std::cos(pose_.headingRad)*worldX_+std::sin(pose_.headingRad)*worldY_;
    leftSpeed_=-std::sin(pose_.headingRad)*worldX_+std::cos(pose_.headingRad)*worldY_;
    for (int i=0;i<3;++i) previous_[i]=counts[i];
    healthy_=robot_geometry::Finite(pose_); return healthy_;
  }
  const RobotPose& Pose() const { return pose_; }
  bool Healthy() const { return healthy_; }
  double ForwardSpeed() const { return forwardSpeed_; }
  double LeftSpeed() const { return leftSpeed_; }
  double OffsetDegrees() const { return offset_*180/robot_geometry::PI; }
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
                        double rawY, double rawRightX, double dt,
                        bool liftControlsNeutral = true) {
    if (!connected) {
      Stop("Controller inputs missing - check connection/axes", true);
      return;
    }
    if (!Finite(rawX) || !Finite(rawY) || !Finite(rawRightX) ||
        Abs(rawX) > 1.01 || Abs(rawY) > 1.01 || Abs(rawRightX) > 1.01 ||
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
        !Finite(localization_config::DRIVE_FORWARD_DIRECTION_DEG) ||
        Abs(localization_config::DRIVE_YAW_DIRECTION_SIGN) != 1.0 ||
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
      if (rawMagnitude > deadband || Abs(rawRightX) > deadband || !liftControlsNeutral) {
        Stop("Center both sticks and release D-pad to arm", true);
        return;
      }
      armed_ = true;
    }
    const double x=Abs(rawX)<=deadband?0:rawX;
    const double y=Abs(rawY)<=deadband?0:rawY;
    const double magnitude=std::hypot(x,y);
    const double seconds = Clamp(dt, 0.0, 0.050);
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
  constexpr const char* GetTranslationMode() const {
    if (forward_MMps_>0 && left_MMps_>0) return "Forward-left";
    if (forward_MMps_>0 && left_MMps_<0) return "Forward-right";
    if (forward_MMps_<0 && left_MMps_>0) return "Backward-left";
    if (forward_MMps_<0 && left_MMps_<0) return "Backward-right";
    if (forward_MMps_>0) return "Forward";
    if (forward_MMps_<0) return "Backward";
    if (left_MMps_>0) return "Left";
    if (left_MMps_<0) return "Right";
    return "Stopped";
  }

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
// Each Hardware instance has separate manual/autonomous controller state.
// Autonomous uses the direct mixer; the driver's historical startup/ramp is
// requested explicitly with Teleop context, never inferred from wheel motion.
class WheelDriveController {
 public:
  constexpr void Reset() { *this=WheelDriveController{}; }
  constexpr void Update(const double (&desired)[3],const double (&counts)[3],double dt,double maxOutput,
                        WheelDriveContext context=WheelDriveContext::Autonomous) {
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
    const bool teleop=context==WheelDriveContext::Teleop;
    if (teleop && !baseline_) BeginStartup(desired,counts,peak);
    sampleElapsed_+=dt;
    const bool observe=baseline_ && sampleElapsed_+1e-9>=wheel_drive_config::SPEED_WINDOW_S;
    const double alpha=sampleElapsed_/(wheel_drive_config::SPEED_FILTER_S+sampleElapsed_);
    double wanted[3]={};
    for (int i=0;i<3;++i) {
      if (!baseline_) previousCounts_[i]=counts[i];
      if (observe) {
        const double measured=(counts[i]-previousCounts_[i])*localization_config::WHEEL_ENCODER_SIGN[i]*
            localization_config::ENCODER_MM_PER_COUNT/sampleElapsed_;
        if (!Finite(measured) || Abs(measured)>3*localization_config::MAX_WHEEL_SPEED_MMPS) {
          Fault("Encoder jump/out-of-range speed",i); return;
        }
        measured_[i]+=alpha*(measured-measured_[i]); previousCounts_[i]=counts[i];
      }
      target_[i]=desired[i]*scale;
      wanted[i]=desired[i]/denominator;
    }
    if (!baseline_ || observe) sampleElapsed_=0;
    baseline_=true;
    if (teleop && starting_) {
      startupTime_+=dt;
      for (int i=0;i<3;++i) {
        const bool required=Abs(desired[i])/peak>=teleop_wheel_config::STARTUP_MIN_WHEEL_RATIO;
        if (required!=startupRequired_[i] || (required && desired[i]*startupDirection_[i]<=0)) starting_=false;
      }
      if (starting_) {
        bool allMoving=true;
        for (int i=0;i<3;++i) if (startupRequired_[i]) {
          const double movement=(counts[i]-startupCounts_[i])*localization_config::WHEEL_ENCODER_SIGN[i]*startupDirection_[i];
          if (movement>=teleop_wheel_config::STARTUP_CONFIRM_COUNTS) startupMoved_[i]=true;
          allMoving=allMoving && startupMoved_[i];
        }
        if (allMoving) starting_=false;
        else if (startupTime_>=teleop_wheel_config::STARTUP_TIMEOUT_S) {
          Fault("Wheel did not start - increase stick slightly or check encoder; center stick",WaitingWheel()); return;
        } else for (int i=0;i<3;++i) if (startupRequired_[i] && startupMoved_[i]) wanted[i]=0;
      }
    }
    double fraction=1;
    if (teleop) {
      const double seconds=dt<0.05?dt:0.05;
      for (int i=0;i<3;++i) {
        const double change=Abs(wanted[i]-output_[i]);
        const double rate=wanted[i]*output_[i]>=0 && Abs(wanted[i])>Abs(output_[i])?
            teleop_wheel_config::PWM_RISE_PER_S:teleop_wheel_config::PWM_FALL_PER_S;
        if (change>rate*seconds && rate*seconds/change<fraction) fraction=rate*seconds/change;
      }
    }
    for (int i=0;i<3;++i) output_[i]+=fraction*(wanted[i]-output_[i]);
    for (int i=0;i<3;++i) {
      const double request=target_[i];
      const int direction=Abs(request)<=wheel_drive_config::ZERO_SPEED_MMPS?0:(request>0?1:-1);
      if (direction!=direction_[i]) {
        stallTime_[i]=wrongDirectionTime_[i]=directionTime_[i]=0; direction_[i]=direction;
      }
      if (direction==0) continue;
      directionTime_[i]=direction*output_[i]>0?directionTime_[i]+dt:0;
      const bool stalled=Abs(request)>=wheel_drive_config::STALL_REQUEST_MMPS &&
          Abs(measured_[i])<wheel_drive_config::STALL_MEASURED_MMPS &&
          direction*output_[i]>=wheel_drive_config::STALL_OUTPUT;
      stallTime_[i]=stalled?stallTime_[i]+dt:0;
      const bool backwards=directionTime_[i]>wheel_drive_config::DIRECTION_GRACE_S &&
          direction*measured_[i]< -wheel_drive_config::STALL_REQUEST_MMPS;
      wrongDirectionTime_[i]=backwards?wrongDirectionTime_[i]+dt:0;
      if (stallTime_[i]>=wheel_drive_config::STALL_TIMEOUT_S) { Fault("Wheel stalled/encoder not changing",i); return; }
      if (wrongDirectionTime_[i]>=wheel_drive_config::WRONG_DIRECTION_TIMEOUT_S) { Fault("Encoder direction mismatch",i); return; }
    }
    status_=teleop?(starting_?"Startup - waiting for wheel encoder response":
        "Teleop mixer; encoders monitored; no power boost"):
        "Autonomous direct wheel mixer; no per-wheel corrections or holds";
  }
  constexpr double Output(int i) const { return i>=0 && i<3?output_[i]:0; }
  constexpr double TargetMMps(int i) const { return i>=0 && i<3?target_[i]:0; }
  constexpr double MeasuredMMps(int i) const { return i>=0 && i<3?measured_[i]:0; }
  constexpr bool HasFault() const { return fault_; }
  constexpr int FaultWheel() const { return faultWheel_; }
  constexpr const char* Status() const { return status_; }
  constexpr bool IsStarting() const { return starting_; }
  constexpr int WaitingWheel() const {
    if (starting_) for (int i=0;i<3;++i) if (startupRequired_[i] && !startupMoved_[i]) return i;
    return -1;
  }
 private:
  static constexpr double Abs(double x) { return x<0?-x:x; }
  static constexpr bool Finite(double x) { return std::isfinite(x) && x>=-1e15 && x<=1e15; }
  constexpr void BeginStartup(const double (&desired)[3],const double (&counts)[3],double peak) {
    int required=0;
    for (int i=0;i<3;++i) {
      startupRequired_[i]=Abs(desired[i])/peak>=teleop_wheel_config::STARTUP_MIN_WHEEL_RATIO;
      startupDirection_[i]=desired[i]>0?1:-1; startupCounts_[i]=counts[i]; startupMoved_[i]=false;
      if (startupRequired_[i]) ++required;
    }
    starting_=required>1; startupTime_=0;
  }
  constexpr void Fault(const char* message,int wheel) {
    fault_=true; faultWheel_=wheel; status_=message;
    starting_=false;
    for (int i=0;i<3;++i) output_[i]=0;
  }
  double previousCounts_[3]={},measured_[3]={},target_[3]={},output_[3]={};
  double stallTime_[3]={},wrongDirectionTime_[3]={},directionTime_[3]={};
  int direction_[3]={};
  bool baseline_=false,fault_=false;
  double sampleElapsed_=0;
  int faultWheel_=-1;
  bool starting_=false,startupRequired_[3]={},startupMoved_[3]={};
  double startupCounts_[3]={},startupTime_=0;
  int startupDirection_[3]={};
  const char* status_="Stopped";
};
// Hold-to-run lift control.
// Hold-to-run manual lift. Release after enable/reconnect/limit before moving.
class LiftTeleopController {
 public:
  constexpr void Update(bool allowed, bool inputsPresent, bool up, bool down,
              bool upperLimit, bool lowerLimit) {
    output_=0;
    if (!allowed || !inputsPresent) {
      armed_=false;
      status_=!allowed?"Stopped - enabled Teleop and neutral controls required":"D-pad POV missing";
      return;
    }
    if (!(manipulator_config::LIFT_UP_SIGN==1 || manipulator_config::LIFT_UP_SIGN==-1) ||
        !(manipulator_config::LIFT_OUTPUT>0 && manipulator_config::LIFT_OUTPUT<=1)) {
      armed_=false; status_="Invalid lift configuration"; return;
    }
    if (!armed_) {
      if (up || down) { status_="Release D-pad to arm lift"; return; }
      armed_=true;
    }
    if (up && down) { status_="Up + down cancelled"; return; }
    if ((up && upperLimit) || (down && lowerLimit)) {
      armed_=false; status_=up?"Upper limit - release D-pad":"Lower limit - release D-pad"; return;
    }
    output_=(up?1.0:(down?-1.0:0.0))*manipulator_config::LIFT_UP_SIGN*manipulator_config::LIFT_OUTPUT;
    status_=up?"Raising - hold D-pad up":(down?"Lowering - hold D-pad down":"Ready - D-pad up/down");
  }
  constexpr double Output() const { return output_; }
  constexpr const char* Status() const { return status_; }
 private:
  double output_=0;
  bool armed_=false;
  const char* status_="Stopped";
};

// Manual servo controls.
// Independent Triangle toggle and manual L1/R1 servo steps.
// Release the relevant controls after enable/reconnect before output is armed.
class ServoTeleopController {
 public:
  constexpr void Update(bool allowed, bool inputsPresent, bool triangle,
                        bool l1=false, bool r1=false) {
    enabled_=allowed && inputsPresent;
    if (!enabled_) { Disarm(); return; }
    if (!triangle) triangleArmed_=true;
    if (triangleArmed_ && triangle && !previousTriangle_) {
      triangleActive_=!triangleActive_;
      targets_[1]=triangleActive_?manipulator_config::SERVO_TRIANGLE_ANGLE_DEG:
          manipulator_config::SERVO_HOME_DEG;
    }
    previousTriangle_=triangle;
    if (!l1 && !r1) manualArmed_=true;
    if (manualArmed_) {
      const int steps=(l1 && !previousL1_?1:0)-(r1 && !previousR1_?1:0);
      targets_[0]+=steps*manipulator_config::SERVO_STEP_DEG;
      if (targets_[0]<manipulator_config::SERVO_MIN_DEG) targets_[0]=manipulator_config::SERVO_MIN_DEG;
      if (targets_[0]>manipulator_config::SERVO_MAX_DEG) targets_[0]=manipulator_config::SERVO_MAX_DEG;
    }
    previousL1_=l1;
    previousR1_=r1;
  }
  constexpr void Disarm() {
    enabled_=manualArmed_=triangleArmed_=false;
    previousTriangle_=previousL1_=previousR1_=false;
  }
  constexpr bool OutputEnabled(int i) const {
    return enabled_ && (i==0?manualArmed_:(i==1 && triangleArmed_));
  }
  constexpr double TargetDegrees(int i) const {
    return i>=0 && i<2?targets_[i]:manipulator_config::SERVO_HOME_DEG;
  }
 private:
  bool triangleActive_=false,triangleArmed_=false,manualArmed_=false;
  bool previousTriangle_=false,previousL1_=false,previousR1_=false,enabled_=false;
  double targets_[2]={manipulator_config::SERVO_HOME_DEG,manipulator_config::SERVO_HOME_DEG};
};
