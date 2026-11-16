#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include "RobotConfig.h"

inline IrReading SharpIrFromVoltage(double voltage) {
  IrReading reading;
  // Keep raw finite voltage visible even when it cannot yield a valid range.
  if (!std::isfinite(voltage)) return reading;
  reading.voltage=voltage;
  if (voltage<=0 || voltage>localization_config::IR_MAX_INPUT_V) return reading;
  // Convert Studica's cm formula to mm and apply the requested x2 correction.
  const double distance=277.26*std::pow(voltage,-1.2045)*2.0;
  // Range validity is based ONLY on this final doubled face-to-object gap.
  // The sensor's mounting offset is added later during pose referencing.
  if (!std::isfinite(distance) || distance<localization_config::IR_MIN_RANGE_MM ||
      distance>localization_config::IR_MAX_RANGE_MM) return reading;
  reading.distanceMM=distance;
  reading.valid=true;
  // The physical response is ambiguous below 100 mm: voltage alone cannot
  // reliably detect every too-close target or a floating/disconnected input.
  return reading;
}

// navX 3.1.413 GetRate() returns an angle increment per sensor sample. Use
// sensor timestamps (milliseconds) to obtain actual CCW degrees/second. Do
// not divide by the robot loop period or treat repeated samples as zero speed.
class YawRateEstimator {
 public:
  double Update(double yawDeg,std::uint32_t timestampMS) {
    if (!std::isfinite(yawDeg)) { haveSample_=false; rate_=0; return rate_; }
    if (!haveSample_) {
      haveSample_=true; previousYaw_=yawDeg; previousTime_=timestampMS; rate_=0; return rate_;
    }
    const std::uint32_t elapsedMS=timestampMS-previousTime_;
    if (elapsedMS==0) return rate_;
    // Unsigned subtraction handles the normal 32-bit timestamp rollover.
    // A board restart or interrupted stream establishes a fresh baseline.
    rate_=elapsedMS<=100?robot_geometry::Wrap(robot_geometry::Radians(yawDeg-previousYaw_))*
        (180.0/robot_geometry::PI)*1000.0/elapsedMS:0;
    previousYaw_=yawDeg; previousTime_=timestampMS;
    return rate_;
  }
  double Rate() const { return rate_; }
 private:
  bool haveSample_=false;
  double previousYaw_=0,rate_=0;
  std::uint32_t previousTime_=0;
};

// Fixed user-provided references. Runtime only observes fresh sensor readings.
class CobraCheck {
 public:
  void Update(const LineReading& reading,double now) {
    reading_=reading;
    if (!std::isfinite(now)) { reading_.communicationValid=false; status_="Invalid clock"; }
    else if (!reading_.communicationValid) status_="No Cobra I2C response: check J1, power and I2C cable";
    else if (!VoltagesValid()) status_="Invalid Cobra voltage";
    else status_="Supplied calibration: CH0 leftmost through CH3 rightmost";
  }
  bool ChannelReadingValid(int i) const {
    return i>=0 && i<4 && reading_.communicationValid && std::isfinite(reading_.volts[i]) &&
        reading_.volts[i]>=0 && reading_.volts[i]<=cobra_config::COBRA_REFERENCE_V+0.05;
  }
  bool VoltagesValid() const {
    for (int i=0;i<4;++i) if (!ChannelReadingValid(i)) return false;
    return true;
  }
  bool Ready() const {
    for (int i=0;i<4;++i) if (std::fabs(Black(i)-White(i))<0.30) return false;
    return true;
  }
  double White(int i) const { return i>=0 && i<4?cobra_config::COBRA_FLOOR_V[i]:-1; }
  double Black(int i) const { return i>=0 && i<4?cobra_config::COBRA_TAPE_V[i]:-1; }
  double BlackLevel(int i) const {
    if (i<0 || i>=4 || !Ready() || !VoltagesValid()) return -1;
    return std::max(0.0,std::min(1.0,(reading_.volts[i]-White(i))/(Black(i)-White(i))));
  }
  bool IsBlack(int i) const { return BlackLevel(i)>=0.5; }
  const std::string& Status() const { return status_; }
 private:
  LineReading reading_;
  std::string status_="Supplied calibration; waiting for I2C readings";
};
