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

// Cobra line readings and calibration checks.
// Positive error: tape lies LEFT of array center. Calibration supports either
// light or dark tape; no assumed voltage polarity.
inline LineReading TrackLine(const double (&volts)[4]) {
  using namespace autonomous_config;
  LineReading result;
  double weighted=0, peak=0, minimum=1;
  double weights[4]={};
  // Keep all four raw voltages visible even if one channel invalidates tracking.
  for (int i=0;i<4;++i) result.volts[i]=volts[i];
  // Raw diagnostics work before calibration; autonomy must not classify tape
  // using placeholder voltages. The Cobra check has separate session captures.
  if (!COBRA_CALIBRATED) return result;
  for (int i=0;i<4;++i) {
    const double span=COBRA_TAPE_V[i]-COBRA_FLOOR_V[i];
    if (!std::isfinite(volts[i]) || volts[i]<0 || volts[i]>COBRA_REFERENCE_V+0.05 || std::fabs(span)<0.2) return result;
    weights[i]=std::max(0.0,std::min(1.0,(volts[i]-COBRA_FLOOR_V[i])/span));
    result.strength+=weights[i];
    weighted+=weights[i]*COBRA_CHANNEL_LEFT_MM[i];
    peak=std::max(peak,weights[i]); minimum=std::min(minimum,weights[i]);
  }
  if (result.strength<LINE_MIN_STRENGTH || peak<LINE_MIN_PEAK || minimum>0.7) return result;
  result.leftErrorMM=weighted/result.strength;
  double variance=0;
  for (int i=0;i<4;++i) {
    const double delta=COBRA_CHANNEL_LEFT_MM[i]-result.leftErrorMM;
    variance+=weights[i]*delta*delta;
  }
  result.valid=2.0*std::sqrt(variance/result.strength)<=LINE_MAX_WIDTH_MM;
  return result;
}

// Live calibration uses user-measured defaults, then restores saved references.
class CobraCheck {
 public:
  explicit CobraCheck(bool useDefaults=true) {
    if (useDefaults && autonomous_config::COBRA_CALIBRATED) {
      for (int i=0;i<4;++i) { white_[i]=autonomous_config::COBRA_FLOOR_V[i]; black_[i]=autonomous_config::COBRA_TAPE_V[i]; }
      haveWhite_=haveBlack_=true;
      status_="Using supplied white/black calibration; live readings required";
    }
  }
  bool LoadCalibration(const std::string& path,std::string& error);
  bool SaveCalibration(const std::string& path,std::string& error) const;
  unsigned CalibrationRevision() const { return revision_; }
  void Update(const LineReading& reading, double now, bool disabled,
              bool captureWhite, bool captureBlack) {
    reading_=reading;
    if (!std::isfinite(now)) { capture_=0; status_="Invalid clock"; return; }
    if (!disabled && (capture_ || captureWhite || captureBlack)) {
      capture_=0; status_="Capture cancelled: keep Control Station Disabled"; return;
    }
    if (captureWhite && captureBlack) {
      capture_=0; status_="Choose just one surface capture"; return;
    }
    if (captureWhite || captureBlack) {
      capture_=captureWhite?1:2;
      start_=lastSample_=now; samples_=0;
      for (int i=0;i<4;++i) { sum_[i]=0; minimum_[i]=1e9; maximum_[i]=-1e9; }
    }
    const bool recovered=!readingWasValid_ && VoltagesValid();
    readingWasValid_=VoltagesValid();
    if (!readingWasValid_) {
      capture_=0;
      status_=reading_.communicationValid?"Invalid Cobra voltage: check sensor power and reference voltage":
          "No Cobra I2C response: check analog module J1, power and VMX I2C cable";
      return;
    }
    if (!capture_) {
      if (recovered) status_=Ready()?"Ready: sweep tape across CH0..3; green means black":
          "I2C responding: capture WHITE floor and BLACK tape under all four sensors";
      return;
    }
    if (now<lastSample_ || now-lastSample_>0.20) {
      capture_=0; status_="Capture cancelled: readings interrupted; retry"; return;
    }
    lastSample_=now;
    ++samples_;
    for (int i=0;i<4;++i) {
      const double v=reading_.volts[i];
      sum_[i]+=v; minimum_[i]=std::min(minimum_[i],v); maximum_[i]=std::max(maximum_[i],v);
    }
    status_=capture_==1?"Hold still: sampling WHITE floor":"Hold still: sampling BLACK tape";
    if (now-start_<0.75) return;
    if (samples_<10) { capture_=0; status_="Too few fresh samples; retry"; return; }
    for (int i=0;i<4;++i) if (maximum_[i]-minimum_[i]>0.20) {
      capture_=0; status_="Unstable CH"+std::to_string(i)+": hold height and surface still; retry"; return;
    }
    // Commit a surface only after checking the complete candidate. Failed or
    // cancelled recaptures preserve the previous working calibration.
    for (int i=0;i<4;++i) {
      const double average=sum_[i]/samples_;
      const bool haveOther=capture_==1?haveBlack_:haveWhite_;
      const double other=capture_==1?black_[i]:white_[i];
      if (haveOther && std::fabs(average-other)<0.30) {
        capture_=0; status_="Low white/black contrast on CH"+std::to_string(i)+"; previous calibration retained"; return;
      }
    }
    for (int i=0;i<4;++i) {
      if (capture_==1) white_[i]=sum_[i]/samples_; else black_[i]=sum_[i]/samples_;
    }
    if (capture_==1) haveWhite_=true; else haveBlack_=true;
    capture_=0;
    if (!haveWhite_) { status_="Black saved; capture WHITE under all four sensors"; return; }
    if (!haveBlack_) { status_="White saved; capture BLACK under all four sensors"; return; }
    for (int i=0;i<4;++i) if (!ChannelCalibrated(i)) {
      status_="Low white/black contrast on CH"+std::to_string(i)+"; check height and repeat captures"; return;
    }
    ++revision_;
    status_="Ready: sweep tape across CH0..3; green means black";
  }
  bool VoltagesValid() const {
    if (!reading_.communicationValid) return false;
    for (double v:reading_.volts)
      if (!std::isfinite(v) || v<0 || v>autonomous_config::COBRA_REFERENCE_V+0.05) return false;
    return true;
  }
  bool ChannelCalibrated(int i) const {
    return haveWhite_ && haveBlack_ && std::fabs(black_[i]-white_[i])>=0.30;
  }
  bool Ready() const {
    if (capture_) return false;
    for (int i=0;i<4;++i) if (!ChannelCalibrated(i)) return false;
    return true;
  }
  bool Capturing() const { return capture_!=0; }
  bool IsBlack(int i) const {
    return Ready() && VoltagesValid() &&
        (reading_.volts[i]-white_[i])/(black_[i]-white_[i])>=0.5;
  }
  double BlackLevel(int i) const {
    if (i<0 || i>=4 || !Ready() || !VoltagesValid()) return -1;
    return std::max(0.0,std::min(1.0,(reading_.volts[i]-white_[i])/(black_[i]-white_[i])));
  }
  bool ChannelReadingValid(int i) const {
    return i>=0 && i<4 && reading_.communicationValid && std::isfinite(reading_.volts[i]) &&
        reading_.volts[i]>=0 && reading_.volts[i]<=autonomous_config::COBRA_REFERENCE_V+0.05;
  }
  double White(int i) const { return haveWhite_?white_[i]:-1; }
  double Black(int i) const { return haveBlack_?black_[i]:-1; }
  const std::string& Status() const { return status_; }
 private:
  LineReading reading_;
  bool haveWhite_=false,haveBlack_=false;
  bool readingWasValid_=false;
  unsigned revision_=0;
  int capture_=0,samples_=0;
  double start_=0,lastSample_=0;
  double white_[4]={},black_[4]={},sum_[4]={},minimum_[4]={},maximum_[4]={};
  std::string status_="Place all four sensors over WHITE floor, then Capture white";
};
