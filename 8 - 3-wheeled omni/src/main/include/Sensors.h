#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include "RobotConfig.h"

inline IrReading SharpIrFromVoltage(double voltage) {
  IrReading reading;
  // Keep raw finite voltage visible even when it cannot yield a valid range.
  if (!std::isfinite(voltage)) return reading;
  reading.voltage=voltage;
  if (voltage<=0 || voltage>localization_config::IR_MAX_INPUT_V) return reading;
  // Studica's GP2Y0A21 approximation is 27.726*V^-1.2045 in CENTIMETRES.
  // Convert exactly once to mm. Do not clamp an invalid range to a valid edge.
  const double distance=277.26*std::pow(voltage,-1.2045);
  if (!std::isfinite(distance) || distance<localization_config::IR_MIN_RANGE_MM ||
      distance>localization_config::IR_MAX_RANGE_MM) return reading;
  reading.distanceMM=distance;
  reading.valid=true;
  // The physical response is ambiguous below 100 mm: voltage alone cannot
  // reliably detect every too-close target or a floating/disconnected input.
  return reading;
}

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

// Diagnostic calibration only. Never controls motors or changes autonomous
// calibration/confirmation flags. White/black references last until restart.
class CobraCheck {
 public:
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
      if (captureWhite) haveWhite_=false; else haveBlack_=false;
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
  int capture_=0,samples_=0;
  double start_=0,lastSample_=0;
  double white_[4]={},black_[4]={},sum_[4]={},minimum_[4]={},maximum_[4]={};
  std::string status_="Place all four sensors over WHITE floor, then Capture white";
};
