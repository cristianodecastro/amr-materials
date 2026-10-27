#pragma once

#include <algorithm>
#include <string>
#include "LineTracker.h"

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
    if (!VoltagesValid()) {
      capture_=0; status_="Invalid voltage: check Cobra power, J1/I2C cable and supply"; return;
    }
    if (!capture_) return;
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
    for (double v:reading_.volts)
      if (!std::isfinite(v) || v<0 || v>autonomous_config::COBRA_REFERENCE_V+0.05) return false;
    return true; // electrical range only; the vendor API exposes no I2C ACK flag
  }
  bool ChannelCalibrated(int i) const {
    return haveWhite_ && haveBlack_ && std::fabs(black_[i]-white_[i])>=0.30;
  }
  bool Ready() const {
    if (capture_) return false;
    for (int i=0;i<4;++i) if (!ChannelCalibrated(i)) return false;
    return true;
  }
  bool IsBlack(int i) const {
    return Ready() && VoltagesValid() &&
        (reading_.volts[i]-white_[i])/(black_[i]-white_[i])>=0.5;
  }
  double White(int i) const { return haveWhite_?white_[i]:-1; }
  double Black(int i) const { return haveBlack_?black_[i]:-1; }
  const std::string& Status() const { return status_; }
 private:
  LineReading reading_;
  bool haveWhite_=false,haveBlack_=false;
  int capture_=0,samples_=0;
  double start_=0,lastSample_=0;
  double white_[4]={},black_[4]={},sum_[4]={},minimum_[4]={},maximum_[4]={};
  std::string status_="Place all four sensors over WHITE floor, then Capture white";
};
