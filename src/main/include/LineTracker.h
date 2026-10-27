#pragma once

#include <algorithm>
#include "AutonomousConfig.h"

struct LineReading {
  bool valid = false;
  double leftErrorMM = 0.0;
  double strength = 0.0;
  double volts[4] = {-1,-1,-1,-1};
};

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
