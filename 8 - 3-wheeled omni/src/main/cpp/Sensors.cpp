#include "Sensors.h"

#include <cstdio>
#include <fstream>
#include <iomanip>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace {
bool ValidReferences(const double (&white)[4],const double (&black)[4]) {
  for (int i=0;i<4;++i) {
    if (!std::isfinite(white[i]) || !std::isfinite(black[i]) || white[i]<0 || black[i]<0 ||
        white[i]>autonomous_config::COBRA_REFERENCE_V+0.05 ||
        black[i]>autonomous_config::COBRA_REFERENCE_V+0.05 || std::fabs(white[i]-black[i])<0.30) return false;
  }
  return true;
}
}
bool CobraCheck::LoadCalibration(const std::string& path,std::string& error) {
  std::ifstream file(path);
  if (!file) { error="No saved Cobra calibration; using supplied measurements"; return false; }
  std::string signature;
  int version=0;
  double white[4]={},black[4]={};
  if (!(file>>signature>>version) || signature!="CobraCalibration" || version!=1) {
    error="Invalid saved Cobra calibration format; supplied measurements retained"; return false;
  }
  for (auto& v:white) if (!(file>>v)) { error="Incomplete Cobra white reference"; return false; }
  for (auto& v:black) if (!(file>>v)) { error="Incomplete Cobra black reference"; return false; }
  file>>std::ws;
  if (!file.eof() || !ValidReferences(white,black)) { error="Invalid saved Cobra voltages/contrast"; return false; }
  for (int i=0;i<4;++i) { white_[i]=white[i]; black_[i]=black[i]; }
  haveWhite_=haveBlack_=true; capture_=0;
  status_="Saved Cobra calibration restored";
  error.clear(); return true;
}
bool CobraCheck::SaveCalibration(const std::string& path,std::string& error) const {
  if (!Ready() || !ValidReferences(white_,black_)) { error="Cobra references are not ready to save"; return false; }
  const std::string temporary=path+".tmp";
  std::ofstream file(temporary,std::ios::trunc);
  if (!file) { error="Cannot write Cobra calibration; current values remain in memory"; return false; }
  file<<"CobraCalibration 1\n"<<std::setprecision(12);
  for (double v:white_) file<<v<<' ';
  file<<'\n';
  for (double v:black_) file<<v<<' ';
  file<<'\n'; file.close();
  bool replaced=false;
  if (file) {
#ifdef _WIN32
    replaced=MoveFileExA(temporary.c_str(),path.c_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH)!=0;
#else
    replaced=std::rename(temporary.c_str(),path.c_str())==0;
#endif
  }
  if (!replaced) {
    std::remove(temporary.c_str()); error="Cannot finish Cobra calibration save; previous file retained"; return false;
  }
  error.clear(); return true;
}
