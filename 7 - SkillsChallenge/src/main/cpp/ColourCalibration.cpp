#include "RobotConfig.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>

namespace {
std::string Lower(std::string text) {
  for (auto& c:text) c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return text;
}
}
ColourCalibration DefaultColourCalibration() {
  return {camera_config::COLOUR_ROI,{camera_config::DEFAULT_COLOUR_REFERENCES,
      camera_config::DEFAULT_COLOUR_REFERENCES+camera_config::COLOUR_REFERENCE_COUNT}};
}
bool ValidateColourCalibration(const ColourCalibration& data,std::string& error) {
  error.clear();
  if (!ValidColourRegion(data.roi) || data.references.size()!=camera_config::COLOUR_REFERENCE_COUNT) {
    error="Invalid ROI or reference slot count"; return false;
  }
  for (unsigned i=0;i<data.references.size();++i) {
    const auto& r=data.references[i];
    if (!ValidRgb(r.mean)) { error="Reference RGB must be finite and in 0..255"; return false; }
    if (!r.enabled) continue;
    if (r.name.empty() || r.name.size()>32 || Lower(r.name)=="unknown" ||
        std::isspace(static_cast<unsigned char>(r.name.front())) ||
        std::isspace(static_cast<unsigned char>(r.name.back())) ||
        std::any_of(r.name.begin(),r.name.end(),[](unsigned char c) { return c<32 || c==127; })) {
      error="Use a 1..32 character colour name; no surrounding spaces or reserved name Unknown"; return false;
    }
    for (unsigned j=0;j<i;++j) if (data.references[j].enabled && Lower(data.references[j].name)==Lower(r.name)) {
      error="Colour name already enabled in another slot"; return false;
    }
  }
  return true;
}
bool SaveColourCalibration(const std::string& path,const ColourCalibration& data,std::string& error) {
  if (!ValidateColourCalibration(data,error)) return false;
  const std::string temporary=path+".tmp";
  std::ofstream file(temporary,std::ios::trunc);
  if (!file) { error="Cannot write calibration temporary file"; return false; }
  file<<"SkillsChallengeColourCalibration 1\n"<<data.roi.x<<' '<<data.roi.y<<' '
      <<data.roi.width<<' '<<data.roi.height<<'\n'<<data.references.size()<<'\n'<<std::setprecision(12);
  for (const auto& r:data.references)
    file<<r.enabled<<' '<<std::quoted(r.name)<<' '<<r.mean.red<<' '<<r.mean.green<<' '<<r.mean.blue<<'\n';
  file.close();
  if (!file) { error="Cannot finish calibration write"; std::remove(temporary.c_str()); return false; }
  // POSIX rename atomically replaces the previous file on the VMX. A failed
  // write leaves its previous calibration intact; captures are serialised.
  if (std::rename(temporary.c_str(),path.c_str())!=0) {
    error=std::string("Cannot replace calibration: ")+std::strerror(errno);
    std::remove(temporary.c_str()); return false;
  }
  error.clear(); return true;
}
bool LoadColourCalibration(const std::string& path,ColourCalibration& data,std::string& error) {
  error.clear();
  std::ifstream file(path);
  std::string signature;
  int version=0,count=0;
  ColourCalibration candidate;
  if (!(file>>signature>>version>>candidate.roi.x>>candidate.roi.y>>candidate.roi.width>>candidate.roi.height>>count) ||
      signature!="SkillsChallengeColourCalibration" || version!=1 || count!=camera_config::COLOUR_REFERENCE_COUNT) {
    error="Invalid colour calibration format/version"; return false;
  }
  for (int i=0;i<count;++i) {
    ColourReference r;
    int enabled=-1;
    if (!(file>>enabled>>std::quoted(r.name)>>r.mean.red>>r.mean.green>>r.mean.blue) || (enabled!=0 && enabled!=1)) {
      error="Invalid colour reference data"; return false;
    }
    r.enabled=enabled==1; candidate.references.push_back(r);
  }
  file>>std::ws;
  if (!file.eof() || !ValidateColourCalibration(candidate,error)) {
    if (error.empty()) error="Unexpected data after colour references";
    return false;
  }
  data=std::move(candidate); error.clear(); return true;
}
