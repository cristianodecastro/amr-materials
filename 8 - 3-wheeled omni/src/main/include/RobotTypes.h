#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

// Shared units, geometry and sensor/action data.
struct RobotPose { double x_MM; double y_MM; double headingRad; };
struct ArenaPoint { double x; double y; };

namespace robot_geometry {
static constexpr double PI = 3.14159265358979323846;
inline double Wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double Radians(double degrees) { return degrees * PI / 180.0; }
inline double Distance(ArenaPoint a, ArenaPoint b) { return std::hypot(a.x-b.x, a.y-b.y); }
inline bool Finite(const RobotPose& p) {
  return std::isfinite(p.x_MM) && std::isfinite(p.y_MM) && std::isfinite(p.headingRad);
}
}

struct LineReading {
  bool valid = false;
  double leftErrorMM = 0.0;
  double strength = 0.0;
  double volts[4] = {-1,-1,-1,-1};
};

enum class ObjectActionStatus { Unavailable, Busy, Complete, Failed };

// Generic names and measured RGB means allow competition colours to be added
// as data. No enum/switch or hard-coded hue band is required for a new colour.
struct RgbColour { double red=0,green=0,blue=0; };
inline bool ValidRgb(const RgbColour& c) {
  return std::isfinite(c.red) && std::isfinite(c.green) && std::isfinite(c.blue) &&
      c.red>=0 && c.red<=255 && c.green>=0 && c.green<=255 && c.blue>=0 && c.blue<=255;
}
struct ColourReference { std::string name; RgbColour mean; bool enabled=false; };
struct ColourEstimate {
  std::string name="Unknown";
  int reference=-1;
  RgbColour mean;
  double distance=-1,margin=0,maxStdDev=0;
};
struct ImageRegion { int x,y,width,height; };
inline bool ValidColourRegion(const ImageRegion& r) {
  return r.x>=0 && r.y>=0 && r.width>0 && r.height>0 &&
      r.x<640 && r.y<480 && r.width<=640-r.x && r.height<=480-r.y;
}
inline ImageRegion ScaleColourRegion(ImageRegion r,int width,int height) {
  if (!ValidColourRegion(r) || width<=0 || height<=0) return {0,0,0,0};
  const int x=static_cast<int>(r.x*(width/640.0)),y=static_cast<int>(r.y*(height/480.0));
  return {x,y,static_cast<int>((r.x+r.width)*(width/640.0))-x,
      static_cast<int>((r.y+r.height)*(height/480.0))-y};
}
inline ColourEstimate ClassifyAverageColour(RgbColour mean,double maxStdDev,
    const ColourReference* references,int count,double maxDistance,double minMargin,double maxAllowedStdDev) {
  ColourEstimate result; result.mean=mean; result.maxStdDev=maxStdDev;
  if (!ValidRgb(mean) || !std::isfinite(maxStdDev) || maxStdDev<0 ||
      !references || count<=0 || !std::isfinite(maxDistance) || maxDistance<=0 ||
      !std::isfinite(minMargin) || minMargin<0 || !std::isfinite(maxAllowedStdDev) || maxAllowedStdDev<0) return result;
  int best=-1;
  double nearest=std::numeric_limits<double>::infinity(),second=nearest;
  for (int i=0;i<count;++i) {
    const auto& r=references[i];
    if (!r.enabled || r.name.empty() || !ValidRgb(r.mean)) continue;
    const double dr=mean.red-r.mean.red,dg=mean.green-r.mean.green,db=mean.blue-r.mean.blue;
    const double d=std::sqrt(dr*dr+dg*dg+db*db);
    if (d<nearest) { second=nearest; nearest=d; best=i; }
    else if (d<second) second=d;
  }
  if (best<0) return result;
  result.distance=nearest;
  result.margin=std::isfinite(second)?second-nearest:std::sqrt(3.0)*255;
  if (nearest<=maxDistance && result.margin>=minMargin && maxStdDev<=maxAllowedStdDev) {
    result.name=references[best].name; result.reference=best;
  }
  return result;
}
struct ColourCalibration { ImageRegion roi; std::vector<ColourReference> references; };
ColourCalibration DefaultColourCalibration();
bool ValidateColourCalibration(const ColourCalibration& data,std::string& error);
bool SaveColourCalibration(const std::string& path,const ColourCalibration& data,std::string& error);
bool LoadColourCalibration(const std::string& path,ColourCalibration& data,std::string& error);

// Future autonomous object handling must report physically confirmed success.
