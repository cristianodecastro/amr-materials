#pragma once

#include <cmath>

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

enum class PieceColour { Unknown, Red, Green, Yellow, Blue };
inline const char* PieceColourName(PieceColour colour) {
  switch (colour) {
    case PieceColour::Red:return "Red";
    case PieceColour::Green:return "Green";
    case PieceColour::Yellow:return "Yellow";
    case PieceColour::Blue:return "Blue";
    default:return "Unknown";
  }
}
struct ColourEstimate { PieceColour colour=PieceColour::Unknown; double dominance=0,coverage=0; };
// Pure HSV voting, independently testable without camera/hardware libraries.
class ColourVotes {
 public:
  void AddHsv(int hue,int saturation,int value,int minSaturation,int minValue) {
    ++total_;
    if (saturation<minSaturation || value<minValue) return;
    ++saturated_;
    const int colour=hue<=10 || hue>=170?1:
        (hue>=20 && hue<=38?3:(hue>=39 && hue<=90?2:(hue>=95 && hue<=135?4:0)));
    ++counts_[colour];
  }
  ColourEstimate Estimate(double minCoverage,double minDominance) const {
    ColourEstimate result;
    int best=1;
    for (int i=2;i<5;++i) if (counts_[i]>counts_[best]) best=i;
    result.dominance=saturated_?static_cast<double>(counts_[best])/saturated_:0;
    result.coverage=total_?static_cast<double>(counts_[best])/total_:0;
    if (result.coverage>=minCoverage && result.dominance>=minDominance)
      result.colour=static_cast<PieceColour>(best);
    return result;
  }
 private:
  unsigned counts_[5]={},saturated_=0,total_=0;
};

// Future autonomous object handling must report physically confirmed success.
