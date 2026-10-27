#pragma once

#include <vector>
#include "AutonomousConfig.h"

struct ArenaRect { double x0,y0,x1,y1; bool trustedWall; };
struct ArenaRayHit { double range = -1.0; int surface = -1; bool trustedWall = false; };

// Fixed map of solid divider segments and station/object envelopes. The large
// central start square and tape are floor markings, not solid obstacles.
class ArenaMap {
 public:
  ArenaMap();
  bool IsFree(ArenaPoint p) const;
  bool SegmentFree(ArenaPoint a, ArenaPoint b) const;
  bool Plan(ArenaPoint start, ArenaPoint goal, std::vector<ArenaPoint>& path) const;
  ArenaRayHit Raycast(ArenaPoint origin, double angle) const;
 private:
  std::vector<ArenaRect> obstacles_;
};
