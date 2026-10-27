#include "ArenaMap.h"

#include <algorithm>
#include <limits>

namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
double Padding() { return autonomous_config::ROBOT_ENVELOPE_RADIUS_MM + autonomous_config::CLEARANCE_MM; }
ArenaRect Inflate(ArenaRect r) {
  const double p=Padding();
  return {r.x0-p,r.y0-p,r.x1+p,r.y1+p,r.trustedWall};
}
// Slab intersection, including tangency. Works for axis-aligned rays/segments.
bool Intersect(ArenaPoint p, ArenaPoint d, const ArenaRect& r, double& lo, double& hi) {
  const double origin[2]={p.x,p.y}, direction[2]={d.x,d.y};
  const double minimum[2]={r.x0,r.y0}, maximum[2]={r.x1,r.y1};
  for (int i=0;i<2;++i) {
    if (std::fabs(direction[i])<1e-9) {
      if (origin[i]<minimum[i] || origin[i]>maximum[i]) return false;
    } else {
      double a=(minimum[i]-origin[i])/direction[i];
      double b=(maximum[i]-origin[i])/direction[i];
      if (a>b) std::swap(a,b);
      lo=std::max(lo,a); hi=std::min(hi,b);
      if (lo>hi) return false;
    }
  }
  return hi>=lo;
}
}

ArenaMap::ArenaMap() {
  using namespace autonomous_config;
  obstacles_.push_back({DIVIDER_X_MIN_MM,0,DIVIDER_X_MAX_MM,LOWER_DIVIDER_END_Y_MM,true});
  obstacles_.push_back({DIVIDER_X_MIN_MM,UPPER_DIVIDER_START_Y_MM,DIVIDER_X_MAX_MM,localization_config::ARENA_HEIGHT_MM,true});
  const double h=STATION_SIDE_MM/2.0;
  for (const auto& s:STORAGES) obstacles_.push_back({s.center.x-h,s.center.y-h,s.center.x+h,s.center.y+h,false});
  for (const auto& s:DELIVERIES) obstacles_.push_back({s.center.x-h,s.center.y-h,s.center.x+h,s.center.y+h,false});
}

bool ArenaMap::IsFree(ArenaPoint p) const {
  const double pad=Padding();
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x<pad || p.y<pad ||
      p.x>localization_config::ARENA_WIDTH_MM-pad || p.y>localization_config::ARENA_HEIGHT_MM-pad) return false;
  for (const auto& obstacle:obstacles_) {
    const auto r=Inflate(obstacle);
    if (p.x>=r.x0 && p.x<=r.x1 && p.y>=r.y0 && p.y<=r.y1) return false;
  }
  return true;
}

bool ArenaMap::SegmentFree(ArenaPoint a, ArenaPoint b) const {
  if (!IsFree(a) || !IsFree(b)) return false;
  for (const auto& obstacle:obstacles_) {
    double lo=0,hi=1;
    if (Intersect(a,{b.x-a.x,b.y-a.y},Inflate(obstacle),lo,hi)) return false;
  }
  return true;
}

bool ArenaMap::Plan(ArenaPoint start, ArenaPoint goal, std::vector<ArenaPoint>& path) const {
  path.clear();
  if (!IsFree(start) || !IsFree(goal)) return false;
  // Visibility graph over inflated obstacle corners; at most 42 nodes.
  // Leave room for waypoint tolerance; a waypoint must not permit corner cuts.
  std::vector<ArenaPoint> nodes={start,goal};
  for (const auto& obstacle:obstacles_) {
    const auto r=Inflate(obstacle);
    const double extra=autonomous_config::POSITION_TOLERANCE_MM+10.0;
    for (auto p : {ArenaPoint{r.x0-extra,r.y0-extra},ArenaPoint{r.x0-extra,r.y1+extra},
                   ArenaPoint{r.x1+extra,r.y0-extra},ArenaPoint{r.x1+extra,r.y1+extra}})
      if (IsFree(p)) nodes.push_back(p);
  }
  const int n=static_cast<int>(nodes.size());
  std::vector<double> cost(n,kInfinity);
  std::vector<int> parent(n,-1);
  std::vector<bool> visited(n,false);
  cost[0]=0;
  for (int iteration=0;iteration<n;++iteration) {
    int best=-1;
    for (int i=0;i<n;++i)
      if (!visited[i] && (best<0 || cost[i]<cost[best])) best=i;
    if (best<0 || !std::isfinite(cost[best])) break;
    if (best==1) break;
    visited[best]=true;
    for (int next=0;next<n;++next) {
      if (visited[next] || next==best || !SegmentFree(nodes[best],nodes[next])) continue;
      const double candidate=cost[best]+robot_geometry::Distance(nodes[best],nodes[next]);
      if (candidate<cost[next]) { cost[next]=candidate; parent[next]=best; }
    }
  }
  if (!std::isfinite(cost[1])) return false;
  for (int i=1;i!=0;i=parent[i]) {
    if (i<0) { path.clear(); return false; }
    path.push_back(nodes[i]);
  }
  std::reverse(path.begin(),path.end());
  // Regular stationary observation opportunities, including long straight legs.
  std::vector<ArenaPoint> observations;
  ArenaPoint previous=start;
  for (auto p:path) {
    const int steps=std::max(1,static_cast<int>(std::ceil(robot_geometry::Distance(previous,p)/600.0)));
    for (int i=1;i<=steps;++i) {
      const double fraction=static_cast<double>(i)/steps;
      observations.push_back({previous.x+(p.x-previous.x)*fraction,previous.y+(p.y-previous.y)*fraction});
    }
    previous=p;
  }
  path=std::move(observations);
  return true;
}

ArenaRayHit ArenaMap::Raycast(ArenaPoint origin, double angle) const {
  ArenaRayHit hit;
  if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(angle) ||
      origin.x<=0 || origin.y<=0 || origin.x>=localization_config::ARENA_WIDTH_MM ||
      origin.y>=localization_config::ARENA_HEIGHT_MM) return hit;
  const ArenaPoint d={std::cos(angle),std::sin(angle)};
  double nearest=kInfinity;
  // Outer-wall IDs 0..3; divider IDs 4,5; untrusted objects IDs >=6.
  const double width=localization_config::ARENA_WIDTH_MM, height=localization_config::ARENA_HEIGHT_MM;
  const double ranges[4]={d.x<-1e-9?-origin.x/d.x:kInfinity,
      d.x>1e-9?(width-origin.x)/d.x:kInfinity,
      d.y<-1e-9?-origin.y/d.y:kInfinity,
      d.y>1e-9?(height-origin.y)/d.y:kInfinity};
  for (int i=0;i<4;++i) if (ranges[i]<nearest) {
    nearest=ranges[i]; hit={nearest,i,true};
  }
  for (unsigned i=0;i<obstacles_.size();++i) {
    double lo=0,hi=kInfinity;
    if (Intersect(origin,d,obstacles_[i],lo,hi) && lo<nearest) {
      nearest=lo; hit={lo,static_cast<int>(i)+4,obstacles_[i].trustedWall};
    }
  }
  return hit;
}
