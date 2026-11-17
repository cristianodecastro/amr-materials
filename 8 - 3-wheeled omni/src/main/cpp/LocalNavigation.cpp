#include "Autonomous.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace {
using navigation_config::Block;
double Gap(ArenaPoint p,const Block& b,double heading) {
  const double angle=robot_geometry::ArenaAngle(heading),co=std::cos(angle),si=std::sin(angle);
  const double r=navigation_config::BODY_RADIUS_MM,f=navigation_config::FRONT_REACH_MM;
  const double dx=std::max(b.xmin-p.x,std::max(0.0,p.x-b.xmax));
  const double dy=std::max(b.ymin-p.y,std::max(0.0,p.y-b.ymax));
  const double disk=std::hypot(dx,dy)-r;
  // Separating axes for the full-width front manipulator rectangle [0,f] x [-r,r].
  const double cx=(b.xmin+b.xmax)/2-p.x-co*f/2,cy=(b.ymin+b.ymax)/2-p.y-si*f/2;
  const double hx=(b.xmax-b.xmin)/2,hy=(b.ymax-b.ymin)/2;
  const double front=std::max(std::max(std::fabs(cx)-hx-std::fabs(co)*f/2-std::fabs(si)*r,
      std::fabs(cy)-hy-std::fabs(si)*f/2-std::fabs(co)*r),
      std::max(std::fabs(cx*co+cy*si)-hx*std::fabs(co)-hy*std::fabs(si)-f/2,
      std::fabs(-cx*si+cy*co)-hx*std::fabs(si)-hy*std::fabs(co)-r));
  return std::min(disk,front);
}
}
void LocalObstacleMap::AddHit(ArenaPoint p,double beam) {
  if (!std::isfinite(p.x) || !std::isfinite(p.y)) return;
  // Outer walls and the known structures already have their true extents.
  if (court_) {
    if (p.x<50 || p.y<50 || p.x>localization_config::ARENA_WIDTH_MM-50 || p.y>localization_config::ARENA_HEIGHT_MM-50) return;
    for (const auto& b:navigation_config::COURT_BLOCKS)
      if (p.x>=b.xmin-50 && p.x<=b.xmax+50 && p.y>=b.ymin-50 && p.y<=b.ymax+50) return;
  }
  const double r=navigation_config::SENSED_OBSTACLE_RADIUS_MM;
  // The return is the NEAR surface. Put assumed depth behind it, not toward us.
  const ArenaPoint center={p.x+r*std::cos(beam),p.y+r*std::sin(beam)};
  const double extent=r*(std::fabs(std::cos(beam))+std::fabs(std::sin(beam)));
  for (const auto& b:hits_) if (std::hypot(center.x-(b.xmin+b.xmax)/2,center.y-(b.ymin+b.ymax)/2)<r) return;
  if (hits_.size()>=24) hits_.erase(hits_.begin());
  hits_.push_back({center.x-extent,center.y-extent,center.x+extent,center.y+extent});
}
double LocalObstacleMap::Clearance(ArenaPoint p,double heading) const {
  double gap=1e9;
  if (court_) {
    const double r=navigation_config::BODY_RADIUS_MM;
    gap=std::min(std::min(p.x-r,localization_config::ARENA_WIDTH_MM-p.x-r),
        std::min(p.y-r,localization_config::ARENA_HEIGHT_MM-p.y-r));
    const double a=robot_geometry::ArenaAngle(heading),co=std::cos(a),si=std::sin(a);
    for (double side:{-r,r}) {
      const double x=p.x+co*navigation_config::FRONT_REACH_MM-si*side;
      const double y=p.y+si*navigation_config::FRONT_REACH_MM+co*side;
      gap=std::min(gap,std::min(std::min(x,localization_config::ARENA_WIDTH_MM-x),
          std::min(y,localization_config::ARENA_HEIGHT_MM-y)));
    }
    for (const auto& b:navigation_config::COURT_BLOCKS) gap=std::min(gap,Gap(p,b,heading));
  }
  for (const auto& b:hits_) gap=std::min(gap,Gap(p,b,heading));
  return gap;
}
void LocalObstacleMap::ShiftHits(ArenaPoint delta) {
  for (auto& b:hits_) { b.xmin+=delta.x; b.xmax+=delta.x; b.ymin+=delta.y; b.ymax+=delta.y; }
}
bool LocalObstacleMap::TurnClear(ArenaPoint center,double from,double to,double margin) const {
  const double angle=robot_geometry::Wrap(to-from);
  const int steps=std::max(1,static_cast<int>(std::ceil(std::fabs(angle)/robot_geometry::Radians(5))));
  for (int i=0;i<=steps;++i)
    if (Clearance(center,from+angle*i/steps)<margin) return false;
  return true;
}
bool LocalObstacleMap::SegmentClear(ArenaPoint a,ArenaPoint b,double heading,double margin,bool allowEscape) const {
  const int steps=std::max(1,static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/10)));
  double previous=Clearance(a,heading); bool escaping=allowEscape && previous>=0 && previous<margin;
  for (int i=0;i<=steps;++i) {
    const double t=static_cast<double>(i)/steps;
    const double gap=Clearance({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)},heading);
    if (gap>=margin) escaping=false;
    if (escaping ? gap<previous-1e-6 : gap<margin) return false;
    previous=gap;
  }
  return true;
}
bool LocalObstacleMap::Detour(ArenaPoint start,ArenaPoint goal,double heading,std::vector<ArenaPoint>& points) const {
  points.clear();
  const double margin=navigation_config::OBSTACLE_CLEARANCE_MM;
  if (Clearance(goal,heading)<margin || Clearance(start,heading)<0) return false;
  if (SegmentClear(start,goal,heading,margin,true)) { points.push_back(goal); return true; }
  const double step=navigation_config::DETOUR_GRID_MM;
  const int nx=static_cast<int>(localization_config::ARENA_WIDTH_MM/step)+1;
  const int ny=static_cast<int>(localization_config::ARENA_HEIGHT_MM/step)+1,n=nx*ny;
  const auto point=[=](int i) { return ArenaPoint{(i%nx)*step,(i/nx)*step}; };
  const auto distance=[](ArenaPoint a,ArenaPoint b) { return std::hypot(a.x-b.x,a.y-b.y); };
  std::vector<double> cost(n,std::numeric_limits<double>::infinity());
  std::vector<int> parent(n,-2); std::vector<bool> closed(n,false);
  using Entry=std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  // Exact start/goal are connected to grid nodes; neither is snapped into a wall.
  for (int y=std::max(0,static_cast<int>(start.y/step)-2);y<=std::min(ny-1,static_cast<int>(start.y/step)+2);++y)
    for (int x=std::max(0,static_cast<int>(start.x/step)-2);x<=std::min(nx-1,static_cast<int>(start.x/step)+2);++x) {
      const int i=y*nx+x; const auto p=point(i);
      if (SegmentClear(start,p,heading,margin,true)) {
        cost[i]=distance(start,p); parent[i]=-1; open.push({cost[i]+distance(p,goal),i});
      }
    }
  int end=-1;
  while (!open.empty()) {
    const int i=open.top().second; open.pop(); if (closed[i]) continue; closed[i]=true;
    const auto p=point(i);
    if (distance(p,goal)<=2*step && SegmentClear(p,goal,heading)) { end=i; break; }
    for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) {
      if (dx==0 && dy==0) continue;
      const int x=i%nx+dx,y=i/nx+dy;
      if (x<0 || x>=nx || y<0 || y>=ny) continue;
      const int j=y*nx+x; if (closed[j]) continue;
      const auto next=point(j);
      // Extra waypoint tolerance protects corners as the chassis blends segments.
      if (!SegmentClear(p,next,heading,margin+navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM)) continue;
      const double tentative=cost[i]+distance(p,next);
      if (tentative<cost[j]) { cost[j]=tentative; parent[j]=i; open.push({tentative+distance(next,goal),j}); }
    }
  }
  if (end<0) return false;
  std::vector<ArenaPoint> raw;
  for (int i=end;i>=0;i=parent[i]) raw.push_back(point(i));
  std::reverse(raw.begin(),raw.end()); raw.push_back(goal);
  ArenaPoint previous=start;
  for (unsigned i=0;i<raw.size();) {
    unsigned furthest=i;
    for (unsigned j=i+1;j<raw.size();++j)
      if (SegmentClear(previous,raw[j],heading,margin+navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM)) furthest=j;
    if (distance(previous,raw[furthest])>1) { points.push_back(raw[furthest]); previous=raw[furthest]; }
    i=furthest+1;
  }
  return !points.empty();
}
