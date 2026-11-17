#include "Autonomous.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#ifdef __linux__
#include <sys/resource.h>
#endif

struct NavigationPlanWorker::Impl {
  std::mutex mutex;
  std::condition_variable wake;
  std::function<void()> work;
  std::atomic<bool> busy{false},ready{false};
  std::atomic<double> elapsed{0};
  bool stopping=false;
  std::thread thread;
};
NavigationPlanWorker::NavigationPlanWorker():impl_(new Impl) {
  auto* state=impl_.get();
  state->thread=std::thread([state] {
#ifdef __linux__
    setpriority(PRIO_PROCESS,0,10); // reduce only this Linux worker's priority
#endif
    for (;;) {
      std::function<void()> work;
      {
        std::unique_lock<std::mutex> lock(state->mutex);
        state->wake.wait(lock,[state] { return state->stopping || static_cast<bool>(state->work); });
        if (state->stopping) return;
        work=std::move(state->work);
      }
      const auto start=std::chrono::steady_clock::now();
      work();
      state->elapsed.store(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
      state->ready.store(true,std::memory_order_release);
      state->busy.store(false,std::memory_order_release);
    }
  });
}
NavigationPlanWorker::~NavigationPlanWorker() {
  { std::lock_guard<std::mutex> lock(impl_->mutex); impl_->stopping=true; }
  impl_->wake.notify_one();
  impl_->thread.join();
}
bool NavigationPlanWorker::Submit(std::function<void()> work) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->busy.load(std::memory_order_acquire)) return false;
  impl_->ready.store(false,std::memory_order_release);
  impl_->busy.store(true,std::memory_order_release);
  impl_->work=std::move(work); impl_->wake.notify_one(); return true;
}
bool NavigationPlanWorker::Busy() const { return impl_->busy.load(std::memory_order_acquire); }
bool NavigationPlanWorker::Ready() const { return impl_->ready.load(std::memory_order_acquire) && !Busy(); }
double NavigationPlanWorker::ElapsedMS() const { return impl_->elapsed.load(); }

namespace {
using navigation_config::Block;
void BodyAxes(double heading,double& co,double& si) {
  // Grid checks repeat one orientation thousands of times. Cache per thread;
  // the robot and worker never share mutable trigonometry state.
  thread_local double previous=std::numeric_limits<double>::quiet_NaN(),cachedCo=0,cachedSi=0;
  if (heading!=previous) { previous=heading; cachedCo=std::cos(heading+robot_geometry::PI/2); cachedSi=std::sin(heading+robot_geometry::PI/2); }
  co=cachedCo; si=cachedSi;
}
double Gap(ArenaPoint p,const Block& b,double co,double si) {
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
  for (auto& hit:hits_) {
    const auto& b=hit.bounds;
    if (std::hypot(center.x-(b.xmin+b.xmax)/2,center.y-(b.ymin+b.ymax)/2)<r) { hit.clearVotes=0; return; }
  }
  if (hits_.size()>=24) hits_.erase(hits_.begin());
  hits_.push_back({{center.x-extent,center.y-extent,center.x+extent,center.y+extent},0});
  ++revision_;
}
void LocalObstacleMap::ObserveHit(ArenaPoint origin,ArenaPoint hit,double beam,double stamp,int sensor,bool urgent,bool allowObstacle) {
  if (sensor<0 || sensor>=4 || !std::isfinite(stamp)) return;
  if (stamp<=rayStamp_[sensor]) return;
  rayStamp_[sensor]=stamp;
  // A new ray that passes entirely through an old patch twice disproves it.
  // This clears bad echoes without forgetting an unseen, confirmed obstacle.
  const double range=std::hypot(hit.x-origin.x,hit.y-origin.y),direction[]={std::cos(beam),std::sin(beam)};
  for (auto it=hits_.begin();it!=hits_.end();) {
    double entry=0,exit=range; bool intersects=true;
    const double starts[]={origin.x,origin.y},lo[]={it->bounds.xmin,it->bounds.ymin},hi[]={it->bounds.xmax,it->bounds.ymax};
    for (int axis=0;axis<2;++axis) {
      if (std::fabs(direction[axis])<1e-9) {
        if (starts[axis]<lo[axis] || starts[axis]>hi[axis]) intersects=false;
      } else {
        double a=(lo[axis]-starts[axis])/direction[axis],b=(hi[axis]-starts[axis])/direction[axis];
        if (a>b) std::swap(a,b);
        entry=std::max(entry,a); exit=std::min(exit,b);
      }
    }
    if (intersects && exit>=entry && exit<range-navigation_config::OBSTACLE_FREE_RAY_MARGIN_MM) {
      if (++it->clearVotes>=2) { it=hits_.erase(it); ++revision_; continue; }
    }
    ++it;
  }
  auto& pending=pending_[sensor];
  if (!allowObstacle) { pending.stamp=-1; return; }
  const bool repeated=pending.stamp>=0 && stamp>pending.stamp && stamp-pending.stamp<=navigation_config::OBSTACLE_CONFIRM_WINDOW_S &&
      std::hypot(hit.x-pending.hit.x,hit.y-pending.hit.y)<=navigation_config::OBSTACLE_MATCH_MM;
  if (urgent || repeated) AddHit(hit,beam);
  pending={hit,stamp};
}
double LocalObstacleMap::Clearance(ArenaPoint p,double heading) const {
  double co,si; BodyAxes(heading,co,si);
  double gap=1e9;
  if (court_) {
    const double r=navigation_config::BODY_RADIUS_MM;
    gap=std::min(std::min(p.x-r,localization_config::ARENA_WIDTH_MM-p.x-r),
        std::min(p.y-r,localization_config::ARENA_HEIGHT_MM-p.y-r));
    for (double side:{-r,r}) {
      const double x=p.x+co*navigation_config::FRONT_REACH_MM-si*side;
      const double y=p.y+si*navigation_config::FRONT_REACH_MM+co*side;
      gap=std::min(gap,std::min(std::min(x,localization_config::ARENA_WIDTH_MM-x),
          std::min(y,localization_config::ARENA_HEIGHT_MM-y)));
    }
    for (const auto& b:navigation_config::COURT_BLOCKS) gap=std::min(gap,Gap(p,b,co,si));
  }
  for (const auto& hit:hits_) gap=std::min(gap,Gap(p,hit.bounds,co,si));
  return gap;
}
void LocalObstacleMap::ShiftHits(ArenaPoint delta) {
  for (auto& hit:hits_) { auto& b=hit.bounds; b.xmin+=delta.x; b.xmax+=delta.x; b.ymin+=delta.y; b.ymax+=delta.y; }
  for (auto& pending:pending_) if (pending.stamp>=0) { pending.hit.x+=delta.x; pending.hit.y+=delta.y; }
}
bool LocalObstacleMap::TurnClear(ArenaPoint center,double from,double to,double margin) const {
  if (Cancelled()) return false;
  const double angle=robot_geometry::Wrap(to-from);
  const int steps=std::max(1,static_cast<int>(std::ceil(std::fabs(angle)/robot_geometry::Radians(5))));
  for (int i=0;i<=steps;++i)
    if (Clearance(center,from+angle*i/steps)<margin) return false;
  return true;
}
bool LocalObstacleMap::SegmentClear(ArenaPoint a,ArenaPoint b,double heading,double margin,bool allowEscape) const {
  if (Cancelled()) return false;
  const int steps=std::max(1,static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/10)));
  double previous=Clearance(a,heading); bool escaping=allowEscape && previous>=0 && previous<margin;
  for (int i=0;i<=steps;++i) {
    if ((i&7)==0 && Cancelled()) return false;
    const double t=static_cast<double>(i)/steps;
    const double gap=Clearance({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)},heading);
    if (gap>=margin) escaping=false;
    if (escaping ? gap<previous-1e-6 : gap<margin) return false;
    previous=gap;
  }
  return true;
}
bool LocalObstacleMap::Detour(ArenaPoint start,ArenaPoint goal,double heading,std::vector<ArenaPoint>& points) const {
  if (SearchGrid(start,goal,heading,points,navigation_config::DETOUR_GRID_MM,navigation_config::INTERMEDIATE_POSITION_TOLERANCE_MM)) return true;
  // A coarse grid / passing buffer can close a real, clear passage. Refine
  // discretization and tracking padding; never shrink the robot or clearance.
  return SearchGrid(start,goal,heading,points,navigation_config::DETOUR_GRID_MM/2,10);
}
bool LocalObstacleMap::SearchGrid(ArenaPoint start,ArenaPoint goal,double heading,std::vector<ArenaPoint>& points,double step,double padding) const {
  points.clear();
  const double margin=navigation_config::OBSTACLE_CLEARANCE_MM;
  if (Clearance(goal,heading)<margin || Clearance(start,heading)<0) return false;
  if (SegmentClear(start,goal,heading,margin,true)) { points.push_back(goal); return true; }
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
    if (Cancelled()) return false;
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
      if (!SegmentClear(p,next,heading,margin+padding)) continue;
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
    for (unsigned j=raw.size()-1;j>i;--j)
      if (SegmentClear(previous,raw[j],heading,margin+padding)) { furthest=j; break; }
    if (distance(previous,raw[furthest])>1) { points.push_back(raw[furthest]); previous=raw[furthest]; }
    i=furthest+1;
  }
  return !points.empty();
}
