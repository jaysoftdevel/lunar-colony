#include "game/robots/path_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <queue>
#include <thread>

namespace game::robots {
namespace {

using eng::Vec3;

float dist_xz(Vec3 a,Vec3 b){
    const float dx=a.x-b.x;
    const float dz=a.z-b.z;
    return std::sqrt(dx*dx+dz*dz);
}

} // namespace

bool plan_corridor_path(const PathPlanRequest& request,
                        const PathPlannerCallbacks& callbacks,
                        std::vector<Vec3>& geometric){
    geometric.clear();
    if(!callbacks.floor_height || !callbacks.point_clear || !callbacks.segment_clear)
        return false;

    Vec3 start=request.start;
    Vec3 goal=request.goal;
    start.y=callbacks.floor_height(start);
    goal.y=callbacks.floor_height(goal);

    // Reject an impossible destination before allocating/searching a grid.
    // Construction logistics can temporarily select a service point that is
    // invalid for the rover's complete hull; searching thousands of cells for
    // such a goal only freezes the main thread and can never succeed.
    if(!callbacks.point_clear(goal)) return false;

    Vec3 delta=goal-start;
    delta.y=0.0f;
    const float direct_length=std::sqrt(delta.x*delta.x+delta.z*delta.z);
    if(direct_length<0.05f){
        geometric.push_back(goal);
        return true;
    }

    const Vec3 forward{delta.x/direct_length,0.0f,delta.z/direct_length};
    const Vec3 lateral{-forward.z,0.0f,forward.x};

    // Most rover travel in the colony is geometrically simple. Do not launch a
    // grid search for an open straight line or for a single building/parked
    // rover that can be passed with one stable dogleg. These candidates are
    // still validated by the caller's complete point/segment callbacks; this
    // stage only removes unnecessary A* work.
    if(callbacks.segment_clear(start,goal)){
        geometric.push_back(goal);
        return true;
    }

    std::vector<Vec3> best_simple;
    float best_simple_length=std::numeric_limits<float>::infinity();
    auto try_simple=[&](std::initializer_list<Vec3> points){
        std::vector<Vec3> route(points.begin(),points.end());
        if(route.empty()) return;
        Vec3 previous=start;
        float length=0.0f;
        for(auto& p:route){
            p.y=callbacks.floor_height(p);
            if(!callbacks.point_clear(p) || !callbacks.segment_clear(previous,p)) return;
            length+=dist_xz(previous,p);
            previous=p;
        }
        if(!callbacks.segment_clear(previous,goal)) return;
        length+=dist_xz(previous,goal);
        route.push_back(goal);
        if(length<best_simple_length){
            best_simple_length=length;
            best_simple=std::move(route);
        }
    };

    const float max_side=std::clamp(request.corridor_half_width,2.0f,20.0f);
    // Try a few wider doglegs before paying for A*. This handles long but
    // geometrically simple barriers (for example a corridor spanning between
    // two structures) with a handful of segment tests instead of thousands of
    // grid-cell checks.
    constexpr std::array<float,8> offsets{{1.6f,2.6f,3.8f,5.4f,7.2f,10.0f,14.0f,18.0f}};
    constexpr std::array<std::array<float,2>,3> spans{{
        std::array<float,2>{{0.18f,0.72f}},
        std::array<float,2>{{0.28f,0.72f}},
        std::array<float,2>{{0.28f,0.82f}}
    }};
    for(float raw_offset:offsets){
        const float offset=std::min(raw_offset,max_side);
        if(offset<0.75f) continue;
        for(float side:{-1.0f,1.0f}){
            const Vec3 mid=start+forward*(direct_length*0.50f)+lateral*(offset*side);
            try_simple({mid});
            for(const auto& span:spans){
                const Vec3 a=start+forward*(direct_length*span[0])+lateral*(offset*side);
                const Vec3 b=start+forward*(direct_length*span[1])+lateral*(offset*side);
                try_simple({a,b});
            }
        }
        // The first successful narrow bypass is normally preferable to a much
        // wider one. Keep searching only one additional band for a shorter
        // path, then stop; this bounds work in dense scenes.
        if(!best_simple.empty() && raw_offset>=2.6f) break;
    }
    if(!best_simple.empty()){
        geometric=std::move(best_simple);
        return true;
    }
    if(!request.allow_grid_fallback) return false;

    const float cell=std::max(0.45f,request.cell_size);
    const float along_margin=1.45f;
    const float side_extent=std::max(2.2f,request.corridor_half_width);
    const float u_min=-along_margin;
    const float u_max=direct_length+along_margin;
    const int nu=std::max(3,(int)std::ceil((u_max-u_min)/cell)+1);
    const int nv=std::max(5,(int)std::ceil((side_extent*2.0f)/cell)+1);
    const int total=nu*nv;
    auto index=[&](int u,int v){ return v*nu+u; };
    // Terrain height is one of the more expensive callbacks in a wide search.
    // A* used to query the same grid point repeatedly from occupancy, expansion and simplification. Cache each grid coordinate on first use.
    std::vector<Vec3> point_cache((std::size_t)total);
    std::vector<std::uint8_t> point_ready((std::size_t)total,0);
    auto point=[&](int u,int v){
        const int idx=index(u,v);
        if(point_ready[(std::size_t)idx]) return point_cache[(std::size_t)idx];
        const float along=u_min+(float)u*cell;
        const float side=-side_extent+(float)v*cell;
        Vec3 p=start+forward*along+lateral*side;
        p.y=callbacks.floor_height(p);
        point_cache[(std::size_t)idx]=p;
        point_ready[(std::size_t)idx]=1;
        return p;
    };

    const int su=std::clamp((int)std::lround((0.0f-u_min)/cell),0,nu-1);
    const int sv=std::clamp((int)std::lround(side_extent/cell),0,nv-1);
    const int start_index=index(su,sv);

    std::vector<std::int8_t> walkable((std::size_t)total,-1);
    // Occupancy is evaluated lazily by A*. The caller may include current
    // fleet positions as temporary local obstacles for route choice; dynamic
    // time-indexed arbitration and continuous collision certification still
    // belong exclusively to the safety coordinator.
    auto cell_clear=[&](int u,int v){
        const int idx=index(u,v);
        if(idx==start_index) return true;
        auto& cached=walkable[(std::size_t)idx];
        if(cached>=0) return cached!=0;
        const bool ok=callbacks.point_clear(point(u,v));
        cached=ok?1:0;
        return ok;
    };
    std::vector<float> g((std::size_t)total,std::numeric_limits<float>::infinity());
    std::vector<int> parent((std::size_t)total,-1);
    std::vector<std::uint8_t> closed((std::size_t)total,0);
    struct OpenNode {
        float f;
        int idx;
        bool operator<(const OpenNode& other) const { return f>other.f; }
    };
    std::priority_queue<OpenNode> open;
    g[(std::size_t)start_index]=0.0f;
    open.push({direct_length,start_index});

    constexpr std::array<int,8> du{{1,-1,0,0,1,1,-1,-1}};
    constexpr std::array<int,8> dv{{0,0,1,-1,1,-1,1,-1}};
    int goal_index=-1;
    int expansions=0;
    const int max_expansions=std::clamp(request.max_expansions,64,std::max(64,total));
    const float goal_tolerance=std::max(request.stop_radius+0.16f,cell*0.82f);

    while(!open.empty() && expansions<max_expansions){
        const int cur=open.top().idx;
        open.pop();
        if(closed[(std::size_t)cur]) continue;
        closed[(std::size_t)cur]=1;
        ++expansions;

        const int cu=cur%nu;
        const int cv=cur/nu;
        const Vec3 cp=(cur==start_index)?start:point(cu,cv);
        if(dist_xz(cp,goal)<=goal_tolerance && callbacks.segment_clear(cp,goal)){
            goal_index=cur;
            break;
        }

        for(std::size_t n=0;n<du.size();++n){
            const int u=cu+du[n];
            const int v=cv+dv[n];
            if(u<0 || v<0 || u>=nu || v>=nv) continue;
            const int ni=index(u,v);
            if(closed[(std::size_t)ni] || !cell_clear(u,v)) continue;
            if(du[n]!=0 && dv[n]!=0 && (!cell_clear(cu+du[n],cv) || !cell_clear(cu,cv+dv[n]))) continue;
            const Vec3 np=point(u,v);
            if(std::abs(np.y-cp.y)>0.38f) continue;
            if(callbacks.neighbor_clear){
                if(!callbacks.neighbor_clear(cp,np)) continue;
            }else if(!callbacks.segment_clear(cp,np)) continue;
            const float step=dist_xz(cp,np);
            const float ng=g[(std::size_t)cur]+step;
            if(ng>=g[(std::size_t)ni]) continue;
            g[(std::size_t)ni]=ng;
            parent[(std::size_t)ni]=cur;
            open.push({ng+dist_xz(np,goal),ni});
        }
    }
    if(goal_index<0) return false;

    std::vector<Vec3> raw;
    raw.reserve((std::size_t)std::max(8,(int)(direct_length/cell)+8));
    for(int cur=goal_index,guard=0;cur!=start_index && cur>=0 && guard++<total;cur=parent[(std::size_t)cur]){
        raw.push_back(point(cur%nu,cur/nu));
        if(parent[(std::size_t)cur]<0) break;
    }
    std::reverse(raw.begin(),raw.end());
    if(raw.empty()) return false;
    if(dist_xz(raw.back(),goal)>0.04f){
        if(!callbacks.segment_clear(raw.back(),goal)) return false;
        raw.push_back(goal);
    }

    // Theta*-style bounded line-of-sight simplification. It intentionally keeps
    // the lookahead bounded so a large map cannot turn one replan into O(N^2)
    // segment validation.
    Vec3 anchor=start;
    std::size_t i=0;
    const std::size_t max_skip=std::max<std::size_t>(1,request.visibility_skip_limit);
    while(i<raw.size()){
        const std::size_t farthest=std::min(raw.size()-1,i+max_skip);
        std::size_t best=i;
        for(std::size_t candidate=farthest+1;candidate>i;--candidate){
            const std::size_t j=candidate-1;
            if(callbacks.segment_clear(anchor,raw[j])){
                best=j;
                break;
            }
        }
        geometric.push_back(raw[best]);
        anchor=raw[best];
        i=best+1;
    }
    return !geometric.empty();
}

} // namespace game::robots
