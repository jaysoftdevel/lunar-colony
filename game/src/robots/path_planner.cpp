#include "game/robots/path_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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
    if(!callbacks.floor_height || !callbacks.point_clear || !callbacks.segment_clear || !callbacks.traffic_cost)
        return false;

    Vec3 start=request.start;
    Vec3 goal=request.goal;
    start.y=callbacks.floor_height(start);
    goal.y=callbacks.floor_height(goal);

    Vec3 delta=goal-start;
    delta.y=0.0f;
    const float direct_length=std::sqrt(delta.x*delta.x+delta.z*delta.z);
    if(direct_length<0.05f){
        geometric.push_back(goal);
        return true;
    }

    const Vec3 forward{delta.x/direct_length,0.0f,delta.z/direct_length};
    const Vec3 lateral{-forward.z,0.0f,forward.x};
    const float cell=std::max(0.25f,request.cell_size);
    const float along_margin=1.45f;
    const float side_extent=std::max(2.2f,request.corridor_half_width);
    const float u_min=-along_margin;
    const float u_max=direct_length+along_margin;
    const int nu=std::max(3,(int)std::ceil((u_max-u_min)/cell)+1);
    const int nv=std::max(5,(int)std::ceil((side_extent*2.0f)/cell)+1);
    const int total=nu*nv;
    auto index=[&](int u,int v){ return v*nu+u; };
    auto point=[&](int u,int v){
        const float along=u_min+(float)u*cell;
        const float side=-side_extent+(float)v*cell;
        Vec3 p=start+forward*along+lateral*side;
        p.y=callbacks.floor_height(p);
        return p;
    };

    const int su=std::clamp((int)std::lround((0.0f-u_min)/cell),0,nu-1);
    const int sv=std::clamp((int)std::lround(side_extent/cell),0,nv-1);
    const int start_index=index(su,sv);

    std::vector<std::int8_t> walkable((std::size_t)total,-1);
    std::vector<float> traffic_cache((std::size_t)total,-1.0f);

    // Failed wide-corridor searches used to spend all of their time on one
    // core evaluating the same static occupancy callbacks serially. For broad
    // searches, precompute those independent cells on a bounded worker set.
    // A* itself remains deterministic/serial; only immutable clearance and
    // traffic samples are evaluated concurrently.
    const unsigned requested_workers=std::max(1u,request.worker_count);
    const unsigned workers=std::min<unsigned>(requested_workers,(unsigned)std::max(1,total/160));
    if(workers>1u && total>=640){
        std::vector<std::thread> pool;
        pool.reserve(workers);
        for(unsigned worker=0;worker<workers;++worker){
            const int begin=(int)((std::int64_t)total*(std::int64_t)worker/(std::int64_t)workers);
            const int end=(int)((std::int64_t)total*(std::int64_t)(worker+1u)/(std::int64_t)workers);
            pool.emplace_back([&,begin,end]{
                for(int idx=begin;idx<end;++idx){
                    if(idx==start_index){ walkable[(std::size_t)idx]=1; traffic_cache[(std::size_t)idx]=0.0f; continue; }
                    const int u=idx%nu;
                    const int v=idx/nu;
                    const Vec3 p=point(u,v);
                    walkable[(std::size_t)idx]=callbacks.point_clear(p)?1:0;
                    float traffic=std::max(0.0f,callbacks.traffic_cost(p));
                    if(std::abs(request.alternating_side_preference)>0.001f){
                        const float signed_side=-side_extent+(float)v*cell;
                        traffic+=std::max(0.0f,signed_side*request.alternating_side_preference)*0.035f;
                    }
                    traffic_cache[(std::size_t)idx]=traffic;
                }
            });
        }
        for(auto& thread:pool) thread.join();
    }
    auto cell_clear=[&](int u,int v){
        const int idx=index(u,v);
        if(idx==start_index) return true;
        auto& cached=walkable[(std::size_t)idx];
        if(cached>=0) return cached!=0;
        const bool ok=callbacks.point_clear(point(u,v));
        cached=ok?1:0;
        return ok;
    };
    auto traffic_cost=[&](int u,int v){
        const int idx=index(u,v);
        float& cached=traffic_cache[(std::size_t)idx];
        if(cached>=0.0f) return cached;
        const Vec3 p=point(u,v);
        cached=std::max(0.0f,callbacks.traffic_cost(p));
        if(std::abs(request.alternating_side_preference)>0.001f){
            const float signed_side=-side_extent+(float)v*cell;
            cached+=std::max(0.0f,signed_side*request.alternating_side_preference)*0.035f;
        }
        return cached;
    };

    std::vector<float> g((std::size_t)total,std::numeric_limits<float>::infinity());
    // Geometric distance is tracked separately from weighted A* cost so the
    // dynamic collision callback receives a stable ETA even when traffic costs
    // deliberately bias the selected lane.
    std::vector<float> travelled((std::size_t)total,std::numeric_limits<float>::infinity());
    std::vector<int> parent((std::size_t)total,-1);
    std::vector<std::uint8_t> closed((std::size_t)total,0);
    struct OpenNode {
        float f;
        int idx;
        bool operator<(const OpenNode& other) const { return f>other.f; }
    };
    std::priority_queue<OpenNode> open;
    g[(std::size_t)start_index]=0.0f;
    travelled[(std::size_t)start_index]=0.0f;
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
        const float nominal_speed=std::max(0.10f,request.nominal_speed_mps);
        const float current_travel=std::isfinite(travelled[(std::size_t)cur])?travelled[(std::size_t)cur]:0.0f;
        if(dist_xz(cp,goal)<=goal_tolerance && callbacks.segment_clear(cp,goal)){
            const float goal_step=dist_xz(cp,goal);
            const float t0=current_travel/nominal_speed;
            const float t1=(current_travel+goal_step)/nominal_speed;
            if(!callbacks.dynamic_segment_clear || callbacks.dynamic_segment_clear(cp,goal,t0,t1)){
                goal_index=cur;
                break;
            }
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
            const float next_travel=current_travel+step;
            if(callbacks.dynamic_segment_clear){
                const float t0=current_travel/nominal_speed;
                const float t1=next_travel/nominal_speed;
                if(!callbacks.dynamic_segment_clear(cp,np,t0,t1)) continue;
            }
            const float ng=g[(std::size_t)cur]+step+traffic_cost(u,v)*request.traffic_weight;
            if(ng>=g[(std::size_t)ni]) continue;
            g[(std::size_t)ni]=ng;
            travelled[(std::size_t)ni]=next_travel;
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
    float simplified_travel=0.0f;
    const float nominal_speed=std::max(0.10f,request.nominal_speed_mps);
    std::size_t i=0;
    const std::size_t max_skip=std::max<std::size_t>(1,request.visibility_skip_limit);
    while(i<raw.size()){
        const std::size_t farthest=std::min(raw.size()-1,i+max_skip);
        std::size_t best=i;
        for(std::size_t candidate=farthest+1;candidate>i;--candidate){
            const std::size_t j=candidate-1;
            const float edge=dist_xz(anchor,raw[j]);
            const float t0=simplified_travel/nominal_speed;
            const float t1=(simplified_travel+edge)/nominal_speed;
            if(callbacks.segment_clear(anchor,raw[j]) &&
               (!callbacks.dynamic_segment_clear || callbacks.dynamic_segment_clear(anchor,raw[j],t0,t1))){
                best=j;
                break;
            }
        }
        simplified_travel+=dist_xz(anchor,raw[best]);
        geometric.push_back(raw[best]);
        anchor=raw[best];
        i=best+1;
    }
    return !geometric.empty();
}

} // namespace game::robots
