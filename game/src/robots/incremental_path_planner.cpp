#include "game/robots/path_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>

namespace game::robots {
namespace {

using eng::Vec3;
using State=IncrementalPathPlannerState;
constexpr float kInf=std::numeric_limits<float>::infinity();
constexpr float kKeyEpsilon=1.0e-5f;
constexpr std::array<int,8> kDx{{1,-1,0,0,1,1,-1,-1}};
constexpr std::array<int,8> kDz{{0,0,1,-1,1,-1,1,-1}};
constexpr std::array<int,8> kOpposite{{1,0,3,2,7,6,5,4}};

float dist_xz(Vec3 a,Vec3 b){
    const float dx=a.x-b.x;
    const float dz=a.z-b.z;
    return std::sqrt(dx*dx+dz*dz);
}

bool finite_cost(float value){ return std::isfinite(value); }
bool same_value(float a,float b){
    if(!finite_cost(a) || !finite_cost(b)) return finite_cost(a)==finite_cost(b);
    return std::abs(a-b)<=1.0e-4f;
}

bool key_less(const State::OpenEntry& a,const State::OpenEntry& b){
    if(a.k1<b.k1-kKeyEpsilon) return true;
    if(a.k1>b.k1+kKeyEpsilon) return false;
    return a.k2<b.k2-kKeyEpsilon;
}

struct OpenCompare {
    bool operator()(const State::OpenEntry& a,const State::OpenEntry& b) const {
        // std::push_heap is a max heap by default. Reverse lexicographic key
        // ordering so the smallest D* Lite key remains at the front.
        return key_less(b,a);
    }
};

int index_of(const State& s,int x,int z){ return z*s.nx+x; }
bool in_bounds(const State& s,int x,int z){ return x>=0 && z>=0 && x<s.nx && z<s.nz; }

std::array<int,2> coordinates(const State& s,int index){
    return {index%s.nx,index/s.nx};
}

Vec3 point_for(const State& s,int index,const PathPlannerCallbacks& callbacks){
    const auto c=coordinates(s,index);
    Vec3 p{s.origin_x+(float)c[0]*s.cell_size,0.0f,
           s.origin_z+(float)c[1]*s.cell_size};
    p.y=callbacks.floor_height?callbacks.floor_height(p):0.0f;
    return p;
}

float heuristic(const State& s,int a,int b){
    const auto ac=coordinates(s,a);
    const auto bc=coordinates(s,b);
    const float dx=(float)(ac[0]-bc[0])*s.cell_size;
    const float dz=(float)(ac[1]-bc[1])*s.cell_size;
    return std::sqrt(dx*dx+dz*dz);
}

State::OpenEntry calculate_key(const State& s,int index){
    const float v=std::min(s.g[(std::size_t)index],s.rhs[(std::size_t)index]);
    return {v+heuristic(s,s.last_start_index,index)+s.km,v,index,0};
}

void heap_push(State& s,int index){
    if(index<0 || index>=(int)s.g.size()) return;
    if(same_value(s.g[(std::size_t)index],s.rhs[(std::size_t)index])) return;
    auto key=calculate_key(s,index);
    key.serial=++s.open_serial;
    s.open_heap.push_back(key);
    std::push_heap(s.open_heap.begin(),s.open_heap.end(),OpenCompare{});
}

bool heap_pop(State& s,State::OpenEntry& result){
    if(s.open_heap.empty()) return false;
    std::pop_heap(s.open_heap.begin(),s.open_heap.end(),OpenCompare{});
    result=s.open_heap.back();
    s.open_heap.pop_back();
    return true;
}

float sample_penalty(State& s,int index,std::uint64_t epoch,
                     const PathPlannerCallbacks& callbacks,bool* changed=nullptr){
    if(index<0 || index>=(int)s.node_penalty.size()) return kInf;
    const std::size_t ui=(std::size_t)index;
    if(s.node_penalty_epoch[ui]==epoch) return s.node_penalty[ui];

    const float old=s.node_penalty[ui];
    const bool was_known=s.known_node[ui]!=0;
    const Vec3 p=point_for(s,index,callbacks);
    float value=kInf;
    if(callbacks.point_clear && callbacks.point_clear(p)){
        value=0.0f;
        if(callbacks.traversal_cost){
            value=callbacks.traversal_cost(p);
            if(std::isnan(value)) value=kInf;
            if(finite_cost(value)) value=std::max(0.0f,value);
        }
    }
    s.node_penalty[ui]=value;
    s.node_penalty_epoch[ui]=epoch;
    if(!was_known){
        s.known_node[ui]=1;
        s.known_nodes.push_back(index);
    }
    if(changed) *changed=was_known && !same_value(old,value);
    return value;
}

int neighbor_index(const State& s,int index,int dir){
    const auto c=coordinates(s,index);
    const int x=c[0]+kDx[(std::size_t)dir];
    const int z=c[1]+kDz[(std::size_t)dir];
    return in_bounds(s,x,z)?index_of(s,x,z):-1;
}

float compute_edge_cost(State& s,int index,int dir,std::uint64_t epoch,
                        const PathPlannerCallbacks& callbacks,bool* changed=nullptr){
    const int next=neighbor_index(s,index,dir);
    if(next<0) return kInf;
    const std::size_t edge_slot=(std::size_t)index*8u+(std::size_t)dir;
    if(s.edge_cost_epoch[edge_slot]==epoch) return s.edge_cost[edge_slot];

    const float old=s.edge_cost[edge_slot];
    const bool had_old=s.edge_cost_epoch[edge_slot]!=0;
    const float p0=sample_penalty(s,index,epoch,callbacks);
    const float p1=sample_penalty(s,next,epoch,callbacks);
    float value=kInf;
    if(finite_cost(p0) && finite_cost(p1)){
        const auto c=coordinates(s,index);
        bool diagonal_clear=true;
        if(kDx[(std::size_t)dir]!=0 && kDz[(std::size_t)dir]!=0){
            const int side_a=index_of(s,c[0]+kDx[(std::size_t)dir],c[1]);
            const int side_b=index_of(s,c[0],c[1]+kDz[(std::size_t)dir]);
            if(!finite_cost(sample_penalty(s,side_a,epoch,callbacks)) ||
               !finite_cost(sample_penalty(s,side_b,epoch,callbacks))){
                diagonal_clear=false;
            }
        }
        if(diagonal_clear){
            const Vec3 a=point_for(s,index,callbacks);
            const Vec3 b=point_for(s,next,callbacks);
            if(std::abs(a.y-b.y)<=0.38f){
                const bool clear=callbacks.neighbor_clear?callbacks.neighbor_clear(a,b):
                    (callbacks.segment_clear && callbacks.segment_clear(a,b));
                if(clear){
                    const float step=(kDx[(std::size_t)dir]!=0 && kDz[(std::size_t)dir]!=0)?
                        s.cell_size*1.41421356237f:s.cell_size;
                    // Soft obstacle inflation: a path through the preferred
                    // clearance halo is legal but pays increasingly more than
                    // open terrain. This lets D* choose narrow passages only
                    // when no sensible wider route exists.
                    value=step*(1.0f+0.24f*(p0+p1));
                }
            }
        }
    }

    s.edge_cost[edge_slot]=value;
    s.edge_cost_epoch[edge_slot]=epoch;
    const std::size_t reverse_slot=(std::size_t)next*8u+(std::size_t)kOpposite[(std::size_t)dir];
    if(reverse_slot<s.edge_cost.size()){
        s.edge_cost[reverse_slot]=value;
        s.edge_cost_epoch[reverse_slot]=epoch;
    }
    if(changed) *changed=had_old && !same_value(old,value);
    return value;
}

void update_vertex(State& s,int index,std::uint64_t epoch,const PathPlannerCallbacks& callbacks){
    if(index<0 || index>=(int)s.g.size()) return;
    if(!s.terminal[(std::size_t)index]){
        float best=kInf;
        for(int dir=0;dir<8;++dir){
            const int next=neighbor_index(s,index,dir);
            if(next<0) continue;
            const float edge=compute_edge_cost(s,index,dir,epoch,callbacks);
            if(!finite_cost(edge) || !finite_cost(s.g[(std::size_t)next])) continue;
            best=std::min(best,edge+s.g[(std::size_t)next]);
        }
        s.rhs[(std::size_t)index]=best;
    }else{
        s.rhs[(std::size_t)index]=0.0f;
    }
    heap_push(s,index);
}

void update_around(State& s,int index,std::uint64_t epoch,const PathPlannerCallbacks& callbacks){
    update_vertex(s,index,epoch,callbacks);
    for(int dir=0;dir<8;++dir){
        const int n=neighbor_index(s,index,dir);
        if(n>=0) update_vertex(s,n,epoch,callbacks);
    }
}

enum class ComputeStatus { Converged, Pending };

ComputeStatus compute_shortest_path(State& s,std::uint64_t epoch,const PathPlannerCallbacks& callbacks,
                                    int max_expansions,std::chrono::steady_clock::time_point deadline){
    int expansions=0;
    const int bounded_max=std::max(64,max_expansions);
    while(expansions<bounded_max){
        if(s.global_search && std::chrono::steady_clock::now()>=deadline)return ComputeStatus::Pending;
        const auto start_key=calculate_key(s,s.last_start_index);
        if(s.open_heap.empty())
            return ComputeStatus::Converged;
        const auto top=s.open_heap.front();
        const bool start_inconsistent=!same_value(s.rhs[(std::size_t)s.last_start_index],s.g[(std::size_t)s.last_start_index]);
        if(!key_less(top,start_key) && !start_inconsistent) return ComputeStatus::Converged;

        State::OpenEntry old_key{};
        if(!heap_pop(s,old_key)) break;
        const int u=old_key.index;
        if(u<0 || u>=(int)s.g.size()) continue;
        // Lazy heap deletion: entries for vertices that became consistent are
        // harmless stale duplicates and must not execute the D* overconsistent
        // branch a second time.
        if(same_value(s.g[(std::size_t)u],s.rhs[(std::size_t)u])) continue;

        const auto new_key=calculate_key(s,u);
        if(key_less(old_key,new_key)){
            heap_push(s,u);
            continue;
        }

        ++expansions;
        ++s.expanded_node_count;
        if(s.g[(std::size_t)u]>s.rhs[(std::size_t)u]){
            s.g[(std::size_t)u]=s.rhs[(std::size_t)u];
            for(int dir=0;dir<8;++dir){
                const int pred=neighbor_index(s,u,dir);
                if(pred>=0) update_vertex(s,pred,epoch,callbacks);
            }
        }else{
            s.g[(std::size_t)u]=kInf;
            update_vertex(s,u,epoch,callbacks);
            for(int dir=0;dir<8;++dir){
                const int pred=neighbor_index(s,u,dir);
                if(pred>=0) update_vertex(s,pred,epoch,callbacks);
            }
        }
    }
    return ComputeStatus::Pending;
}

int nearest_grid_index(const State& s,Vec3 p){
    const int x=(int)std::lround((p.x-s.origin_x)/s.cell_size);
    const int z=(int)std::lround((p.z-s.origin_z)/s.cell_size);
    return in_bounds(s,x,z)?index_of(s,x,z):-1;
}

int find_start_index(State& s,Vec3 actual,std::uint64_t epoch,const PathPlannerCallbacks& callbacks){
    const int nearest=nearest_grid_index(s,actual);
    if(nearest<0) return -1;
    const auto base=coordinates(s,nearest);
    int best=-1;
    float best_distance=kInf;
    for(int radius=0;radius<=(s.global_search?12:3);++radius){
        for(int dz=-radius;dz<=radius;++dz){
            for(int dx=-radius;dx<=radius;++dx){
                if(radius>0 && std::max(std::abs(dx),std::abs(dz))!=radius) continue;
                const int x=base[0]+dx,z=base[1]+dz;
                if(!in_bounds(s,x,z)) continue;
                const int idx=index_of(s,x,z);
                if(!finite_cost(sample_penalty(s,idx,epoch,callbacks))) continue;
                const Vec3 p=point_for(s,idx,callbacks);
                if(callbacks.segment_clear && !callbacks.segment_clear(actual,p)) continue;
                const float d=dist_xz(actual,p);
                if(d<best_distance){ best=idx; best_distance=d; }
            }
        }
        if(best>=0) return best;
    }
    return -1;
}

bool initialize_state(const PathPlanRequest& request,std::uint64_t epoch,
                      const PathPlannerCallbacks& callbacks,State& s,bool global=false){
    reset_incremental_path_state(s);
    s.global_search=global;
    if(!callbacks.floor_height || !callbacks.point_clear || !callbacks.segment_clear) return false;
    if(!callbacks.point_clear(request.goal)) return false;

    float cell=global?std::clamp(request.cell_size,0.78f,1.05f):std::max(0.78f,request.cell_size);
    const float margin=std::clamp(request.corridor_half_width,6.0f,34.0f);
    float min_x=std::min(request.start.x,request.goal.x)-margin;
    float max_x=std::max(request.start.x,request.goal.x)+margin;
    float min_z=std::min(request.start.z,request.goal.z)-margin;
    float max_z=std::max(request.start.z,request.goal.z)+margin;
    if(global && request.world_half_extent>0){
        min_x=min_z=-request.world_half_extent;
        max_x=max_z=request.world_half_extent;
    }
    // Anchor the lattice at the EXACT destination. A valid service bay can be
    // narrower than one cell; the old start-relative phase could put every
    // nearby row inside an adjacent building, returning NoPath without even
    // expanding a node. The goal and its approach now exist in the graph.
    const auto dimensions=[&](){
        if(global){
            min_x=request.goal.x-std::ceil((request.goal.x-min_x)/cell)*cell;
            min_z=request.goal.z-std::ceil((request.goal.z-min_z)/cell)*cell;
        }
        return std::array<int,2>{{std::max(5,(int)std::ceil((max_x-min_x)/cell)+1),
                                 std::max(5,(int)std::ceil((max_z-min_z)/cell)+1)}};
    };
    auto dims=dimensions();
    const int max_cells=global?40000:10000;
    if((std::int64_t)dims[0]*dims[1]>max_cells){
        cell*=std::sqrt((float)((std::int64_t)dims[0]*dims[1])/(float)max_cells)*(global?1.01f:1.0f);
        dims=dimensions();
    }
    const int nx=dims[0],nz=dims[1];

    s.initialized=true;
    s.cell_size=cell;
    s.origin_x=min_x;
    s.origin_z=min_z;
    s.nx=nx;
    s.nz=nz;
    s.goal=request.goal;
    s.world_epoch=epoch;
    const std::size_t total=(std::size_t)nx*(std::size_t)nz;
    s.g.assign(total,kInf);
    s.rhs.assign(total,kInf);
    s.node_penalty.assign(total,kInf);
    s.node_penalty_epoch.assign(total,0);
    s.known_node.assign(total,0);
    s.terminal.assign(total,0);
    s.edge_cost.assign(total*8u,kInf);
    s.edge_cost_epoch.assign(total*8u,0);
    s.known_nodes.clear();
    s.open_heap.clear();

    s.last_start_index=find_start_index(s,request.start,epoch,callbacks);
    if(s.last_start_index<0) return false;
    s.km=0.0f;

    const int goal_nearest=nearest_grid_index(s,request.goal);
    if(goal_nearest<0) return false;
    const auto gc=coordinates(s,goal_nearest);
    const float goal_tolerance=std::max(request.stop_radius+0.20f,s.cell_size*1.20f);
    const int terminal_radius=std::clamp((int)std::ceil(goal_tolerance/s.cell_size)+1,1,4);
    int terminal_count=0;
    for(int dz=-terminal_radius;dz<=terminal_radius;++dz){
        for(int dx=-terminal_radius;dx<=terminal_radius;++dx){
            const int x=gc[0]+dx,z=gc[1]+dz;
            if(!in_bounds(s,x,z)) continue;
            const int idx=index_of(s,x,z);
            const Vec3 p=point_for(s,idx,callbacks);
            if(dist_xz(p,request.goal)>goal_tolerance) continue;
            if(!finite_cost(sample_penalty(s,idx,epoch,callbacks))) continue;
            if(!callbacks.segment_clear(p,request.goal)) continue;
            s.terminal[(std::size_t)idx]=1;
            s.rhs[(std::size_t)idx]=0.0f;
            heap_push(s,idx);
            ++terminal_count;
        }
    }
    if(terminal_count==0) return false;
    ++s.initialization_count;
    return true;
}

bool state_compatible(const PathPlanRequest& request,const State& s){
    if(!s.initialized || s.nx<=0 || s.nz<=0) return false;
    if(dist_xz(request.goal,s.goal)>std::max(0.18f,s.cell_size*0.30f)) return false;
    const int idx=nearest_grid_index(s,request.start);
    if(idx<0) return false;
    if(s.global_search && request.world_half_extent>0)return true; // full-map bounds never need a moving repair border
    const auto c=coordinates(s,idx);
    // Leave a small repair border. If local avoidance moved the robot outside
    // the retained planning window, rebuilding a fresh D* state is safer and
    // cheaper than growing every per-node array in place.
    return c[0]>=2 && c[1]>=2 && c[0]<s.nx-2 && c[1]<s.nz-2;
}

void refresh_known_nodes(State& s,std::uint64_t epoch,const PathPlannerCallbacks& callbacks){
    if(s.world_epoch==epoch) return;
    std::vector<int> changed;
    changed.reserve(32);
    // Snapshot the known list: sampling may lazily discover a few neighboring
    // cells through callbacks later, but this revision pass only needs to
    // repair state that was part of the previous search graph.
    const auto known=s.known_nodes;
    for(int idx:known){
        bool did_change=false;
        sample_penalty(s,idx,epoch,callbacks,&did_change);
        if(did_change) changed.push_back(idx);
    }
    s.world_epoch=epoch;
    if(changed.empty()) return;
    ++s.incremental_repair_count;
    s.repaired_node_count+=(std::uint32_t)changed.size();
    for(int idx:changed) update_around(s,idx,epoch,callbacks);
}

bool extract_route(State& s,const PathPlanRequest& request,std::uint64_t epoch,
                   const PathPlannerCallbacks& callbacks,std::vector<Vec3>& raw,
                   bool& repair_requested){
    raw.clear();
    repair_requested=false;
    int current=s.last_start_index;
    // Retain the validated connection from the exact robot pose to the grid.
    // Dropping this vertex made smoothing try to cut a corner directly to the
    // first grid NEIGHBOR, falsely rejecting a valid exit from a narrow bay.
    if(s.global_search && current>=0){const Vec3 entry=point_for(s,current,callbacks);if(dist_xz(request.start,entry)>0.03f)raw.push_back(entry);}
    std::vector<std::uint8_t> visited(s.g.size(),0);
    const int guard_limit=std::min<int>((int)s.g.size(),4096);
    for(int guard=0;guard<guard_limit;++guard){
        if(current<0 || current>=(int)s.g.size()) return false;
        if(visited[(std::size_t)current]) return false;
        visited[(std::size_t)current]=1;

        if(s.terminal[(std::size_t)current]){
            const Vec3 p=point_for(s,current,callbacks);
            if(callbacks.segment_clear(p,request.goal)){
                if(dist_xz(p,request.goal)>0.03f) raw.push_back(request.goal);
                return true;
            }
            // The terminal approach itself changed even if the terminal grid
            // point stayed free. Remove only this terminal and repair locally.
            s.terminal[(std::size_t)current]=0;
            update_around(s,current,epoch,callbacks);
            repair_requested=true;
            return false;
        }

        int best=-1;
        float best_value=kInf;
        bool changed_edge=false;
        int changed_neighbor=-1;
        for(int dir=0;dir<8;++dir){
            const int n=neighbor_index(s,current,dir);
            if(n<0) continue;
            bool edge_changed=false;
            const float edge=compute_edge_cost(s,current,dir,epoch,callbacks,&edge_changed);
            if(edge_changed){ changed_edge=true; changed_neighbor=n; }
            if(!finite_cost(edge) || !finite_cost(s.g[(std::size_t)n])) continue;
            const float candidate=edge+s.g[(std::size_t)n];
            if(candidate<best_value-1.0e-5f){
                best_value=candidate;
                best=n;
            }
        }
        if(changed_edge){
            update_around(s,current,epoch,callbacks);
            if(changed_neighbor>=0) update_around(s,changed_neighbor,epoch,callbacks);
            repair_requested=true;
            return false;
        }
        if(best<0 || !finite_cost(best_value)) return false;
        raw.push_back(point_for(s,best,callbacks));
        current=best;
    }
    return false;
}

float sampled_segment_penalty(Vec3 a,Vec3 b,const PathPlanRequest& request,
                              const PathPlannerCallbacks& callbacks){
    if(!callbacks.traversal_cost) return 0.0f;
    const float length=dist_xz(a,b);
    const float sample_step=std::clamp(request.cell_size*0.45f,0.30f,0.65f);
    const int samples=std::max(1,(int)std::ceil(length/sample_step));
    float maximum=0.0f;
    // Endpoints are included because a line-of-sight shortcut can otherwise
    // jump from open space directly into the high-cost side of a corner.
    for(int sample=0;sample<=samples;++sample){
        const float t=(float)sample/(float)samples;
        Vec3 p=a+(b-a)*t;
        if(callbacks.floor_height) p.y=callbacks.floor_height(p);
        float cost=callbacks.traversal_cost(p);
        if(std::isnan(cost) || !std::isfinite(cost)) return kInf;
        maximum=std::max(maximum,std::max(0.0f,cost));
    }
    return maximum;
}

float raw_section_penalty(const std::vector<Vec3>& raw,std::size_t first,std::size_t last,
                          const PathPlannerCallbacks& callbacks){
    if(!callbacks.traversal_cost) return 0.0f;
    float maximum=0.0f;
    for(std::size_t i=first;i<=last && i<raw.size();++i){
        float cost=callbacks.traversal_cost(raw[i]);
        if(std::isnan(cost) || !std::isfinite(cost)) return kInf;
        maximum=std::max(maximum,std::max(0.0f,cost));
    }
    return maximum;
}

bool smooth_route(const PathPlanRequest& request,const PathPlannerCallbacks& callbacks,
                  const std::vector<Vec3>& raw,std::vector<Vec3>& geometric){
    geometric.clear();
    if(raw.empty()) return false;
    Vec3 anchor=request.start;
    std::size_t i=0;
    const std::size_t max_skip=std::max<std::size_t>(1,request.visibility_skip_limit);
    while(i<raw.size()){
        const std::size_t farthest=std::min(raw.size()-1,i+max_skip);
        std::size_t best=raw.size();
        for(std::size_t candidate=farthest+1;candidate>i;--candidate){
            const std::size_t j=candidate-1;
            if(!callbacks.segment_clear(anchor,raw[j])) continue;

            // D* intentionally pays a soft inflation cost to keep the rover
            // away from structure corners. v0.40.50's Theta-style smoothing
            // ignored that cost and could shortcut the safe D* arc back across
            // the 1 m halo. Preserve the worst clearance class of the raw
            // section. The immediately adjacent raw edge is always permitted:
            // D* already certified that edge, which keeps narrow but physically
            // valid passages usable.
            if(callbacks.traversal_cost && j>i){
                const float raw_penalty=raw_section_penalty(raw,i,j,callbacks);
                const float shortcut_penalty=sampled_segment_penalty(anchor,raw[j],request,callbacks);
                if(!std::isfinite(shortcut_penalty) ||
                   shortcut_penalty>raw_penalty+0.35f) continue;
            }
            best=j;
            break;
        }
        if(best==raw.size()){
            // The first edge is the topology-preserving fallback. It should be
            // reachable whenever extraction produced a valid D* successor.
            if(!callbacks.segment_clear(anchor,raw[i])) return false;
            best=i;
        }
        if(geometric.empty() || dist_xz(geometric.back(),raw[best])>0.03f)
            geometric.push_back(raw[best]);
        anchor=raw[best];
        i=best+1;
    }
    return !geometric.empty();
}

} // namespace

void reset_incremental_path_state(IncrementalPathPlannerState& state){
    const std::uint32_t initializations=state.initialization_count;
    const std::uint32_t repairs=state.incremental_repair_count;
    const std::uint32_t repaired_nodes=state.repaired_node_count;
    const std::uint32_t expansions=state.expanded_node_count;
    state=IncrementalPathPlannerState{};
    // Lifetime diagnostics survive a goal/state reset.
    state.initialization_count=initializations;
    state.incremental_repair_count=repairs;
    state.repaired_node_count=repaired_nodes;
    state.expanded_node_count=expansions;
}

IncrementalPathResult plan_incremental_path(const PathPlanRequest& request,
                                            std::uint64_t world_epoch,
                                            const PathPlannerCallbacks& callbacks,
                                            IncrementalPathPlannerState& state,
                                            std::vector<Vec3>& geometric){
    geometric.clear();
    if(!callbacks.floor_height || !callbacks.point_clear || !callbacks.segment_clear) return IncrementalPathResult::NoPath;

    const auto global_fallback=[&](){
        if(state.global_search || request.world_half_extent<=0)return IncrementalPathResult::NoPath;
        if(!initialize_state(request,world_epoch,callbacks,state,true)){
            reset_incremental_path_state(state);return IncrementalPathResult::NoPath;
        }
        // Defer graph propagation to the next granted search slice, preserving
        // the normal per-call expansion budget even when a legacy search fails.
        return IncrementalPathResult::Pending;
    };
    if(!state_compatible(request,state)){
        if(!initialize_state(request,world_epoch,callbacks,state))return global_fallback();
    }else{
        if(state.global_search && state.world_epoch!=world_epoch){
            // Do not resample thousands of retained global nodes in one frame
            // after construction changes. A fresh lazy graph is cheap to seed
            // and its propagation resumes under the normal slice budget.
            if(!initialize_state(request,world_epoch,callbacks,state,true))return IncrementalPathResult::NoPath;
            return IncrementalPathResult::Pending;
        }
        refresh_known_nodes(state,world_epoch,callbacks);
        const int next_start=find_start_index(state,request.start,world_epoch,callbacks);
        if(next_start<0)return global_fallback();
        if(next_start!=state.last_start_index){
            state.km+=heuristic(state,state.last_start_index,next_start);
            state.last_start_index=next_start;
        }
    }

    // D* Lite repairs only as much of the prior search tree as is needed for
    // the moving start. A route extraction may discover a newly blocked edge
    // whose endpoints stayed free; feed that edge back into the inconsistent
    // set and allow two bounded repair passes before falling back to A*.
    const auto deadline=state.global_search?std::chrono::steady_clock::now()+std::chrono::milliseconds(4):std::chrono::steady_clock::time_point::max();
    constexpr int extraction_repairs=3;
    for(int attempt=0;attempt<extraction_repairs;++attempt){
        const auto compute=compute_shortest_path(state,world_epoch,callbacks,request.max_expansions,deadline);
        if(compute==ComputeStatus::Pending) return IncrementalPathResult::Pending;
        if(!finite_cost(state.g[(std::size_t)state.last_start_index]) &&
           !finite_cost(state.rhs[(std::size_t)state.last_start_index]))
            return global_fallback();

        std::vector<Vec3> raw;
        bool repair_requested=false;
        if(extract_route(state,request,world_epoch,callbacks,raw,repair_requested))
            return smooth_route(request,callbacks,raw,geometric)?IncrementalPathResult::Success:global_fallback();
        if(!repair_requested) return global_fallback();
        ++state.incremental_repair_count;
    }
    return IncrementalPathResult::Pending;
}

} // namespace game::robots
