#include "game/world/corridor/detail.hpp"
#include <array>
#include <cstdint>
#include <queue>
#include <vector>

namespace game::corridor_detail {
namespace {
float segment_distance_approx(Vec3 a0,Vec3 a1,Vec3 b0,Vec3 b1){
    float best=std::numeric_limits<float>::max();
    for(int i=0;i<=8;++i){
        const float t=(float)i/8.0f;
        const Vec3 p=a0+(a1-a0)*t;
        best=std::min(best,point_segment_distance_xz(p,b0,b1));
    }
    for(int i=0;i<=8;++i){
        const float t=(float)i/8.0f;
        const Vec3 p=b0+(b1-b0)*t;
        best=std::min(best,point_segment_distance_xz(p,a0,a1));
    }
    return best;
}

bool near_allowed_junction(Vec3 route_mid,Vec3 host_mid,std::size_t host_index,
                           const CorridorEndpoint& a,const CorridorEndpoint& b){
    constexpr float JUNCTION_ZONE=CORRIDOR_HALF_WIDTH*2.0f+0.32f;
    for(const CorridorEndpoint* ep:{&a,&b}){
        if(ep->kind!=CorridorEndpointKind::Corridor || ep->index!=host_index) continue;
        const Vec3 junction=ep->surface_matched_junction?ep->host_position:ep->position;
        if(distance_xz(route_mid,junction)<=JUNCTION_ZONE && distance_xz(host_mid,junction)<=JUNCTION_ZONE) return true;
    }
    return false;
}

bool route_overlaps_corridor(const std::vector<Vec3>& path,
                             const std::vector<CorridorConnection>& existing,
                             const CorridorEndpoint& a,const CorridorEndpoint& b){
    constexpr float CENTERLINE_CLEARANCE=CORRIDOR_HALF_WIDTH*2.0f+0.16f;
    constexpr float AUTO_JUNCTION_ZONE=CORRIDOR_HALF_WIDTH*1.75f;
    for(std::size_t ci=0;ci<existing.size();++ci){
        const auto& host=existing[ci];
        if(host.path.size()<2) continue;

        // Any sufficiently transversal centerline crossing is a legal corridor
        // junction.  Collect it once, then exempt only the compact pressure
        // chamber around that point. Parallel/coincident runs remain invalid.
        std::vector<Vec3> automatic_junctions;
        for(std::size_t i=0;i+1<path.size();++i){
            for(std::size_t j=0;j+1<host.path.size();++j){
                Vec3 hit{};
                if(mergeable_corridor_crossing(path[i],path[i+1],host.path[j],host.path[j+1],hit)){
                    bool duplicate=false;
                    for(const Vec3 p:automatic_junctions) if(distance_xz(p,hit)<0.25f){ duplicate=true; break; }
                    if(!duplicate) automatic_junctions.push_back(hit);
                }
            }
        }

        for(std::size_t i=0;i+1<path.size();++i){
            const Vec3 route_mid=(path[i]+path[i+1])*0.5f;
            for(std::size_t j=0;j+1<host.path.size();++j){
                if(segment_distance_approx(path[i],path[i+1],host.path[j],host.path[j+1])>=CENTERLINE_CLEARANCE) continue;
                // Use the host point nearest this route sample rather than the
                // midpoint of the whole host segment. Host paths may contain a
                // long straight segment; using its midpoint makes an explicit
                // branch junction fail everywhere except near that midpoint.
                const Vec3 host_near=closest_point_segment_xz(route_mid,host.path[j],host.path[j+1]);
                if(near_allowed_junction(route_mid,host_near,ci,a,b)) continue;
                bool in_auto_junction=false;
                for(const Vec3 hit:automatic_junctions){
                    if(point_segment_distance_xz(hit,path[i],path[i+1])<0.05f ||
                       distance_xz(route_mid,hit)<AUTO_JUNCTION_ZONE){
                        if(point_segment_distance_xz(hit,host.path[j],host.path[j+1])<0.05f ||
                           distance_xz(host_near,hit)<AUTO_JUNCTION_ZONE){
                            in_auto_junction=true; break;
                        }
                    }
                }
                if(in_auto_junction) continue;
                return true;
            }
        }
    }
    return false;
}

float edge_distance_xz(Vec3 p,Vec3 a,Vec3 b){
    return point_segment_distance_xz(p,a,b);
}

float signed_area_xz(Vec3 a,Vec3 b,Vec3 c){
    return (b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x);
}

bool point_in_triangle_xz(Vec3 p,Vec3 a,Vec3 b,Vec3 c){
    const float area=signed_area_xz(a,b,c);
    if(std::abs(area)<1.0e-6f) return false;
    const float s0=signed_area_xz(a,b,p);
    const float s1=signed_area_xz(b,c,p);
    const float s2=signed_area_xz(c,a,p);
    const bool neg=(s0<-1.0e-5f)||(s1<-1.0e-5f)||(s2<-1.0e-5f);
    const bool pos=(s0> 1.0e-5f)||(s1> 1.0e-5f)||(s2> 1.0e-5f);
    return !(neg&&pos);
}

bool point_near_triangle_set_xz(Vec3 local,const std::vector<Vec3>& tris,float clearance){
    for(std::size_t i=0;i+2<tris.size();i+=3){
        const Vec3 a=tris[i],b=tris[i+1],c=tris[i+2];
        if(point_in_triangle_xz(local,a,b,c)) return true;
        if(edge_distance_xz(local,a,b)<=clearance || edge_distance_xz(local,b,c)<=clearance ||
           edge_distance_xz(local,c,a)<=clearance) return true;
    }
    return false;
}

bool endpoint_dock_exception(Vec3 p,const CorridorEndpoint& ep,CorridorEndpointKind kind,std::size_t index){
    if(ep.kind!=kind || ep.index!=index) return false;
    // Only the short, straight departure from a selected docking port may
    // enter the structure's own hull clearance. This is intentionally narrow:
    // the remainder of the route must go around the actual structure geometry.
    const Vec3 rel=flatten(p-ep.position);
    const Vec3 tangent=flat_normal(ep.tangent);
    const float forward=dot(rel,tangent);
    const float lateral=std::abs(dot(rel,perp_xz(tangent)));
    return forward>=-0.35f && forward<=3.20f && lateral<=CORRIDOR_HALF_WIDTH*0.78f;
}

bool point_overlaps_structures(Vec3 p,
                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                               const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                               const CorridorEndpoint& a,const CorridorEndpoint& b){
    const float clearance=CORRIDOR_HALF_WIDTH+ROUTE_CLEARANCE;
    for(std::size_t hi=0;hi<habitats.size();++hi){
        if(endpoint_dock_exception(p,a,CorridorEndpointKind::Habitat,hi) ||
           endpoint_dock_exception(p,b,CorridorEndpointKind::Habitat,hi)) continue;
        const auto& hm=library.mesh(habitats[hi]);
        const Vec3 local=rotate_y(p-habitats[hi].position,-habitats[hi].rotation_degrees);
        if(std::abs(local.x)>hm.footprint_x*0.5f+clearance ||
           std::abs(local.z)>hm.footprint_z*0.5f+clearance) continue;
        // Test the actual pressure hull/foundation projection, not the authored
        // rectangular placement footprint. This lets corridors pass close to
        // rounded/irregular structures without reserving empty corner squares.
        if(point_near_triangle_set_xz(local,hm.shell,clearance) ||
           point_near_triangle_set_xz(local,hm.foundation,clearance) ||
           point_near_triangle_set_xz(local,hm.glazing,clearance)) return true;
    }
    for(std::size_t ri=0;ri<resources.size();++ri){
        if(endpoint_dock_exception(p,a,CorridorEndpointKind::Resource,ri) ||
           endpoint_dock_exception(p,b,CorridorEndpointKind::Resource,ri)) continue;
        const auto& rm=resource_library.mesh(resources[ri]);
        const Vec3 local=rotate_y(p-resources[ri].position,-resources[ri].rotation_degrees);
        if(std::abs(local.x)>rm.footprint_x*0.5f+clearance ||
           std::abs(local.z)>rm.footprint_z*0.5f+clearance) continue;
        if(point_near_triangle_set_xz(local,rm.body,clearance) ||
           point_near_triangle_set_xz(local,rm.foundation,clearance)) return true;
    }
    return false;
}

bool point_overlaps_corridors(Vec3 p,const std::vector<CorridorConnection>& existing,
                              const CorridorEndpoint& a,const CorridorEndpoint& b){
    constexpr float CENTERLINE_CLEARANCE=CORRIDOR_HALF_WIDTH*2.0f+0.16f;
    for(std::size_t ci=0;ci<existing.size();++ci){
        const auto& host=existing[ci];
        for(std::size_t j=0;j+1<host.path.size();++j){
            const Vec3 q=closest_point_segment_xz(p,host.path[j],host.path[j+1]);
            if(distance_xz(p,q)>=CENTERLINE_CLEARANCE) continue;
            if(near_allowed_junction(p,q,ci,a,b)) continue;
            return true;
        }
    }
    return false;
}

bool route_overlaps_structures(const std::vector<Vec3>& path,
                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                               const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                               const CorridorEndpoint& a,const CorridorEndpoint& b){
    constexpr float SAMPLE=0.20f;
    for(std::size_t si=0;si+1<path.size();++si){
        const float len=distance_xz(path[si],path[si+1]);
        const int samples=std::max(1,(int)std::ceil(len/SAMPLE));
        for(int n=0;n<=samples;++n){
            const float t=(float)n/(float)samples;
            const Vec3 p=path[si]+(path[si+1]-path[si])*t;
            if(point_overlaps_structures(p,habitats,library,resources,resource_library,a,b)) return true;
        }
    }
    return false;
}

std::vector<Vec3> smooth_candidate(std::vector<Vec3> controls){
    if(controls.size()<2) return controls;
    for(int it=0;it<3;++it){
        if(controls.size()<3) break;
        std::vector<Vec3> out;
        out.reserve(controls.size()*2);
        out.push_back(controls.front());
        for(std::size_t i=0;i+1<controls.size();++i){
            out.push_back(controls[i]*0.75f+controls[i+1]*0.25f);
            out.push_back(controls[i]*0.25f+controls[i+1]*0.75f);
        }
        out.push_back(controls.back());
        controls=std::move(out);
    }
    std::vector<Vec3> compact;
    compact.reserve(controls.size());
    for(const Vec3& p:controls){
        if(compact.empty() || distance_xz(compact.back(),p)>0.07f || std::abs(compact.back().y-p.y)>0.035f)
            compact.push_back(p);
    }
    return compact;
}


std::vector<Vec3> compact_connection_route(const CorridorEndpoint& a,const CorridorEndpoint& b){
    const float d=distance_xz(a.position,b.position);
    if(d<0.015f) return {};
    Vec3 ta=flat_normal(a.tangent);
    Vec3 tb=flat_normal(b.tangent);
    if(length(ta)<0.001f) ta=flat_normal(b.position-a.position);
    if(length(tb)<0.001f) tb=flat_normal(a.position-b.position);

    // Short structure-to-corridor and corridor-to-corridor links need a much
    // smaller tangent lead than the generic route planner.  Oversized leads
    // were the main reason close entrances produced loops/overlaps instead of
    // a compact merged connection.  A sampled cubic keeps both terminal ring
    // planes stable while fitting into the available gap.
    const float lead=std::clamp(d*0.18f,0.05f,0.48f);
    const Vec3 p0=a.position;
    const Vec3 p1=a.position+ta*lead;
    const Vec3 p2=b.position+tb*lead;
    const Vec3 p3=b.position;
    constexpr int SAMPLES=10;
    std::vector<Vec3> path;
    path.reserve(SAMPLES+1);
    for(int i=0;i<=SAMPLES;++i){
        const float t=(float)i/(float)SAMPLES;
        const float u=1.0f-t;
        Vec3 p=p0*(u*u*u)+p1*(3.0f*u*u*t)+p2*(3.0f*u*t*t)+p3*(t*t*t);
        p.y=a.position.y+(b.position.y-a.position.y)*t;
        if(path.empty() || distance_xz(path.back(),p)>0.012f || std::abs(path.back().y-p.y)>0.008f)
            path.push_back(p);
    }
    if(path.size()<2) return {a.position,b.position};
    path.front()=a.position;
    path.back()=b.position;
    return path;
}

bool route_is_valid(const std::vector<Vec3>& path,
                    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                    const std::vector<CorridorConnection>& existing,
                    const CorridorEndpoint& a,const CorridorEndpoint& b){
    if(path.size()<2 || corridor_path_self_overlaps(path)) return false;
    if(route_overlaps_structures(path,habitats,library,resources,resource_library,a,b)) return false;
    if(route_overlaps_corridor(path,existing,a,b)) return false;
    return true;
}


bool grid_segment_clear(Vec3 a_pos,Vec3 b_pos,
                        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                        const std::vector<CorridorConnection>& existing,
                        const CorridorEndpoint& a,const CorridorEndpoint& b){
    const float len=distance_xz(a_pos,b_pos);
    const int samples=std::max(1,(int)std::ceil(len/0.34f));
    for(int i=0;i<=samples;++i){
        const float t=(float)i/(float)samples;
        const Vec3 p=a_pos+(b_pos-a_pos)*t;
        if(point_overlaps_structures(p,habitats,library,resources,resource_library,a,b)) return false;
        if(point_overlaps_corridors(p,existing,a,b)) return false;
    }
    return true;
}

struct GridQueueNode {
    int index{};
    float f{};
    bool operator<(const GridQueueNode& rhs) const { return f>rhs.f; }
};

std::vector<Vec3> grid_detour_route(const CorridorEndpoint& a,const CorridorEndpoint& b,
                                    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                                    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                                    const std::vector<CorridorConnection>& existing,
                                    float playfield_half_extent,float margin){
    Vec3 ta=flat_normal(a.tangent),tb=flat_normal(b.tangent);
    if(length(ta)<0.001f) ta=flat_normal(b.position-a.position);
    if(length(tb)<0.001f) tb=flat_normal(a.position-b.position);
    const float d=distance_xz(a.position,b.position);
    const float lead=std::clamp(d*0.10f,1.15f,2.70f);
    const Vec3 start=a.position+ta*lead;
    const Vec3 goal=b.position+tb*lead;

    // The search is local to the requested connection but clipped to the real
    // playfield. Repeated calls with larger margins progressively discover
    // wider detours without paying the cost of rasterising a 512 m map every
    // preview frame.
    constexpr float STEP=1.05f;
    const float min_x=std::max(-playfield_half_extent,std::min(start.x,goal.x)-margin);
    const float max_x=std::min( playfield_half_extent,std::max(start.x,goal.x)+margin);
    const float min_z=std::max(-playfield_half_extent,std::min(start.z,goal.z)-margin);
    const float max_z=std::min( playfield_half_extent,std::max(start.z,goal.z)+margin);
    if(max_x-min_x<STEP || max_z-min_z<STEP) return {};
    const int nx=std::max(2,(int)std::floor((max_x-min_x)/STEP)+1);
    const int nz=std::max(2,(int)std::floor((max_z-min_z)/STEP)+1);
    const int total=nx*nz;
    if(total>120000) return {};

    const auto clamp_ix=[&](float x){ return std::clamp((int)std::lround((x-min_x)/STEP),0,nx-1); };
    const auto clamp_iz=[&](float z){ return std::clamp((int)std::lround((z-min_z)/STEP),0,nz-1); };
    const int sx=clamp_ix(start.x),sz=clamp_iz(start.z);
    const int gx=clamp_ix(goal.x),gz=clamp_iz(goal.z);
    const int start_index=sz*nx+sx,goal_index=gz*nx+gx;
    const float route_y=(start.y+goal.y)*0.5f;
    const auto node_pos=[&](int index){
        if(index==start_index) return start;
        if(index==goal_index) return goal;
        const int x=index%nx,z=index/nx;
        return Vec3{min_x+(float)x*STEP,route_y,min_z+(float)z*STEP};
    };
    const auto heuristic=[&](int index){ return distance_xz(node_pos(index),goal); };

    std::vector<float> gscore((std::size_t)total,std::numeric_limits<float>::max());
    std::vector<int> parent((std::size_t)total,-1);
    std::vector<std::int8_t> blocked((std::size_t)total,-1);
    auto node_blocked=[&](int index){
        if(index==start_index || index==goal_index) return false;
        auto& cached=blocked[(std::size_t)index];
        if(cached>=0) return cached!=0;
        const Vec3 p=node_pos(index);
        // Corridors are not absolute grid obstacles: a transversal encounter is
        // allowed to become a merged pressure junction. Structures remain hard
        // obstacles. The corridor penalty field below strongly discourages
        // running alongside an existing tube, while still permitting a short
        // perpendicular crossing when that is the cleanest detour.
        const bool value=point_overlaps_structures(p,habitats,library,resources,resource_library,a,b);
        cached=value?1:0;
        return value;
    };

    std::vector<float> corridor_penalty((std::size_t)total,0.0f);
    constexpr float SOFT_RADIUS=CORRIDOR_HALF_WIDTH*3.25f;
    constexpr float HARD_RADIUS=CORRIDOR_HALF_WIDTH*2.0f+0.16f;
    for(std::size_t ci=0;ci<existing.size();++ci){
        const auto& host=existing[ci];
        for(std::size_t si=0;si+1<host.path.size();++si){
            const Vec3 p0=host.path[si],p1=host.path[si+1];
            const int x0=std::clamp((int)std::floor((std::min(p0.x,p1.x)-SOFT_RADIUS-min_x)/STEP),0,nx-1);
            const int x1=std::clamp((int)std::ceil ((std::max(p0.x,p1.x)+SOFT_RADIUS-min_x)/STEP),0,nx-1);
            const int z0=std::clamp((int)std::floor((std::min(p0.z,p1.z)-SOFT_RADIUS-min_z)/STEP),0,nz-1);
            const int z1=std::clamp((int)std::ceil ((std::max(p0.z,p1.z)+SOFT_RADIUS-min_z)/STEP),0,nz-1);
            for(int z=z0;z<=z1;++z) for(int x=x0;x<=x1;++x){
                const int index=z*nx+x;
                const Vec3 p=node_pos(index);
                if(near_allowed_junction(p,closest_point_segment_xz(p,p0,p1),ci,a,b)) continue;
                const float dseg=point_segment_distance_xz(p,p0,p1);
                if(dseg>=SOFT_RADIUS) continue;
                const float weight=dseg<HARD_RADIUS?9.0f:2.5f*(SOFT_RADIUS-dseg)/(SOFT_RADIUS-HARD_RADIUS);
                corridor_penalty[(std::size_t)index]=std::max(corridor_penalty[(std::size_t)index],weight);
            }
        }
    }

    std::priority_queue<GridQueueNode> open;
    gscore[(std::size_t)start_index]=0.0f;
    open.push({start_index,heuristic(start_index)});
    constexpr std::array<std::array<int,2>,8> dirs{{
        {{1,0}},{{-1,0}},{{0,1}},{{0,-1}},{{1,1}},{{1,-1}},{{-1,1}},{{-1,-1}}
    }};
    std::vector<bool> closed((std::size_t)total,false);
    bool found=false;
    while(!open.empty()){
        const int current=open.top().index;
        open.pop();
        if(closed[(std::size_t)current]) continue;
        closed[(std::size_t)current]=true;
        if(current==goal_index){ found=true; break; }
        const int cx=current%nx,cz=current/nx;
        const Vec3 cp=node_pos(current);
        for(const auto& dir:dirs){
            const int x=cx+dir[0],z=cz+dir[1];
            if(x<0 || x>=nx || z<0 || z>=nz) continue;
            const int next=z*nx+x;
            if(closed[(std::size_t)next] || node_blocked(next)) continue;
            const Vec3 np=node_pos(next);
            // Prevent diagonal corner cutting through a hard structure cell.
            if(dir[0]!=0 && dir[1]!=0){
                const int side_a=cz*nx+x;
                const int side_b=z*nx+cx;
                if(node_blocked(side_a) || node_blocked(side_b)) continue;
            }
            const float tentative=gscore[(std::size_t)current]+distance_xz(cp,np)+corridor_penalty[(std::size_t)next];
            if(tentative>=gscore[(std::size_t)next]) continue;
            gscore[(std::size_t)next]=tentative;
            parent[(std::size_t)next]=current;
            open.push({next,tentative+heuristic(next)});
        }
    }
    if(!found) return {};

    std::vector<Vec3> raw;
    for(int at=goal_index;at>=0;at=parent[(std::size_t)at]){
        raw.push_back(node_pos(at));
        if(at==start_index) break;
    }
    if(raw.empty() || distance_xz(raw.back(),start)>0.02f) return {};
    std::reverse(raw.begin(),raw.end());

    // Visibility compression removes the staircase produced by the grid while
    // retaining the obstacle-safe topology. The later smoothing pass therefore
    // starts from only the meaningful detour corners.
    std::vector<Vec3> compact;
    for(std::size_t i=0;i<raw.size();){
        compact.push_back(raw[i]);
        if(i+1>=raw.size()) break;
        std::size_t best=i+1;
        for(std::size_t j=raw.size()-1;j>i+1;--j){
            if(grid_segment_clear(raw[i],raw[j],habitats,library,resources,resource_library,existing,a,b)){
                best=j;
                break;
            }
        }
        i=best;
    }

    std::vector<Vec3> controls;
    controls.reserve(compact.size()+4);
    controls.push_back(a.position);
    if(distance_xz(a.position,start)>0.03f) controls.push_back(start);
    for(std::size_t i=1;i+1<compact.size();++i) controls.push_back(compact[i]);
    if(distance_xz(goal,b.position)>0.03f) controls.push_back(goal);
    controls.push_back(b.position);

    auto smooth=smooth_candidate(controls);
    if(route_is_valid(smooth,habitats,library,resources,resource_library,existing,a,b)) return smooth;
    // Tight passages may not tolerate corner cutting. In that case keep the
    // visibility-compressed polyline; it is still a valid, non-overlapping
    // corridor and the swept D-profile will follow its explicit bends.
    if(route_is_valid(controls,habitats,library,resources,resource_library,existing,a,b)) return controls;
    return {};
}

std::vector<Vec3> find_non_overlapping_route(const CorridorEndpoint& a,const CorridorEndpoint& b,
                    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                    const std::vector<CorridorConnection>& existing,float playfield_half_extent){
    const float d=distance_xz(a.position,b.position);

    // When at least one endpoint is an existing corridor, first try a compact
    // tangent-preserving connector.  This is what makes an entrance directly
    // beside a corridor connectable without forcing a large detour or allowing
    // the new tube to run through the host.  The normal overlap validator still
    // applies; only the explicitly selected shared-junction volume is exempt.
    if((a.kind==CorridorEndpointKind::Corridor || b.kind==CorridorEndpointKind::Corridor) && d<=5.25f){
        auto compact=compact_connection_route(a,b);
        if(route_is_valid(compact,habitats,library,resources,resource_library,existing,a,b)) return compact;
    }

    auto direct=smooth_route(a,b,habitats,library);
    if(route_is_valid(direct,habitats,library,resources,resource_library,existing,a,b)) return direct;

    if(d<0.10f) return {};
    Vec3 forward=flat_normal(b.position-a.position);
    if(length(forward)<0.001f) forward={1,0,0};
    const Vec3 lateral=perp_xz(forward);
    const float lead=std::clamp(d*0.16f,1.15f,4.25f);
    const Vec3 a_lead=a.position+flat_normal(a.tangent)*lead;
    const Vec3 b_lead=b.position+flat_normal(b.tangent)*lead;

    // Search increasingly wide lanes on both sides of the direct route.  This
    // turns corridor placement into a deterministic local router: occupied
    // corridor volumes are treated as obstacles unless they are the selected
    // shared endpoint/junction.  The shortest valid candidate wins.
    const float offsets[]={3.2f,-3.2f,5.4f,-5.4f,7.8f,-7.8f,10.5f,-10.5f,14.0f,-14.0f,18.0f,-18.0f,23.0f,-23.0f};
    std::vector<Vec3> best;
    float best_len=std::numeric_limits<float>::max();
    for(float off:offsets){
        const Vec3 q1=a.position+(b.position-a.position)*0.32f+lateral*off;
        const Vec3 q2=a.position+(b.position-a.position)*0.68f+lateral*off;
        auto candidate=smooth_candidate({a.position,a_lead,q1,q2,b_lead,b.position});
        if(!route_is_valid(candidate,habitats,library,resources,resource_library,existing,a,b)) continue;
        const float len=path_length(candidate);
        if(len<best_len){ best_len=len; best=std::move(candidate); }
    }

    // A single lateral lane can still be blocked by a large building.  Try
    // asymmetric two-bend lanes, which route around one side of the obstacle
    // near the source and can return toward the destination on the other side.
    const float doglegs[]={6.0f,9.5f,13.5f,18.0f,24.0f};
    for(float mag:doglegs){
        for(float sa:{-1.0f,1.0f}) for(float sb:{-1.0f,1.0f}){
            const Vec3 q1=a.position+(b.position-a.position)*0.28f+lateral*(mag*sa);
            const Vec3 q2=a.position+(b.position-a.position)*0.72f+lateral*(mag*sb);
            auto candidate=smooth_candidate({a.position,a_lead,q1,q2,b_lead,b.position});
            if(!route_is_valid(candidate,habitats,library,resources,resource_library,existing,a,b)) continue;
            const float len=path_length(candidate);
            if(len<best_len){ best_len=len; best=std::move(candidate); }
        }
    }
    if(!best.empty()) return best;

    // Fixed-offset candidates are fast and cover the common case. If all of
    // them are blocked, escalate to an A* centerline search so distant or
    // irregular obstacles can be routed around instead of making two otherwise
    // connectable entrances/corridors appear impossible.
    const float primary_margin=std::clamp(d*0.42f+10.0f,14.0f,44.0f);
    const float secondary_margin=std::min(playfield_half_extent,std::max(primary_margin+12.0f,62.0f));
    for(float margin:{primary_margin,secondary_margin}){
        auto detour=grid_detour_route(a,b,habitats,library,resources,resource_library,existing,playfield_half_extent,margin);
        if(detour.size()>=2) return detour;
        if(std::abs(secondary_margin-primary_margin)<1.0f) break;
    }
    return {};
}
}
std::optional<CorridorConnection> make_generic(const CorridorBuildAnchor& a_anchor,const CorridorBuildAnchor& b_anchor,
                                               CorridorConnectionKind kind,
                                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                                               const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                                               const std::vector<CorridorConnection>& existing,float playfield_half_extent){
    Vec3 a_hint=a_anchor.position,b_hint=b_anchor.position;
    if(a_anchor.kind==CorridorEndpointKind::Habitat && a_anchor.index<habitats.size()) a_hint=habitats[a_anchor.index].position;
    if(b_anchor.kind==CorridorEndpointKind::Habitat && b_anchor.index<habitats.size()) b_hint=habitats[b_anchor.index].position;
    if(a_anchor.kind==CorridorEndpointKind::Resource && a_anchor.index<resources.size()) a_hint=resources[a_anchor.index].position;
    if(b_anchor.kind==CorridorEndpointKind::Resource && b_anchor.index<resources.size()) b_hint=resources[b_anchor.index].position;
    auto a=resolve_anchor(a_anchor,b_hint,habitats,library,resources,resource_library,existing);
    auto b=resolve_anchor(b_anchor,a_hint,habitats,library,resources,resource_library,existing);
    if(!a || !b) return std::nullopt;

    // Re-resolve habitats once using the actual opposite endpoint so generated
    // door placement tracks corridor/corridor/open targets precisely.
    if(a_anchor.kind==CorridorEndpointKind::Habitat || a_anchor.kind==CorridorEndpointKind::Resource)
        a=resolve_anchor(a_anchor,b->position,habitats,library,resources,resource_library,existing);
    if(b_anchor.kind==CorridorEndpointKind::Habitat || b_anchor.kind==CorridorEndpointKind::Resource)
        b=resolve_anchor(b_anchor,a->position,habitats,library,resources,resource_library,existing);
    if(!a || !b) return std::nullopt;

    // Habitat-to-habitat runs shorter than eight raster cells stay elevated
    // and connect directly. The corridor walkway is aligned close to the door
    // thresholds instead of dropping to the lunar surface and climbing again.
    if(a->kind==CorridorEndpointKind::Habitat && b->kind==CorridorEndpointKind::Habitat &&
       distance_xz(a->position,b->position)<DIRECT_CORRIDOR_MAX_RASTERS){
        const float threshold_a=std::max(WALKWAY_Y,a->door.position.y);
        const float threshold_b=std::max(WALKWAY_Y,b->door.position.y);
        const float base=std::max(DIRECT_CORRIDOR_MIN_BASE_Y,std::min(threshold_a,threshold_b)-WALKWAY_Y);
        a->position.y=base;
        b->position.y=base;
    }

    auto path=find_non_overlapping_route(*a,*b,habitats,library,resources,resource_library,existing,playfield_half_extent);
    if(path.size()<2) return std::nullopt;

    // A corridor extended from an existing open end must share the exact end
    // ring plane. Pin the first/last sampled segment to the host terminal
    // tangent; otherwise curve sampling can rotate the D-profile a few degrees
    // and leave a wedge-shaped pressure gap.
    if(path.size()>=3){
        const float terminal=std::clamp(distance_xz(path.front(),path.back())*0.08f,0.05f,0.18f);
        if(a->seamless_splice) path[1]=a->position+a->tangent*terminal;
        if(b->seamless_splice) path[path.size()-2]=b->position+b->tangent*terminal;
    }

    // Re-check after terminal tangent pinning.  If the small terminal edit
    // invalidates the chosen lane, fail rather than introducing an overlap.
    if(!route_is_valid(path,habitats,library,resources,resource_library,existing,*a,*b)) return std::nullopt;

    CorridorConnection out;
    out.kind=kind;
    out.endpoint_a=*a;
    out.endpoint_b=*b;
    out.path=std::move(path);
    out.mesh=build_mesh(out.endpoint_a,out.endpoint_b,out.path,existing);
    return out;
}


std::optional<CorridorConnection> make_generic_via_waypoint(const CorridorBuildAnchor& a_anchor,const CorridorBuildAnchor& b_anchor,
                                                            Vec3 waypoint,CorridorConnectionKind kind,
                                                            const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                                                            const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library,
                                                            const std::vector<CorridorConnection>& existing,float playfield_half_extent){
    Vec3 a_hint=a_anchor.position,b_hint=b_anchor.position;
    if(a_anchor.kind==CorridorEndpointKind::Habitat && a_anchor.index<habitats.size()) a_hint=habitats[a_anchor.index].position;
    if(b_anchor.kind==CorridorEndpointKind::Habitat && b_anchor.index<habitats.size()) b_hint=habitats[b_anchor.index].position;
    if(a_anchor.kind==CorridorEndpointKind::Resource && a_anchor.index<resources.size()) a_hint=resources[a_anchor.index].position;
    if(b_anchor.kind==CorridorEndpointKind::Resource && b_anchor.index<resources.size()) b_hint=resources[b_anchor.index].position;
    auto a=resolve_anchor(a_anchor,b_hint,habitats,library,resources,resource_library,existing);
    auto b=resolve_anchor(b_anchor,a_hint,habitats,library,resources,resource_library,existing);
    if(!a || !b) return std::nullopt;
    if(a_anchor.kind==CorridorEndpointKind::Habitat || a_anchor.kind==CorridorEndpointKind::Resource)
        a=resolve_anchor(a_anchor,b->position,habitats,library,resources,resource_library,existing);
    if(b_anchor.kind==CorridorEndpointKind::Habitat || b_anchor.kind==CorridorEndpointKind::Resource)
        b=resolve_anchor(b_anchor,a->position,habitats,library,resources,resource_library,existing);
    if(!a || !b) return std::nullopt;

    waypoint.y=(a->position.y+b->position.y)*0.5f;
    if(distance_xz(a->position,waypoint)<1.0f || distance_xz(b->position,waypoint)<1.0f) return std::nullopt;

    CorridorEndpoint mid_from_a;
    mid_from_a.kind=CorridorEndpointKind::OpenDoor;
    mid_from_a.position=waypoint;
    mid_from_a.tangent=flat_normal(a->position-waypoint);
    if(length(mid_from_a.tangent)<0.001f) mid_from_a.tangent={1,0,0};
    CorridorEndpoint mid_to_b=mid_from_a;
    mid_to_b.tangent=flat_normal(b->position-waypoint);
    if(length(mid_to_b.tangent)<0.001f) mid_to_b.tangent={-1,0,0};

    auto first=find_non_overlapping_route(*a,mid_from_a,habitats,library,resources,resource_library,existing,playfield_half_extent);
    if(first.size()<2) return std::nullopt;
    auto second=find_non_overlapping_route(mid_to_b,*b,habitats,library,resources,resource_library,existing,playfield_half_extent);
    if(second.size()<2) return std::nullopt;

    std::vector<Vec3> path;
    path.reserve(first.size()+second.size()-1);
    path.insert(path.end(),first.begin(),first.end());
    if(distance_xz(path.back(),second.front())<0.10f) path.insert(path.end(),second.begin()+1,second.end());
    else path.insert(path.end(),second.begin(),second.end());
    if(path.size()<3) return std::nullopt;

    // The waypoint is purely an internal bend. Validate the complete corridor
    // against the real source/target endpoints so no intermediate air-lock or
    // overlap exception is introduced.
    if(!route_is_valid(path,habitats,library,resources,resource_library,existing,*a,*b)) return std::nullopt;

    CorridorConnection out;
    out.kind=kind;
    out.endpoint_a=*a;
    out.endpoint_b=*b;
    out.path=std::move(path);
    out.mesh=build_mesh(out.endpoint_a,out.endpoint_b,out.path,existing);
    return out;
}
} // namespace game::corridor_detail
