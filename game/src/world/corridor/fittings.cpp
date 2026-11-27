#include "game/world/corridor/detail.hpp"
#include "game/world/structural_frame.hpp"
#include "game/world/corridor/pressure_shell.hpp"
#include "game/world/mesh_portals.hpp"
#include "game/world/corridor/corridor_module.hpp"
#include <array>
#include <cmath>
#include <vector>

namespace game::corridor_detail {
void append_oriented_box(std::vector<Vec3>& dst,Vec3 center,Vec3 forward,float half_length,float half_width,float half_height){
    forward=flat_normal(forward);
    if(length(forward)<0.001f) return;
    const Vec3 side=perp_xz(forward);
    const Vec3 f=forward*half_length,s=side*half_width,u{0,half_height,0};
    Vec3 p[8]={center-f-s-u,center+f-s-u,center+f-s+u,center-f-s+u,
               center-f+s-u,center+f+s-u,center+f+s+u,center-f+s+u};
    const int faces[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
    for(const auto& q:faces){ tri(dst,p[q[0]],p[q[1]],p[q[2]]); tri(dst,p[q[0]],p[q[2]],p[q[3]]); }
}

void append_open_rect_tunnel(CorridorMesh& mesh,Vec3 inner,Vec3 outer,Vec3 normal,float half_width,float height){
    normal=flat_normal(normal);
    const Vec3 side=perp_xz(normal);
    const Vec3 a0=inner-side*half_width+Vec3{0,0.05f,0};
    const Vec3 a1=inner+side*half_width+Vec3{0,0.05f,0};
    const Vec3 a2=inner+side*half_width+Vec3{0,height,0};
    const Vec3 a3=inner-side*half_width+Vec3{0,height,0};
    const Vec3 b0=outer-side*half_width+Vec3{0,0.05f,0};
    const Vec3 b1=outer+side*half_width+Vec3{0,0.05f,0};
    const Vec3 b2=outer+side*half_width+Vec3{0,height,0};
    const Vec3 b3=outer-side*half_width+Vec3{0,height,0};
    tri(mesh.shell,a0,b0,b3); tri(mesh.shell,a0,b3,a3);
    tri(mesh.shell,b1,a1,a2); tri(mesh.shell,b1,a2,b2);
    tri(mesh.shell,a3,b3,b2); tri(mesh.shell,a3,b2,a2);
    tri(mesh.details,a0,b1,b0); tri(mesh.details,a0,a1,b1); // sealed vestibule floor
    for(const auto& e:std::array<std::pair<Vec3,Vec3>,8>{{{a0,b0},{a1,b1},{a2,b2},{a3,b3},{a0,a3},{a1,a2},{b0,b3},{b1,b2}}}) edge(mesh.edges,e.first,e.second);
}

namespace {
using Ring=std::vector<Vec3>;
Ring profile_ring(Vec3 center,Vec3 side,const std::vector<Vec2>& profile){
    Ring out;for(const auto q:profile)out.push_back(ring_vertex(center,side,q));return out;
}
Ring interpolate_ring(const Ring& a,const Ring& b,float t){
    Ring out;for(std::size_t i=0;i<a.size();++i)out.push_back(a[i]+(b[i]-a[i])*t);return out;
}
// A continuous annular flange rather than a stack of intersecting I-beams.
// Both inner faces follow the actual loft; the outer face is offset from it.
// Keep the flat bottom of the oval on the same bearing plane as its feet.
Ring offset_ring(const Ring& ring,float thickness){
    Ring out=ring;
    const auto side=normalize(ring[1]-ring[0]);
    for(std::size_t i=0;i<ring.size();++i){
        const auto p=ring[i];
        const auto tangent=[&](std::size_t j,std::size_t k){return normalize(ring[k]-ring[j]);};
        const auto outward=[&](Vec3 t){return side*t.y-Vec3{0,dot(t,side),0};};
        const auto n0=outward(tangent((i+ring.size()-1)%ring.size(),i)),n1=outward(tangent(i,(i+1)%ring.size()));
        const auto n=normalize(n0+n1);
        const float m=thickness/std::max(.6f,dot(n,n1));
        out[i]=p+n*m;
        // A low ground-level sleeve has no room for a projecting flange below grade.
        out[i].y=std::max(.025f,out[i].y);
    }
    return out;
}
void band(CorridorMesh& mesh,const Ring& a,const Ring& b,float thickness){
    const auto oa=offset_ring(a,thickness),ob=offset_ring(b,thickness);
    const auto quad=[&](Vec3 p,Vec3 q,Vec3 r,Vec3 s){
        if(length(cross(q-p,r-p))>1e-8f)tri(mesh.details,p,q,r);
        if(length(cross(r-p,s-p))>1e-8f)tri(mesh.details,p,r,s);
    };
    const bool reverse=cross(a[1]-a[0],b[0]-a[0]).y>0;
    const auto face=[&](Vec3 p,Vec3 q,Vec3 r,Vec3 s){if(reverse)quad(p,s,r,q);else quad(p,q,r,s);};
    for(std::size_t i=0;i<a.size();++i){const auto j=(i+1)%a.size();
        face(oa[i],oa[j],ob[j],ob[i]);
        // The shared pressure skin is the inner surface: no duplicate coplanar face.
        face(a[i],a[j],oa[j],oa[i]);
        face(b[j],b[i],ob[i],ob[j]);
        edge(mesh.edges,oa[i],oa[j]);edge(mesh.edges,ob[i],ob[j]);
    }
}
void bearing(CorridorMesh& mesh,const Ring& a,const Ring& b,float thickness,float spacing){
    const auto oa=offset_ring(a,thickness),ob=offset_ring(b,thickness);
    const auto mid=interpolate_ring(oa,ob,.5f);
    const auto side=flat_normal(mid[1]-mid[0]);const Vec3 axis{side.z,0,-side.x};
    structural::Builder frame{mesh.details,mesh.edges,{}};
    const auto point=[&](float along,float across){
        const auto left=oa[0]+(ob[0]-oa[0])*along,right=oa[1]+(ob[1]-oa[1])*along;
        return left+(right-left)*across;
    };
    // The support tops are cut to the actual sloping underside of the hoop,
    // not sunk into it. Foot pads and columns share the same boundary plane.
    for(float t:{.08f,.92f}){
        const float width=distance_xz(mid[0],mid[1]);
        const float dt=std::min(.06f/width,.055f);
        const std::array<Vec3,4> top{{point(.18f,t-dt),point(.82f,t-dt),point(.82f,t+dt),point(.18f,t+dt)}};
        float low=top[0].y;for(auto p:top)low=std::min(low,p.y);
        const float pad=std::min(.035f,low);if(pad<=.001f)continue;
        const auto center=point(.5f,t);const float pad_width=std::min(.29f,spacing*.70f);
        frame.box({center.x,pad*.5f,center.z},side,{0,1,0},axis,{pad_width,pad,pad_width});
        std::array<Vec3,4> base=top;for(auto& p:base)p.y=pad;
        const auto face=[&](Vec3 p,Vec3 q,Vec3 r,Vec3 s){
            const auto c=(p+q+r+s)*.25f;const Vec3 middle{center.x,(pad+center.y)*.5f,center.z};
            if(dot(cross(q-p,r-p),c-middle)<0)std::swap(q,s);
            if(length(cross(q-p,r-p))>1e-8f)tri(mesh.details,p,q,r);
            if(length(cross(r-p,s-p))>1e-8f)tri(mesh.details,p,r,s);
        };
        for(int j=0;j<4;++j){const int k=(j+1)%4;face(base[j],base[k],top[k],top[j]);}
        // Shared pad/column and hoop/column faces deliberately omitted.
    }
}
void append_oval_transition_frame(CorridorMesh& mesh,const Ring& a,const Ring& b,bool supports){
    if(a.size()!=b.size()||a.size()<3)return;
    const auto start=(a[0]+a[1])*.5f,finish=(b[0]+b[1])*.5f;
    const float run=distance_xz(start,finish);if(run<.12f)return;
    const int count=std::max(1,int(std::ceil(run/2.4f)));
    const float half=std::min(.065f/run,.28f/count);
    constexpr float depth=.075f;
    for(int i=0;i<count;++i){const float t=(i+.5f)/count;
        const auto left=interpolate_ring(a,b,t-half),right=interpolate_ring(a,b,t+half);
        band(mesh,left,right,depth);if(supports)bearing(mesh,left,right,depth,run/count);
    }
    // Longitudinal stringers stop at the flange faces, so there are no
    // intersecting beam ends. Mount them along the same swept skin vertices.
    if(run<.85f)return; // The single supported flange spans a short door adapter.
    structural::Builder rails{mesh.details,mesh.edges,{}};
    const auto outer_a=offset_ring(a,.035f),outer_b=offset_ring(b,.035f);
    for(int i=0;i<=count;++i){
        const float t0=i==0?0.f:(i-.5f)/count+half;
        const float t1=i==count?1.f:(i+.5f)/count-half;
        if((t1-t0)*run<.01f)continue;
        for(std::size_t k:{std::size_t(2),std::size_t(12)}){
            if(k>=a.size())continue;
            const auto p=outer_a[k]+(outer_b[k]-outer_a[k])*t0,q=outer_a[k]+(outer_b[k]-outer_a[k])*t1;
            rails.beam(p,q,.035f,.045f,{0,1,0});
        }
    }
}
} // namespace

std::vector<Vec2> doorway_cross_section(float half_width,float height){
    std::vector<Vec2> out;
    out.reserve(13);
    constexpr float floor_y=0.05f;
    const float side_y=height-geometry::kGeneratedEntranceCrownRise; // Rounded crown, clear hatch header.
    out.push_back({-half_width,floor_y});
    out.push_back({ half_width,floor_y});
    constexpr int ARC=10;
    for(int i=0;i<=ARC;++i){
        const float a=(float)i/(float)ARC*PI;
        out.push_back({std::cos(a)*half_width,
                       side_y+std::sin(a)*(height-side_y)});
    }
    return out;
}

// Connect two pressure profiles vertex-for-vertex. The terminal rings are not
// overlapped: the first profile lies exactly on the door plane and the second
// profile is exactly the corridor D-profile used by append_swept_shell().
// This makes the habitat/door/corridor seam deterministic instead of relying
// on intersecting sleeves to hide small orientation or position errors.
void append_profile_transition(CorridorMesh& mesh,Vec3 a,Vec3 b,Vec3 normal,
                               const std::vector<Vec2>& profile_a,
                               const std::vector<Vec2>& profile_b){
    normal=flat_normal(normal);
    if(length(normal)<0.001f || profile_a.size()!=profile_b.size() || profile_a.size()<3) return;
    const Vec3 side=perp_xz(normal);
    for(std::size_t i=0;i<profile_a.size();++i){
        const std::size_t j=(i+1)%profile_a.size();
        const Vec3 a0=ring_vertex(a,side,profile_a[i]);
        const Vec3 a1=ring_vertex(a,side,profile_a[j]);
        const Vec3 b0=ring_vertex(b,side,profile_b[i]);
        const Vec3 b1=ring_vertex(b,side,profile_b[j]);
        tri(mesh.shell,a0,b0,b1);
        tri(mesh.shell,a0,b1,a1);
        edge(mesh.edges,a0,a1);
    }
}

void append_generated_entrance(CorridorMesh& mesh,const CorridorDoor& door){
    if(!door.generated || length(door.position-door.wall_position)<.01f) return;
    const Vec3 n=flat_normal(door.normal);
    const Vec3 side=perp_xz(n);
    const Vec3 door_base=door.position-Vec3{0,geometry::kCorridorDeckY,0};
    const auto profile=doorway_cross_section(GENERATED_ENTRANCE_HALF_WIDTH,GENERATED_ENTRANCE_HEIGHT);

    // The inner ring follows the actual habitat pressure-envelope vertices.
    // This is intentionally not a flat rectangle: domes and faceted hulls get
    // a conformal collar with every edge terminating on the real structure.
    if(door.wall_profile.size()==profile.size()){
        std::vector<Vec3> outer;
        outer.reserve(profile.size());
        for(const Vec2 q:profile) outer.push_back(ring_vertex(door_base,side,q));
        for(std::size_t i=0;i<profile.size();++i){
            const std::size_t j=(i+1)%profile.size();
            tri(mesh.shell,door.wall_profile[i],outer[i],outer[j]);
            tri(mesh.shell,door.wall_profile[i],outer[j],door.wall_profile[j]);
            edge(mesh.edges,door.wall_profile[i],door.wall_profile[j]);
        }
    }else{
        const Vec3 wall_base=door.wall_position-Vec3{0,geometry::kCorridorDeckY,0};
        append_profile_transition(mesh,wall_base,door_base,n,profile,profile);
    }
    // The actual pressure hatch is rendered at the wall by draw_sliding_door.
    // Reinforce the vestibule with the same skin section as its pressure loft.
    const auto wall_base=door.wall_position-Vec3{0,WALKWAY_Y,0};
    append_oval_transition_frame(mesh,profile_ring(wall_base,side,profile),profile_ring(door_base,side,profile),true);
}

// Continuous deck ramp: geometry and the shared crew surface query use the
// exact same endpoints. No oversized final riser or unsampled stair gap.
void append_entrance_stairs(CorridorMesh& mesh,const CorridorEndpoint& endpoint,Vec3 terminal_tangent){
    if(!building_endpoint(endpoint))return;
    const auto& door=endpoint.door;
    const Vec3 n=flat_normal(door.normal),side=perp_xz(n)*WALKWAY_HALF_WIDTH;
    Vec3 end_side=perp_xz(terminal_tangent)*WALKWAY_HALF_WIDTH;
    if(dot(side,end_side)<0)end_side=end_side*-1.f;
    Vec3 inner=door.wall_position;inner.y=door.position.y;
    Vec3 outer=endpoint.position;outer.y+=WALKWAY_Y;
    const auto deck=[&](Vec3 a,Vec3 sa,Vec3 b,Vec3 sb){
        if(distance_xz(a,b)<.001f)return;
        const auto uptri=[&](Vec3 p,Vec3 q,Vec3 r){if(cross(q-p,r-p).y<0)std::swap(q,r);tri(mesh.details,p,q,r);};
        const auto quad=[&](Vec3 p,Vec3 q,Vec3 r,Vec3 s){uptri(p,q,r);uptri(p,r,s);};
        const int steps=docking_step_count(a,b);
        const int panels=steps>1?steps:std::max(1,int(std::ceil(distance_xz(a,b)/.60f)));
        for(int i=0;i<panels;++i){
            const float t0=float(i)/panels,t1=float(i+1)/panels;
            auto p=a+(b-a)*t0,q=a+(b-a)*t1;
            const auto u=sa+(sb-sa)*t0,v=sa+(sb-sa)*t1;
            if(steps>1)p.y=q.y=docking_floor_y(a,b,(t0+t1)*.5f);
            quad(p-u,q-v,q+v,p+u);
            // Continuous underside fascia and transverse tread nose, no holes between panels.
            for(float sign:{-1.f,1.f}){auto x=p+u*sign,y=q+v*sign;tri(mesh.details,x,y,y-Vec3{0,.08f,0});tri(mesh.details,x,y-Vec3{0,.08f,0},x-Vec3{0,.08f,0});}
            const auto nose=q+(p-q)*std::min(.15f,.035f/std::max(.001f,distance_xz(p,q)));
            quad(nose-v*.91f+Vec3{0,.008f,0},q-v*.91f+Vec3{0,.008f,0},q+v*.91f+Vec3{0,.008f,0},nose+v*.91f+Vec3{0,.008f,0});
            if(steps>1){const float next=i+1==panels?b.y:docking_floor_y(a,b,(float(i)+1.5f)/panels);if(std::abs(next-q.y)>.0001f){auto low=q;low.y=next;tri(mesh.details,q-v,low-v,low+v);tri(mesh.details,q-v,low+v,q+v);}}
        }
        if(steps>1){auto first=a;first.y=docking_floor_y(a,b,.5f/panels);if(std::abs(first.y-a.y)>.0001f){tri(mesh.details,a-sa,first-sa,first+sa);tri(mesh.details,a-sa,first+sa,a+sa);}}
        edge(mesh.edges,a-sa,b-sb);edge(mesh.edges,a+sa,b+sb);
    };
    deck(inner-n*.7f,side,inner,side);
    deck(inner,side,door.position,side);
    deck(door.position,side,outer,end_side);
}

void append_docking_collar(CorridorMesh& mesh,const CorridorEndpoint& endpoint,Vec3 terminal_tangent){
    if(endpoint.kind!=CorridorEndpointKind::Habitat && endpoint.kind!=CorridorEndpointKind::Resource) return;
    const CorridorDoor& door=endpoint.door;
    const Vec3 n=flat_normal(door.normal);
    append_generated_entrance(mesh,door);

    // Start exactly on the actual door plane. For generated entrances this is
    // the outer vestibule door; for built-in entrances it is the structure's
    // declared hatch plane. The adapter then morphs once, without overlap, into
    // the exact D-profile used by the corridor shell at endpoint.position.
    const Vec3 door_base=door.position-Vec3{0,geometry::kCorridorDeckY,0};
    // Match the pressure-union solver's generated hatch section, including
    // the terminal side orientation on the opposite end of the corridor.
    const auto door_profile=doorway_cross_section(GENERATED_ENTRANCE_HALF_WIDTH,GENERATED_ENTRANCE_HEIGHT);
    const auto corridor_profile=corridor_cross_section();
    const Vec3 side=perp_xz(n);
    Vec3 terminal_side=perp_xz(terminal_tangent);
    if(dot(side,terminal_side)<0)terminal_side=terminal_side*-1.f;
    const auto hatch=profile_ring(door_base,side,door_profile),terminal=profile_ring(endpoint.position,terminal_side,corridor_profile);
    for(std::size_t i=0;i<hatch.size();++i){const auto j=(i+1)%hatch.size();
        tri(mesh.shell,hatch[i],terminal[i],terminal[j]);tri(mesh.shell,hatch[i],terminal[j],hatch[j]);
        edge(mesh.edges,hatch[i],hatch[j]);
    }
    append_oval_transition_frame(mesh,hatch,terminal,true);
    append_entrance_stairs(mesh,endpoint,terminal_tangent);
    // The main run already supplies its first oval hoop and bearing pair.
    // No duplicated terminal hoop, threshold box or free-standing hatch.

}

void append_sealed_open_end(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    if(endpoint.kind!=CorridorEndpointKind::OpenDoor || endpoint.suppress_open_door) return;
    Vec3 n=flat_normal(endpoint.tangent);
    if(length(n)<0.001f) n={1,0,0};
    const Vec3 side=perp_xz(n);
    const auto section=corridor_cross_section();
    if(section.size()<3) return;

    std::vector<Vec3> ring;
    ring.reserve(section.size());
    Vec3 centroid{};
    for(const Vec2 q:section){
        const Vec3 p=ring_vertex(endpoint.position,side,q);
        ring.push_back(p);
        centroid=centroid+p;
    }
    centroid=centroid*(1.0f/(float)ring.size());

    // A dead-end corridor terminates in a structural pressure bulkhead, not a
    // hatch.  The cap lives in the shell itself so FPV cannot carve or animate
    // it as a door and there is no exterior vestibule/deck beyond the endpoint.
    for(std::size_t i=0;i<ring.size();++i){
        const std::size_t j=(i+1)%ring.size();
        tri(mesh.shell,centroid,ring[i],ring[j]);
        edge(mesh.edges,ring[i],ring[j]);
    }
    append_ring(mesh,endpoint.position,n,1.08f);
    append_oriented_box(mesh.details,
                        endpoint.position-n*0.055f+Vec3{0,1.12f,0},
                        n,0.07f,0.48f,0.58f);
    // Cross-braced pressure plate communicates that this is a structural
    // bulkhead, not an unimplemented/open doorway.
    append_oriented_box(mesh.details,endpoint.position-n*0.075f+Vec3{0,1.12f,0},n,0.045f,1.05f,0.055f);
    append_oriented_box(mesh.details,endpoint.position-n*0.078f+Vec3{0,1.12f,0},side,0.045f,0.055f,0.92f);
}

// Pressure union generates the contact curve; this helper adds the walking bridge.
void append_surface_matched_junction(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    Vec3 branch=flat_normal(endpoint.tangent);
    Vec3 host=flat_normal(endpoint.host_tangent);
    if(length(branch)<0.001f || length(host)<0.001f) return;
    const Vec3 host_side=perp_xz(host);
    const float side_sign=dot(branch,host_side)>=0.0f?1.0f:-1.0f;

    append_ring(mesh,endpoint.position,branch,1.0f);

    // Bridge the branch walkway to the host walkway edge inside the shared
    // pressure volume. This stays below the pressure shell and removes the
    // small floor gap between the host's 0.72 m walkway and 1.08 m hull side.
    const Vec3 host_walk_edge=endpoint.host_position+host_side*(side_sign*WALKWAY_HALF_WIDTH)+Vec3{0,WALKWAY_Y,0};
    const Vec3 shell_edge=endpoint.host_position+host_side*(side_sign*CORRIDOR_HALF_WIDTH)+Vec3{0,WALKWAY_Y,0};
    Vec3 hs=host*WALKWAY_HALF_WIDTH;
    Vec3 bs=perp_xz(branch)*WALKWAY_HALF_WIDTH;if(dot(hs,bs)<0)hs=hs*-1.f;
    const auto quad=[&](Vec3 a,Vec3 sa,Vec3 b,Vec3 sb){
        for(auto t:std::array<std::array<Vec3,3>,2>{{{{a-sa,b-sb,b+sb}},{{a-sa,b+sb,a+sa}}}}){if(cross(t[1]-t[0],t[2]-t[0]).y<0)std::swap(t[1],t[2]);tri(mesh.details,t[0],t[1],t[2]);}
        edge(mesh.edges,a-sa,b-sb);edge(mesh.edges,a+sa,b+sb);
    };
    // Include the actual adapter run outside the host shell, not only its inner lip.
    quad(endpoint.position+Vec3{0,WALKWAY_Y,0},bs,shell_edge,hs);
    quad(shell_edge,hs,host_walk_edge,hs);
}

void append_corridor_junction(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    if(endpoint.kind!=CorridorEndpointKind::Corridor || endpoint.seamless_splice) return;
    if(endpoint.surface_matched_junction){
        append_surface_matched_junction(mesh,endpoint);
        return;
    }
    Vec3 branch=flat_normal(endpoint.tangent);
    if(length(branch)<0.001f) branch={1,0,0};
    Vec3 host=flat_normal(endpoint.host_tangent);
    if(length(host)<0.001f) host=branch;

    // Both tubes terminate at the exact same centerline point. Use shared rings
    // instead of an oversized overlapping hub so the visual connection has no
    // gap and no duplicate sleeve extending along either corridor.
    append_ring(mesh,endpoint.position,branch,1.08f);
    append_ring(mesh,endpoint.position,host,1.08f);
    const Vec3 s0=perp_xz(branch)*WALKWAY_HALF_WIDTH;
    const Vec3 s1=perp_xz(host)*WALKWAY_HALF_WIDTH;
    const Vec3 c=endpoint.position+Vec3{0,WALKWAY_Y,0};
    edge(mesh.edges,c-s0,c+s0);
    edge(mesh.edges,c-s1,c+s1);
}

struct AutomaticCrossing {
    Vec3 position{};
    Vec3 self_tangent{1,0,0};
    Vec3 other_tangent{0,0,1};
    std::size_t other_index{};
};

struct MergedCrossingJunction {
    Vec3 position{};
    std::size_t samples{0};
    std::vector<Vec3> directions{};
    std::vector<std::size_t> participants{};
    std::vector<Vec3> footprint{};
    std::vector<Vec3> cut_footprint{};
};

std::vector<AutomaticCrossing> collect_automatic_crossings(const std::vector<Vec3>& path,
                                                            const std::vector<CorridorConnection>& existing,
                                                            std::size_t self_index){
    std::vector<AutomaticCrossing> out;
    for(std::size_t i=0;i+1<path.size();++i){
        for(std::size_t ci=0;ci<existing.size();++ci){
            if(ci==self_index) continue;
            if(self_index<existing.size() && existing[self_index].construction.complete()!=existing[ci].construction.complete())continue;
            const auto& c=existing[ci];
            for(std::size_t j=0;j+1<c.path.size();++j){
                Vec3 hit{};
                if(!mergeable_corridor_crossing(path[i],path[i+1],c.path[j],c.path[j+1],hit)) continue;
                bool duplicate=false;
                for(const auto& e:out) if(e.other_index==ci && distance_xz(e.position,hit)<0.25f){ duplicate=true; break; }
                if(duplicate) continue;
                out.push_back({hit,flat_normal(path[i+1]-path[i]),flat_normal(c.path[j+1]-c.path[j]),ci});
            }
        }
    }
    return out;
}

namespace {
constexpr float JUNCTION_CLUSTER_RADIUS=CORRIDOR_HALF_WIDTH*1.10f;
constexpr float JUNCTION_ARM_LENGTH=CORRIDOR_HALF_WIDTH*1.48f;
constexpr float JUNCTION_HALF_WIDTH=CORRIDOR_HALF_WIDTH*1.10f;
constexpr float JUNCTION_CUT_SCALE=0.90f;

float cross_xz(Vec3 o,Vec3 a,Vec3 b){
    return (a.x-o.x)*(b.z-o.z)-(a.z-o.z)*(b.x-o.x);
}

std::vector<Vec3> convex_hull_xz(std::vector<Vec3> points){
    if(points.size()<=3) return points;
    std::sort(points.begin(),points.end(),[](Vec3 a,Vec3 b){
        if(std::abs(a.x-b.x)>1.0e-5f) return a.x<b.x;
        return a.z<b.z;
    });
    points.erase(std::unique(points.begin(),points.end(),[](Vec3 a,Vec3 b){
        return distance_xz(a,b)<0.001f;
    }),points.end());
    if(points.size()<=3) return points;

    std::vector<Vec3> lower,upper;
    for(const Vec3 p:points){
        while(lower.size()>=2 && cross_xz(lower[lower.size()-2],lower.back(),p)<=0.0001f) lower.pop_back();
        lower.push_back(p);
    }
    for(auto it=points.rbegin();it!=points.rend();++it){
        const Vec3 p=*it;
        while(upper.size()>=2 && cross_xz(upper[upper.size()-2],upper.back(),p)<=0.0001f) upper.pop_back();
        upper.push_back(p);
    }
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(),upper.begin(),upper.end());
    return lower;
}

void add_unique_direction(std::vector<Vec3>& directions,Vec3 d){
    d=flat_normal(d);
    if(length(d)<0.001f) return;
    // Corridor axes are bidirectional. Canonicalising the sign keeps +X/-X
    // from becoming two separate arms while still preserving different angles.
    if(d.x<-0.0001f || (std::abs(d.x)<=0.0001f && d.z<0.0f)) d=d*-1.0f;
    for(const Vec3 e:directions) if(std::abs(dot(e,d))>0.965f) return;
    directions.push_back(d);
}

void add_unique_participant(std::vector<std::size_t>& participants,std::size_t index){
    if(index==NONE_INDEX) return;
    if(std::find(participants.begin(),participants.end(),index)==participants.end()) participants.push_back(index);
}

std::vector<Vec3> junction_footprint(Vec3 center,const std::vector<Vec3>& directions,float scale){
    std::vector<Vec3> points;
    points.reserve(directions.size()*4);
    const float arm=JUNCTION_ARM_LENGTH*scale;
    const float half=JUNCTION_HALF_WIDTH*scale;
    for(Vec3 d:directions){
        d=flat_normal(d);
        if(length(d)<0.001f) continue;
        const Vec3 side=perp_xz(d)*half;
        for(float sign:{-1.0f,1.0f}){
            const Vec3 end=center+d*(arm*sign);
            points.push_back(end-side);
            points.push_back(end+side);
        }
    }
    return convex_hull_xz(std::move(points));
}

std::vector<MergedCrossingJunction> consolidate_crossings(const std::vector<AutomaticCrossing>& crossings,
                                                           std::size_t self_index){
    std::vector<MergedCrossingJunction> groups;
    for(const auto& c:crossings){
        MergedCrossingJunction* group=nullptr;
        for(auto& g:groups){
            if(distance_xz(g.position,c.position)<=JUNCTION_CLUSTER_RADIUS){ group=&g; break; }
        }
        if(!group){
            groups.push_back({});
            group=&groups.back();
            group->position=c.position;
        }else{
            group->position=(group->position*(float)group->samples+c.position)*(1.0f/(float)(group->samples+1));
        }
        ++group->samples;
        add_unique_direction(group->directions,c.self_tangent);
        add_unique_direction(group->directions,c.other_tangent);
        add_unique_participant(group->participants,self_index);
        add_unique_participant(group->participants,c.other_index);
    }
    for(auto& g:groups){
        g.footprint=junction_footprint(g.position,g.directions,1.0f);
        g.cut_footprint=junction_footprint(g.position,g.directions,JUNCTION_CUT_SCALE);
    }
    return groups;
}

bool junction_edge_opening(Vec3 a,Vec3 b,const MergedCrossingJunction& junction,Vec3& outward){
    const Vec3 mid=(a+b)*0.5f;
    Vec3 radial=flat_normal(mid-junction.position);
    const Vec3 edge_dir=flat_normal(b-a);
    if(length(radial)<0.001f || length(edge_dir)<0.001f) return false;
    float best=0.0f;
    Vec3 best_dir{};
    for(Vec3 d:junction.directions){
        d=flat_normal(d);
        const float alignment=std::abs(dot(radial,d));
        const float perpendicular=std::abs(dot(edge_dir,d));
        if(alignment>0.84f && perpendicular<0.34f && alignment>best){
            best=alignment;
            best_dir=dot(radial,d)>=0.0f?d:d*-1.0f;
        }
    }
    if(best<=0.0f) return false;
    outward=best_dir;
    return true;
}

void append_wall_quad(CorridorMesh& mesh,Vec3 a0,Vec3 a1,Vec3 b1,Vec3 b0){
    tri(mesh.shell,a0,a1,b1);
    tri(mesh.shell,a0,b1,b0);
}

void append_d_profile_opening_wall(CorridorMesh& mesh,Vec3 edge_a,Vec3 edge_b,Vec3 outward,float base_y){
    const Vec3 side=flat_normal(edge_b-edge_a);
    if(length(side)<0.001f) return;
    const Vec3 center=(edge_a+edge_b)*0.5f;
    const float half_edge=distance_xz(edge_a,edge_b)*0.5f;
    const float aperture_half=std::min(CORRIDOR_HALF_WIDTH,half_edge-0.03f);
    const float floor_y=base_y+CORRIDOR_FLOOR_Y;
    const float top_y=base_y+CORRIDOR_SIDE_Y+CORRIDOR_ROOF_RADIUS;

    // Side pressure plates leave exactly the corridor-width D-profile opening.
    if(half_edge>aperture_half+0.015f){
        const Vec3 l0=center-side*half_edge+Vec3{0,floor_y-center.y,0};
        const Vec3 l1=center-side*aperture_half+Vec3{0,floor_y-center.y,0};
        const Vec3 lt0={l0.x,top_y,l0.z},lt1={l1.x,top_y,l1.z};
        append_wall_quad(mesh,l0,l1,lt1,lt0);
        const Vec3 r0=center+side*aperture_half+Vec3{0,floor_y-center.y,0};
        const Vec3 r1=center+side*half_edge+Vec3{0,floor_y-center.y,0};
        const Vec3 rt0={r0.x,top_y,r0.z},rt1={r1.x,top_y,r1.z};
        append_wall_quad(mesh,r0,r1,rt1,rt0);
    }

    // Above the rounded D-profile roof, a faceted spandrel closes the chamber
    // up to its flat service ceiling. The opening itself therefore matches the
    // normal corridor pressure shell without any circular/cylindrical hub.
    const auto section=corridor_cross_section();
    for(std::size_t i=2;i+1<section.size();++i){
        const Vec2 qa=section[i],qb=section[i+1];
        const Vec3 ca=center+side*qa.x+Vec3{0,base_y+qa.y-center.y,0};
        const Vec3 cb=center+side*qb.x+Vec3{0,base_y+qb.y-center.y,0};
        const Vec3 ta{ca.x,top_y,ca.z},tb{cb.x,top_y,cb.z};
        append_wall_quad(mesh,ca,cb,tb,ta);
    }
    append_ring(mesh,{center.x,base_y,center.z},outward,1.0f);
}

} // namespace

void carve_automatic_crossings(CorridorMesh& mesh,const std::vector<MergedCrossingJunction>& junctions){
    // Trim deck/services to exact boundaries; pressure panels use the shared union.
    for(const auto& junction:junctions){
        std::vector<std::pair<Vec3,float>> planes;const auto& poly=junction.footprint;
        Vec3 center{};for(auto p:poly)center=center+p;center=center/float(poly.size());
        for(std::size_t i=0;i<poly.size();++i){
            const auto a=poly[i],b=poly[(i+1)%poly.size()];Vec3 normal=flat_normal({b.z-a.z,0,a.x-b.x});
            if(dot(normal,center-a)>0)normal=normal*-1.f;
            planes.push_back({normal,dot(normal,a)});
        }
        planes.push_back({{0,1,0},junction.position.y+CORRIDOR_SIDE_Y+CORRIDOR_ROOF_RADIUS+.12f});
        planes.push_back({{0,-1,0},-junction.position.y-CORRIDOR_FLOOR_Y+.12f});
        std::vector<Vec3> details;
        for(std::size_t i=0;i+2<mesh.details.size();i+=3){
            std::vector<Vec3> inside{mesh.details[i],mesh.details[i+1],mesh.details[i+2]};
            for(const auto& [normal,offset]:planes){
                const auto outside=mesh_portals::clip(inside,normal,offset,true);
                for(std::size_t j=1;j+1<outside.size();++j)if(length(cross(outside[j]-outside[0],outside[j+1]-outside[0]))>1.e-7f)tri(details,outside[0],outside[j],outside[j+1]);
                inside=mesh_portals::clip(inside,normal,offset,false);if(inside.empty())break;
            }
        }
        mesh.details=std::move(details);
    }
}

void append_merged_crossing_manifold(CorridorMesh& mesh,const MergedCrossingJunction& junction){
    const auto& poly=junction.footprint;
    if(poly.size()<4) return;
    const float base_y=junction.position.y;
    const float floor_y=base_y+CORRIDOR_FLOOR_Y;
    const float deck_y=base_y+WALKWAY_Y;
    const float top_y=base_y+CORRIDOR_SIDE_Y+CORRIDOR_ROOF_RADIUS;
    const Vec3 floor_center{junction.position.x,floor_y,junction.position.z};
    const Vec3 deck_center{junction.position.x,deck_y,junction.position.z};
    const Vec3 roof_center{junction.position.x,top_y,junction.position.z};

    // One convex, faceted pressure manifold replaces the old round/cylindrical
    // crossing hub. Its footprint is the convex union of all corridor approach
    // throats, so X/T/Y/angled crossings share one structural volume even when
    // several directions arrive at nearly the same point.
    for(std::size_t i=0;i<poly.size();++i){
        const std::size_t j=(i+1)%poly.size();
        const Vec3 f0{poly[i].x,floor_y,poly[i].z},f1{poly[j].x,floor_y,poly[j].z};
        const Vec3 d0{poly[i].x,deck_y,poly[i].z},d1{poly[j].x,deck_y,poly[j].z};
        const Vec3 t0{poly[i].x,top_y,poly[i].z},t1{poly[j].x,top_y,poly[j].z};
        tri(mesh.shell,floor_center,f1,f0);
        tri(mesh.shell,roof_center,t0,t1);
        tri(mesh.details,deck_center,d0,d1);
        edge(mesh.edges,d0,d1);
        edge(mesh.edges,t0,t1);

        structural::Builder frame{mesh.details,mesh.edges,{}};
        const auto out=flat_normal((poly[i]+poly[j])*.5f-junction.position);
        frame.beam(t0+Vec3{0,.08f,0},t1+Vec3{0,.08f,0},.16f,.20f,{0,1,0},true);
        frame.beam(f0,f1,.15f,.16f,{0,-1,0},true);
        frame.beam(f0+out*.055f,t0+out*.055f,.14f,.18f,out,true);
        Vec3 outward{};
        if(junction_edge_opening(poly[i],poly[j],junction,outward)){
            append_d_profile_opening_wall(mesh,poly[i],poly[j],outward,base_y);
        }else{
            append_wall_quad(mesh,f0,f1,t1,t0);
            edge(mesh.edges,f0,t0);
        }
        frame.box({f0.x,base_y+.018f,f0.z},{1,0,0},{0,1,0},{0,0,1},{.34f,.036f,.32f});
        frame.beam({f0.x,base_y+.035f,f0.z},f0,.18f,.16f,out);
    }
    Vec3 low=poly.front(),high=low;
    for(const auto p:poly){low.x=std::min(low.x,p.x);low.z=std::min(low.z,p.z);high.x=std::max(high.x,p.x);high.z=std::max(high.z,p.z);}
    for(float x=low.x;x<high.x;x+=2*WALKWAY_HALF_WIDTH){
        CorridorMesh panels;append_deck_panels(panels,{{x+WALKWAY_HALF_WIDTH,base_y,low.z},{x+WALKWAY_HALF_WIDTH,base_y,high.z}});
        for(std::size_t t=0;t+2<panels.details.size();t+=3){
            std::vector<Vec3> fragment{panels.details[t],panels.details[t+1],panels.details[t+2]};
            for(std::size_t j=0;j<poly.size()&&!fragment.empty();++j){const auto a=poly[j],b=poly[(j+1)%poly.size()];
                auto n=flat_normal({b.z-a.z,0,a.x-b.x});if(dot(n,junction.position-a)>0)n=n*-1.f;
                fragment=mesh_portals::clip(fragment,n,dot(n,a),false);
            }
            for(std::size_t j=1;j+1<fragment.size();++j)if(length(cross(fragment[j]-fragment[0],fragment[j+1]-fragment[0]))>1.e-7f)tri(mesh.details,fragment[0],fragment[j],fragment[j+1]);
        }
    }
}

void append_crossing_hubs(CorridorMesh& mesh,const std::vector<MergedCrossingJunction>& junctions,std::size_t self_index){
    for(const auto& junction:junctions){
        if(self_index!=NONE_INDEX){
            std::size_t owner=self_index;
            for(const std::size_t p:junction.participants) owner=std::min(owner,p);
            if(owner!=self_index) continue;
        }
        append_merged_crossing_manifold(mesh,junction);
    }
}

CorridorMesh build_endpoint_fitting(const CorridorEndpoint& endpoint){
    CorridorMesh mesh;
    append_docking_collar(mesh,endpoint,endpoint.tangent);
    append_corridor_junction(mesh,endpoint);
    return mesh;
}

CorridorMesh build_mesh(const CorridorEndpoint& a,const CorridorEndpoint& b,const std::vector<Vec3>& path,
                        const std::vector<CorridorConnection>& existing,std::size_t self_index){
    CorridorMesh mesh;
    corridor_module().build_outer(mesh,path);
    corridor_module().build_inner(mesh,path);
    append_docking_collar(mesh,a,path_tangent(path,0));
    append_docking_collar(mesh,b,path_tangent(path,path.empty()?0:path.size()-1));
    append_corridor_junction(mesh,a);
    append_corridor_junction(mesh,b);
    append_sealed_open_end(mesh,a);
    append_sealed_open_end(mesh,b);
    // Physical host/branch apertures are computed by the closed-volume union.
    // Transversal path crossings are true shared junctions. Carve both pressure
    // tubes locally and let exactly one mesh own the replacement chamber so the
    // intersection contains no doubled shell, floor or rib geometry.
    const auto automatic_crossings=collect_automatic_crossings(path,existing,self_index);
    const auto merged_junctions=consolidate_crossings(automatic_crossings,self_index);
    carve_automatic_crossings(mesh,merged_junctions);
    append_crossing_hubs(mesh,merged_junctions,self_index);
    if(self_index==NONE_INDEX){
        CorridorConnection candidate;candidate.endpoint_a=a;candidate.endpoint_b=b;candidate.path=path;
        std::vector<CorridorConnection> preview;preview.push_back(std::move(candidate));
        rebuild_pressure_shells(preview);mesh.shell=std::move(preview.front().mesh.shell);
    }
    mesh.prepare_surfaces(extended_floor_path(a,b,path));
    return mesh;
}

std::vector<PressureChamber> pressure_chambers(const std::vector<CorridorConnection>& connections){
    std::vector<PressureChamber> out;
    for(std::size_t i=0;i<connections.size();++i){
        const auto crossings=collect_automatic_crossings(connections[i].path,connections,i);
        for(const auto& junction:consolidate_crossings(crossings,i)){
            std::size_t owner=i;for(auto index:junction.participants)owner=std::min(owner,index);
            if(owner==i)out.push_back({junction.position,junction.footprint,owner});
        }
    }
    return out;
}

void rebuild_all_meshes(std::vector<CorridorConnection>& connections){
    for(std::size_t i=0;i<connections.size();++i){
        auto& c=connections[i];
        c.mesh=build_mesh(c.endpoint_a,c.endpoint_b,c.path,connections,i);
    }
    rebuild_pressure_shells(connections);
}

} // namespace game::corridor_detail
