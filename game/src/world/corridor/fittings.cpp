#include "game/world/corridor/detail.hpp"
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

std::vector<Vec2> doorway_cross_section(float half_width,float height){
    std::vector<Vec2> out;
    out.reserve(13);
    constexpr float floor_y=0.05f;
    const float side_y=std::min(0.72f,height*0.30f);
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

void append_door_frame(CorridorMesh& mesh,Vec3 center,Vec3 normal,float half_width=geometry::kDoorFrameHalfWidth,float height=geometry::kDoorFrameHeight){
    normal=flat_normal(normal);
    const Vec3 side=perp_xz(normal);
    const float bar=0.15f;
    const Vec3 base{center.x,0,center.z};
    append_oriented_box(mesh.details,base-side*(half_width-bar)+Vec3{0,height*0.5f,0},normal,0.12f,bar,height*0.5f);
    append_oriented_box(mesh.details,base+side*(half_width-bar)+Vec3{0,height*0.5f,0},normal,0.12f,bar,height*0.5f);
    append_oriented_box(mesh.details,base+Vec3{0,height-bar,0},normal,0.12f,half_width,bar);
    append_oriented_box(mesh.details,base+Vec3{0,bar,0},normal,0.12f,half_width,bar);
}

void append_pressure_seal(CorridorMesh& mesh,Vec3 center,Vec3 normal,float half_width,float height){
    normal=flat_normal(normal);
    if(length(normal)<0.001f) return;
    const Vec3 side=perp_xz(normal);
    const float bar=0.070f;
    const float depth=0.045f;
    const Vec3 base{center.x,0.055f,center.z};
    // A narrow continuous gasket/flange sits inside the structural door frame.
    // It is deliberately separate geometry so no visual gap survives between
    // the pressure sleeve and a habitat/resource hull.
    append_oriented_box(mesh.details,base-side*(half_width-bar)+Vec3{0,height*0.5f,0},normal,depth,bar,height*0.5f);
    append_oriented_box(mesh.details,base+side*(half_width-bar)+Vec3{0,height*0.5f,0},normal,depth,bar,height*0.5f);
    append_oriented_box(mesh.details,base+Vec3{0,height-bar,0},normal,depth,half_width,bar);
    append_oriented_box(mesh.details,base+Vec3{0,bar,0},normal,depth,half_width,bar);
}

void append_generated_entrance(CorridorMesh& mesh,const CorridorDoor& door){
    if(!door.generated) return;
    const Vec3 n=flat_normal(door.normal);
    const Vec3 side=perp_xz(n);
    const Vec3 door_base{door.position.x,0.0f,door.position.z};
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
        const Vec3 wall_base{door.wall_position.x,0.0f,door.wall_position.z};
        append_profile_transition(mesh,wall_base,door_base,n,profile,profile);
    }
    append_door_frame(mesh,door.wall_position,n,geometry::kDoorFrameHalfWidth,geometry::kDoorFrameHeight);
    append_door_frame(mesh,door.position,n,geometry::kDoorFrameHalfWidth,geometry::kDoorFrameHeight);
}

// Habitat doors sit higher than the corridor walkway on the current habitat
// models. Add shallow internal steps only when there is enough height to
// justify them. The stair run lives entirely inside the pressure docking sleeve.
void append_entrance_stairs(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    if(endpoint.kind!=CorridorEndpointKind::Habitat && endpoint.kind!=CorridorEndpointKind::Resource) return;
    const CorridorDoor& door=endpoint.door;
    const Vec3 n=flat_normal(door.normal);
    // door.position.y is the structure module's authoritative interior deck.
    // wall_position.y remains the generic pressure-hull slice used for carving.
    const float threshold_y=std::max(WALKWAY_Y,door.position.y);
    const float corridor_walkway_y=endpoint.position.y+WALKWAY_Y;
    const float rise=threshold_y-corridor_walkway_y;
    if(rise<0.16f) return;

    const float available=std::max(0.70f,distance_xz(door.position,endpoint.position)-0.12f);
    const float run=std::min(1.22f,available);
    const int steps=std::clamp((int)std::ceil(rise/0.17f),2,6);
    const float tread=run/(float)steps;
    const float step_rise=rise/(float)steps;
    for(int i=0;i<steps;++i){
        const float along=tread*((float)i+0.5f);
        const float top=threshold_y-step_rise*(float)i;
        const Vec3 door_base{door.position.x,0.0f,door.position.z};
        const Vec3 center=door_base+n*along+Vec3{0,top-0.045f,0};
        append_oriented_box(mesh.details,center,n,tread*0.52f,WALKWAY_HALF_WIDTH*0.90f,0.045f);
        edge(mesh.edges,center-perp_xz(n)*(WALKWAY_HALF_WIDTH*0.90f)+n*(tread*0.48f)+Vec3{0,0.05f,0},
                        center+perp_xz(n)*(WALKWAY_HALF_WIDTH*0.90f)+n*(tread*0.48f)+Vec3{0,0.05f,0});
    }
}

void append_docking_collar(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    if(endpoint.kind!=CorridorEndpointKind::Habitat && endpoint.kind!=CorridorEndpointKind::Resource) return;
    const CorridorDoor& door=endpoint.door;
    const Vec3 n=flat_normal(door.normal);
    append_generated_entrance(mesh,door);

    // Start exactly on the actual door plane. For generated entrances this is
    // the outer vestibule door; for built-in entrances it is the structure's
    // declared hatch plane. The adapter then morphs once, without overlap, into
    // the exact D-profile used by the corridor shell at endpoint.position.
    const Vec3 door_base{door.position.x,0.0f,door.position.z};
    const auto door_profile=doorway_cross_section(DOCK_SLEEVE_HALF_WIDTH,DOCK_SLEEVE_HEIGHT);
    const auto corridor_profile=corridor_cross_section();
    append_profile_transition(mesh,door_base,endpoint.position,n,door_profile,corridor_profile);
    append_door_frame(mesh,door.position,n,geometry::kDoorFrameHalfWidth,geometry::kDoorFrameHeight);
    append_pressure_seal(mesh,door.position,n,DOCK_SLEEVE_HALF_WIDTH*0.94f,DOCK_SLEEVE_HEIGHT*0.94f);
    // The hull contact gets a second flange. Generated entrances use the exact
    // sampled wall profile for the pressure skin; this inner flange visually
    // and mechanically consolidates that surface-to-sleeve seam.
    append_pressure_seal(mesh,door.wall_position,n,geometry::kDoorClearWidth*0.62f,geometry::kDoorClearHeight*1.02f);
    const Vec3 side=perp_xz(n);
    const Vec3 threshold=door_base+Vec3{0,0.105f,0};
    append_oriented_box(mesh.details,threshold,n,0.18f,geometry::kDoorClearWidth*0.52f,0.055f);
    edge(mesh.edges,threshold-side*(geometry::kDoorClearWidth*0.52f)+Vec3{0,0.06f,0},
                    threshold+side*(geometry::kDoorClearWidth*0.52f)+Vec3{0,0.06f,0});
    append_entrance_stairs(mesh,endpoint);
    append_ring(mesh,endpoint.position,n,1.0f);
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

float host_half_width_at_height(float y){
    if(y<=CORRIDOR_SIDE_Y) return CORRIDOR_HALF_WIDTH;
    const float dy=y-CORRIDOR_SIDE_Y;
    if(dy>=CORRIDOR_ROOF_RADIUS) return 0.0f;
    const float normalized=dy/CORRIDOR_ROOF_RADIUS;
    return CORRIDOR_HALF_WIDTH*std::sqrt(std::max(0.0f,1.0f-normalized*normalized));
}

// A corridor branch is joined to a host by cutting a real aperture out of the
// host pressure shell. Earlier versions left the host shell intact and drew the
// branch adapter on top of it, which created coincident/overlapping triangles
// at T/Y junctions. The aperture follows the host D-profile and removes only
// the side/roof patch occupied by the branch; the host deck remains continuous.
bool point_in_host_junction_aperture(Vec3 p,const CorridorEndpoint& branch){
    Vec3 host=flat_normal(branch.host_tangent);
    Vec3 leave=flat_normal(branch.tangent);
    if(length(host)<0.001f || length(leave)<0.001f) return false;
    const Vec3 host_side=perp_xz(host);
    const float side_sign=dot(leave,host_side)>=0.0f?1.0f:-1.0f;
    const Vec3 rel=flatten(p-branch.host_position);
    const float axial=std::abs(dot(rel,host));
    const float local_y=p.y-branch.host_position.y;
    const float top=CORRIDOR_SIDE_Y+CORRIDOR_ROOF_RADIUS;
    if(local_y<CORRIDOR_FLOOR_Y-0.08f || local_y>top+0.08f) return false;
    if(axial>CORRIDOR_HALF_WIDTH*1.14f) return false;
    const float half=host_half_width_at_height(local_y);
    const float lateral=dot(rel,host_side)*side_sign;
    // Keep a narrow boundary strip so the adapter and host share a clean seam
    // instead of leaving a visible crack after triangle removal.
    return lateral>=half-0.30f;
}

bool triangle_hits_host_junction(Vec3 a,Vec3 b,Vec3 c,const CorridorEndpoint& branch){
    const Vec3 m=(a+b+c)*(1.0f/3.0f);
    return point_in_host_junction_aperture(a,branch) ||
           point_in_host_junction_aperture(b,branch) ||
           point_in_host_junction_aperture(c,branch) ||
           point_in_host_junction_aperture(m,branch);
}

void carve_host_junction_apertures(CorridorMesh& mesh,std::size_t self_index,
                                    const std::vector<CorridorConnection>& existing){
    if(self_index==NONE_INDEX) return;
    std::vector<const CorridorEndpoint*> cuts;
    for(std::size_t ci=0;ci<existing.size();++ci){
        if(ci==self_index) continue;
        const auto& c=existing[ci];
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind==CorridorEndpointKind::Corridor && ep->index==self_index &&
               ep->surface_matched_junction) cuts.push_back(ep);
        }
    }
    if(cuts.empty()) return;

    std::vector<Vec3> shell;
    shell.reserve(mesh.shell.size());
    for(std::size_t i=0;i+2<mesh.shell.size();i+=3){
        bool remove=false;
        for(const CorridorEndpoint* cut:cuts){
            if(triangle_hits_host_junction(mesh.shell[i],mesh.shell[i+1],mesh.shell[i+2],*cut)){
                remove=true; break;
            }
        }
        if(!remove){ shell.push_back(mesh.shell[i]); shell.push_back(mesh.shell[i+1]); shell.push_back(mesh.shell[i+2]); }
    }
    mesh.shell=std::move(shell);

    // Ribs/outline lines crossing the new opening are removed as well. The
    // branch adapter contributes the replacement seam, avoiding doubled rings.
    std::vector<Vec3> edges;
    edges.reserve(mesh.edges.size());
    for(std::size_t i=0;i+1<mesh.edges.size();i+=2){
        const Vec3 mid=(mesh.edges[i]+mesh.edges[i+1])*0.5f;
        bool remove=false;
        for(const CorridorEndpoint* cut:cuts){
            if(point_in_host_junction_aperture(mesh.edges[i],*cut) ||
               point_in_host_junction_aperture(mesh.edges[i+1],*cut) ||
               point_in_host_junction_aperture(mid,*cut)){ remove=true; break; }
        }
        if(!remove){ edges.push_back(mesh.edges[i]); edges.push_back(mesh.edges[i+1]); }
    }
    mesh.edges=std::move(edges);
}

// Join a branch corridor to the actual swept surface of its host corridor.
// The outer ring is a normal corridor D-profile. Each vertex of the inner ring
// is projected onto the host's D-profile at the same height, which makes the
// terminal seam follow the host's vertical walls and curved roof exactly rather
// than ending at the host centerline or at an approximate rectangular sleeve.
void append_surface_matched_junction(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    Vec3 branch=flat_normal(endpoint.tangent);
    Vec3 host=flat_normal(endpoint.host_tangent);
    if(length(branch)<0.001f || length(host)<0.001f) return;
    const Vec3 host_side=perp_xz(host);
    const float side_sign=dot(branch,host_side)>=0.0f?1.0f:-1.0f;
    const Vec3 branch_side=perp_xz(branch);
    const auto section=corridor_cross_section();

    std::vector<Vec3> outer;
    std::vector<Vec3> contact;
    outer.reserve(section.size());
    contact.reserve(section.size());
    for(const Vec2& q:section){
        const Vec3 ov=ring_vertex(endpoint.position,branch_side,q);
        const float axial=dot(flatten(ov-endpoint.host_position),host);
        const float half=host_half_width_at_height(q.y);
        const Vec3 cv=endpoint.host_position+host*axial+host_side*(side_sign*half)+Vec3{0,q.y,0};
        outer.push_back(ov);
        contact.push_back(cv);
    }
    for(std::size_t i=0;i<section.size();++i){
        const std::size_t j=(i+1)%section.size();
        tri(mesh.shell,outer[i],contact[i],contact[j]);
        tri(mesh.shell,outer[i],contact[j],outer[j]);
        edge(mesh.edges,contact[i],contact[j]);
    }
    append_ring(mesh,endpoint.position,branch,1.0f);

    // Bridge the branch walkway to the host walkway edge inside the shared
    // pressure volume. This stays below the pressure shell and removes the
    // small floor gap between the host's 0.72 m walkway and 1.08 m hull side.
    const Vec3 host_walk_edge=endpoint.host_position+host_side*(side_sign*WALKWAY_HALF_WIDTH)+Vec3{0,WALKWAY_Y,0};
    const Vec3 shell_edge=endpoint.host_position+host_side*(side_sign*CORRIDOR_HALF_WIDTH)+Vec3{0,WALKWAY_Y,0};
    const Vec3 hs=host*(WALKWAY_HALF_WIDTH*0.92f);
    tri(mesh.details,shell_edge-hs,host_walk_edge-hs,host_walk_edge+hs);
    tri(mesh.details,shell_edge-hs,host_walk_edge+hs,shell_edge+hs);
    edge(mesh.edges,shell_edge-hs,host_walk_edge-hs);
    edge(mesh.edges,shell_edge+hs,host_walk_edge+hs);
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

bool point_in_convex_xz(Vec3 p,const std::vector<Vec3>& poly){
    if(poly.size()<3) return false;
    float sign=0.0f;
    for(std::size_t i=0;i<poly.size();++i){
        const Vec3 a=poly[i],b=poly[(i+1)%poly.size()];
        const float c=cross_xz(a,b,p);
        if(std::abs(c)<0.001f) continue;
        if(sign==0.0f) sign=c;
        else if((sign>0.0f)!=(c>0.0f)) return false;
    }
    return true;
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

bool point_in_auto_crossing_cut(Vec3 p,const MergedCrossingJunction& junction){
    const float y=p.y-junction.position.y;
    if(y<CORRIDOR_FLOOR_Y-0.12f || y>CORRIDOR_SIDE_Y+CORRIDOR_ROOF_RADIUS+0.12f) return false;
    return point_in_convex_xz(p,junction.cut_footprint);
}

void carve_automatic_crossings(CorridorMesh& mesh,const std::vector<MergedCrossingJunction>& junctions){
    if(junctions.empty()) return;
    auto cut_triangle=[&](Vec3 a,Vec3 b,Vec3 c){
        const Vec3 m=(a+b+c)*(1.0f/3.0f);
        for(const auto& x:junctions){
            if(point_in_auto_crossing_cut(a,x)||point_in_auto_crossing_cut(b,x)||point_in_auto_crossing_cut(c,x)||point_in_auto_crossing_cut(m,x)) return true;
        }
        return false;
    };
    std::vector<Vec3> shell; shell.reserve(mesh.shell.size());
    for(std::size_t i=0;i+2<mesh.shell.size();i+=3){
        if(!cut_triangle(mesh.shell[i],mesh.shell[i+1],mesh.shell[i+2])){
            shell.push_back(mesh.shell[i]); shell.push_back(mesh.shell[i+1]); shell.push_back(mesh.shell[i+2]);
        }
    }
    mesh.shell=std::move(shell);
    std::vector<Vec3> details; details.reserve(mesh.details.size());
    for(std::size_t i=0;i+2<mesh.details.size();i+=3){
        if(!cut_triangle(mesh.details[i],mesh.details[i+1],mesh.details[i+2])){
            details.push_back(mesh.details[i]); details.push_back(mesh.details[i+1]); details.push_back(mesh.details[i+2]);
        }
    }
    mesh.details=std::move(details);
    std::vector<Vec3> edges; edges.reserve(mesh.edges.size());
    for(std::size_t i=0;i+1<mesh.edges.size();i+=2){
        const Vec3 m=(mesh.edges[i]+mesh.edges[i+1])*0.5f;
        bool cut=false;
        for(const auto& x:junctions){
            if(point_in_auto_crossing_cut(mesh.edges[i],x)||point_in_auto_crossing_cut(mesh.edges[i+1],x)||point_in_auto_crossing_cut(m,x)){ cut=true; break; }
        }
        if(!cut){ edges.push_back(mesh.edges[i]); edges.push_back(mesh.edges[i+1]); }
    }
    mesh.edges=std::move(edges);
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

        Vec3 outward{};
        if(junction_edge_opening(poly[i],poly[j],junction,outward)){
            append_d_profile_opening_wall(mesh,poly[i],poly[j],outward,base_y);
        }else{
            append_wall_quad(mesh,f0,f1,t1,t0);
            edge(mesh.edges,f0,t0);
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
    append_docking_collar(mesh,endpoint);
    append_corridor_junction(mesh,endpoint);
    return mesh;
}

CorridorMesh build_mesh(const CorridorEndpoint& a,const CorridorEndpoint& b,const std::vector<Vec3>& path,
                        const std::vector<CorridorConnection>& existing,std::size_t self_index){
    CorridorMesh mesh;
    corridor_module().build_outer(mesh,path);
    corridor_module().build_inner(mesh,path);
    append_docking_collar(mesh,a);
    append_docking_collar(mesh,b);
    append_corridor_junction(mesh,a);
    append_corridor_junction(mesh,b);
    append_sealed_open_end(mesh,a);
    append_sealed_open_end(mesh,b);
    // Only committed parent/branch apertures depend on a stable self index.
    carve_host_junction_apertures(mesh,self_index,existing);
    // Transversal path crossings are true shared junctions. Carve both pressure
    // tubes locally and let exactly one mesh own the replacement chamber so the
    // intersection contains no doubled shell, floor or rib geometry.
    const auto automatic_crossings=collect_automatic_crossings(path,existing,self_index);
    const auto merged_junctions=consolidate_crossings(automatic_crossings,self_index);
    carve_automatic_crossings(mesh,merged_junctions);
    append_crossing_hubs(mesh,merged_junctions,self_index);
    return mesh;
}

void rebuild_all_meshes(std::vector<CorridorConnection>& connections){
    for(std::size_t i=0;i<connections.size();++i){
        auto& c=connections[i];
        c.mesh=build_mesh(c.endpoint_a,c.endpoint_b,c.path,connections,i);
    }
}

} // namespace game::corridor_detail
