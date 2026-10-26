#include "game/world/corridor/detail.hpp"
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

void append_door_frame(CorridorMesh& mesh,Vec3 center,Vec3 normal,float half_width=1.17f,float height=2.68f){
    normal=flat_normal(normal);
    const Vec3 side=perp_xz(normal);
    const float bar=0.15f;
    const Vec3 base{center.x,0,center.z};
    append_oriented_box(mesh.details,base-side*(half_width-bar)+Vec3{0,height*0.5f,0},normal,0.12f,bar,height*0.5f);
    append_oriented_box(mesh.details,base+side*(half_width-bar)+Vec3{0,height*0.5f,0},normal,0.12f,bar,height*0.5f);
    append_oriented_box(mesh.details,base+Vec3{0,height-bar,0},normal,0.12f,half_width,bar);
    append_oriented_box(mesh.details,base+Vec3{0,bar,0},normal,0.12f,half_width,bar);
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
    append_door_frame(mesh,door.wall_position,n,1.22f,2.76f);
    append_door_frame(mesh,door.position,n,1.22f,2.76f);
}

// Habitat doors sit higher than the corridor walkway on the current habitat
// models. Add shallow internal steps only when there is enough height to
// justify them. The stair run lives entirely inside the pressure docking sleeve.
void append_entrance_stairs(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    if(endpoint.kind!=CorridorEndpointKind::Habitat) return;
    const CorridorDoor& door=endpoint.door;
    const Vec3 n=flat_normal(door.normal);
    const float threshold_y=std::clamp(door.wall_position.y-0.30f,WALKWAY_Y,1.05f);
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
    if(endpoint.kind!=CorridorEndpointKind::Habitat) return;
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
    append_door_frame(mesh,door.position,n,1.22f,2.76f);
    append_entrance_stairs(mesh,endpoint);
    append_ring(mesh,endpoint.position,n,1.0f);
}

void append_open_end_door(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    if(endpoint.kind!=CorridorEndpointKind::OpenDoor || endpoint.suppress_open_door) return;
    const Vec3 n=flat_normal(endpoint.tangent);
    const Vec3 side=perp_xz(n);
    const auto section=corridor_cross_section(1.01f);
    Vec3 center{endpoint.position.x,endpoint.position.y,endpoint.position.z};
    Vec3 centroid{center.x,1.0f,center.z};
    for(std::size_t j=0;j<section.size();++j){
        const std::size_t k=(j+1)%section.size();
        tri(mesh.details,centroid,ring_vertex(center,side,section[j]),ring_vertex(center,side,section[k]));
    }
    append_ring(mesh,center,n,1.08f);
    append_door_frame(mesh,center,n,0.68f,1.72f);
    append_oriented_box(mesh.details,center-n*0.035f+Vec3{0,0.92f,0},n,0.055f,0.55f,0.73f);
}

float host_half_width_at_height(float y){
    if(y<=CORRIDOR_SIDE_Y) return CORRIDOR_HALF_WIDTH;
    const float dy=y-CORRIDOR_SIDE_Y;
    if(dy>=CORRIDOR_ROOF_RADIUS) return 0.0f;
    return std::sqrt(std::max(0.0f,CORRIDOR_ROOF_RADIUS*CORRIDOR_ROOF_RADIUS-dy*dy));
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

bool segment_intersection_xz(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3& out){
    const float r_x=b.x-a.x,r_z=b.z-a.z;
    const float s_x=d.x-c.x,s_z=d.z-c.z;
    const float den=r_x*s_z-r_z*s_x;
    if(std::abs(den)<0.00001f) return false;
    const float qx=c.x-a.x,qz=c.z-a.z;
    const float t=(qx*s_z-qz*s_x)/den;
    const float u=(qx*r_z-qz*r_x)/den;
    if(t<=0.02f || t>=0.98f || u<=0.02f || u>=0.98f) return false;
    out={a.x+r_x*t,a.y+(b.y-a.y)*t,a.z+r_z*t};
    return true;
}

void append_crossing_hubs(CorridorMesh& mesh,const std::vector<Vec3>& path,const std::vector<CorridorConnection>& existing){
    for(std::size_t i=0;i+1<path.size();++i){
        for(const auto& c:existing){
            for(std::size_t j=0;j+1<c.path.size();++j){
                Vec3 hit{};
                if(segment_intersection_xz(path[i],path[i+1],c.path[j],c.path[j+1],hit)){
                    CorridorEndpoint ep;
                    ep.kind=CorridorEndpointKind::Corridor;
                    ep.position=hit;
                    ep.tangent=flat_normal(path[i+1]-path[i]);
                    ep.host_tangent=flat_normal(c.path[j+1]-c.path[j]);
                    append_corridor_junction(mesh,ep);
                }
            }
        }
    }
}

CorridorMesh build_mesh(const CorridorEndpoint& a,const CorridorEndpoint& b,const std::vector<Vec3>& path,
                        const std::vector<CorridorConnection>& existing){
    CorridorMesh mesh;
    append_swept_shell(mesh,path);
    append_walkway(mesh,path);
    append_ribs(mesh,path);
    append_docking_collar(mesh,a);
    append_docking_collar(mesh,b);
    append_corridor_junction(mesh,a);
    append_corridor_junction(mesh,b);
    append_open_end_door(mesh,a);
    append_open_end_door(mesh,b);
    append_crossing_hubs(mesh,path,existing);
    return mesh;
}

} // namespace game::corridor_detail
