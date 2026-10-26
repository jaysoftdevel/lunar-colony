#include "game/world/corridor_network.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace game {
using namespace eng;

namespace {
constexpr float PI=3.14159265358979323846f;
constexpr float CORRIDOR_HALF_WIDTH=1.08f;
constexpr float CORRIDOR_FLOOR_Y=0.12f;
constexpr float CORRIDOR_SIDE_Y=0.62f;
constexpr float CORRIDOR_ROOF_RADIUS=1.08f;
constexpr float WALKWAY_Y=0.23f;
constexpr float WALKWAY_HALF_WIDTH=0.72f;
constexpr float DOCK_RUN=0.92f;
constexpr float GENERATED_ENTRANCE_DEPTH=1.65f;
constexpr float GENERATED_ENTRANCE_HALF_WIDTH=1.18f;
constexpr float GENERATED_ENTRANCE_HEIGHT=2.65f;
constexpr float GENERATED_HULL_OVERLAP=0.34f;
constexpr float ENTRANCE_MIN_SPACING=2.75f;
constexpr float ROUTE_CLEARANCE=0.42f;
constexpr float SNAP_EPS=0.05f;
constexpr std::size_t NONE_INDEX=std::numeric_limits<std::size_t>::max();

struct AttachmentExtents {
    float half_x{};
    float half_z{};
    bool rounded_xz{false};
};

Vec3 rotate_y(Vec3 v,float degrees){
    const float a=radians(degrees);
    const float c=std::cos(a),s=std::sin(a);
    return {v.x*c+v.z*s,v.y,-v.x*s+v.z*c};
}

Vec3 flatten(Vec3 v){ v.y=0.0f; return v; }
Vec3 flat_normal(Vec3 v){ return normalize(flatten(v)); }
Vec3 perp_xz(Vec3 v){ v=flat_normal(v); return {-v.z,0.0f,v.x}; }

float distance_xz(Vec3 a,Vec3 b){
    const float dx=a.x-b.x,dz=a.z-b.z;
    return std::sqrt(dx*dx+dz*dz);
}

void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c){ dst.push_back(a); dst.push_back(b); dst.push_back(c); }
void edge(std::vector<Vec3>& dst,Vec3 a,Vec3 b){ dst.push_back(a); dst.push_back(b); }

std::vector<Vec2> corridor_cross_section(float scale=1.0f){
    std::vector<Vec2> out;
    out.reserve(12);
    out.push_back({-CORRIDOR_HALF_WIDTH*scale,CORRIDOR_FLOOR_Y});
    out.push_back({ CORRIDOR_HALF_WIDTH*scale,CORRIDOR_FLOOR_Y});
    // Flat floor + short vertical walls + a rounded roof. This keeps the
    // pressure tube curved without producing a fully circular tunnel.
    constexpr int ARC=10;
    for(int i=0;i<=ARC;++i){
        const float a=(float)i/(float)ARC*PI;
        out.push_back({std::cos(a)*CORRIDOR_ROOF_RADIUS*scale,
                       CORRIDOR_SIDE_Y+std::sin(a)*CORRIDOR_ROOF_RADIUS*scale});
    }
    return out;
}

Vec3 ring_vertex(Vec3 center,Vec3 side,const Vec2& q){
    return {center.x+side.x*q.x,q.y,center.z+side.z*q.x};
}

Vec3 path_tangent(const std::vector<Vec3>& path,std::size_t i){
    if(path.size()<2) return {1,0,0};
    if(i==0) return flat_normal(path[1]-path[0]);
    if(i+1>=path.size()) return flat_normal(path.back()-path[path.size()-2]);
    const Vec3 t=flat_normal(path[i+1]-path[i-1]);
    return length(t)>0.001f?t:flat_normal(path[i+1]-path[i]);
}

void append_swept_shell(CorridorMesh& mesh,const std::vector<Vec3>& path){
    if(path.size()<2) return;
    const auto section=corridor_cross_section();
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 t0=path_tangent(path,i);
        const Vec3 t1=path_tangent(path,i+1);
        const Vec3 s0=perp_xz(t0);
        const Vec3 s1=perp_xz(t1);
        const Vec3 c0{path[i].x,0.0f,path[i].z};
        const Vec3 c1{path[i+1].x,0.0f,path[i+1].z};
        for(std::size_t j=0;j<section.size();++j){
            const std::size_t k=(j+1)%section.size();
            const Vec3 a=ring_vertex(c0,s0,section[j]);
            const Vec3 b=ring_vertex(c1,s1,section[j]);
            const Vec3 c=ring_vertex(c1,s1,section[k]);
            const Vec3 d=ring_vertex(c0,s0,section[k]);
            tri(mesh.shell,a,b,c); tri(mesh.shell,a,c,d);
        }
    }
}

void append_walkway(CorridorMesh& mesh,const std::vector<Vec3>& path){
    if(path.size()<2) return;
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 t0=path_tangent(path,i),t1=path_tangent(path,i+1);
        const Vec3 s0=perp_xz(t0)*WALKWAY_HALF_WIDTH;
        const Vec3 s1=perp_xz(t1)*WALKWAY_HALF_WIDTH;
        const Vec3 p0{path[i].x,WALKWAY_Y,path[i].z};
        const Vec3 p1{path[i+1].x,WALKWAY_Y,path[i+1].z};
        tri(mesh.details,p0-s0,p1-s1,p1+s1);
        tri(mesh.details,p0-s0,p1+s1,p0+s0);
        edge(mesh.edges,p0-s0,p1-s1);
        edge(mesh.edges,p0+s0,p1+s1);
    }
}

void append_ring(CorridorMesh& mesh,Vec3 center,Vec3 tangent,float scale=1.055f){
    tangent=flat_normal(tangent);
    if(length(tangent)<0.001f) return;
    const Vec3 side=perp_xz(tangent);
    const auto section=corridor_cross_section(scale);
    for(std::size_t j=0;j<section.size();++j){
        const std::size_t k=(j+1)%section.size();
        edge(mesh.edges,ring_vertex(center,side,section[j]),ring_vertex(center,side,section[k]));
    }
}

void append_ribs(CorridorMesh& mesh,const std::vector<Vec3>& path){
    if(path.size()<2) return;
    float next=1.7f;
    float travelled=0.0f;
    for(std::size_t i=0;i+1<path.size();++i){
        const float seg=distance_xz(path[i],path[i+1]);
        if(seg<0.001f) continue;
        while(travelled+seg>=next){
            const float t=(next-travelled)/seg;
            const Vec3 p=path[i]+(path[i+1]-path[i])*t;
            append_ring(mesh,{p.x,0,p.z},path[i+1]-path[i]);
            next+=1.7f;
        }
        travelled+=seg;
    }
}

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
    tri(mesh.details,a0,b1,b0); tri(mesh.details,a0,a1,b1); // vestibule floor
    for(const auto& e:std::array<std::pair<Vec3,Vec3>,8>{{{a0,b0},{a1,b1},{a2,b2},{a3,b3},{a0,a3},{a1,a2},{b0,b3},{b1,b2}}}) edge(mesh.edges,e.first,e.second);
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
    const Vec3 inner=door.wall_position-n*GENERATED_HULL_OVERLAP;
    const Vec3 outer=door.position+n*0.10f;
    append_open_rect_tunnel(mesh,inner,outer,n,GENERATED_ENTRANCE_HALF_WIDTH,GENERATED_ENTRANCE_HEIGHT);
    append_door_frame(mesh,door.wall_position,n,1.22f,2.76f);
    append_door_frame(mesh,door.position,n,1.22f,2.76f);
}

void append_docking_collar(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    if(endpoint.kind!=CorridorEndpointKind::Habitat) return;
    const CorridorDoor& door=endpoint.door;
    const Vec3 n=flat_normal(door.normal);
    append_generated_entrance(mesh,door);
    append_door_frame(mesh,door.position,n);

    // Short D-profile pressure adapter. It starts slightly inside the entrance
    // plane and ends inside the corridor shell, wrapping all around the door.
    std::vector<Vec3> collar_path={door.position-n*0.16f,endpoint.position+n*0.08f};
    append_swept_shell(mesh,collar_path);
    append_walkway(mesh,collar_path);
    append_ring(mesh,door.position,n,1.10f);
    append_ring(mesh,endpoint.position,n,1.10f);
}

void append_open_end_door(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    if(endpoint.kind!=CorridorEndpointKind::OpenDoor) return;
    const Vec3 n=flat_normal(endpoint.tangent);
    const Vec3 side=perp_xz(n);
    const auto section=corridor_cross_section(1.01f);
    // Solid pressure bulkhead at the end. It lives in details so it stays
    // visible when the corridor hull is transparent in FULL view mode.
    Vec3 center{endpoint.position.x,0,endpoint.position.z};
    Vec3 centroid{center.x,1.0f,center.z};
    for(std::size_t j=0;j<section.size();++j){
        const std::size_t k=(j+1)%section.size();
        tri(mesh.details,centroid,ring_vertex(center,side,section[j]),ring_vertex(center,side,section[k]));
    }
    append_ring(mesh,center,n,1.08f);
    append_door_frame(mesh,center,n,0.68f,1.72f);
    // inset door panel
    append_oriented_box(mesh.details,center-n*0.035f+Vec3{0,0.92f,0},n,0.055f,0.55f,0.73f);
}

void append_corridor_junction(CorridorMesh& mesh,const CorridorEndpoint& endpoint){
    if(endpoint.kind!=CorridorEndpointKind::Corridor) return;
    // Wider short D-profile sleeve around the branch point. Since the existing
    // corridor shell already passes through this volume, the overlap reads as
    // one sealed intersection rather than two tubes merely touching.
    Vec3 t=flat_normal(endpoint.tangent);
    if(length(t)<0.001f) t={1,0,0};
    std::vector<Vec3> hub={endpoint.position-t*0.48f,endpoint.position+t*0.48f};
    const auto section=corridor_cross_section(1.18f);
    for(std::size_t i=0;i+1<hub.size();++i){
        const Vec3 side=perp_xz(t);
        for(std::size_t j=0;j<section.size();++j){
            const std::size_t k=(j+1)%section.size();
            const Vec3 a=ring_vertex(hub[i],side,section[j]);
            const Vec3 b=ring_vertex(hub[i+1],side,section[j]);
            const Vec3 c=ring_vertex(hub[i+1],side,section[k]);
            const Vec3 d=ring_vertex(hub[i],side,section[k]);
            tri(mesh.details,a,b,c); tri(mesh.details,a,c,d);
        }
    }
    append_ring(mesh,endpoint.position,t,1.20f);
}

AttachmentExtents attachment_extents(HabitatKind kind){
    switch(kind){
        case HabitatKind::AirLock: return {3.62f,2.16f,false};
        case HabitatKind::OxygenGeneratorDome: return {4.38f,4.38f,true};
        case HabitatKind::CantineDome: return {4.55f,4.55f,true};
        case HabitatKind::BioDome: return {5.26f,3.31f,false};
        case HabitatKind::CrewQuarters: return {5.42f,2.36f,false};
        default: return {2.0f,2.0f,false};
    }
}

CorridorDoor transformed_default_door(const PlacedHabitat& habitat,const HabitatMesh& mesh){
    HabitatEntrance local{};
    if(!mesh.entrances.empty()) local=mesh.entrances.front();
    CorridorDoor d;
    d.position=habitat.position+rotate_y(local.local_position,habitat.rotation_degrees);
    d.wall_position=d.position;
    d.normal=flat_normal(rotate_y(local.local_normal,habitat.rotation_degrees));
    d.generated=false;
    return d;
}

bool endpoint_uses_habitat(const CorridorEndpoint& e,std::size_t index){
    return e.kind==CorridorEndpointKind::Habitat && e.index==index;
}

bool door_occupied(std::size_t habitat_index,const CorridorDoor& candidate,const std::vector<CorridorConnection>& existing){
    for(const auto& c:existing){
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(!endpoint_uses_habitat(*ep,habitat_index)) continue;
            if(distance_xz(candidate.position,ep->door.position)<1.20f) return true;
        }
    }
    return false;
}

bool entrance_slot_clear(std::size_t habitat_index,const CorridorDoor& candidate,
                         const PlacedHabitat& habitat,const HabitatMesh& mesh,
                         const std::vector<CorridorConnection>& existing){
    const CorridorDoor main=transformed_default_door(habitat,mesh);
    if(distance_xz(candidate.wall_position,main.wall_position)<ENTRANCE_MIN_SPACING) return false;
    for(const auto& c:existing){
        for(const CorridorEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(!endpoint_uses_habitat(*ep,habitat_index)) continue;
            if(distance_xz(candidate.wall_position,ep->door.wall_position)<ENTRANCE_MIN_SPACING) return false;
            if(distance_xz(candidate.position,ep->door.position)<ENTRANCE_MIN_SPACING) return false;
        }
    }
    return true;
}

Vec3 local_wall_point(const AttachmentExtents& e,Vec3 local_normal,float tangent){
    if(std::abs(local_normal.x)>0.5f){
        const float z=std::clamp(tangent,-e.half_z*0.55f,e.half_z*0.55f);
        float x=e.half_x;
        if(e.rounded_xz){
            const float q=std::clamp(z/e.half_z,-0.80f,0.80f);
            x=e.half_x*std::sqrt(std::max(0.18f,1.0f-q*q));
        }
        return {local_normal.x>0.0f?x:-x,1.18f,z};
    }
    const float x=std::clamp(tangent,-e.half_x*0.55f,e.half_x*0.55f);
    float z=e.half_z;
    if(e.rounded_xz){
        const float q=std::clamp(x/e.half_x,-0.80f,0.80f);
        z=e.half_z*std::sqrt(std::max(0.18f,1.0f-q*q));
    }
    return {x,1.18f,local_normal.z>0.0f?z:-z};
}

std::optional<CorridorDoor> generated_face_door(std::size_t habitat_index,
                                                const PlacedHabitat& habitat,
                                                const HabitatMesh& mesh,
                                                Vec3 target,
                                                const std::vector<CorridorConnection>& existing){
    const AttachmentExtents ext=attachment_extents(habitat.kind);
    const Vec3 target_dir=flat_normal(target-habitat.position);
    const std::array<Vec3,4> normals={Vec3{1,0,0},Vec3{-1,0,0},Vec3{0,0,1},Vec3{0,0,-1}};
    const std::array<float,7> slots={0.0f,0.30f,-0.30f,0.50f,-0.50f,0.68f,-0.68f};
    bool found=false;
    float best=std::numeric_limits<float>::max();
    CorridorDoor result{};
    for(const Vec3 ln:normals){
        const Vec3 wn=flat_normal(rotate_y(ln,habitat.rotation_degrees));
        const float facing_penalty=(1.0f-dot(wn,target_dir))*4.0f;
        const float tangent_extent=std::abs(ln.x)>0.5f?ext.half_z:ext.half_x;
        for(float factor:slots){
            CorridorDoor d;
            d.wall_position=habitat.position+rotate_y(local_wall_point(ext,ln,factor*tangent_extent),habitat.rotation_degrees);
            d.normal=wn;
            d.position=d.wall_position+d.normal*GENERATED_ENTRANCE_DEPTH;
            d.generated=true;
            if(!entrance_slot_clear(habitat_index,d,habitat,mesh,existing)) continue;
            const float score=distance_xz(d.position,target)+facing_penalty+std::abs(factor)*0.4f;
            if(score<best){ best=score; result=d; found=true; }
        }
    }
    if(!found) return std::nullopt;
    return result;
}

float point_segment_distance_xz(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<0.0001f) return distance_xz(p,a);
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return distance_xz(p,{a.x+dx*t,0,a.z+dz*t});
}

Vec3 closest_point_segment_xz(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<0.0001f) return {a.x,0,a.z};
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return {a.x+dx*t,0,a.z+dz*t};
}

bool segment_clear(Vec3 a,Vec3 b,const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                   std::size_t ignore_a=NONE_INDEX,std::size_t ignore_b=NONE_INDEX){
    for(std::size_t i=0;i<habitats.size();++i){
        if(i==ignore_a || i==ignore_b) continue;
        const auto& hm=library.mesh(habitats[i].kind);
        const float radius=0.5f*std::sqrt(hm.footprint_x*hm.footprint_x+hm.footprint_z*hm.footprint_z)+CORRIDOR_HALF_WIDTH+ROUTE_CLEARANCE;
        if(point_segment_distance_xz(habitats[i].position,a,b)<radius) return false;
    }
    return true;
}

bool path_clear(const std::vector<Vec3>& path,const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                std::size_t ignore_a=NONE_INDEX,std::size_t ignore_b=NONE_INDEX){
    for(std::size_t i=0;i+1<path.size();++i){
        if(!segment_clear(path[i],path[i+1],habitats,library,ignore_a,ignore_b)) return false;
    }
    return true;
}

float path_length(const std::vector<Vec3>& path){
    float total=0.0f;
    for(std::size_t i=0;i+1<path.size();++i) total+=distance_xz(path[i],path[i+1]);
    return total;
}

std::vector<Vec3> chaikin(std::vector<Vec3> pts,int iterations){
    if(pts.size()<3) return pts;
    for(int it=0;it<iterations;++it){
        std::vector<Vec3> out;
        out.reserve(pts.size()*2);
        out.push_back(pts.front());
        for(std::size_t i=0;i+1<pts.size();++i){
            const Vec3 a=pts[i],b=pts[i+1];
            out.push_back(a*0.75f+b*0.25f);
            out.push_back(a*0.25f+b*0.75f);
        }
        out.push_back(pts.back());
        pts=std::move(out);
    }
    std::vector<Vec3> compact;
    compact.reserve(pts.size());
    for(Vec3 p:pts){
        p.y=0;
        if(compact.empty() || distance_xz(compact.back(),p)>0.08f) compact.push_back(p);
    }
    return compact;
}

std::vector<Vec3> smooth_route(const CorridorEndpoint& a,const CorridorEndpoint& b,
                               const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library){
    const float d=distance_xz(a.position,b.position);
    if(d<0.08f) return {};
    if(d<0.90f) return {a.position,b.position};
    const Vec3 direct=flat_normal(b.position-a.position);
    const Vec3 lateral=perp_xz(direct);
    const float lead=std::clamp(d*0.22f,1.0f,3.2f);
    const float mid_scale=std::max(2.8f,std::min(9.0f,d*0.28f));
    const std::array<float,9> offsets={0.0f,1.0f,-1.0f,1.8f,-1.8f,2.6f,-2.6f,3.4f,-3.4f};
    const std::size_t ignore_a=a.kind==CorridorEndpointKind::Habitat?a.index:NONE_INDEX;
    const std::size_t ignore_b=b.kind==CorridorEndpointKind::Habitat?b.index:NONE_INDEX;

    for(float f:offsets){
        const Vec3 mid=(a.position+b.position)*0.5f+lateral*(f*mid_scale);
        std::vector<Vec3> controls={a.position,a.position+a.tangent*lead,mid,b.position+b.tangent*lead,b.position};
        auto path=chaikin(std::move(controls),3);
        if(path_clear(path,habitats,library,ignore_a,ignore_b)) return path;
    }
    return {};
}

CorridorEndpoint resolve_open_or_corridor(const CorridorBuildAnchor& anchor,Vec3 other,
                                          const std::vector<CorridorConnection>& existing){
    CorridorEndpoint out;
    out.kind=anchor.kind;
    out.index=anchor.index;
    out.position=anchor.position;
    out.position.y=0;
    out.tangent=flat_normal(other-out.position);
    if(length(out.tangent)<0.001f) out.tangent={1,0,0};
    if(anchor.kind==CorridorEndpointKind::Corridor && anchor.index<existing.size()){
        // Keep tangent aimed toward the new route; the junction hardware itself
        // wraps the existing corridor at the exact selected midpoint.
        out.position.y=0;
    }
    return out;
}

std::optional<CorridorEndpoint> resolve_habitat(const CorridorBuildAnchor& anchor,Vec3 other,
                                                const std::vector<PlacedHabitat>& habitats,
                                                const HabitatModelLibrary& library,
                                                const std::vector<CorridorConnection>& existing){
    if(anchor.index>=habitats.size()) return std::nullopt;
    const auto& habitat=habitats[anchor.index];
    const auto& mesh=library.mesh(habitat.kind);
    CorridorDoor d=transformed_default_door(habitat,mesh);
    const Vec3 toward=flat_normal(other-d.position);
    const bool facing=length(toward)>0.001f && dot(d.normal,toward)>0.45f;
    const bool available=!door_occupied(anchor.index,d,existing);
    if(!facing || !available){
        auto generated=generated_face_door(anchor.index,habitat,mesh,other,existing);
        if(!generated) return std::nullopt;
        d=*generated;
    }
    CorridorEndpoint out;
    out.kind=CorridorEndpointKind::Habitat;
    out.index=anchor.index;
    out.door=d;
    out.tangent=d.normal;
    out.position=d.position+d.normal*DOCK_RUN;
    out.position.y=0;
    return out;
}

std::optional<CorridorEndpoint> resolve_anchor(const CorridorBuildAnchor& anchor,Vec3 other,
                                               const std::vector<PlacedHabitat>& habitats,
                                               const HabitatModelLibrary& library,
                                               const std::vector<CorridorConnection>& existing){
    if(anchor.kind==CorridorEndpointKind::Habitat) return resolve_habitat(anchor,other,habitats,library,existing);
    return resolve_open_or_corridor(anchor,other,existing);
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
    out={a.x+r_x*t,0,a.z+r_z*t};
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

std::optional<CorridorConnection> make_generic(const CorridorBuildAnchor& a_anchor,const CorridorBuildAnchor& b_anchor,
                                               CorridorConnectionKind kind,
                                               const std::vector<PlacedHabitat>& habitats,
                                               const HabitatModelLibrary& library,
                                               const std::vector<CorridorConnection>& existing){
    Vec3 a_hint=a_anchor.position,b_hint=b_anchor.position;
    if(a_anchor.kind==CorridorEndpointKind::Habitat && a_anchor.index<habitats.size()) a_hint=habitats[a_anchor.index].position;
    if(b_anchor.kind==CorridorEndpointKind::Habitat && b_anchor.index<habitats.size()) b_hint=habitats[b_anchor.index].position;
    auto a=resolve_anchor(a_anchor,b_hint,habitats,library,existing);
    auto b=resolve_anchor(b_anchor,a_hint,habitats,library,existing);
    if(!a || !b) return std::nullopt;

    // Re-resolve habitats once using the actual opposite endpoint so generated
    // door placement tracks corridor/corridor/open targets precisely.
    if(a_anchor.kind==CorridorEndpointKind::Habitat) a=resolve_anchor(a_anchor,b->position,habitats,library,existing);
    if(b_anchor.kind==CorridorEndpointKind::Habitat) b=resolve_anchor(b_anchor,a->position,habitats,library,existing);
    if(!a || !b) return std::nullopt;

    auto path=smooth_route(*a,*b,habitats,library);
    if(path.size()<2) return std::nullopt;

    CorridorConnection out;
    out.kind=kind;
    out.endpoint_a=*a;
    out.endpoint_b=*b;
    out.path=std::move(path);
    out.mesh=build_mesh(out.endpoint_a,out.endpoint_b,out.path,existing);
    return out;
}

Vec3 nearest_path_point(const std::vector<Vec3>& path,Vec3 p){
    Vec3 best=p;
    float best_d=std::numeric_limits<float>::max();
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 q=closest_point_segment_xz(p,path[i],path[i+1]);
        const float d=distance_xz(p,q);
        if(d<best_d){ best_d=d; best=q; }
    }
    return best;
}

std::pair<Vec3,Vec3> closest_pair(const std::vector<Vec3>& a,const std::vector<Vec3>& b){
    Vec3 best_a{},best_b{};
    float best=std::numeric_limits<float>::max();
    for(std::size_t i=0;i+1<a.size();++i){
        for(std::size_t j=0;j+1<b.size();++j){
            for(int s=0;s<=6;++s){
                const float t=(float)s/6.0f;
                const Vec3 p=a[i]+(a[i+1]-a[i])*t;
                const Vec3 q=closest_point_segment_xz(p,b[j],b[j+1]);
                const float d=distance_xz(p,q);
                if(d<best){ best=d;best_a=p;best_b=q; }
            }
        }
    }
    return {best_a,best_b};
}

} // namespace

void CorridorNetwork::connect_new_habitat(std::size_t new_index,
                                          const std::vector<PlacedHabitat>& habitats,
                                          const HabitatModelLibrary& library){
    if(auto c=preview_connection(new_index,habitats,library)) connections_.push_back(std::move(*c));
}

std::optional<CorridorConnection> CorridorNetwork::preview_connection(
    std::size_t new_index,const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const {
    if(new_index>=habitats.size()) return std::nullopt;

    const Vec3 newcomer=habitats[new_index].position;
    float best=std::numeric_limits<float>::max();
    CorridorBuildAnchor target{};
    bool found=false;

    // Existing habitats remain valid automatic targets.
    for(std::size_t i=0;i<habitats.size();++i){
        if(i==new_index) continue;
        const float d=distance_xz(newcomer,habitats[i].position);
        if(d<best){
            best=d; found=true;
            target={CorridorEndpointKind::Habitat,i,habitats[i].position};
        }
    }

    // Existing corridors are also first-class targets. A newly placed habitat
    // therefore joins the nearest part of the colony network, not necessarily
    // the nearest building.
    for(std::size_t i=0;i<connections_.size();++i){
        const Vec3 p=nearest_path_point(connections_[i].path,newcomer);
        const float d=distance_xz(newcomer,p);
        if(d<best){
            best=d; found=true;
            target={CorridorEndpointKind::Corridor,i,p};
        }
    }

    if(!found) return std::nullopt;
    CorridorBuildAnchor source{CorridorEndpointKind::Habitat,new_index,newcomer};
    return make_generic(source,target,CorridorConnectionKind::Automatic,habitats,library,connections_);
}

std::optional<CorridorConnection> CorridorNetwork::preview_built_corridor(
    const CorridorBuildAnchor& a,const CorridorBuildAnchor& b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const {
    if(a.kind==CorridorEndpointKind::Corridor && a.index>=connections_.size()) return std::nullopt;
    if(b.kind==CorridorEndpointKind::Corridor && b.index>=connections_.size()) return std::nullopt;
    if(a.kind==CorridorEndpointKind::Habitat && a.index>=habitats.size()) return std::nullopt;
    if(b.kind==CorridorEndpointKind::Habitat && b.index>=habitats.size()) return std::nullopt;
    if(a.kind==b.kind && a.index==b.index && a.kind!=CorridorEndpointKind::OpenDoor) return std::nullopt;
    return make_generic(a,b,CorridorConnectionKind::BuiltSection,habitats,library,connections_);
}

bool CorridorNetwork::build_corridor(const CorridorBuildAnchor& a,const CorridorBuildAnchor& b,
                                     const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library){
    auto c=preview_built_corridor(a,b,habitats,library);
    if(!c) return false;
    connections_.push_back(std::move(*c));
    return true;
}

std::optional<CorridorConnection> CorridorNetwork::preview_corridor_link(
    std::size_t corridor_a,std::size_t corridor_b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const {
    if(corridor_a>=connections_.size() || corridor_b>=connections_.size() || corridor_a==corridor_b) return std::nullopt;
    const auto pair=closest_pair(connections_[corridor_a].path,connections_[corridor_b].path);
    if(distance_xz(pair.first,pair.second)<CORRIDOR_HALF_WIDTH*1.25f) return std::nullopt;
    CorridorBuildAnchor a{CorridorEndpointKind::Corridor,corridor_a,pair.first};
    CorridorBuildAnchor b{CorridorEndpointKind::Corridor,corridor_b,pair.second};
    auto c=make_generic(a,b,CorridorConnectionKind::NetworkLink,habitats,library,connections_);
    return c;
}

bool CorridorNetwork::connect_corridors(std::size_t corridor_a,std::size_t corridor_b,
                                        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library){
    auto c=preview_corridor_link(corridor_a,corridor_b,habitats,library);
    if(!c) return false;
    connections_.push_back(std::move(*c));
    return true;
}

Vec3 CorridorNetwork::nearest_point_on_corridor(std::size_t corridor_index,Vec3 p) const {
    if(corridor_index>=connections_.size()) return p;
    return nearest_path_point(connections_[corridor_index].path,p);
}

CorridorBuildAnchor CorridorNetwork::corridor_anchor(std::size_t corridor_index,Vec3 near_point) const {
    return {CorridorEndpointKind::Corridor,corridor_index,nearest_point_on_corridor(corridor_index,near_point)};
}

std::size_t CorridorNetwork::habitat_connection_count(std::size_t habitat_index) const {
    std::size_t count=0;
    for(const auto& c:connections_){
        if(endpoint_uses_habitat(c.endpoint_a,habitat_index) || endpoint_uses_habitat(c.endpoint_b,habitat_index)) ++count;
    }
    return count;
}

float CorridorNetwork::connection_length(std::size_t corridor_index) const {
    if(corridor_index>=connections_.size()) return 0.0f;
    return path_length(connections_[corridor_index].path);
}

} // namespace game
