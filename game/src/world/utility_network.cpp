#include "game/world/utility_network.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace game {
using namespace eng;
using namespace world_math;

namespace {
constexpr float PI=3.14159265358979323846f;

struct Builder {
    UtilityMesh mesh;
    static void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c){ dst.push_back(a); dst.push_back(b); dst.push_back(c); }
    void line(Vec3 a,Vec3 b){ mesh.edges.push_back(a); mesh.edges.push_back(b); }

    void box(std::vector<Vec3>& dst,Vec3 c,Vec3 s,bool outline=true){
        const Vec3 h=s*0.5f;
        Vec3 p[8]={
            {c.x-h.x,c.y-h.y,c.z-h.z},{c.x+h.x,c.y-h.y,c.z-h.z},
            {c.x+h.x,c.y+h.y,c.z-h.z},{c.x-h.x,c.y+h.y,c.z-h.z},
            {c.x-h.x,c.y-h.y,c.z+h.z},{c.x+h.x,c.y-h.y,c.z+h.z},
            {c.x+h.x,c.y+h.y,c.z+h.z},{c.x-h.x,c.y+h.y,c.z+h.z}
        };
        const int f[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
        for(const auto& q:f){ tri(dst,p[q[0]],p[q[1]],p[q[2]]); tri(dst,p[q[0]],p[q[2]],p[q[3]]); }
        if(outline){
            const int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
            for(const auto& q:e) line(p[q[0]],p[q[1]]);
        }
    }

    void oriented_box(std::vector<Vec3>& dst,Vec3 c,Vec3 axis_x,Vec3 axis_y,Vec3 axis_z,Vec3 size,bool outline=true){
        axis_x=normalize(axis_x); axis_y=normalize(axis_y); axis_z=normalize(axis_z);
        const Vec3 hx=axis_x*(size.x*0.5f), hy=axis_y*(size.y*0.5f), hz=axis_z*(size.z*0.5f);
        Vec3 p[8]={
            c-hx-hy-hz,c+hx-hy-hz,c+hx+hy-hz,c-hx+hy-hz,
            c-hx-hy+hz,c+hx-hy+hz,c+hx+hy+hz,c-hx+hy+hz
        };
        const int f[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
        for(const auto& q:f){ tri(dst,p[q[0]],p[q[1]],p[q[2]]); tri(dst,p[q[0]],p[q[2]],p[q[3]]); }
        if(outline){
            const int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
            for(const auto& q:e) line(p[q[0]],p[q[1]]);
        }
    }

    void cylinder_between(std::vector<Vec3>& dst,Vec3 a,Vec3 b,float radius,int segments=10,bool outline=true){
        const Vec3 axis=b-a;
        const float len=length(axis);
        if(len<0.0001f) return;
        const Vec3 n=axis*(1.0f/len);
        const Vec3 ref=std::abs(n.y)<0.92f?Vec3{0,1,0}:Vec3{1,0,0};
        const Vec3 u=normalize(cross(n,ref));
        const Vec3 v=normalize(cross(n,u));
        const int seg=std::max(6,segments);
        for(int i=0;i<seg;++i){
            const float a0=2.0f*PI*(float)i/(float)seg;
            const float a1=2.0f*PI*(float)(i+1)/(float)seg;
            const Vec3 r0=u*std::cos(a0)*radius+v*std::sin(a0)*radius;
            const Vec3 r1=u*std::cos(a1)*radius+v*std::sin(a1)*radius;
            const Vec3 p0=a+r0,p1=a+r1,q0=b+r0,q1=b+r1;
            tri(dst,p0,q0,q1); tri(dst,p0,q1,p1);
            if(outline){ line(p0,p1); line(q0,q1); if(i%(std::max(1,seg/4))==0) line(p0,q0); }
        }
    }
};

float channel_y(UtilityKind kind){ return kind==UtilityKind::PowerCable?0.22f:0.34f; }

bool resource_supports(UtilityKind kind,ResourceKind resource){
    if(kind==UtilityKind::PowerCable) return true;
    return resource==ResourceKind::WaterDriller || resource==ResourceKind::WaterStorage;
}

Vec3 structure_center(const UtilityBuildAnchor& a,
                      const std::vector<PlacedHabitat>& habitats,
                      const std::vector<PlacedResource>& resources){
    if(a.kind==UtilityEndpointKind::Habitat && a.index<habitats.size()) return habitats[a.index].position;
    if(a.kind==UtilityEndpointKind::Resource && a.index<resources.size()) return resources[a.index].position;
    return a.hit;
}

Vec3 rotated_local(Vec3 v,float degrees){ return rotate_y(v,degrees); }

UtilityEndpoint surface_endpoint(const UtilityBuildAnchor& anchor,Vec3 toward,UtilityKind utility,
                                 const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
                                 const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    Vec3 center{};
    float rotation=0.0f, sx=1.0f, sz=1.0f;
    if(anchor.kind==UtilityEndpointKind::Habitat){
        const auto& placed=habitats[anchor.index];
        const auto& mesh=habitat_library.mesh(placed.kind);
        center=placed.position; rotation=placed.rotation_degrees; sx=mesh.footprint_x; sz=mesh.footprint_z;
    }else{
        const auto& placed=resources[anchor.index];
        const auto& mesh=resource_library.mesh(placed.kind);
        center=placed.position; rotation=placed.rotation_degrees; sx=mesh.footprint_x; sz=mesh.footprint_z;
    }

    Vec3 d=toward-center; d.y=0.0f;
    if(length(d)<0.001f) d=anchor.hit-center;
    if(length(d)<0.001f) d={1,0,0};
    const Vec3 local=rotate_y(normalize(d),-rotation);
    Vec3 local_normal{};
    Vec3 local_port{};
    const float hx=sx*0.5f, hz=sz*0.5f;
    if(std::abs(local.x/std::max(0.01f,hx))>=std::abs(local.z/std::max(0.01f,hz))){
        const float sign=local.x>=0.0f?1.0f:-1.0f;
        local_normal={sign,0,0};
        local_port={sign*hx,0,0};
    }else{
        const float sign=local.z>=0.0f?1.0f:-1.0f;
        local_normal={0,0,sign};
        local_port={0,0,sign*hz};
    }
    UtilityEndpoint out;
    out.kind=anchor.kind;
    out.index=anchor.index;
    out.normal=normalize(rotated_local(local_normal,rotation));
    out.position=center+rotated_local(local_port,rotation);
    out.position.y=channel_y(utility);
    return out;
}

bool point_in_footprint(Vec3 p,Vec3 center,float rotation,float sx,float sz,float padding){
    const Vec3 local=rotate_y(p-center,-rotation);
    return std::abs(local.x)<sx*0.5f+padding && std::abs(local.z)<sz*0.5f+padding;
}

bool segment_hits_structure(Vec3 a,Vec3 b,UtilityEndpointKind skip_kind_a,std::size_t skip_a,
                            UtilityEndpointKind skip_kind_b,std::size_t skip_b,
                            const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
                            const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    const float len=distance_xz(a,b);
    const int steps=std::max(2,(int)std::ceil(len/0.35f));
    for(int i=1;i<steps;++i){
        const float t=(float)i/(float)steps;
        const Vec3 p=a+(b-a)*t;
        for(std::size_t h=0;h<habitats.size();++h){
            if((skip_kind_a==UtilityEndpointKind::Habitat && h==skip_a) || (skip_kind_b==UtilityEndpointKind::Habitat && h==skip_b)) continue;
            const auto& placed=habitats[h];
            const auto& mesh=habitat_library.mesh(placed.kind);
            if(point_in_footprint(p,placed.position,placed.rotation_degrees,mesh.footprint_x,mesh.footprint_z,0.28f)) return true;
        }
        for(std::size_t r=0;r<resources.size();++r){
            if((skip_kind_a==UtilityEndpointKind::Resource && r==skip_a) || (skip_kind_b==UtilityEndpointKind::Resource && r==skip_b)) continue;
            const auto& placed=resources[r];
            const auto& mesh=resource_library.mesh(placed.kind);
            if(point_in_footprint(p,placed.position,placed.rotation_degrees,mesh.footprint_x,mesh.footprint_z,0.28f)) return true;
        }
    }
    return false;
}

bool path_clear(const std::vector<Vec3>& path,const UtilityEndpoint& a,const UtilityEndpoint& b,
                const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
                const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    for(std::size_t i=0;i+1<path.size();++i){
        if(segment_hits_structure(path[i],path[i+1],a.kind,a.index,b.kind,b.index,habitats,habitat_library,resources,resource_library)) return false;
    }
    return true;
}

std::vector<Vec3> make_route(const UtilityEndpoint& a,const UtilityEndpoint& b,
                             const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
                             const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    std::vector<Vec3> direct={a.position,b.position};
    if(path_clear(direct,a,b,habitats,habitat_library,resources,resource_library)) return direct;

    const float y=a.position.y;
    std::vector<Vec3> x_first={a.position,{b.position.x,y,a.position.z},b.position};
    std::vector<Vec3> z_first={a.position,{a.position.x,y,b.position.z},b.position};
    const bool x_ok=path_clear(x_first,a,b,habitats,habitat_library,resources,resource_library);
    const bool z_ok=path_clear(z_first,a,b,habitats,habitat_library,resources,resource_library);
    if(x_ok && z_ok){
        const float xl=distance_xz(x_first[0],x_first[1])+distance_xz(x_first[1],x_first[2]);
        const float zl=distance_xz(z_first[0],z_first[1])+distance_xz(z_first[1],z_first[2]);
        return xl<=zl?x_first:z_first;
    }
    if(x_ok) return x_first;
    if(z_ok) return z_first;
    return {};
}

void add_power_segment(Builder& b,Vec3 a,Vec3 c){
    Vec3 d=c-a; d.y=0;
    const float len=length(d);
    if(len<0.001f) return;
    const Vec3 n=d*(1.0f/len);
    const Vec3 side{-n.z,0,n.x};
    const Vec3 mid=(a+c)*0.5f;
    b.oriented_box(b.mesh.shell,mid+Vec3{0,-0.05f,0},n,{0,1,0},side,{len,0.16f,0.52f},false);
    for(float s:{-0.16f,0.0f,0.16f}) b.cylinder_between(b.mesh.details,a+side*s+Vec3{0,0.05f,0},c+side*s+Vec3{0,0.05f,0},0.045f,8,true);
    const int supports=std::max(1,(int)std::floor(len/1.4f));
    for(int i=0;i<=supports;++i){
        const float t=(float)i/(float)std::max(1,supports);
        const Vec3 p=a+(c-a)*t;
        b.line(p-side*0.27f+Vec3{0,-0.09f,0},p+side*0.27f+Vec3{0,-0.09f,0});
    }
}

void add_water_segment(Builder& b,Vec3 a,Vec3 c){
    b.cylinder_between(b.mesh.shell,a,c,0.17f,10,true);
    Vec3 d=c-a; d.y=0;
    const float len=length(d);
    if(len<0.001f) return;
    const Vec3 n=d*(1.0f/len);
    const Vec3 side{-n.z,0,n.x};
    const int supports=std::max(1,(int)std::floor(len/1.6f));
    for(int i=0;i<=supports;++i){
        const float t=(float)i/(float)std::max(1,supports);
        const Vec3 p=a+(c-a)*t;
        b.line(p-side*0.26f+Vec3{0,-0.22f,0},p+side*0.26f+Vec3{0,-0.22f,0});
        b.line(p-side*0.18f+Vec3{0,-0.22f,0},p-side*0.18f+Vec3{0,-0.03f,0});
        b.line(p+side*0.18f+Vec3{0,-0.22f,0},p+side*0.18f+Vec3{0,-0.03f,0});
    }
}

void add_power_port(Builder& b,const UtilityEndpoint& ep){
    const Vec3 c=ep.position-ep.normal*0.16f+Vec3{0,0.08f,0};
    b.box(b.mesh.details,c,{0.46f,0.46f,0.30f},true);
}

void add_water_port(Builder& b,const UtilityEndpoint& ep){
    const Vec3 inside=ep.position-ep.normal*0.20f;
    const Vec3 outside=ep.position+ep.normal*0.16f;
    b.cylinder_between(b.mesh.details,inside,outside,0.26f,12,true);
    b.cylinder_between(b.mesh.details,ep.position-ep.normal*0.03f,ep.position+ep.normal*0.03f,0.33f,12,true);
}

UtilityMesh make_mesh(UtilityKind kind,const UtilityEndpoint& a,const UtilityEndpoint& b,const std::vector<Vec3>& path){
    Builder builder;
    if(kind==UtilityKind::PowerCable){ add_power_port(builder,a); add_power_port(builder,b); }
    else { add_water_port(builder,a); add_water_port(builder,b); }

    for(std::size_t i=0;i+1<path.size();++i){
        if(kind==UtilityKind::PowerCable) add_power_segment(builder,path[i],path[i+1]);
        else add_water_segment(builder,path[i],path[i+1]);
    }
    // Mark bends with a compact service joint so orthogonal routes read as one
    // continuous channel instead of two unrelated line segments.
    for(std::size_t i=1;i+1<path.size();++i){
        if(kind==UtilityKind::PowerCable) builder.box(builder.mesh.details,path[i],{0.58f,0.26f,0.58f},true);
        else builder.cylinder_between(builder.mesh.details,path[i]+Vec3{0,-0.05f,0},path[i]+Vec3{0,0.05f,0},0.26f,12,true);
    }
    return builder.mesh;
}

bool same_anchor(const UtilityBuildAnchor& a,const UtilityBuildAnchor& b){ return a.kind==b.kind && a.index==b.index; }

} // namespace

bool UtilityNetwork::anchor_compatible(UtilityKind kind,const UtilityBuildAnchor& anchor,
                                       const std::vector<PlacedHabitat>& habitats,
                                       const std::vector<PlacedResource>& resources) const {
    if(anchor.kind==UtilityEndpointKind::Habitat) return anchor.index<habitats.size();
    if(anchor.index>=resources.size()) return false;
    return resource_supports(kind,resources[anchor.index].kind);
}

std::optional<UtilityConnection> UtilityNetwork::preview_connection(
    UtilityKind kind,const UtilityBuildAnchor& a,const UtilityBuildAnchor& b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const {
    if(same_anchor(a,b)) return std::nullopt;
    if(!anchor_compatible(kind,a,habitats,resources) || !anchor_compatible(kind,b,habitats,resources)) return std::nullopt;

    const Vec3 center_a=structure_center(a,habitats,resources);
    const Vec3 center_b=structure_center(b,habitats,resources);
    UtilityEndpoint ea=surface_endpoint(a,center_b,kind,habitats,habitat_library,resources,resource_library);
    UtilityEndpoint eb=surface_endpoint(b,center_a,kind,habitats,habitat_library,resources,resource_library);
    const std::vector<Vec3> path=make_route(ea,eb,habitats,habitat_library,resources,resource_library);
    if(path.size()<2) return std::nullopt;

    UtilityConnection out;
    out.kind=kind;
    out.endpoint_a=ea;
    out.endpoint_b=eb;
    out.path=path;
    out.mesh=make_mesh(kind,ea,eb,path);
    return out;
}

bool UtilityNetwork::build_connection(
    UtilityKind kind,const UtilityBuildAnchor& a,const UtilityBuildAnchor& b,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library){
    auto preview=preview_connection(kind,a,b,habitats,habitat_library,resources,resource_library);
    if(!preview) return false;
    for(const auto& existing:connections_){
        if(existing.kind!=kind) continue;
        const bool same=(existing.endpoint_a.kind==preview->endpoint_a.kind && existing.endpoint_a.index==preview->endpoint_a.index &&
                         existing.endpoint_b.kind==preview->endpoint_b.kind && existing.endpoint_b.index==preview->endpoint_b.index) ||
                        (existing.endpoint_a.kind==preview->endpoint_b.kind && existing.endpoint_a.index==preview->endpoint_b.index &&
                         existing.endpoint_b.kind==preview->endpoint_a.kind && existing.endpoint_b.index==preview->endpoint_a.index);
        if(same) return false;
    }
    connections_.push_back(std::move(*preview));
    return true;
}

float UtilityNetwork::connection_length(std::size_t index) const {
    if(index>=connections_.size()) return 0.0f;
    float total=0.0f;
    const auto& path=connections_[index].path;
    for(std::size_t i=0;i+1<path.size();++i) total+=distance_xz(path[i],path[i+1]);
    return total;
}

std::size_t UtilityNetwork::endpoint_connection_count(UtilityEndpointKind kind,std::size_t index) const {
    std::size_t count=0;
    for(const auto& c:connections_){
        if((c.endpoint_a.kind==kind && c.endpoint_a.index==index) || (c.endpoint_b.kind==kind && c.endpoint_b.index==index)) ++count;
    }
    return count;
}

const char* utility_kind_name(UtilityKind kind){ return kind==UtilityKind::PowerCable?"POWER CABLE":"WATER PIPE"; }
const char* utility_endpoint_name(UtilityEndpointKind kind){ return kind==UtilityEndpointKind::Habitat?"HABITAT":"RESOURCE"; }

} // namespace game
