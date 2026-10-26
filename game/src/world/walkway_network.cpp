#include "game/world/walkway_network.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace game {
using namespace eng;
using namespace world_math;
namespace {
constexpr float PI=3.14159265358979323846f;
constexpr float HALF_WIDTH=0.78f;
constexpr float DECK_Y=0.20f;
constexpr float RAIL_H=0.95f;
constexpr float CLEARANCE=0.55f;

Vec3 flat(Vec3 v){ v.y=0.0f; return v; }
Vec3 flat_normal(Vec3 v){ v=flat(v); const float l=length(v); return l>0.0001f?v*(1.0f/l):Vec3{1,0,0}; }
Vec3 side_of(Vec3 v){ v=flat_normal(v); return {-v.z,0,v.x}; }
float dist_xz(Vec3 a,Vec3 b){ const float dx=a.x-b.x,dz=a.z-b.z; return std::sqrt(dx*dx+dz*dz); }

struct Builder {
    WalkwayMesh mesh;
    static void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c){ dst.push_back(a); dst.push_back(b); dst.push_back(c); }
    void line(Vec3 a,Vec3 b){ mesh.edges.push_back(a); mesh.edges.push_back(b); }
    void cylinder(std::vector<Vec3>& dst,Vec3 a,Vec3 b,float radius,int seg=8){
        const Vec3 axis=b-a; const float len=length(axis); if(len<0.0001f) return;
        const Vec3 n=axis*(1.0f/len); const Vec3 ref=std::abs(n.y)<0.92f?Vec3{0,1,0}:Vec3{1,0,0};
        const Vec3 u=normalize(cross(n,ref)),v=normalize(cross(n,u));
        seg=std::max(6,seg);
        for(int i=0;i<seg;++i){
            const float a0=2.0f*PI*(float)i/(float)seg,a1=2.0f*PI*(float)(i+1)/(float)seg;
            const Vec3 r0=u*std::cos(a0)*radius+v*std::sin(a0)*radius;
            const Vec3 r1=u*std::cos(a1)*radius+v*std::sin(a1)*radius;
            tri(dst,a+r0,b+r0,b+r1); tri(dst,a+r0,b+r1,a+r1);
            line(a+r0,a+r1); line(b+r0,b+r1);
        }
    }
};

Vec3 closest_on_rotated_rect(Vec3 hit,const PlacedResource& item,const ResourceMesh& mesh){
    const Vec3 local=rotate_y(hit-item.position,-item.rotation_degrees);
    const float hx=mesh.footprint_x*0.5f,hz=mesh.footprint_z*0.5f;
    Vec3 q{std::clamp(local.x,-hx,hx),0,std::clamp(local.z,-hz,hz)};
    const float dx=hx-std::abs(q.x),dz=hz-std::abs(q.z);
    if(dx<dz) q.x=(q.x<0?-hx:hx); else q.z=(q.z<0?-hz:hz);
    return item.position+rotate_y(q,item.rotation_degrees);
}

Vec3 resource_normal(Vec3 point,const PlacedResource& item,const ResourceMesh& mesh){
    const Vec3 local=rotate_y(point-item.position,-item.rotation_degrees);
    const float hx=mesh.footprint_x*0.5f,hz=mesh.footprint_z*0.5f;
    Vec3 n{};
    if(std::abs(std::abs(local.x)-hx)<std::abs(std::abs(local.z)-hz)) n={local.x<0?-1.0f:1.0f,0,0};
    else n={0,0,local.z<0?-1.0f:1.0f};
    return flat_normal(rotate_y(n,item.rotation_degrees));
}

bool door_clear(const CorridorDoor& door,const std::vector<WalkwayConnection>& existing){
    for(const auto& c:existing){
        for(const WalkwayEndpoint* ep:{&c.endpoint_a,&c.endpoint_b}){
            if(ep->kind!=WalkwayEndpointKind::Habitat) continue;
            if(dist_xz(ep->door.position,door.position)<2.35f) return false;
        }
    }
    return true;
}

std::optional<WalkwayEndpoint> resolve_habitat_endpoint(const WalkwayBuildAnchor& a,Vec3 other,
                                                        const CorridorNetwork& corridors,
                                                        const std::vector<PlacedHabitat>& habitats,
                                                        const HabitatModelLibrary& library,
                                                        const std::vector<WalkwayConnection>& existing){
    if(a.index>=habitats.size()) return std::nullopt;
    CorridorBuildAnchor ca{CorridorEndpointKind::Habitat,a.index,a.hit};
    auto ep=corridors.preview_anchor(ca,other,habitats,library);
    if(!ep || ep->kind!=CorridorEndpointKind::Habitat || !door_clear(ep->door,existing)) return std::nullopt;
    WalkwayEndpoint out;
    out.kind=WalkwayEndpointKind::Habitat; out.index=a.index; out.door=ep->door;
    out.position=ep->door.position; out.position.y=0.0f; out.normal=flat_normal(ep->door.normal);
    return out;
}

std::optional<WalkwayEndpoint> resolve_resource_endpoint(const WalkwayBuildAnchor& a,
                                                         const std::vector<PlacedResource>& resources,
                                                         const ResourceModelLibrary& library){
    if(a.index>=resources.size()) return std::nullopt;
    const auto& item=resources[a.index]; const auto& mesh=library.mesh(item.kind);
    WalkwayEndpoint out; out.kind=WalkwayEndpointKind::Resource; out.index=a.index;
    if(item.kind==ResourceKind::LandingPad){
        // The landing pad has one deliberate crew approach: the outer end of
        // its built-in approach ramp.  End the walkway flush with that deck so
        // it never grows a fake entrance, vestibule, or stair block.
        const Vec3 local{0.0f,0.50f,-10.15f};
        out.position=item.position+rotate_y(local,item.rotation_degrees);
        out.normal=flat_normal(rotate_y(Vec3{0,0,-1},item.rotation_degrees));
        out.landing_pad=true;
        return out;
    }
    out.position=closest_on_rotated_rect(a.hit,item,mesh);
    out.normal=resource_normal(out.position,item,mesh);
    out.generated_access=true;
    return out;
}

float point_segment_distance(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z,den=dx*dx+dz*dz;
    if(den<0.0001f) return dist_xz(p,a);
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return dist_xz(p,{a.x+dx*t,0,a.z+dz*t});
}

bool segment_clear(Vec3 a,Vec3 b,const WalkwayEndpoint& ea,const WalkwayEndpoint& eb,
                   const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                   const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib){
    for(std::size_t i=0;i<habitats.size();++i){
        if((ea.kind==WalkwayEndpointKind::Habitat && ea.index==i)||(eb.kind==WalkwayEndpointKind::Habitat && eb.index==i)) continue;
        const auto& m=hlib.mesh(habitats[i]);
        const float r=0.46f*std::sqrt(m.footprint_x*m.footprint_x+m.footprint_z*m.footprint_z)+CLEARANCE;
        if(point_segment_distance(habitats[i].position,a,b)<r) return false;
    }
    for(std::size_t i=0;i<resources.size();++i){
        if((ea.kind==WalkwayEndpointKind::Resource && ea.index==i)||(eb.kind==WalkwayEndpointKind::Resource && eb.index==i)) continue;
        const auto& m=rlib.mesh(resources[i].kind);
        const float r=0.46f*std::sqrt(m.footprint_x*m.footprint_x+m.footprint_z*m.footprint_z)+CLEARANCE;
        if(point_segment_distance(resources[i].position,a,b)<r) return false;
    }
    return true;
}

Vec3 cubic(Vec3 p0,Vec3 p1,Vec3 p2,Vec3 p3,float t){
    const float u=1.0f-t;
    return p0*(u*u*u)+p1*(3.0f*u*u*t)+p2*(3.0f*u*t*t)+p3*(t*t*t);
}

void append_curve(std::vector<Vec3>& out,Vec3 p0,Vec3 p1,Vec3 p2,Vec3 p3,int samples,bool include_first){
    for(int i=include_first?0:1;i<=samples;++i){
        const float t=(float)i/(float)samples;
        const Vec3 p=cubic(p0,p1,p2,p3,t);
        if(out.empty()||dist_xz(out.back(),p)>0.04f||std::abs(out.back().y-p.y)>0.03f) out.push_back(p);
    }
}

bool path_clear(const std::vector<Vec3>& path,const WalkwayEndpoint& a,const WalkwayEndpoint& b,
                const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib){
    if(path.size()<2) return false;
    for(std::size_t i=0;i+1<path.size();++i)
        if(!segment_clear(path[i],path[i+1],a,b,habitats,hlib,resources,rlib)) return false;
    return true;
}

float path_len(const std::vector<Vec3>& path){ float n=0; for(std::size_t i=0;i+1<path.size();++i) n+=dist_xz(path[i],path[i+1]); return n; }

std::vector<Vec3> smooth_candidate(const WalkwayEndpoint& a,const WalkwayEndpoint& b,float lateral){
    const float d=std::max(1.0f,dist_xz(a.position,b.position));
    const float lead=std::clamp(d*0.24f,1.15f,5.0f);
    const Vec3 chord=flat_normal(b.position-a.position);
    const Vec3 side=side_of(chord);
    if(std::abs(lateral)<0.01f){
        std::vector<Vec3> path;
        append_curve(path,a.position,a.position+a.normal*lead,b.position+b.normal*lead,b.position,std::clamp((int)(d*2.0f),12,40),true);
        return path;
    }

    const Vec3 mid=(a.position+b.position)*0.5f+side*lateral;
    const Vec3 mid_tangent=flat_normal(b.position-a.position);
    const float half=std::max(1.0f,d*0.24f);
    std::vector<Vec3> path;
    append_curve(path,a.position,a.position+a.normal*lead,mid-mid_tangent*half,mid,std::clamp((int)d,8,24),true);
    append_curve(path,mid,mid+mid_tangent*half,b.position+b.normal*lead,b.position,std::clamp((int)d,8,24),false);
    return path;
}

std::vector<Vec3> route(const WalkwayEndpoint& a,const WalkwayEndpoint& b,
                        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib){
    const float d=dist_xz(a.position,b.position);
    const float offset=std::clamp(d*0.28f,3.0f,9.0f);
    std::array<float,5> offsets={0.0f,offset,-offset,offset*1.55f,-offset*1.55f};
    float best=std::numeric_limits<float>::max(); std::vector<Vec3> chosen;
    for(float lateral:offsets){
        auto p=smooth_candidate(a,b,lateral);
        if(!path_clear(p,a,b,habitats,hlib,resources,rlib)) continue;
        const float len=path_len(p);
        if(len<best){ best=len; chosen=std::move(p); }
    }
    return chosen;
}

void append_generated_entrance(Builder& b,const CorridorDoor& door){
    if(!door.generated) return;
    const Vec3 n=flat_normal(door.normal),side=side_of(n);
    constexpr float half=1.18f;
    std::vector<Vec3> outer;
    if(!door.wall_profile.empty()){
        outer.reserve(door.wall_profile.size());
        for(const Vec3& wp:door.wall_profile){
            const float lateral=dot(flat(wp-door.wall_position),side);
            outer.push_back({door.position.x+side.x*lateral,wp.y,door.position.z+side.z*lateral});
        }
        if(outer.size()==door.wall_profile.size()&&outer.size()>=3){
            for(std::size_t i=0;i<outer.size();++i){
                const std::size_t j=(i+1)%outer.size();
                Builder::tri(b.mesh.details,door.wall_profile[i],outer[i],outer[j]);
                Builder::tri(b.mesh.details,door.wall_profile[i],outer[j],door.wall_profile[j]);
                b.line(door.wall_profile[i],door.wall_profile[j]); b.line(outer[i],outer[j]);
            }
        }
    }
    const Vec3 base{door.position.x,0.0f,door.position.z};
    b.line(base-side*half,base+side*half);
    b.line(base-side*half,base-side*half+Vec3{0,2.65f,0});
    b.line(base+side*half,base+side*half+Vec3{0,2.65f,0});
    b.line(base-side*half+Vec3{0,2.65f,0},base+side*half+Vec3{0,2.65f,0});
}

void append_resource_access(Builder& b,const WalkwayEndpoint& ep){
    if(ep.kind!=WalkwayEndpointKind::Resource || !ep.generated_access || ep.landing_pad) return;
    const Vec3 n=flat_normal(ep.normal),side=side_of(n),base=ep.position+Vec3{0,DECK_Y,0};
    constexpr float half=0.92f;
    // Open exterior access gate: clearly a crew access point, intentionally no
    // door leaf / pressure shell because walkways never provide atmosphere.
    b.line(base-side*half,base-side*half+Vec3{0,1.35f,0});
    b.line(base+side*half,base+side*half+Vec3{0,1.35f,0});
    b.line(base-side*half+Vec3{0,1.35f,0},base+side*half+Vec3{0,1.35f,0});
    b.line(base-side*half,base+side*half);
    b.cylinder(b.mesh.details,base-side*half-n*0.10f,base-side*half+n*0.10f,0.055f,6);
    b.cylinder(b.mesh.details,base+side*half-n*0.10f,base+side*half+n*0.10f,0.055f,6);
}

void append_habitat_steps(Builder& b,const WalkwayEndpoint& ep){
    if(ep.kind!=WalkwayEndpointKind::Habitat) return;
    const float target=std::max(0.24f,ep.door.wall_position.y-0.25f);
    const float rise=target-DECK_Y; if(rise<0.12f) return;
    const Vec3 n=flat_normal(ep.normal),side=side_of(n);
    const int steps=std::clamp((int)std::ceil(rise/0.16f),2,7);
    const float run=1.18f,step_run=run/(float)steps,step_rise=rise/(float)steps;
    for(int i=0;i<steps;++i){
        const float d=step_run*((float)i+0.5f);
        const float y=target-step_rise*(float)i;
        const Vec3 c=ep.position+n*d+Vec3{0,y,0};
        const Vec3 s=side*(HALF_WIDTH*0.88f),f=n*(step_run*0.50f);
        Builder::tri(b.mesh.deck,c-f-s,c+f-s,c+f+s); Builder::tri(b.mesh.deck,c-f-s,c+f+s,c-f+s);
        b.line(c-f-s,c-f+s); b.line(c+f-s,c+f+s);
    }
}

void append_landing_terminal(Builder& b,const WalkwayEndpoint& ep){
    if(!ep.landing_pad) return;
    const Vec3 n=flat_normal(ep.normal),side=side_of(n);
    const Vec3 edge=ep.position+Vec3{0,DECK_Y,0};
    const Vec3 outer=edge+n*1.15f;
    const Vec3 s0=side*(HALF_WIDTH*1.55f),s1=side*HALF_WIDTH;
    Builder::tri(b.mesh.deck,outer-s1,edge-s0,edge+s0);
    Builder::tri(b.mesh.deck,outer-s1,edge+s0,outer+s1);
    b.line(edge-s0,edge+s0); // clean, regular pad threshold
    b.line(outer-s1,edge-s0); b.line(outer+s1,edge+s0);
}

WalkwayMesh make_mesh(const WalkwayEndpoint& a,const WalkwayEndpoint& b,const std::vector<Vec3>& path){
    Builder builder;
    append_generated_entrance(builder,a.door); append_generated_entrance(builder,b.door);
    append_resource_access(builder,a); append_resource_access(builder,b);
    append_habitat_steps(builder,a); append_habitat_steps(builder,b);
    append_landing_terminal(builder,a); append_landing_terminal(builder,b);

    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 p0=path[i]+Vec3{0,DECK_Y,0},p1=path[i+1]+Vec3{0,DECK_Y,0};
        Vec3 tangent=flat_normal(p1-p0); if(length(tangent)<0.001f) continue;
        Vec3 next_tangent=tangent;
        if(i+2<path.size()) next_tangent=flat_normal(path[i+2]-path[i+1]);
        const Vec3 side0=side_of(tangent),side1=side_of(next_tangent);
        const Vec3 s0=side0*HALF_WIDTH,s1=side1*HALF_WIDTH;
        Builder::tri(builder.mesh.deck,p0-s0,p1-s1,p1+s1); Builder::tri(builder.mesh.deck,p0-s0,p1+s1,p0+s0);
        builder.line(p0-s0,p1-s1); builder.line(p0+s0,p1+s1);
        builder.line(p0-s0+Vec3{0,RAIL_H,0},p1-s1+Vec3{0,RAIL_H,0});
        builder.line(p0+s0+Vec3{0,RAIL_H,0},p1+s1+Vec3{0,RAIL_H,0});
        const float len=dist_xz(p0,p1); const int posts=std::max(1,(int)std::floor(len/1.8f));
        for(int j=0;j<=posts;++j){
            const float t=(float)j/(float)std::max(1,posts); const Vec3 p=p0+(p1-p0)*t;
            const Vec3 side=normalize(side0*(1.0f-t)+side1*t); const Vec3 s=side*HALF_WIDTH;
            builder.line(p-s,p-s+Vec3{0,RAIL_H,0}); builder.line(p+s,p+s+Vec3{0,RAIL_H,0});
        }
        // Integrated supply backbone: water pipe plus paired electrical conductors.
        // It follows the same smooth path but does not imply a pressurized volume.
        const Vec3 util0=side0*(HALF_WIDTH+0.16f),util1=side1*(HALF_WIDTH+0.16f);
        builder.cylinder(builder.mesh.details,p0+util0+Vec3{0,-0.02f,0},p1+util1+Vec3{0,-0.02f,0},0.10f,8);
        builder.cylinder(builder.mesh.details,p0-util0+Vec3{0,0.02f,0},p1-util1+Vec3{0,0.02f,0},0.035f,6);
        builder.cylinder(builder.mesh.details,p0-util0+Vec3{0,0.11f,0},p1-util1+Vec3{0,0.11f,0},0.035f,6);
    }
    return std::move(builder.mesh);
}
}

std::optional<WalkwayConnection> WalkwayNetwork::preview_connection(
    const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,const CorridorNetwork& corridors,
    const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
    const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib) const {
    std::optional<WalkwayEndpoint> a,b;
    if(source.kind==WalkwayEndpointKind::Habitat) a=resolve_habitat_endpoint(source,target.hit,corridors,habitats,hlib,connections_);
    else if(source.kind==WalkwayEndpointKind::Resource) a=resolve_resource_endpoint(source,resources,rlib);
    if(!a) return std::nullopt;

    if(target.kind==WalkwayEndpointKind::Habitat) b=resolve_habitat_endpoint(target,a->position,corridors,habitats,hlib,connections_);
    else if(target.kind==WalkwayEndpointKind::Resource) b=resolve_resource_endpoint(target,resources,rlib);
    if(!b) return std::nullopt;
    if(a->kind==b->kind && a->index==b->index) return std::nullopt;

    auto path=route(*a,*b,habitats,hlib,resources,rlib); if(path.size()<2) return std::nullopt;
    WalkwayConnection out; out.endpoint_a=*a; out.endpoint_b=*b; out.path=std::move(path);
    out.mesh=make_mesh(out.endpoint_a,out.endpoint_b,out.path); out.pressurized=false; return out;
}

bool WalkwayNetwork::build_connection(const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,const CorridorNetwork& corridors,
                                      const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& hlib,
                                      const std::vector<PlacedResource>& resources,const ResourceModelLibrary& rlib){
    auto c=preview_connection(source,target,corridors,habitats,hlib,resources,rlib); if(!c) return false; connections_.push_back(std::move(*c)); return true;
}

float WalkwayNetwork::connection_length(std::size_t index) const { return index<connections_.size()?path_len(connections_[index].path):0.0f; }
std::size_t WalkwayNetwork::endpoint_connection_count(WalkwayEndpointKind kind,std::size_t index) const {
    std::size_t n=0; for(const auto& c:connections_) for(const WalkwayEndpoint* e:{&c.endpoint_a,&c.endpoint_b}) if(e->kind==kind&&e->index==index) ++n; return n;
}
void WalkwayNetwork::remove_connection(std::size_t index){ if(index<connections_.size()) connections_.erase(connections_.begin()+(std::ptrdiff_t)index); }
void WalkwayNetwork::remove_habitat(std::size_t index){
    for(auto& c:connections_) for(WalkwayEndpoint* e:{&c.endpoint_a,&c.endpoint_b}){
        if(e->kind==WalkwayEndpointKind::Habitat&&e->index==index){ e->kind=WalkwayEndpointKind::Open; e->normal=e->door.normal; }
        else if(e->kind==WalkwayEndpointKind::Habitat&&e->index>index) --e->index;
    }
}
void WalkwayNetwork::remove_resource(std::size_t index){
    for(auto& c:connections_) for(WalkwayEndpoint* e:{&c.endpoint_a,&c.endpoint_b}){
        if(e->kind==WalkwayEndpointKind::Resource&&e->index==index) e->kind=WalkwayEndpointKind::Open;
        else if(e->kind==WalkwayEndpointKind::Resource&&e->index>index) --e->index;
    }
}
const char* walkway_endpoint_name(WalkwayEndpointKind kind){ switch(kind){ case WalkwayEndpointKind::Habitat:return "HABITAT"; case WalkwayEndpointKind::Resource:return "STRUCTURE"; case WalkwayEndpointKind::Open:return "OPEN END"; } return "END"; }

} // namespace game
