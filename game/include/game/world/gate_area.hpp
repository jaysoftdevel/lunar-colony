#pragma once
#include "game/world/corridor_network.hpp"
#include "game/world/storage_loading_gate.hpp"
#include "game/world/robot_workshop.hpp"
#include <vector>
namespace game::gate_area {
struct Area {
    eng::Vec3 center{},normal{0,0,1};float half_width{0.8f},half_depth{1.0f};
    world_math::OrientedFootprint bounds() const {return world_math::footprint(center,std::atan2(normal.x,normal.z)*180.0f/3.14159265358979323846f,2*half_width,2*half_depth);}
    bool hits(eng::Vec3 a,eng::Vec3 b,float margin=0) const {
        const eng::Vec3 side{normal.z,0,-normal.x},d=b-a;float lo=0,hi=1;
        const auto clip=[&](float start,float delta,float extent){
            if(std::abs(delta)<1e-7f)return std::abs(start)<=extent;
            float u=(-extent-start)/delta,v=(extent-start)/delta;if(u>v)std::swap(u,v);lo=std::max(lo,u);hi=std::min(hi,v);return lo<=hi;
        };
        return clip(eng::dot(a-center,side),eng::dot(d,side),half_width+margin) && clip(eng::dot(a-center,normal),eng::dot(d,normal),half_depth+margin);
    }
};
// A placing pad beside the threshold keeps the central corridor aperture free.
inline Area beside(eng::Vec3 wall,eng::Vec3 normal){normal.y=0;normal=eng::normalize(normal);return {wall+normal*1.4f+eng::Vec3{normal.z,0,-normal.x}*3.0f,normal,0.8f,1.0f};}
inline void add_unique(std::vector<Area>& areas,Area a){for(const auto& old:areas)if(world_math::distance_xz(old.center,a.center)<0.25f)return;areas.push_back(a);}
inline std::vector<Area> habitat(const PlacedHabitat& h,const HabitatMesh& m,std::size_t index,const std::vector<CorridorConnection>& corridors){
    std::vector<Area> out;
    for(const auto& e:m.entrances)add_unique(out,beside(h.position+world_math::rotate_y(e.local_position,h.rotation_degrees),world_math::rotate_y(e.local_normal,h.rotation_degrees)));
    for(const auto& c:corridors)for(const auto* e:{&c.endpoint_a,&c.endpoint_b})if(e->kind==CorridorEndpointKind::Habitat && e->index==index)add_unique(out,beside(e->door.wall_position,e->door.normal));
    return out;
}
inline std::vector<Area> storage(const PlacedResource& r,const ResourceMesh& m,std::size_t index,const std::vector<CorridorConnection>& corridors){
    std::vector<Area> out;
    if(r.kind==ResourceKind::RobotWorkshop){out.push_back({workshop::world(r,{0,0,(workshop::kApproachFar-workshop::kHalfZ)*0.5f}),world_math::rotate_y({0,0,-1},r.rotation_degrees),workshop::kApproachHalfWidth,(-workshop::kHalfZ-workshop::kApproachFar)*0.5f});return out;}
    if(r.kind!=ResourceKind::StorageDome)return out;
    const float radius=std::min(m.footprint_x,m.footprint_z)*(5.35f/13.0f);
    for(int f=0;f<4;++f){const auto n=world_math::rotate_y(storage_loading::local_normal(f),r.rotation_degrees);add_unique(out,beside(r.position+n*radius,n));}
    for(const auto& c:corridors)for(const auto* e:{&c.endpoint_a,&c.endpoint_b})if(e->kind==CorridorEndpointKind::Resource && e->index==index)add_unique(out,beside(e->door.wall_position,e->door.normal));
    for(int f:storage_loading::faces(r))if(f>=0)add_unique(out,{storage_loading::service(r,f),storage_loading::normal(r,f),1.0f,0.75f});
    return out;
}
inline void draw(eng::Renderer& r,const Area& a,bool valid=true){
    const auto n=a.normal;const eng::Vec3 side{n.z,0,-n.x};auto c=a.center;c.y=0.07f;
    const auto p=c-side*a.half_width-n*a.half_depth,q=c+side*a.half_width-n*a.half_depth,s=c+side*a.half_width+n*a.half_depth,t=c-side*a.half_width+n*a.half_depth;
    const eng::Color color=valid?eng::Color{1.0f,0.72f,0.15f,1}:eng::Color{1.0f,0.16f,0.12f,1};
    r.draw_triangles_3d({p,q,s,p,s,t},{color.r,color.g,color.b,0.22f});r.draw_lines_3d({p,q,q,s,s,t,t,p,p,s,q,t},color,2.5f);
}

}
