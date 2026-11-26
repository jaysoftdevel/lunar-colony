#pragma once
#include "game/world/habitat_mesh.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/util/world_math.hpp"
namespace game {
const HabitatMesh& interior_unit_mesh(InteriorUnitKind);
const HabitatMesh& interior_unit_mesh(InteriorUnitKind,InteriorUnitSize);
inline const HabitatMesh& interior_unit_mesh(const InteriorUnit& unit){return interior_unit_mesh(unit.kind,unit.size);}
// Every installable unit follows the local tangent of its host pressure hull.
inline bool interior_unit_wall_aligned(InteriorUnitKind kind){
    return kind<InteriorUnitKind::Count;
}
// Alignment does not imply that freestanding furnishings require wall anchors.
inline bool interior_unit_wall_anchored(InteriorUnitKind kind){
    return interior_unit_wall_aligned(kind)&&kind!=InteriorUnitKind::DiningTable&&kind!=InteriorUnitKind::SeatingBench&&kind!=InteriorUnitKind::HydroponicRack;
}
inline float interior_unit_wall_yaw(const HabitatMesh& host,runtime::Vec3 p){
    const auto r=host.dome_radii;
    if(r.x<=0||r.z<=0||p.x*p.x+p.z*p.z<.01f)return 0;
    return std::atan2(p.x/(r.x*r.x),p.z/(r.z*r.z))*57.2957795f;
}
inline bool interior_units_overlap(const InteriorUnit& a,const InteriorUnit& b,float margin=.18f){
    const auto sa=interior_unit_profile(a).size,sb=interior_unit_profile(b).size;
    const auto ax=world_math::rotate_y({1,0,0},a.rotation_degrees),az=world_math::rotate_y({0,0,1},a.rotation_degrees);
    const auto bx=world_math::rotate_y({1,0,0},b.rotation_degrees),bz=world_math::rotate_y({0,0,1},b.rotation_degrees);
    for(const auto n:{ax,az,bx,bz}){
        const float ra=(sa.x*.5f+margin)*std::abs(runtime::dot(n,ax))+(sa.z*.5f+margin)*std::abs(runtime::dot(n,az));
        const float rb=(sb.x*.5f+margin)*std::abs(runtime::dot(n,bx))+(sb.z*.5f+margin)*std::abs(runtime::dot(n,bz));
        if(std::abs(runtime::dot(b.position-a.position,n))>=ra+rb)return false;
    }return true;
}
inline bool interior_unit_reservation_hits(const InteriorUnit& unit,runtime::Vec3 wall,runtime::Vec3 normal){
    const auto size=interior_unit_profile(unit).size;
    const auto ux=world_math::rotate_y({1,0,0},unit.rotation_degrees),uz=world_math::rotate_y({0,0,1},unit.rotation_degrees);
    const runtime::Vec3 side{-normal.z,0,normal.x},center=wall-normal*1.10f;
    for(const auto n:{ux,uz,normal,side}){
        const float a=(size.x*.5f+.18f)*std::abs(runtime::dot(n,ux))+(size.z*.5f+.18f)*std::abs(runtime::dot(n,uz));
        const float b=1.55f*std::abs(runtime::dot(n,normal))+(geometry::kDoorClearWidth*.5f+.20f)*std::abs(runtime::dot(n,side));
        if(std::abs(runtime::dot(unit.position-center,n))>=a+b)return false;
    }return true;
}
inline std::pair<runtime::Vec3,runtime::Vec3> interior_unit_bounds(const InteriorUnit& unit,float margin=.18f){
    const auto size=interior_unit_profile(unit).size;
    const auto x=world_math::rotate_y({size.x*.5f+margin,0,0},unit.rotation_degrees);
    const auto z=world_math::rotate_y({0,0,size.z*.5f+margin},unit.rotation_degrees);
    const runtime::Vec3 half{std::abs(x.x)+std::abs(z.x),size.y*.5f,std::abs(x.z)+std::abs(z.z)};
    const auto center=unit.position+runtime::Vec3{0,size.y*.5f,0};return {center-half,center+half};
}
}
