#pragma once
#include "game/world/habitat_mesh.hpp"
#include "game/util/world_math.hpp"
namespace game {
const HabitatMesh& interior_unit_mesh(InteriorUnitKind);
inline std::pair<runtime::Vec3,runtime::Vec3> interior_unit_bounds(const InteriorUnit& unit,float margin=.18f){
    const auto size=interior_unit_profile(unit.kind).size;
    const auto x=world_math::rotate_y({size.x*.5f+margin,0,0},unit.rotation_degrees);
    const auto z=world_math::rotate_y({0,0,size.z*.5f+margin},unit.rotation_degrees);
    const runtime::Vec3 half{std::abs(x.x)+std::abs(z.x),size.y*.5f,std::abs(x.z)+std::abs(z.z)};
    const auto center=unit.position+runtime::Vec3{0,size.y*.5f,0};return {center-half,center+half};
}
}
