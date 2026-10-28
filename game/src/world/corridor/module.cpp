#include "game/world/corridor/corridor_module.hpp"
#include "game/world/corridor/detail.hpp"
#include "game/world/geometry_standards.hpp"

namespace game::corridor_detail {

game::StructureAccessProfile CorridorModule::access_profile() const {
    return {game::geometry::kCorridorDeckY,true,true,false};
}

void CorridorModule::build_outer(CorridorMesh& mesh,const std::vector<eng::Vec3>& path) const {
    append_swept_shell(mesh,path);
    append_ribs(mesh,path);
    append_pressure_services(mesh,path);
}

void CorridorModule::build_inner(CorridorMesh& mesh,const std::vector<eng::Vec3>& path) const {
    append_walkway(mesh,path);
    append_deck_panels(mesh,path);
}

const CorridorModule& corridor_module(){
    static const CorridorModule value{};
    return value;
}

} // namespace game::corridor_detail
