#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace game::runtime;
using namespace interior_detail;

void build_cantine_dome_inner(Builder& b){
    const float floor=b.mesh.walkable_floor_y;
    // Dining groups are furnished through the internal-unit catalog.
    for(float x:{-1.77f,1.77f}){
        interior_layout::StationScope station(b.mesh);
        bench(b,{x,floor,-3.24f},1.01f,.43f);
        b.box(b.mesh.interior,{x,floor+1.26f,-3.37f},{.84f,.49f,.24f},false);
        ceiling_light(b,{x,3.79f,-1.55f},{x,3.79f,1.55f});
    }
}

} // namespace game::habitats
