#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_cantine_dome_inner(Builder& b){
    const float floor=b.mesh.walkable_floor_y;
    // Dining groups and the galley face the circulation lanes.
    for(float x:{-1.75f,1.75f})for(float z:{-1.56f,1.56f}){
        b.cylinder_y(b.mesh.interior,{x,floor+.37f,z},.74f,.10f,10,false);
        b.cylinder_y(b.mesh.interior,{x,floor+.75f,z},.12f,.50f,16,false);
        for(float dx:{-.43f,.43f})stool(b,{x+dx,floor,z+std::copysign(.83f,z)});
    }
    for(float x:{-1.77f,1.77f}){
        bench(b,{x,floor,-3.24f},1.01f,.43f);
        b.box(b.mesh.interior,{x,floor+1.26f,-3.37f},{.84f,.49f,.24f},false);
        ceiling_light(b,{x,3.79f,-1.55f},{x,3.79f,1.55f});
    }
}

} // namespace game::habitats
