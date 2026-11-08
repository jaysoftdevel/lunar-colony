#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_bio_dome_inner(Builder& b){
    const float floor=b.mesh.walkable_floor_y;
    // Grow trays on independent frames leave service gaps between the racks.
    for(float x:{-3.52f,-1.40f,1.40f,3.52f})for(float z:{-1.70f,1.70f}){
        bench(b,{x,floor,z},.92f,.56f,.61f);
        b.box(b.mesh.interior,{x,floor+.69f,z},{.91f,.15f,.51f},false);
        for(float dx:{-.38f,.38f})
            b.box(b.mesh.interior,{x+dx,floor+.49f,z},{.07f,.98f,.45f},false);
        b.cylinder_between(b.mesh.interior,{x-.40f,floor+1.02f,z},{x+.40f,floor+1.02f,z},.035f,6,true,false);
        ceiling_light(b,{x-.42f,3.15f,z},{x+.42f,3.15f,z});
    }
    for(float z:{-2.55f,2.55f})tank(b,{4.30f,floor,z},1.04f,.24f);
}

} // namespace game::habitats
