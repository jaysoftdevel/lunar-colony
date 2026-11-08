#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_sick_bay_inner(Builder& b){
    const float floor=b.mesh.walkable_floor_y;
    b.cylinder_y(b.mesh.interior,{0,floor-.07f,0},.14f,4.55f,24,false);
    for(float x:{-1.85f,1.85f})for(float z:{-1.61f,1.61f}){
        bench(b,{x,floor,z},1.72f,.62f,.48f);
        b.box(b.mesh.interior,{x,floor+.54f,z},{1.71f,.12f,.64f},false);
        b.box(b.mesh.interior,{x-.58f,floor+.635f,z},{.37f,.07f,.49f},false);
        const float edge=z+std::copysign(.49f,z);
        b.cylinder_y(b.mesh.interior,{x+.61f,floor+.65f,edge},1.30f,.07f,8,false);
        b.box(b.mesh.interior,{x+.61f,floor+1.36f,edge},{.45f,.31f,.12f},false);
    }
    for(float z:{-3.35f,3.35f})for(float x:{-1.27f,1.27f})
        locker_bank(b,{x,floor+.775f,z},2,.57f);
    for(float x:{-1.85f,1.85f})ceiling_light(b,{x,3.75f,-1.30f},{x,3.75f,1.30f});
}

} // namespace game::habitats
