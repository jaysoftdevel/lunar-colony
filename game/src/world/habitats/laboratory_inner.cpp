#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_laboratory_inner(Builder& b){
    const float floor=b.mesh.walkable_floor_y;
    b.box(b.mesh.interior,{0,floor-.08f,0},{10.8f,.16f,3.52f},false);
    for(float x:{-3.31f,-1.36f,1.45f,3.40f}){
        bench(b,{x,floor,-1.54f},1.01f,.58f);
        b.box(b.mesh.interior,{x,floor+1.01f,-1.65f},{.81f,.46f,.37f},false);
        bench(b,{x,floor,1.54f},1.01f,.58f,.72f);
        for(float dx:{-.24f,.24f})tank(b,{x+dx,floor+.72f,1.54f},.50f,.12f);
        ceiling_light(b,{x,4.78f,-1.25f},{x,4.78f,1.25f});
    }
}

} // namespace game::habitats
