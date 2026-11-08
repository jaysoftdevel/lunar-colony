#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_processing_facility_inner(Builder& b){
    const float floor=b.mesh.walkable_floor_y;
    b.box(b.mesh.interior,{0,floor-.08f,0},{11.5f,.16f,5.5f},false);
    for(float x:{-2.65f,1.25f,3.65f}){
        bench(b,{x,floor,-1.74f},1.35f,.81f,.86f);
        b.cylinder_y(b.mesh.interior,{x,floor+1.33f,-1.74f},.95f,.35f,12,false);
        b.cylinder_y(b.mesh.interior,{x,floor+1.91f,-1.74f},.22f,.22f,10,false);
        bench(b,{x,floor,1.74f},1.37f,.79f,.64f);
        for(float dx:{-.40f,0.f,.40f})b.box(b.mesh.interior,{x+dx,floor+.72f,1.74f},{.21f,.16f,.54f},false);
        ceiling_light(b,{x,4.75f,-1.65f},{x,4.75f,1.65f});
    }
}

} // namespace game::habitats
