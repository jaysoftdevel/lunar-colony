#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_airlock_inner(Builder& b){
    // Interior deck and equipment remain opaque when hull view modes are active.
    b.box(b.mesh.interior,{0,0.72f,0},{6.4f,0.16f,3.2f},false);
    b.box(b.mesh.interior,{-1.7f,1.55f,-1.35f},{1.6f,0.65f,0.48f},false);
    b.box(b.mesh.interior,{0.9f,1.55f,-1.35f},{1.6f,0.65f,0.48f},false);
    b.cylinder_y(b.mesh.interior,{2.35f,1.55f,0.95f},1.6f,0.30f,10,false);

    // Suit prep, storage and pressure-control equipment along the cylindrical walls.
    locker_bank(b,{-1.7f,1.75f,-1.65f},4);
    locker_bank(b,{1.55f,1.75f,-1.65f},3);
    for(float x:{-2.4f,-0.8f,0.8f,2.4f}){
        b.cylinder_y(b.mesh.interior,{x,2.15f,1.60f},1.75f,0.16f,8,false);
        b.box(b.mesh.interior,{x,1.12f,1.60f},{0.55f,0.24f,0.55f},false);
    }
    console(b,{0.2f,1.38f,1.52f},1.45f,1.0f);
    b.box(b.mesh.interior,{0,1.02f,0},{1.9f,0.22f,0.75f},false);
    ceiling_light(b,{-2.7f,4.15f,0},{2.7f,4.15f,0});
    b.cylinder_between(b.mesh.interior,{-3.0f,3.82f,1.3f},{3.0f,3.82f,1.3f},0.06f,6,true,false);
}

} // namespace game::habitats
