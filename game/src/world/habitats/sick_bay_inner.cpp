#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_sick_bay_inner(Builder& b){
    // Medical interior: four treatment beds, central diagnostic ring and cabinets.
    b.box(b.mesh.interior,{0,1.15f,0},{6.6f,0.18f,6.6f},false);
    for(Vec3 c:std::initializer_list<Vec3>{{-2.0f,1.45f,-1.65f},{2.0f,1.45f,-1.65f},{-2.0f,1.45f,1.65f},{2.0f,1.45f,1.65f}}){
        b.box(b.mesh.interior,c,{1.65f,0.32f,0.72f},false);
        b.cylinder_y(b.mesh.interior,{c.x,2.25f,c.z},1.1f,0.18f,8,false);
    }
    b.torus_x(b.mesh.interior,{0,3.25f,0},1.35f,0.09f,18,6);
    for(float z:{-2.7f,2.7f}) b.box(b.mesh.interior,{0,2.0f,z},{3.4f,1.6f,0.48f},false);

    // Medical storage, scanner arches, operating table, monitors and isolation cubicle.
    b.box(b.mesh.interior,{0,1.35f,0},{2.2f,0.32f,0.85f},false);
    b.torus_x(b.mesh.interior,{0,2.15f,0},0.85f,0.07f,16,6);
    for(float z:{-2.85f,2.85f}) locker_bank(b,{0,1.95f,z},5,0.6f);
    for(float x:{-2.0f,2.0f}){
        b.box(b.mesh.interior,{x,2.0f,0},{0.65f,1.2f,0.42f},false);
        b.box(b.mesh.interior,{x,2.55f,-0.25f},{0.48f,0.42f,0.08f},false);
    }
    b.box(b.mesh.interior,{-3.15f,1.85f,0},{0.9f,2.2f,2.0f},false);
    b.cylinder_y(b.mesh.interior,{-3.15f,3.15f,0},0.45f,0.28f,10,false);
    console(b,{3.15f,1.45f,0},1.15f,1.0f);
    for(float x:{-2.3f,0.0f,2.3f}) ceiling_light(b,{x,4.85f,-1.8f},{x,4.85f,1.8f});
}

} // namespace game::habitats
