#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_manufactory_inner(Builder& b){
    b.box(b.mesh.interior,{-3.6f,1.15f,0},{7.4f,0.22f,6.5f},false);
    // Fabrication gantry and suspended tooling visible inside.
    b.box(b.mesh.interior,{-3.6f,6.3f,0},{6.8f,0.22f,0.42f},false);
    b.cylinder_between(b.mesh.interior,{-3.6f,6.2f,0},{-3.6f,4.45f,0},0.16f,8,true,false);
    b.box(b.mesh.interior,{-3.6f,4.15f,0},{0.8f,0.65f,0.8f},false);
    for(float x:{-5.8f,-3.5f,-1.2f}){
        b.box(b.mesh.interior,{x,1.55f,-2.0f},{1.6f,0.7f,1.2f},false);
        b.cylinder_y(b.mesh.interior,{x,2.65f,1.75f},2.25f,0.35f,10,false);
    }


    // Heavy fabrication bed, additive/CNC cells, material pallets and operator island.
    b.box(b.mesh.interior,{-3.6f,1.55f,0},{5.8f,0.34f,2.6f},false);
    for(float x:{-5.6f,-3.6f,-1.6f}){
        b.box(b.mesh.interior,{x,2.25f,-2.55f},{1.45f,2.0f,1.1f},false);
        b.cylinder_y(b.mesh.interior,{x,3.55f,-2.55f},0.72f,0.35f,10,false);
    }
    for(int row=0;row<2;++row) for(int col=0;col<4;++col)
        b.box(b.mesh.interior,{-6.1f+1.45f*col,1.45f+0.72f*row,2.65f},{1.1f,0.55f,1.0f},false);
    console(b,{-0.1f,1.65f,2.65f},1.45f,1.0f);
    b.cylinder_between(b.mesh.interior,{-3.6f,5.95f,0},{-3.6f,3.35f,0},0.13f,8,true,false);
    b.box(b.mesh.interior,{-3.6f,3.15f,0},{0.75f,0.55f,1.2f},false);
    for(float x:{-5.8f,-3.4f,-1.0f}) ceiling_light(b,{x,6.05f,-3.2f},{x,6.05f,3.2f});
}

} // namespace game::habitats
