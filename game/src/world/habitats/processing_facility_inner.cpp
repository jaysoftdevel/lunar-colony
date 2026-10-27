#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_processing_facility_inner(Builder& b){
    // Processing interior: furnace cells, ingot line and center service deck.
    b.box(b.mesh.interior,{0,1.05f,0},{8.6f,0.18f,5.2f},false);
    for(float x:{-2.6f,-0.9f,0.9f,2.6f}){
        b.box(b.mesh.interior,{x,2.15f,-1.35f},{1.15f,1.55f,1.25f},false);
        b.box(b.mesh.interior,{x,1.35f,1.35f},{1.35f,0.38f,1.1f},false);
    }

    // Interior refining line: furnaces, separation vessels, ingot molds and belt transfer.
    for(float x:{-4.8f,-2.9f,-1.0f}){
        b.box(b.mesh.interior,{x,2.0f,-1.8f},{1.25f,2.1f,1.15f},false);
        b.cylinder_y(b.mesh.interior,{x,3.35f,-1.8f},0.75f,0.42f,10,false);
    }
    b.box(b.mesh.interior,{-2.7f,1.25f,0.35f},{6.0f,0.22f,0.85f},false);
    for(int i=0;i<7;++i) b.cylinder_z(b.mesh.interior,{-5.1f+0.8f*i,1.42f,0.35f},0.95f,0.11f,8,false);
    for(float x:{0.8f,2.3f,3.8f}){
        b.box(b.mesh.interior,{x,1.28f,1.65f},{1.15f,0.34f,1.65f},false);
        for(int k=0;k<4;++k) b.box(b.mesh.interior,{x-0.33f+0.22f*k,1.54f,1.65f},{0.16f,0.12f,1.25f},false);
    }
    for(float z:{-1.8f,1.8f}) console(b,{4.65f,1.45f,z},1.2f,z<0?-1.0f:1.0f);
    for(float x:{-4.5f,-1.5f,1.5f,4.5f}) ceiling_light(b,{x,5.0f,-2.0f},{x,5.0f,2.0f});
}

} // namespace game::habitats
