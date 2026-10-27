#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_laboratory_inner(Builder& b){
    // Research benches, sample columns and manipulators.
    b.box(b.mesh.interior,{-1.5f,1.25f,0},{6.5f,0.18f,3.8f},false);
    for(float x:{-3.2f,-1.4f,0.4f}){
        b.box(b.mesh.interior,{x,1.55f,-1.25f},{1.25f,0.65f,0.75f},false);
        b.cylinder_y(b.mesh.interior,{x,2.25f,1.2f},1.55f,0.25f,10,false);
    }
    b.cylinder_between(b.mesh.interior,{-2.4f,2.0f,0},{-1.8f,2.75f,0.4f},0.11f,8,true,false);
    b.cylinder_between(b.mesh.interior,{-1.8f,2.75f,0.4f},{-1.2f,2.25f,0.65f},0.09f,8,true,false);

    // Dense research benches, glove boxes, sample carousel, servers and optical/instrument tables.
    for(float x:{-3.8f,-1.9f,0.0f,1.9f}){
        b.box(b.mesh.interior,{x,1.45f,0.2f},{1.45f,0.72f,0.72f},false);
        b.box(b.mesh.interior,{x,2.18f,0.2f},{1.15f,0.55f,0.12f},false);
    }
    b.box(b.mesh.interior,{-3.55f,2.0f,1.75f},{1.7f,1.8f,0.65f},false);
    b.box(b.mesh.interior,{2.65f,2.0f,1.75f},{1.7f,1.8f,0.65f},false);
    b.cylinder_y(b.mesh.interior,{0.2f,1.85f,-1.65f},1.55f,0.75f,18,false);
    for(int i=0;i<8;++i){
        const float a=2.0f*PI*(float)i/8.0f;
        b.cylinder_y(b.mesh.interior,{0.2f+0.52f*std::cos(a),2.55f,-1.65f+0.52f*std::sin(a)},0.35f,0.07f,6,false);
    }
    b.box(b.mesh.interior,{3.95f,1.75f,-1.55f},{0.85f,1.55f,0.65f},false);
    b.cylinder_between(b.mesh.interior,{-1.8f,2.0f,-1.6f},{-1.15f,2.75f,-1.1f},0.10f,8,true,false);
    b.cylinder_between(b.mesh.interior,{-1.15f,2.75f,-1.1f},{-0.6f,2.25f,-0.7f},0.08f,8,true,false);
    for(float x:{-3.6f,-1.2f,1.2f,3.6f}) ceiling_light(b,{x,4.85f,-1.7f},{x,4.85f,1.7f});
}

} // namespace game::habitats
