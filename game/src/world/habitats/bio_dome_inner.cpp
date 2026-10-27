#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_bio_dome_inner(Builder& b){
    // Interior crop racks visible as actual geometry.
    for(float z:{-1.7f,0.0f,1.7f}){
        for(float y:{1.25f,2.05f}){
            b.box(b.mesh.interior,{-0.5f,y,z},{8.3f,0.14f,0.75f},false);
            for(float x:{-3.8f,-1.9f,0.0f,1.9f,3.8f}) b.cylinder_y(b.mesh.interior,{x,y+0.35f,z},0.65f,0.06f,6,false);
        }
    }

    // Denser hydroponics with irrigation plumbing, nutrient tanks and potting station.
    for(float z:{-2.35f,-0.8f,0.8f,2.35f}){
        for(float x:{-3.9f,-2.3f,-0.7f,0.9f,2.5f,4.1f}){
            b.box(b.mesh.interior,{x,1.52f,z},{1.15f,0.12f,0.55f},false);
            b.box(b.mesh.interior,{x,2.35f,z},{1.15f,0.12f,0.55f},false);
            for(float dx:{-0.35f,0.0f,0.35f}) b.cylinder_y(b.mesh.interior,{x+dx,1.90f,z},0.64f,0.055f,6,false);
        }
        b.cylinder_between(b.mesh.interior,{-4.6f,1.18f,z},{4.6f,1.18f,z},0.055f,6,true,false);
    }
    tank(b,{-4.55f,0.95f,2.35f},2.15f,0.42f);
    tank(b,{-4.55f,0.95f,-2.35f},2.15f,0.42f);
    b.box(b.mesh.interior,{4.25f,1.40f,0},{1.25f,0.75f,2.2f},false);
    console(b,{4.2f,2.15f,0},1.0f,1.0f);
    for(float x:{-3.7f,-1.8f,0.0f,1.8f,3.7f}) ceiling_light(b,{x,3.82f,-2.5f},{x,3.82f,2.5f});
}

} // namespace game::habitats
