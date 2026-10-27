#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_crew_quarters_inner(Builder& b){
    // Interior center aisle and bunk modules.
    b.box(b.mesh.interior,{0,0.95f,0},{9.4f,0.16f,3.4f},false);
    for(float x:{-3.5f,-1.2f,1.2f,3.5f}){
        b.box(b.mesh.interior,{x,1.45f,-1.45f},{1.65f,0.55f,0.72f},false);
        b.box(b.mesh.interior,{x,2.25f,-1.45f},{1.65f,0.55f,0.72f},false);
        b.box(b.mesh.interior,{x,1.45f,1.45f},{1.65f,0.55f,0.72f},false);
        b.box(b.mesh.interior,{x,2.25f,1.45f},{1.65f,0.55f,0.72f},false);
    }

    // Personal lockers, fold-down desks, communal table and hygiene/service pods.
    locker_bank(b,{-2.9f,1.95f,-1.85f},4,0.58f);
    locker_bank(b,{2.2f,1.95f,-1.85f},4,0.58f);
    for(float x:{-3.4f,-1.15f,1.15f,3.4f}){
        b.box(b.mesh.interior,{x,1.15f,0.65f},{1.15f,0.10f,0.55f},false);
        stool(b,{x,0.75f,1.20f});
    }
    b.box(b.mesh.interior,{0,1.10f,0},{2.1f,0.16f,1.05f},false);
    for(Vec3 p:std::initializer_list<Vec3>{{-1.25f,0.75f,-0.6f},{1.25f,0.75f,-0.6f},{-1.25f,0.75f,0.6f},{1.25f,0.75f,0.6f}}) stool(b,p);
    b.box(b.mesh.interior,{-4.4f,1.65f,1.65f},{0.85f,1.7f,0.75f},false);
    b.box(b.mesh.interior,{4.4f,1.65f,1.65f},{0.85f,1.7f,0.75f},false);
    for(float x:{-3.2f,0.0f,3.2f}) ceiling_light(b,{x,4.25f,-0.8f},{x,4.25f,0.8f});
}

} // namespace game::habitats
