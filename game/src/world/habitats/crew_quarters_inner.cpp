#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_crew_quarters_inner(Builder& b){
    // Interior center aisle and bunk modules.
    b.box(b.mesh.interior,{0,0.95f,0},{9.4f,0.16f,3.4f},false);
    for(float x:{-1.15f,1.15f,-3.45f,3.45f})for(float side:{-1.f,1.f}){
        const float z=side*1.48f;
        b.mesh.crew_bunks.push_back({{x,1.03f,side*.47f},{x-.78f,1.59f,z}});
    }
    // Keep the full central aisle clear between both exterior docking faces.
    locker_bank(b,{-.56f,1.98f,-1.98f},2,.58f);
    locker_bank(b,{-.56f,1.98f,1.98f},2,.58f);
    b.box(b.mesh.interior,{-4.4f,1.65f,1.65f},{0.85f,1.7f,0.75f},false);
    b.box(b.mesh.interior,{4.4f,1.65f,1.65f},{0.85f,1.7f,0.75f},false);
    for(float x:{-3.2f,0.0f,3.2f}) ceiling_light(b,{x,4.25f,-0.8f},{x,4.25f,0.8f});
}

} // namespace game::habitats
