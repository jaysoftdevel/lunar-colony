#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_oxygen_dome_inner(Builder& b){
    const float floor=b.mesh.walkable_floor_y;
    b.cylinder_y(b.mesh.interior,{0,floor-.06f,0},.12f,4.35f,24,false);
    // Separate electrolysis stations leave space along both axes.
    for(float x:{-1.75f,1.75f}){
        for(float z:{-1.62f,1.62f}){
            bench(b,{x,floor,z},1.20f,.87f,.59f);
            for(float dx:{-.30f,.30f})tank(b,{x+dx,floor+.59f,z},1.12f,.21f);
            b.cylinder_between(b.mesh.interior,{x-.30f,floor+1.69f,z},{x+.30f,floor+1.69f,z},.055f,8,true,false);
        }
        ceiling_light(b,{x,3.72f,-1.65f},{x,3.72f,1.65f});
    }
}

} // namespace game::habitats
