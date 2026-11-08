#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_manufactory_inner(Builder& b){
    const float floor=b.mesh.walkable_floor_y;
    b.box(b.mesh.interior,{0,floor-.09f,0},{15.2f,.18f,6.0f},false);
    for(float x:{-5.25f,-2.35f,1.52f,4.52f}){
        bench(b,{x,floor,-1.97f},1.38f,1.03f,.72f);
        b.box(b.mesh.interior,{x,floor+1.28f,-1.97f},{1.17f,1.12f,.87f},false);
        b.cylinder_y(b.mesh.interior,{x,floor+1.99f,-1.97f},.30f,.37f,10,false);
        bench(b,{x,floor,1.98f},1.43f,.91f,.70f);
        for(float dx:{-.34f,.34f})b.box(b.mesh.interior,{x+dx,floor+.89f,1.98f},{.53f,.38f,.63f},false);
        ceiling_light(b,{x,4.98f,-1.75f},{x,4.98f,1.75f});
    }
    // The gantry and tool head cross overhead with pedestrian clearance.
    b.box(b.mesh.interior,{-3.8f,4.71f,0},{4.4f,.25f,.36f},false);
    b.box(b.mesh.interior,{-3.8f,4.33f,0},{.62f,.58f,.55f},false);
}

} // namespace game::habitats
