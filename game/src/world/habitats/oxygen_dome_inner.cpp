#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_oxygen_dome_inner(Builder& b){
    // Interior electrolyzer / buffer-tank cluster.
    b.box(b.mesh.interior,{0,1.10f,0},{5.6f,0.18f,5.6f},false);
    for(const Vec3 c: {Vec3{-1.25f,2.15f,-1.25f},Vec3{1.25f,2.15f,-1.25f},Vec3{-1.25f,2.15f,1.25f},Vec3{1.25f,2.15f,1.25f}})
        b.cylinder_y(b.mesh.interior,c,2.2f,0.46f,12,false);
    b.cylinder_y(b.mesh.interior,{0,2.45f,0},3.0f,0.34f,12,false);

    // Electrolyzer stacks, gas buffers, manifolds and operator stations.
    for(float z:{-2.1f,2.1f}){
        for(float x:{-2.2f,-0.75f,0.75f,2.2f})
            b.box(b.mesh.interior,{x,1.75f,z},{0.88f,1.65f,0.58f},false);
    }
    for(float a:{0.0f,1.5708f,3.1416f,4.7124f})
        tank(b,{2.85f*std::cos(a),1.05f,2.85f*std::sin(a)},2.25f,0.36f);
    b.torus_x(b.mesh.interior,{0,3.55f,0},1.45f,0.055f,18,6);
    for(float z:{-1.25f,1.25f}) console(b,{0,1.35f,z},1.35f,z<0?-1.0f:1.0f);
    for(float x:{-2.5f,0.0f,2.5f}) ceiling_light(b,{x,4.25f,-1.6f},{x,4.25f,1.6f});
}

} // namespace game::habitats
