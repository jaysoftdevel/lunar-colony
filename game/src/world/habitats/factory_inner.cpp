#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

namespace {
void robot_arm(Builder& b,eng::Vec3 base,float facing){
    using namespace eng;
    const Vec3 p0=base;
    const Vec3 p1=base+Vec3{0,0.55f,0};
    const Vec3 p2=p1+Vec3{0.55f*facing,0.65f,0.15f};
    const Vec3 p3=p2+Vec3{0.50f*facing,0.35f,-0.10f};
    b.cylinder_y(b.mesh.interior,p0+Vec3{0,0.2f,0},0.40f,0.28f,10,false);
    b.cylinder_between(b.mesh.interior,p1,p2,0.15f,8,true,false);
    b.cylinder_between(b.mesh.interior,p2,p3,0.13f,8,true,false);
    b.cylinder_y(b.mesh.interior,p2,0.26f,0.24f,10,false);
    b.box(b.mesh.interior,p3,{0.28f,0.25f,0.28f},false);
}
}

void build_factory_inner(Builder& b){
    // Robotic production cells inside the assembly hall.
    b.box(b.mesh.interior,{-2.6f,1.05f,0},{5.7f,0.18f,5.6f},false);
    robot_arm(b,{-4.25f,1.2f,-1.35f},1.0f);
    robot_arm(b,{-2.55f,1.2f,1.25f},1.0f);
    robot_arm(b,{-0.75f,1.2f,-1.15f},-1.0f);
    for(float x:{-4.2f,-2.6f,-1.0f}) b.box(b.mesh.interior,{x,1.25f,2.15f},{1.2f,0.45f,0.9f},false);

    // Additional assembly conveyor, component shelving, inspection station and tool cabinets.
    b.box(b.mesh.interior,{-2.6f,1.35f,0},{5.3f,0.20f,0.85f},false);
    for(int i=0;i<7;++i) b.cylinder_z(b.mesh.interior,{-4.9f+0.75f*i,1.52f,0},0.92f,0.10f,8,false);
    for(float x:{-4.5f,-2.8f,-1.1f}) for(float y:{1.65f,2.35f,3.05f})
        b.box(b.mesh.interior,{x,y,2.7f},{1.25f,0.18f,0.55f},false);
    b.box(b.mesh.interior,{0.15f,1.65f,-2.65f},{1.8f,1.25f,0.65f},false);
    console(b,{-0.1f,2.6f,-2.65f},1.25f,-1.0f);
    b.cylinder_y(b.mesh.interior,{-4.7f,2.25f,-2.45f},2.3f,0.35f,10,false);
    for(float x:{-4.4f,-2.2f,0.0f}) ceiling_light(b,{x,5.65f,-2.4f},{x,5.65f,2.4f});
}

} // namespace game::habitats
