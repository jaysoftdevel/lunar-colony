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
    const float floor=b.mesh.walkable_floor_y;
    b.box(b.mesh.interior,{0,floor-.08f,0},{12.6f,.16f,5.9f},false);
    for(float x:{-4.14f,-1.42f,1.28f,4.06f}){
        bench(b,{x,floor,-1.88f},1.31f,.92f,.54f);
        robot_arm(b,{x-.39f,floor+.54f,-1.95f},.62f);
        bench(b,{x,floor,1.89f},1.31f,.80f,.72f);
        for(float dx:{-.37f,0.f,.37f})
            b.cylinder_z(b.mesh.interior,{x+dx,floor+.82f,1.89f},.67f,.075f,8,false);
        ceiling_light(b,{x,4.78f,-1.58f},{x,4.78f,1.58f});
    }
}

} // namespace game::habitats
