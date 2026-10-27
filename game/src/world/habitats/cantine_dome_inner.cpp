#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_cantine_dome_inner(Builder& b){
    // Interior communal tables/core visible through the dome.
    b.cylinder_y(b.mesh.interior,{0,2.15f,0},2.2f,0.30f,12,false);
    b.cylinder_y(b.mesh.interior,{0,1.30f,0},0.22f,2.0f,18,false);
    b.cylinder_y(b.mesh.interior,{0,2.95f,0},0.18f,1.65f,18,false);
    for(float a:{0.0f,1.5708f,3.1416f,4.7124f}){
        const Vec3 c{2.35f*std::cos(a),1.28f,2.35f*std::sin(a)};
        b.cylinder_y(b.mesh.interior,c,0.16f,0.82f,14,false);
        b.cylinder_y(b.mesh.interior,{c.x,0.72f,c.z},0.95f,0.13f,8,false);
    }

    // Dining clusters, galley/service counter and food storage.
    for(float a:{0.0f,2.0944f,4.1888f}){
        const Vec3 c{2.45f*std::cos(a),1.38f,2.45f*std::sin(a)};
        b.cylinder_y(b.mesh.interior,c,0.16f,0.86f,16,false);
        for(float da:{-0.65f,0.0f,0.65f}){
            const float aa=a+da;
            stool(b,{c.x+1.15f*std::cos(aa),0.85f,c.z+1.15f*std::sin(aa)});
        }
    }
    b.box(b.mesh.interior,{-2.7f,1.35f,-2.55f},{3.2f,0.82f,0.68f},false);
    b.box(b.mesh.interior,{-3.55f,2.25f,-2.78f},{1.15f,1.35f,0.38f},false);
    b.box(b.mesh.interior,{-2.25f,2.25f,-2.78f},{1.15f,1.35f,0.38f},false);
    b.box(b.mesh.interior,{-3.2f,1.52f,2.7f},{2.1f,1.4f,0.45f},false);
    console(b,{2.7f,1.35f,-2.5f},1.25f,-1.0f);
    for(float x:{-2.4f,0.0f,2.4f}) ceiling_light(b,{x,4.55f,-1.4f},{x,4.55f,1.4f});
}

} // namespace game::habitats
