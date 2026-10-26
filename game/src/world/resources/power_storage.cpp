#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_power_storage(){
    Builder b;
    b.mesh.footprint_x=10.0f; b.mesh.footprint_z=7.0f; b.mesh.height=5.8f;

    b.box(b.mesh.foundation,{0,0.45f,0},{8.5f,0.90f,5.6f});
    for(Vec3 p:std::initializer_list<Vec3>{{-4.6f,0,-2.6f},{4.6f,0,-2.6f},{-4.6f,0,2.6f},{4.6f,0,2.6f}}) b.foot(p,0.44f,0.18f);

    // Four battery cabinets with cooling gaps.
    for(int i=0;i<4;++i){
        const float x=-3.15f+2.10f*(float)i;
        b.box(b.mesh.body,{x,2.65f,0},{1.70f,3.80f,4.35f});
        b.box(b.mesh.details,{x,2.65f,2.22f},{1.42f,3.25f,0.16f});
        for(int fin=0;fin<7;++fin){
            const float fy=1.45f+0.36f*(float)fin;
            b.line({x-0.62f,fy,2.34f},{x+0.62f,fy,2.34f});
        }
    }
    // Top structural truss and power buses.
    b.box(b.mesh.details,{0,5.05f,-2.15f},{8.35f,0.28f,0.28f});
    b.box(b.mesh.details,{0,5.05f,2.15f},{8.35f,0.28f,0.28f});
    for(float x:{-4.0f,4.0f}) b.box(b.mesh.details,{x,3.0f,0},{0.30f,4.0f,4.6f});
    for(int i=0;i<4;++i){
        const float z=-1.3f+0.85f*(float)i;
        b.cylinder_x(b.mesh.details,{0,0.98f,z},7.8f,0.08f,8,true);
    }
    b.box(b.mesh.details,{4.25f,2.1f,0},{0.75f,1.8f,2.1f});
    return std::move(b.mesh);
}
}
