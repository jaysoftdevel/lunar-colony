#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_water_driller(){
    Builder b;
    b.mesh.footprint_x=9.0f; b.mesh.footprint_z=9.0f; b.mesh.height=10.5f;

    b.box(b.mesh.foundation,{0,0.42f,0},{6.0f,0.84f,5.0f});
    for(Vec3 p:std::initializer_list<Vec3>{{-3.4f,0,-2.7f},{3.4f,0,-2.7f},{-3.4f,0,2.7f},{3.4f,0,2.7f}}){
        b.foot(p,0.55f,0.20f);
        b.cylinder_between(b.mesh.details,{p.x,0.20f,p.z},{p.x*0.58f,1.05f,p.z*0.58f},0.18f,8,true,true);
    }

    // Drill mast and guide rails.
    b.box(b.mesh.body,{1.3f,5.1f,0},{1.25f,9.2f,1.7f});
    b.box(b.mesh.details,{1.3f,9.65f,0},{1.75f,0.45f,2.0f});
    for(float z:{-0.78f,0.78f}) for(float x:{0.78f,1.82f}) b.cylinder_y(b.mesh.details,{x,5.2f,z},8.6f,0.09f,8,true);

    // Main drill shaft + auger helix.
    b.cylinder_y(b.mesh.body,{1.3f,4.55f,0},7.4f,0.31f,14,true);
    Vec3 prev{1.67f,1.35f,0};
    const int turns=28;
    for(int i=1;i<=turns;++i){
        const float t=(float)i/(float)turns;
        const float a=t*PI*8.0f;
        Vec3 cur{1.3f+std::cos(a)*0.43f,1.35f+t*4.9f,std::sin(a)*0.43f};
        b.cylinder_between(b.mesh.details,prev,cur,0.055f,6,true,false);
        b.line(prev,cur);
        prev=cur;
    }

    // Service deck, pump package and piping.
    b.box(b.mesh.body,{-1.45f,1.75f,0},{2.55f,2.2f,3.8f});
    b.box(b.mesh.details,{-2.35f,3.15f,-0.85f},{1.1f,0.95f,1.1f});
    b.cylinder_x(b.mesh.details,{-1.5f,2.95f,1.15f},2.1f,0.30f,10,true);
    b.cylinder_between(b.mesh.details,{-1.8f,3.25f,0.7f},{0.82f,7.5f,0.7f},0.12f,8,true,true);
    b.cylinder_between(b.mesh.details,{-1.8f,3.25f,-0.7f},{0.82f,7.5f,-0.7f},0.12f,8,true,true);
    return std::move(b.mesh);
}
}
