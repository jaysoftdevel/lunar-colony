#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_module.hpp"

namespace game::habitats {
namespace {
HabitatEntranceAccess stairs(float x){
    Builder b;
    for(int i=0;i<4;++i) b.box(b.mesh.details,{x+0.40f*i,0.74f-0.17f*i,0},{0.72f,0.14f,2.2f});
    for(float z:{-1.25f,1.25f}) b.cylinder_between(b.mesh.details,{x-0.1f,1.45f,z},{x+1.25f,0.55f,z},0.05f,6,true,true);
    return {std::move(b.mesh.details),std::move(b.mesh.edges)};
}
}

void build_sick_bay_outer(Builder& b){
    b.mesh.footprint_x=13.0f; b.mesh.footprint_z=11.0f; b.mesh.height=8.2f;
    b.cylinder_y(b.mesh.foundation,{0,0.55f,0},1.1f,4.55f,24,true);

    // Low medical dome with broad observation glazing band.
    b.dome(b.mesh.shell,{0,1.0f,0},4.55f,8,24,true,0.78f);
    const float window_y=3.65f;
    for(int s=0;s<12;++s){
        const float a0=2*PI*(float)s/12.0f;
        const float a1=2*PI*(float)(s+1)/12.0f;
        const float r0=4.15f;
        Vec3 p0{r0*std::cos(a0),window_y,r0*std::sin(a0)};
        Vec3 p1{r0*std::cos(a1),window_y,r0*std::sin(a1)};
        b.cylinder_between(b.mesh.details,p0,p1,0.09f,6,true,false);
        b.line(p0,p1);
        Vec3 low0{3.65f*std::cos(a0),2.45f,3.65f*std::sin(a0)};
        b.cylinder_between(b.mesh.details,low0,p0,0.075f,6,true,false);
        b.line(low0,p0);
        Builder::tri(b.mesh.glazing,low0,p0,p1);
        Vec3 low1{3.65f*std::cos(a1),2.45f,3.65f*std::sin(a1)};
        Builder::tri(b.mesh.glazing,low0,p1,low1);
    }

    // Exterior medical support pods / life support tanks.
    for(float z:{-2.6f,-0.9f,0.9f,2.6f}) b.cylinder_y(b.mesh.details,{-4.0f,2.35f,z},2.8f,0.46f,12,true);
    b.box(b.mesh.details,{-4.2f,1.55f,-3.65f},{1.35f,1.35f,0.55f});
    b.cylinder_y(b.mesh.details,{0,6.1f,0},1.35f,0.82f,16,true);
    for(Vec3 c:std::initializer_list<Vec3>{{-2.5f,5.25f,-1.8f},{2.6f,5.0f,1.9f}}) b.cylinder_y(b.mesh.details,c,0.95f,0.35f,12,true);

    // Medical airlock.
    b.box(b.mesh.shell,{4.85f,2.35f,0},{1.75f,3.95f,3.55f});
    b.torus_x(b.mesh.details,{5.78f,2.35f,0},0.65f,0.10f,18,6);
    b.cylinder_x(b.mesh.details,{5.84f,2.35f,0},0.15f,0.55f,16,true);
    b.mesh.entrances.push_back({{5.94f,1.18f,0},{1,0,0}});
    b.mesh.entrance_access.push_back(stairs(5.97f));

    for(float a:{0.5f,2.05f,3.65f,5.2f}){
        Vec3 top{3.85f*std::cos(a),0.8f,3.85f*std::sin(a)};
        Vec3 foot{4.7f*std::cos(a),0.15f,4.7f*std::sin(a)};
        b.cylinder_between(b.mesh.details,top,foot,0.14f,8,true,true);
        b.cylinder_y(b.mesh.details,{foot.x,0.09f,foot.z},0.18f,0.36f,10,true);
    }
}
}
