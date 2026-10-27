#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_module.hpp"

namespace game::habitats {
namespace {
HabitatEntranceAccess stairs(float x){
    Builder b;
    for(int i=0;i<4;++i) b.box(b.mesh.details,{x+0.40f*i,0.74f-0.17f*i,0},{0.72f,0.14f,2.2f});
    return {std::move(b.mesh.details),std::move(b.mesh.edges)};
}
void mast(Builder& b,eng::Vec3 p,float h){
    b.cylinder_y(b.mesh.details,{p.x,p.y+h*0.5f,p.z},h,0.09f,8,true);
    b.box(b.mesh.details,{p.x,p.y+h,p.z},{0.42f,0.35f,0.42f});
}
}

void build_laboratory_outer(Builder& b){
    b.mesh.footprint_x=15.0f; b.mesh.footprint_z=9.5f; b.mesh.height=9.0f;
    b.cylinder_x(b.mesh.shell,{0,3.0f,0},10.8f,2.55f,20,true);
    for(float x:{-4.2f,-2.1f,0.0f,2.1f,4.2f}) b.torus_x(b.mesh.details,{x,3.0f,0},2.58f,0.08f,20,6);

    // Large glazed research bay, following the blueprint's long observation windows.
    for(int i=0;i<4;++i){
        const float x=-3.7f+2.05f*(float)i;
        b.box(b.mesh.glazing,{x,3.15f,-2.54f},{1.72f,2.2f,0.10f},false);
        b.box(b.mesh.details,{x,3.15f,-2.60f},{1.88f,2.35f,0.10f});
    }
    // Put glazing back on top of the structural frames by drawing internal pane edges.
    for(int i=0;i<5;++i){
        const float x=-4.72f+2.05f*(float)i;
        b.cylinder_y(b.mesh.details,{x,3.15f,-2.72f},2.55f,0.065f,6,true);
    }

    // Roof cupola, sensor masts and dish-like instrument.
    b.cylinder_y(b.mesh.shell,{-2.6f,6.15f,0},1.6f,1.35f,14,true);
    for(int i=0;i<6;++i){
        const float a=2*PI*i/6.0f;
        b.box(b.mesh.glazing,{-2.6f+1.05f*std::cos(a),6.15f,1.05f*std::sin(a)},{0.08f,0.75f,0.75f},false);
    }
    mast(b,{0.5f,5.45f,-1.35f},2.2f);
    mast(b,{2.0f,5.35f,1.1f},2.8f);
    mast(b,{3.2f,5.3f,-0.5f},1.8f);
    b.cylinder_between(b.mesh.details,{1.6f,7.0f,-1.9f},{2.55f,7.65f,-2.5f},0.08f,8,true,true);
    b.cylinder_y(b.mesh.details,{2.75f,7.8f,-2.65f},0.18f,0.9f,18,true);

    // External sample tanks / equipment cabinets.
    for(float z:{-1.7f,0.0f,1.7f}) b.cylinder_y(b.mesh.details,{4.35f,2.5f,z},2.7f,0.42f,12,true);
    b.box(b.mesh.details,{3.85f,1.5f,-2.55f},{1.5f,1.1f,0.55f});

    // Airlock at the right end.
    b.box(b.mesh.shell,{5.85f,2.55f,0},{1.55f,4.0f,3.45f});
    b.torus_x(b.mesh.details,{6.68f,2.55f,0},0.64f,0.10f,18,6);
    b.cylinder_x(b.mesh.details,{6.74f,2.55f,0},0.15f,0.54f,16,true);
    b.mesh.entrances.push_back({{6.84f,1.18f,0},{1,0,0}});
    b.mesh.entrance_access.push_back(stairs(6.87f));
    b.supports(-4.4f,4.5f,2.2f,0.9f);
    b.supports(-4.4f,4.5f,-2.2f,0.9f);
}
}
