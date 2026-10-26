#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_generators.hpp"
#include "game/world/habitats/habitat_interiors.hpp"

namespace game::habitats {
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
HabitatEntranceAccess stairs(float x){
    Builder b;
    for(int i=0;i<4;++i) b.box(b.mesh.details,{x+0.40f*(float)i,0.75f-0.17f*(float)i,0},{0.72f,0.15f,2.25f});
    return {std::move(b.mesh.details),std::move(b.mesh.edges)};
}
}

HabitatMesh make_factory(){
    Builder b;
    b.mesh.footprint_x=18.0f; b.mesh.footprint_z=12.0f; b.mesh.height=9.0f;
    b.box(b.mesh.foundation,{0,0.55f,0},{12.8f,1.10f,8.4f});

    // Assembly hall: open framed front bay and rounded pressurized rear module.
    b.box(b.mesh.shell,{-2.5f,3.35f,0},{7.0f,5.3f,7.4f});
    b.cylinder_x(b.mesh.shell,{3.25f,3.40f,0},6.2f,2.75f,18,true);
    for(float x:{1.2f,3.25f,5.25f}) b.torus_x(b.mesh.details,{x,3.4f,0},2.78f,0.09f,18,6);

    // Big front structural portal around the visible robot bay.
    b.box(b.mesh.details,{-5.75f,3.6f,-3.8f},{0.45f,6.1f,0.45f});
    b.box(b.mesh.details,{0.35f,3.6f,-3.8f},{0.45f,6.1f,0.45f});
    b.box(b.mesh.details,{-2.7f,6.55f,-3.8f},{6.5f,0.42f,0.45f});
    for(float x:{-5.15f,-3.55f,-1.95f,-0.35f}) b.box(b.mesh.details,{x,1.35f,-3.55f},{1.1f,0.55f,0.85f});

    // Robotic production cells inside the assembly hall.
    b.box(b.mesh.interior,{-2.6f,1.05f,0},{5.7f,0.18f,5.6f},false);
    robot_arm(b,{-4.25f,1.2f,-1.35f},1.0f);
    robot_arm(b,{-2.55f,1.2f,1.25f},1.0f);
    robot_arm(b,{-0.75f,1.2f,-1.15f},-1.0f);
    for(float x:{-4.2f,-2.6f,-1.0f}) b.box(b.mesh.interior,{x,1.25f,2.15f},{1.2f,0.45f,0.9f},false);

    // Storage racks / material crates at the left end.
    for(int row=0;row<3;++row) for(int col=0;col<2;++col)
        b.box(b.mesh.details,{-6.55f,1.25f+0.8f*(float)row,-2.2f+4.4f*(float)col},{1.1f,0.62f,1.35f});

    // Gantry crane above the production hall.
    b.box(b.mesh.details,{-2.9f,7.15f,0},{7.4f,0.34f,0.40f});
    b.box(b.mesh.details,{-5.95f,5.15f,0},{0.34f,4.2f,0.40f});
    b.cylinder_between(b.mesh.details,{-1.2f,6.95f,0},{-1.2f,4.65f,0},0.10f,8,true,true);
    b.box(b.mesh.details,{-1.2f,4.4f,0},{0.75f,0.45f,0.75f});

    // Roof processing tanks, radiator blocks and manifold pipes.
    for(float x:{-1.8f,0.1f}) b.cylinder_y(b.mesh.details,{x,7.0f,1.5f},2.5f,0.60f,14,true);
    b.box(b.mesh.details,{3.7f,7.0f,-1.3f},{2.0f,1.9f,0.65f});
    for(int i=0;i<8;++i) b.line({2.85f+0.23f*i,6.25f,-1.67f},{2.85f+0.23f*i,7.75f,-1.67f});
    b.cylinder_between(b.mesh.details,{-1.8f,8.25f,1.5f},{3.0f,7.75f,0.8f},0.11f,8,true,true);

    // Pressure vestibule.
    b.box(b.mesh.shell,{6.45f,3.05f,0},{1.65f,4.3f,3.55f});
    b.torus_x(b.mesh.details,{7.32f,3.05f,0},0.65f,0.10f,18,6);
    b.cylinder_x(b.mesh.details,{7.38f,3.05f,0},0.14f,0.55f,16,true);
    b.mesh.entrances.push_back({{7.47f,1.18f,0},{1,0,0}});
    b.mesh.entrance_access.push_back(stairs(7.5f));

    b.supports(-5.3f,4.7f,3.0f,0.95f);
    b.supports(-5.3f,4.7f,-3.0f,0.95f);
    interiors::add_factory(b);
    return std::move(b.mesh);
}
}
