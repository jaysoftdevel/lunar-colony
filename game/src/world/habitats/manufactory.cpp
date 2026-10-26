#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_generators.hpp"
#include "game/world/habitats/habitat_interiors.hpp"

namespace game::habitats {
namespace {
void crane_arm(Builder& b,eng::Vec3 root,float side){
    using namespace eng;
    Vec3 a=root;
    Vec3 c=a+Vec3{1.85f*side,0.25f,0};
    Vec3 d=c+Vec3{1.45f*side,-0.9f,0};
    b.cylinder_y(b.mesh.details,a,0.55f,0.34f,12,true);
    b.cylinder_between(b.mesh.details,a,c,0.24f,10,true,true);
    b.cylinder_between(b.mesh.details,c,d,0.22f,10,true,true);
    b.cylinder_y(b.mesh.details,c,0.42f,0.38f,12,true);
    b.box(b.mesh.details,d,{0.85f,0.55f,1.4f});
}
HabitatEntranceAccess stairs(float x){
    Builder b;
    for(int i=0;i<5;++i) b.box(b.mesh.details,{x+0.38f*i,0.82f-0.15f*i,0},{0.7f,0.14f,2.35f});
    return {std::move(b.mesh.details),std::move(b.mesh.edges)};
}
}

HabitatMesh make_manufactory(){
    Builder b;
    b.mesh.footprint_x=21.0f; b.mesh.footprint_z=14.0f; b.mesh.height=10.8f;
    b.box(b.mesh.foundation,{0,0.62f,0},{15.2f,1.24f,9.8f});

    // Massive fabrication chamber plus long production pressure cylinder.
    b.box(b.mesh.shell,{-3.4f,3.9f,0},{8.6f,6.2f,8.5f});
    b.cylinder_x(b.mesh.shell,{4.2f,3.65f,0},8.8f,3.0f,20,true);
    for(float x:{1.2f,3.5f,5.8f,7.8f}) b.torus_x(b.mesh.details,{x,3.65f,0},3.03f,0.10f,20,6);

    // Huge assembly opening / internal handling platform.
    b.box(b.mesh.interior,{-3.6f,1.15f,0},{7.4f,0.22f,6.5f},false);
    b.box(b.mesh.details,{-7.45f,4.0f,-4.45f},{0.5f,6.5f,0.5f});
    b.box(b.mesh.details,{0.15f,4.0f,-4.45f},{0.5f,6.5f,0.5f});
    b.box(b.mesh.details,{-3.65f,7.1f,-4.45f},{8.1f,0.48f,0.5f});
    b.box(b.mesh.details,{-3.65f,1.3f,-4.45f},{8.1f,0.42f,0.5f});

    // Fabrication gantry and suspended tooling visible inside.
    b.box(b.mesh.interior,{-3.6f,6.3f,0},{6.8f,0.22f,0.42f},false);
    b.cylinder_between(b.mesh.interior,{-3.6f,6.2f,0},{-3.6f,4.45f,0},0.16f,8,true,false);
    b.box(b.mesh.interior,{-3.6f,4.15f,0},{0.8f,0.65f,0.8f},false);
    for(float x:{-5.8f,-3.5f,-1.2f}){
        b.box(b.mesh.interior,{x,1.55f,-2.0f},{1.6f,0.7f,1.2f},false);
        b.cylinder_y(b.mesh.interior,{x,2.65f,1.75f},2.25f,0.35f,10,false);
    }

    // Large external crane arms and cargo fixtures.
    crane_arm(b,{1.1f,9.2f,2.75f},1.0f);
    crane_arm(b,{5.0f,8.9f,-2.8f},1.0f);
    for(int row=0;row<2;++row) for(int col=0;col<3;++col)
        b.box(b.mesh.details,{-7.15f+1.45f*col,1.55f+0.78f*row,3.7f},{1.15f,0.60f,1.2f});

    // Dense service piping / tanks / rooftop process drums.
    for(float x:{-1.5f,1.4f,4.1f}) b.cylinder_y(b.mesh.details,{x,8.15f,-1.0f},2.0f,0.55f,14,true);
    for(int i=0;i<5;++i){
        float z=-2.7f+1.35f*i;
        b.cylinder_between(b.mesh.details,{0.3f,6.2f,z},{5.8f,6.45f,z},0.09f,8,true,true);
    }
    b.box(b.mesh.details,{6.6f,7.65f,2.4f},{1.8f,1.8f,0.65f});

    // Pressure access on the right production module.
    b.box(b.mesh.shell,{8.75f,3.05f,0},{1.65f,4.35f,3.65f});
    b.torus_x(b.mesh.details,{9.62f,3.05f,0},0.66f,0.10f,18,6);
    b.cylinder_x(b.mesh.details,{9.69f,3.05f,0},0.15f,0.56f,16,true);
    b.mesh.entrances.push_back({{9.78f,1.18f,0},{1,0,0}});
    b.mesh.entrance_access.push_back(stairs(9.81f));

    b.supports(-6.4f,6.9f,3.4f,1.05f);
    b.supports(-6.4f,6.9f,-3.4f,1.05f);
    interiors::add_manufactory(b);
    return std::move(b.mesh);
}
}
