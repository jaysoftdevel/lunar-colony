#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_generators.hpp"

namespace game::habitats {
HabitatMesh make_bio_dome(){
    Builder b;
    b.mesh.footprint_x=13.5f; b.mesh.footprint_z=8.0f; b.mesh.height=6.3f;
    b.box(b.mesh.foundation,{0,0.62f,0},{10.5f,1.25f,6.6f});
    b.half_cylinder_x(b.mesh.glazing,{0,1.20f,0},10.5f,3.3f,14,8,false);
    // Repeated greenhouse arch ribs.
    for(float x:{-5.0f,-3.3f,-1.65f,0.0f,1.65f,3.3f,5.0f}) b.arch_rib_x(b.mesh.details,x,{0,1.20f,0},3.31f,0.085f,16);
    // Longitudinal frame rails.
    for(float a:{0.18f,0.42f,0.66f,0.84f}){
        const float theta=PI*a;
        const float y=1.20f+std::sin(theta)*3.31f;
        const float z=std::cos(theta)*3.31f;
        b.cylinder_between(b.mesh.details,{-5.25f,y,z},{5.25f,y,z},0.07f,6,true,true);
    }
    // Interior crop racks visible as actual geometry.
    for(float z:{-1.7f,0.0f,1.7f}){
        for(float y:{1.25f,2.05f}){
            b.box(b.mesh.interior,{-0.5f,y,z},{8.3f,0.14f,0.75f},false);
            for(float x:{-3.8f,-1.9f,0.0f,1.9f,3.8f}) b.cylinder_y(b.mesh.interior,{x,y+0.35f,z},0.65f,0.06f,6,false);
        }
    }
    // Environmental control column and ducting.
    b.cylinder_y(b.mesh.details,{0,3.1f,0},3.6f,0.38f,12,true);
    b.cylinder_between(b.mesh.details,{-4.7f,4.2f,0},{4.7f,4.2f,0},0.12f,8,true,true);
    // Entry/service module on +X.
    b.box(b.mesh.shell,{6.05f,2.05f,0},{1.7f,3.5f,3.1f});
    b.torus_x(b.mesh.details,{6.92f,2.05f,0},0.58f,0.09f,16,6);
    b.cylinder_x(b.mesh.details,{6.97f,2.05f,0},0.12f,0.48f,14,true);
    b.box(b.mesh.details,{5.98f,2.15f,1.8f},{0.7f,1.2f,0.42f});
    // Support legs.
    b.supports(-4.2f,4.1f,2.65f,0.78f);
    b.supports(-4.2f,4.1f,-2.65f,0.78f);
    b.mesh.entrances.push_back({{7.00f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    return std::move(b.mesh);
}

} // namespace game::habitats
