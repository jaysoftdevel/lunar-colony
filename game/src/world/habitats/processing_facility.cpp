#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_generators.hpp"
#include "game/world/habitats/habitat_interiors.hpp"

namespace game::habitats {
namespace {
HabitatEntranceAccess access_stairs(float x){
    Builder b;
    for(int i=0;i<4;++i){
        const float px=x+0.42f*(float)i;
        b.box(b.mesh.details,{px,0.72f-0.16f*(float)i,0},{0.72f,0.15f,2.25f});
    }
    for(float z:{-1.30f,1.30f})
        b.cylinder_between(b.mesh.details,{x-0.12f,1.50f,z},{x+1.45f,0.62f,z},0.055f,6,true,true);
    return {std::move(b.mesh.details),std::move(b.mesh.edges)};
}
}

HabitatMesh make_processing_facility(){
    Builder b;
    b.mesh.footprint_x=17.0f; b.mesh.footprint_z=12.0f; b.mesh.height=9.2f;

    // Heavy industrial foundation and central pressure vessel from the concept.
    b.box(b.mesh.foundation,{0,0.55f,0},{11.8f,1.10f,7.4f});
    b.cylinder_x(b.mesh.shell,{0.4f,3.55f,0},9.7f,2.75f,20,true);
    for(float x:{-3.6f,-1.4f,0.8f,3.0f}) b.torus_x(b.mesh.details,{x,3.55f,0},2.78f,0.09f,20,6);

    // Left feed hoppers and conveyor assembly.
    for(int i=0;i<3;++i){
        const float z=-2.55f+2.55f*(float)i;
        b.box(b.mesh.details,{-5.65f,3.05f,z},{2.15f,3.25f,1.95f});
        b.box(b.mesh.details,{-5.65f,1.28f,z},{1.35f,0.70f,1.35f});
        b.cylinder_y(b.mesh.details,{-5.65f,0.74f,z},0.72f,0.34f,10,true);
    }
    b.box(b.mesh.details,{-6.55f,0.72f,0},{3.8f,0.52f,2.5f});
    for(int i=0;i<5;++i){
        const float t=(float)i/4.0f;
        const Vec3 a{-7.7f+2.8f*t,0.78f,-1.1f};
        const Vec3 c{-5.0f+2.8f*t,1.65f,-1.1f};
        b.cylinder_between(b.mesh.details,a,c,0.075f,6,true,true);
    }

    // Refining columns, filters and pipe manifolds on the right side.
    for(int i=0;i<3;++i){
        const float z=-2.25f+2.25f*(float)i;
        b.cylinder_y(b.mesh.details,{4.65f,3.35f,z},4.5f,0.62f,14,true);
        b.cylinder_y(b.mesh.details,{3.55f,2.75f,z},3.2f,0.28f,10,true);
    }
    for(int i=0;i<4;++i){
        const float z=-2.1f+1.4f*(float)i;
        b.cylinder_y(b.mesh.details,{2.55f,2.15f,z},2.6f,0.22f,10,true);
    }
    b.box(b.mesh.details,{5.0f,5.65f,-2.95f},{1.6f,4.1f,0.55f});
    for(int i=0;i<8;++i){
        const float x=4.45f+0.16f*(float)i;
        b.line({x,3.75f,-3.28f},{x,7.45f,-3.28f});
    }

    // Roof stacks / scrubbers.
    for(const Vec3 c: {Vec3{-3.15f,7.05f,-1.8f},Vec3{-0.8f,7.45f,1.2f},Vec3{2.5f,7.15f,-0.7f}}){
        b.cylinder_y(b.mesh.details,c,2.35f,0.48f,12,true);
        b.cylinder_y(b.mesh.details,{c.x,c.y+1.25f,c.z},0.28f,0.66f,12,true);
    }

    // Processing interior: furnace cells, ingot line and center service deck.
    b.box(b.mesh.interior,{0,1.05f,0},{8.6f,0.18f,5.2f},false);
    for(float x:{-2.6f,-0.9f,0.9f,2.6f}){
        b.box(b.mesh.interior,{x,2.15f,-1.35f},{1.15f,1.55f,1.25f},false);
        b.box(b.mesh.interior,{x,1.35f,1.35f},{1.35f,0.38f,1.1f},false);
    }

    // Pressure entry, hatch and access stairs.
    b.box(b.mesh.shell,{5.45f,3.05f,0},{1.70f,4.35f,3.65f});
    b.box(b.mesh.details,{6.32f,3.05f,0},{0.18f,3.55f,3.05f});
    b.torus_x(b.mesh.details,{6.45f,3.05f,0},0.66f,0.10f,18,6);
    b.cylinder_x(b.mesh.details,{6.52f,3.05f,0},0.16f,0.56f,16,true);
    b.mesh.entrances.push_back({{6.60f,1.18f,0},{1,0,0}});
    b.mesh.entrance_access.push_back(access_stairs(6.63f));

    b.supports(-4.4f,3.8f,2.7f,0.95f);
    b.supports(-4.4f,3.8f,-2.7f,0.95f);
    interiors::add_processing_facility(b);
    return std::move(b.mesh);
}
}
