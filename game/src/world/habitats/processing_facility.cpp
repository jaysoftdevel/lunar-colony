#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_module.hpp"

namespace game::habitats {
void build_processing_facility_outer(Builder& b){
    b.mesh.footprint_x=17.0f; b.mesh.footprint_z=12.0f; b.mesh.height=9.2f;

    // Heavy industrial foundation and central pressure vessel from the concept.
    b.box(b.mesh.foundation,{0,0.55f,0},{11.8f,1.10f,7.4f});
    b.cylinder_x(b.mesh.shell,{0.4f,3.55f,0},9.7f,2.75f,20,true);
    for(float x:{-3.6f,-1.4f,0.8f,3.0f}) b.torus_x(b.mesh.details,{x,3.55f,0},2.78f,0.09f,20,6);

    // Left feed hoppers and conveyor assembly.
    for(int i=0;i<3;++i){
        const float z=-2.55f+2.55f*(float)i;
        b.box(b.mesh.details,{-6.45f,3.05f,z},{2.15f,3.25f,1.95f});
        b.box(b.mesh.details,{-6.45f,1.28f,z},{1.35f,0.70f,1.35f});
        b.cylinder_y(b.mesh.details,{-6.45f,0.74f,z},0.72f,0.34f,10,true);
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
        const float z=-2.55f+2.55f*(float)i;
        b.cylinder_y(b.mesh.details,{5.80f,3.35f,z},4.5f,0.62f,14,true);
        b.cylinder_y(b.mesh.details,{5.46f,2.75f,z},3.2f,0.28f,10,true);
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

    b.supports(-4.4f,3.8f,2.7f,0.95f);
    b.supports(-4.4f,3.8f,-2.7f,0.95f);
}
}
