#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_water_driller(){
    Builder b;
    b.mesh.footprint_x=9.5f; b.mesh.footprint_z=8.0f; b.mesh.height=6.2f;

    // Low pump-station skid: intentionally very different from the ore driller.
    // The water extractor reads as a well head + pump/filter plant instead of a
    // mining mast/auger machine.
    b.box(b.mesh.foundation,{0,0.38f,0},{7.8f,0.76f,6.3f});
    for(Vec3 p:std::initializer_list<Vec3>{{-4.2f,0,-3.0f},{4.2f,0,-3.0f},{-4.2f,0,3.0f},{4.2f,0,3.0f}}){
        b.foot(p,0.48f,0.18f);
        b.cylinder_between(b.mesh.details,{p.x,0.18f,p.z},{p.x*0.78f,0.78f,p.z*0.78f},0.13f,8,true,true);
    }

    // Central subsurface-water well head. Only a short casing enters the
    // regolith; there is deliberately no tall drilling tower or visible auger.
    b.cylinder_y(b.mesh.body,{-2.45f,1.28f,0},2.15f,0.62f,16,true);
    b.cylinder_y(b.mesh.details,{-2.45f,2.47f,0},0.28f,0.78f,16,true);
    b.cylinder_y(b.mesh.details,{-2.45f,0.30f,0},0.60f,0.40f,14,true);
    for(float a:{0.0f,1.5708f,3.1416f,4.7124f}){
        const Vec3 q{-2.45f+std::cos(a)*0.88f,0.80f,std::sin(a)*0.88f};
        b.cylinder_between(b.mesh.details,{-2.45f,1.25f,0},q,0.09f,8,true,true);
    }

    // Twin horizontal pump/motor packages on a service skid.
    for(float z:{-1.25f,1.25f}){
        b.cylinder_x(b.mesh.body,{0.15f,1.35f,z},2.35f,0.48f,14,true);
        b.cylinder_x(b.mesh.details,{1.55f,1.35f,z},0.48f,0.62f,12,true);
        b.box(b.mesh.details,{-1.15f,1.35f,z},{0.72f,1.05f,1.05f});
        b.cylinder_between(b.mesh.details,{-2.0f,1.55f,z},{-1.35f,1.55f,z},0.14f,10,true,true);
    }

    // Filtration / phase-separation vessels: compact vertical pressure tanks.
    for(float x:{2.25f,3.35f}){
        b.cylinder_y(b.mesh.body,{x,2.25f,-1.25f},3.20f,0.56f,14,true);
        b.cylinder_y(b.mesh.details,{x,3.94f,-1.25f},0.22f,0.30f,10,true);
    }
    b.cylinder_y(b.mesh.body,{2.85f,2.05f,1.30f},2.75f,0.72f,16,true);
    b.cylinder_y(b.mesh.details,{2.85f,3.55f,1.30f},0.22f,0.32f,10,true);

    // Manifold piping from the well head to the pumps and storage outlet.
    b.cylinder_between(b.mesh.details,{-2.45f,2.10f,0},{-0.95f,2.10f,0},0.16f,10,true,true);
    b.cylinder_z(b.mesh.details,{-0.95f,2.10f,0},2.50f,0.16f,10,true);
    for(float z:{-1.25f,1.25f})
        b.cylinder_between(b.mesh.details,{-0.95f,2.10f,z},{0.05f,1.75f,z},0.13f,10,true,true);
    b.cylinder_between(b.mesh.details,{1.6f,1.85f,-1.25f},{2.25f,2.25f,-1.25f},0.13f,10,true,true);
    b.cylinder_between(b.mesh.details,{1.6f,1.85f,1.25f},{2.85f,2.05f,1.30f},0.13f,10,true,true);
    b.cylinder_x(b.mesh.details,{2.65f,1.12f,2.55f},2.25f,0.15f,10,true);

    // Low control cabinet / telemetry pod and protective service rails.
    b.box(b.mesh.details,{0.15f,2.65f,2.35f},{1.65f,1.55f,0.62f});
    b.box(b.mesh.details,{-0.15f,0.95f,-2.55f},{2.0f,0.55f,0.42f});
    for(float x:{-3.25f,3.25f}){
        b.cylinder_y(b.mesh.details,{x,1.40f,2.70f},1.70f,0.055f,6,true);
        b.cylinder_y(b.mesh.details,{x,1.40f,-2.70f},1.70f,0.055f,6,true);
    }
    b.cylinder_x(b.mesh.details,{0,2.25f,2.70f},6.5f,0.055f,6,true);
    b.cylinder_x(b.mesh.details,{0,2.25f,-2.70f},6.5f,0.055f,6,true);

    return std::move(b.mesh);
}
}
