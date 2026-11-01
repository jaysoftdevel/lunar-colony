#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_hydrogen_generator(){
    Builder b;
    b.mesh.footprint_x=9.5f;
    b.mesh.footprint_z=7.5f;
    b.mesh.height=5.8f;

    // Raised equipment skid and four vibration-isolated supports.
    b.box(b.mesh.foundation,{0,0.38f,0},{8.5f,0.76f,6.4f});
    for(Vec3 p:std::initializer_list<Vec3>{{-4.15f,0,-2.95f},{4.15f,0,-2.95f},{-4.15f,0,2.95f},{4.15f,0,2.95f}})
        b.foot(p,0.42f,0.18f);

    // Two modular fuel-cell stack cabinets with service gaps.
    for(float x:{-2.05f,0.15f}){
        b.box(b.mesh.body,{x,2.55f,0.25f},{1.75f,3.65f,4.25f});
        b.box(b.mesh.details,{x,2.55f,2.43f},{1.42f,3.10f,0.14f});
        for(int fin=0;fin<7;++fin){
            const float y=1.35f+0.40f*(float)fin;
            b.line({x-0.60f,y,2.52f},{x+0.60f,y,2.52f});
        }
    }

    // Hydrogen inlet conditioning vessel and redundant feed manifold.
    b.cylinder_y(b.mesh.body,{3.10f,2.10f,-1.25f},3.35f,0.78f,16,true);
    b.cylinder_y(b.mesh.details,{3.10f,4.00f,-1.25f},0.36f,0.22f,10,true);
    b.cylinder_x(b.mesh.details,{0.65f,1.10f,-2.15f},5.4f,0.075f,10,true);
    b.cylinder_x(b.mesh.details,{0.65f,1.34f,-1.92f},5.4f,0.055f,10,true);
    b.cylinder_between(b.mesh.details,{3.10f,1.25f,-1.90f},{3.10f,1.25f,-2.15f},0.075f,10,true,true);

    // DC/DC inverter and water/thermal conditioning cabinet.
    b.box(b.mesh.body,{3.10f,2.15f,1.40f},{1.65f,2.85f,1.95f});
    b.box(b.mesh.details,{3.10f,2.20f,2.40f},{1.30f,2.25f,0.12f});
    for(int i=0;i<5;++i){
        const float y=1.45f+0.38f*(float)i;
        b.line({2.60f,y,2.48f},{3.60f,y,2.48f});
    }

    // Overhead electrical bus and compact radiator/heat-rejection rack.
    b.cylinder_x(b.mesh.details,{-0.05f,4.82f,-2.12f},6.9f,0.085f,10,true);
    b.box(b.mesh.details,{-0.50f,5.05f,-1.90f},{5.65f,0.18f,0.18f});
    for(int i=0;i<5;++i){
        const float x=-2.55f+1.05f*(float)i;
        b.oriented_box(b.mesh.details,{x,4.78f,-2.62f},{1,0,0},{0,1,0},{0,0,1},{0.72f,1.10f,0.08f});
    }

    return std::move(b.mesh);
}
}
