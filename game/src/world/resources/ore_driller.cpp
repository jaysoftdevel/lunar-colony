#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_ore_driller(){
    Builder b;
    b.mesh.footprint_x=12.0f; b.mesh.footprint_z=10.0f; b.mesh.height=12.5f;

    // Long tracked service chassis and articulated stabilizers.
    b.box(b.mesh.foundation,{0.7f,0.48f,0},{8.8f,0.96f,5.8f});
    for(Vec3 p:std::initializer_list<Vec3>{{-4.2f,0,-3.3f},{4.7f,0,-3.3f},{-4.2f,0,3.3f},{4.7f,0,3.3f}}){
        b.foot(p,0.62f,0.22f);
        b.cylinder_between(b.mesh.details,{p.x,0.22f,p.z},{p.x*0.68f,1.25f,p.z*0.68f},0.20f,8,true,true);
    }

    // Tall drill mast and guide carriage.
    b.box(b.mesh.body,{-3.55f,6.3f,0},{1.5f,11.4f,1.9f});
    for(float z:{-0.82f,0.82f}) for(float x:{-4.0f,-3.1f})
        b.cylinder_y(b.mesh.details,{x,6.3f,z},10.7f,0.10f,8,true);
    b.box(b.mesh.details,{-3.55f,11.8f,0},{2.0f,0.48f,2.2f});

    // Heavy shaft and large spiral auger visibly entering the lunar surface.
    b.cylinder_y(b.mesh.body,{-3.55f,5.4f,0},9.3f,0.38f,14,true);
    Vec3 prev{-3.10f,0.20f,0};
    const int helix_steps=44;
    for(int i=1;i<=helix_steps;++i){
        const float t=(float)i/(float)helix_steps;
        const float a=t*PI*11.0f;
        Vec3 cur{-3.55f+std::cos(a)*0.58f,0.20f+t*6.1f,std::sin(a)*0.58f};
        b.cylinder_between(b.mesh.details,prev,cur,0.065f,6,true,false);
        b.line(prev,cur);
        prev=cur;
    }

    // Hydraulic pivot boom matching the blueprint's large angled mechanism.
    b.cylinder_between(b.mesh.details,{-2.7f,5.6f,0},{0.0f,3.85f,0},0.30f,12,true,true);
    b.cylinder_between(b.mesh.details,{-2.65f,7.2f,0.65f},{0.45f,4.2f,0.65f},0.16f,10,true,true);
    b.cylinder_y(b.mesh.details,{0.1f,3.8f,0},0.55f,0.42f,12,true);

    // Ore separation housings and twin collection hoppers.
    b.box(b.mesh.body,{0.5f,2.1f,0},{3.1f,2.7f,3.8f});
    b.box(b.mesh.details,{0.4f,3.55f,-1.1f},{1.2f,0.9f,1.2f});
    for(float z:{-1.35f,1.35f}){
        b.box(b.mesh.body,{3.45f,3.25f,z},{2.15f,3.2f,2.2f});
        b.box(b.mesh.details,{3.45f,1.35f,z},{1.25f,0.65f,1.25f});
        for(float x:{2.65f,4.25f}) b.cylinder_y(b.mesh.details,{x,3.35f,z},3.3f,0.08f,8,true);
    }

    // Conveyance / chute from drill processor to the hoppers.
    b.box(b.mesh.details,{2.0f,1.35f,0},{3.5f,0.45f,1.0f});
    b.cylinder_between(b.mesh.details,{0.6f,2.85f,0.8f},{3.1f,4.25f,0.8f},0.13f,8,true,true);
    b.cylinder_between(b.mesh.details,{0.6f,2.85f,-0.8f},{3.1f,4.25f,-0.8f},0.13f,8,true,true);

    // Service rails and control boxes.
    b.box(b.mesh.details,{1.4f,2.8f,-2.25f},{1.6f,1.2f,0.55f});
    b.box(b.mesh.details,{0.1f,1.65f,2.45f},{1.3f,1.1f,0.50f});
    b.cylinder_between(b.mesh.details,{-2.5f,8.0f,-0.75f},{0.3f,4.5f,-0.75f},0.11f,8,true,true);
    b.cylinder_between(b.mesh.details,{-2.5f,8.0f,0.75f},{0.3f,4.5f,0.75f},0.11f,8,true,true);
    return std::move(b.mesh);
}
}
