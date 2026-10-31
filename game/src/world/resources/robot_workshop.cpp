#include "game/world/resource_builder.hpp"
#include "game/world/robot_workshop.hpp"

namespace game::resources {
ResourceMesh make_robot_workshop(){
    Builder b;b.mesh.footprint_x=14;b.mesh.footprint_z=14;b.mesh.height=4.8f;b.mesh.walkable_floor_y=workshop::kDeck;
    b.box(b.mesh.foundation,{0,0.20f,0},{14,0.40f,14});
    // Continuous gentle boarding ramp, sharing its height with rover queries.
    const Vec3 a{-3,0,-12},c{3,0,-12},d{3,0.4f,-7},e{-3,0.4f,-7};
    Builder::tri(b.mesh.foundation,a,c,d);Builder::tri(b.mesh.foundation,a,d,e);
    b.line(a,c);b.line(c,d);b.line(d,e);b.line(e,a);
    for(const auto& box:workshop::kSolids){
        const bool equipment=std::abs(box.center.x)>5.5f && box.half_z<2;
        b.box(equipment?b.mesh.details:b.mesh.body,box.center+Vec3{0,equipment?1.05f:1.60f,0},
              {box.half_x*2,equipment?1.30f:2.40f,box.half_z*2});
    }
    // Ribbed barrel roof. Only its outer perimeter is structural collision;
    // all beams remain above the highest folded robot/tool envelope.
    for(float z:{-6.8f,-3.4f,0.0f,3.4f,6.8f}) for(int i=0;i<12;++i){
        const float p=PI*(float)i/12,q=PI*(float)(i+1)/12;
        const Vec3 u{6.8f*std::cos(p),2.80f+1.85f*std::sin(p),z};
        const Vec3 v{6.8f*std::cos(q),2.80f+1.85f*std::sin(q),z};
        b.cylinder_between(b.mesh.details,u,v,0.065f,6,true,true);
        if(z==6.8f) continue;
        const Vec3 w=v+Vec3{0,0,3.4f},t=u+Vec3{0,0,3.4f};
        Builder::tri(b.mesh.body,u,v,w);Builder::tri(b.mesh.body,u,w,t);
    }
    // Independent charging/data, cargo roller tray and service tooling in
    // each numbered bay. Everything stays outside the central drive aisle.
    for(int slot=0;slot<workshop::kBays;++slot){
        const Vec3 p=workshop::bay_local(slot);const float sign=p.x<0?-1.0f:1.0f;
        b.box(b.mesh.details,{sign*6.1f,1.35f,p.z-0.75f},{0.55f,1.45f,0.50f});
        b.box(b.mesh.details,{sign*6.0f,0.72f,p.z+0.75f},{0.65f,0.50f,0.70f});
        b.cylinder_between(b.mesh.details,{sign*6.1f,2.0f,p.z},{sign*5.55f,2.45f,p.z},0.07f,6);
        b.cylinder_between(b.mesh.details,{sign*5.55f,2.45f,p.z},{sign*4.8f,2.15f,p.z},0.055f,6);
        b.line({p.x-1.4f,0.415f,p.z-1.7f},{p.x+1.4f,0.415f,p.z-1.7f});
        b.line({p.x-1.4f,0.415f,p.z+1.7f},{p.x+1.4f,0.415f,p.z+1.7f});
        for(float x:{p.x-1.4f,p.x+1.4f}) b.line({x,0.415f,p.z-1.7f},{x,0.415f,p.z+1.7f});
        for(int roller=0;roller<4;++roller)
            b.cylinder_z(b.mesh.details,{sign*5.98f,1.0f,p.z+0.48f+0.16f*roller},0.12f,0.04f,6);
    }
    for(float x:{-3.05f,3.05f}){
        b.box(b.mesh.details,{x,0.14f,-9.5f},{0.12f,0.28f,5.0f});
        b.box(b.mesh.details,{x,1.90f,-7.0f},{0.30f,3.0f,0.4f});
    }
    b.box(b.mesh.details,{0,3.55f,-7},{6.4f,0.30f,0.50f});
    b.box(b.mesh.details,{0,3.85f,-7.0f},{0.55f,0.20f,0.30f});
    return std::move(b.mesh);
}
}
