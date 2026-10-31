#include "game/world/resource_builder.hpp"
#include "game/world/robot_workshop.hpp"

namespace game::resources {
ResourceMesh make_robot_workshop(){
    Builder b;b.mesh.footprint_x=11.0f;b.mesh.footprint_z=10.0f;b.mesh.height=4.8f;b.mesh.walkable_floor_y=workshop::kDeck;
    b.box(b.mesh.foundation,{0,0.20f,0},{11.0f,0.40f,10.0f});

    // Compact, full-width approach ramp.  It remains wide enough for a rover
    // plus lateral steering margin, while the protected external apron gives
    // arriving/leaving robots room to align before the hermetic gate.
    const Vec3 a{-workshop::kRampHalf,0,workshop::kRampEnd};
    const Vec3 c{workshop::kRampHalf,0,workshop::kRampEnd};
    const Vec3 d{workshop::kRampHalf,workshop::kDeck,-workshop::kHalfZ};
    const Vec3 e{-workshop::kRampHalf,workshop::kDeck,-workshop::kHalfZ};
    Builder::tri(b.mesh.foundation,a,c,d);Builder::tri(b.mesh.foundation,a,d,e);
    b.line(a,c);b.line(c,d);b.line(d,e);b.line(e,a);

    for(const auto& box:workshop::kSolids){
        const bool equipment=std::abs(box.center.x)>4.2f && box.half_z<1.5f;
        b.box(equipment?b.mesh.details:b.mesh.body,box.center+Vec3{0,equipment?1.05f:1.60f,0},
              {box.half_x*2,equipment?1.30f:2.40f,box.half_z*2});
    }

    // Barrel roof keeps the full internal drive aisle unobstructed.  Reducing
    // the hall footprint does not reduce headroom or the rover service envelope.
    for(float z:{-4.8f,-2.4f,0.0f,2.4f,4.8f}) for(int i=0;i<12;++i){
        const float p=PI*(float)i/12,q=PI*(float)(i+1)/12;
        const Vec3 u{5.30f*std::cos(p),2.80f+1.85f*std::sin(p),z};
        const Vec3 v{5.30f*std::cos(q),2.80f+1.85f*std::sin(q),z};
        b.cylinder_between(b.mesh.details,u,v,0.065f,6,true,true);
        if(z==4.8f) continue;
        const Vec3 w=v+Vec3{0,0,2.4f},t=u+Vec3{0,0,2.4f};
        Builder::tri(b.mesh.body,u,v,w);Builder::tri(b.mesh.body,u,w,t);
    }

    // Four realistic parking/service stations.  Each bay retains a 2.5 x 2.9 m
    // marked parking envelope and wall-side charging/maintenance equipment.
    for(int slot=0;slot<workshop::kBays;++slot){
        const Vec3 p=workshop::bay_local(slot);const float sign=p.x<0?-1.0f:1.0f;
        b.box(b.mesh.details,{sign*4.72f,1.35f,p.z-0.62f},{0.48f,1.45f,0.46f});
        b.box(b.mesh.details,{sign*4.68f,0.72f,p.z+0.66f},{0.56f,0.50f,0.62f});
        b.cylinder_between(b.mesh.details,{sign*4.72f,2.0f,p.z},{sign*4.30f,2.42f,p.z},0.07f,6);
        b.cylinder_between(b.mesh.details,{sign*4.30f,2.42f,p.z},{sign*3.95f,2.15f,p.z},0.055f,6);
        b.line({p.x-1.25f,0.415f,p.z-1.45f},{p.x+1.25f,0.415f,p.z-1.45f});
        b.line({p.x-1.25f,0.415f,p.z+1.45f},{p.x+1.25f,0.415f,p.z+1.45f});
        for(float x:{p.x-1.25f,p.x+1.25f}) b.line({x,0.415f,p.z-1.45f},{x,0.415f,p.z+1.45f});
        for(int roller=0;roller<4;++roller)
            b.cylinder_z(b.mesh.details,{sign*4.62f,1.0f,p.z+0.34f+0.15f*roller},0.11f,0.04f,6);
    }

    const float ramp_mid=(workshop::kRampEnd-workshop::kHalfZ)*0.5f;
    const float ramp_len=-workshop::kHalfZ-workshop::kRampEnd;
    for(float x:{-(workshop::kRampHalf+0.08f),workshop::kRampHalf+0.08f}){
        b.box(b.mesh.details,{x,0.14f,ramp_mid},{0.12f,0.28f,ramp_len});
        b.box(b.mesh.details,{x,1.90f,-workshop::kHalfZ},{0.28f,3.0f,0.36f});
    }
    b.box(b.mesh.details,{0,3.55f,-workshop::kHalfZ},{2.0f*workshop::kGateHalf+0.8f,0.30f,0.46f});
    b.box(b.mesh.details,{0,3.85f,-workshop::kHalfZ},{0.55f,0.20f,0.30f});
    return std::move(b.mesh);
}
}
