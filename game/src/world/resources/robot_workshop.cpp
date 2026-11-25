#include "game/world/resource_builder.hpp"
#include "game/world/robot_workshop.hpp"

namespace game::resources {
ResourceMesh make_robot_workshop(){
    Builder b;b.mesh.footprint_x=11.0f;b.mesh.footprint_z=10.0f;b.mesh.height=6.0f;b.mesh.walkable_floor_y=workshop::kDeck;
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
        b.box(equipment?b.mesh.details:b.mesh.body,box.center+Vec3{0,equipment?1.05f:2.15f,0},
              {box.half_x*2,equipment?1.30f:3.50f,box.half_z*2});
    }

    // Doubly curved dome roof keeps the full internal drive aisle unobstructed.  Reducing
    // the hall footprint does not reduce headroom or the rover service envelope.
    for(float z:{-4.8f,-2.4f,0.0f,2.4f,4.8f}) for(int i=0;i<12;++i){
        const float p=PI*(float)i/12,q=PI*(float)(i+1)/12;
        const Vec3 u{5.30f*std::cos(p),workshop::roof_height(5.30f*std::cos(p),z),z};
        const Vec3 v{5.30f*std::cos(q),workshop::roof_height(5.30f*std::cos(q),z),z};
        b.cylinder_between(b.mesh.details,u,v,0.065f,6,true,true);
        if(z==4.8f) continue;
        const Vec3 w{v.x,workshop::roof_height(v.x,z+2.4f),z+2.4f},t{u.x,workshop::roof_height(u.x,z+2.4f),z+2.4f};
        Builder::tri(b.mesh.glazing,u,v,w);Builder::tri(b.mesh.glazing,u,w,t);
    }

    // Every arch has two grounded stanchions. Keep these in the detail layer
    // so disabling hull rendering still leaves a continuous load path.
    for(float z:{-4.8f,-2.4f,0.f,2.4f,4.8f})for(float x:{-5.30f,5.30f}){
        b.box(b.mesh.details,{x,2.15f,z},{.14f,3.50f,.16f});
        b.box(b.mesh.details,{x,.46f,z},{.30f,.12f,.32f});
    }
    // Longitudinal glazing mullions and pressure seals meet each transverse rib.
    for(int i=0;i<=12;++i){const float a=PI*i/12,x=5.30f*std::cos(a);
        for(int j=0;j<16;++j){const float z=-4.8f+9.6f*j/16,next=-4.8f+9.6f*(j+1)/16;
            b.cylinder_between(b.mesh.details,{x,workshop::roof_height(x,z),z},{x,workshop::roof_height(x,next),next},.038f,6);}
    }

    // Suspended task-light housings are physically tied to the transverse ribs.
    for(float x:{-3.25f,3.25f})for(float z:{-2.4f,2.4f}){
        for(float offset:{-.38f,.38f}){
            const float roof=workshop::roof_height(x+offset,z);
            b.cylinder_between(b.mesh.details,{x+offset,3.60f,z},{x+offset,roof,z},.024f,6);
        }
        b.box(b.mesh.details,{x,3.63f,z},{1.18f,.10f,.26f});
    }

    // Seal both gables to the roof profile. The front cap starts above the
    // sliding door in its central span; the rear has no opening.
    std::vector<float> cuts{-5.30f,-workshop::kGateHalf,workshop::kGateHalf,5.30f};
    for(int i=1;i<24;++i)cuts.push_back(-5.30f+10.60f*i/24.f);
    std::sort(cuts.begin(),cuts.end());
    for(float z:{-4.80f,4.80f})for(std::size_t i=1;i<cuts.size();++i){
        const float a=cuts[i-1],c=cuts[i];
        const auto top=[&](float x){return workshop::roof_height(x,z);};
        const float bottom=z<0 && std::abs((a+c)*.5f)<workshop::kGateHalf?3.30f:3.90f;
        Builder::tri(b.mesh.body,{a,bottom,z},{c,bottom,z},{c,top(c),z});
        Builder::tri(b.mesh.body,{a,bottom,z},{c,top(c),z},{a,top(a),z});
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
        // Wall-mounted tool drawers, diagnostic console, pneumatic reservoir
        // and paired low service rails at every bay. The central aisle stays free.
        b.box(b.mesh.details,{sign*4.82f,1.80f,p.z+.72f},{.34f,.65f,.62f});
        for(int drawer=0;drawer<4;++drawer)
            b.box(b.mesh.details,{sign*4.36f,.66f+drawer*.18f,p.z-.62f},{.07f,.045f,.34f});
        b.cylinder_y(b.mesh.details,{sign*4.95f,.94f,p.z-.10f},.88f,.12f,10);
        for(float offset:{-.68f,.68f})
            b.box(b.mesh.details,{p.x+offset,.435f,p.z},{.075f,.07f,2.55f});
        // Hose runs are attached to the cabinet and floor rather than suspended.
        for(int segment=0;segment<8;++segment){
            const float a=PI*segment/8,c=PI*(segment+1)/8;
            b.cylinder_between(b.mesh.details,{sign*(4.48f+.14f*std::sin(a)),.82f+.30f*std::cos(a),p.z+.52f},
                {sign*(4.48f+.14f*std::sin(c)),.82f+.30f*std::cos(c),p.z+.52f},.024f,6);
        }
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
