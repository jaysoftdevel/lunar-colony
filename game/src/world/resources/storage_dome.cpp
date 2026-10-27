#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_storage_dome(StorageDomeSize size){
    Builder b;
    float diameter=9.0f,height=5.3f;
    switch(size){
        case StorageDomeSize::Small: diameter=9.0f; height=5.3f; break;
        case StorageDomeSize::Mid: diameter=13.0f; height=7.0f; break;
        case StorageDomeSize::Big: diameter=18.0f; height=9.3f; break;
        case StorageDomeSize::Huge: diameter=24.0f; height=12.0f; break;
        default: break;
    }
    const float s=diameter/13.0f;
    const float sy=height/7.0f;
    b.mesh.footprint_x=diameter; b.mesh.footprint_z=diameter; b.mesh.height=height;

    // Reinforced circular foundation and low warehouse dome.
    b.cylinder_y(b.mesh.foundation,{0,0.34f*sy,0},0.68f*sy,5.75f*s,32,true);
    b.cylinder_y(b.mesh.body,{0,0.78f*sy,0},0.48f*sy,5.35f*s,32,true);
    b.dome(b.mesh.body,{0,0.95f*sy,0},5.20f*s,9,32,true,0.92f*sy/s);

    // Structural ribs and four explicit corridor/service docking faces.
    for(int i=0;i<12;++i){
        const float a=2.0f*PI*(float)i/12.0f;
        const Vec3 foot{5.15f*s*std::cos(a),1.02f*sy,5.15f*s*std::sin(a)};
        const Vec3 crown{0.65f*s*std::cos(a),5.70f*sy,0.65f*s*std::sin(a)};
        b.cylinder_between(b.mesh.details,foot,crown,0.075f*s,6,true,true);
    }
    const float wall=5.15f*s;
    for(int side=0;side<4;++side){
        const float a=PI*0.5f*(float)side;
        const Vec3 n{std::sin(a),0.0f,-std::cos(a)};
        const Vec3 c=n*wall+Vec3{0,1.80f*sy,0};
        // A visible rectangular docking/service frame identifies every snap port.
        b.box(b.mesh.details,c,{(side%2==0?3.8f*s:0.55f*s),2.8f*sy,(side%2==0?0.55f*s:3.8f*s)});
        const Vec3 inner=n*(wall+0.33f*s)+Vec3{0,1.75f*sy,0};
        b.box(b.mesh.details,inner,{(side%2==0?2.8f*s:0.26f*s),2.15f*sy,(side%2==0?0.26f*s:2.8f*s)});
    }

    // Internal pallet rows scale with the selected storage tier.
    const int rows=size==StorageDomeSize::Small?2:(size==StorageDomeSize::Mid?3:(size==StorageDomeSize::Big?4:5));
    const float span=3.0f*s;
    for(int iz=0;iz<rows;++iz){
        for(int ix=0;ix<rows;++ix){
            const float fx=rows==1?0.0f:((float)ix/(float)(rows-1)-0.5f)*2.0f*span;
            const float fz=rows==1?0.0f:((float)iz/(float)(rows-1)-0.5f)*2.0f*span;
            if(fz<-span*0.72f && std::abs(fx)<1.1f*s) continue;
            b.box(b.mesh.details,{fx,0.78f*sy,fz},{1.25f*s,0.85f*sy,1.05f*s});
        }
    }
    b.box(b.mesh.details,{3.85f*s,1.30f*sy,-3.15f*s},{1.25f*s,1.55f*sy,0.75f*s});
    b.box(b.mesh.details,{-3.85f*s,1.30f*sy,-3.15f*s},{1.25f*s,1.55f*sy,0.75f*s});
    return std::move(b.mesh);
}
}
