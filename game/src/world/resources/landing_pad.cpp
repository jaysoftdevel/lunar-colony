#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_landing_pad(){
    Builder b;
    b.mesh.footprint_x=20.0f; b.mesh.footprint_z=20.0f; b.mesh.height=2.8f;
    // Low reinforced circular pad with a raised central deck.
    b.cylinder_y(b.mesh.foundation,{0,0.22f,0},0.44f,8.6f,32,true);
    b.cylinder_y(b.mesh.body,{0,0.52f,0},0.22f,7.65f,32,true);
    b.cylinder_y(b.mesh.details,{0,0.66f,0},0.16f,2.0f,24,true);
    // Radial deck segmentation and concentric touchdown rings.
    for(int i=0;i<16;++i){
        const float a=2.0f*PI*(float)i/16.0f;
        b.cylinder_between(b.mesh.details,{1.9f*std::cos(a),0.70f,1.9f*std::sin(a)},{7.55f*std::cos(a),0.70f,7.55f*std::sin(a)},0.035f,6,true,true);
    }
    for(float r:{3.8f,5.8f,7.55f}){
        Vec3 prev{r,0.71f,0};
        for(int i=1;i<=32;++i){
            const float a=2.0f*PI*(float)i/32.0f;
            Vec3 cur{r*std::cos(a),0.71f,r*std::sin(a)};
            b.cylinder_between(b.mesh.details,prev,cur,0.035f,6,true,true); prev=cur;
        }
    }
    // Four retractable-looking landing guidance beacons and service boxes.
    for(float a:{0.7854f,2.3562f,3.9270f,5.4978f}){
        const Vec3 p{8.15f*std::cos(a),0,8.15f*std::sin(a)};
        b.cylinder_y(b.mesh.details,{p.x,0.95f,p.z},1.7f,0.13f,8,true);
        b.cylinder_y(b.mesh.details,{p.x,1.88f,p.z},0.16f,0.34f,10,true);
    }
    b.box(b.mesh.details,{-6.9f,1.15f,0},{1.7f,1.55f,2.2f});
    b.box(b.mesh.details,{6.9f,1.15f,0},{1.7f,1.55f,2.2f});
    b.box(b.mesh.details,{0,1.15f,6.9f},{2.2f,1.55f,1.7f});
    // Low approach ramp on one side.
    for(int i=0;i<4;++i) b.box(b.mesh.foundation,{0,0.12f+0.06f*i,-8.6f-0.55f*i},{4.5f,0.20f,0.65f});
    return std::move(b.mesh);
}
}
