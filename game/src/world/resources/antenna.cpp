#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
namespace {
void ring(Builder& b,eng::Vec3 c,eng::Vec3 u,eng::Vec3 v,float r,int seg){
    using namespace eng;
    Vec3 prev=c+u*r;
    for(int i=1;i<=seg;++i){
        const float a=2.0f*PI*(float)i/(float)seg;
        Vec3 cur=c+(u*std::cos(a)+v*std::sin(a))*r;
        b.cylinder_between(b.mesh.details,prev,cur,0.045f,6,true,true);
        prev=cur;
    }
}
}
ResourceMesh make_antenna(){
    Builder b;
    b.mesh.footprint_x=8.0f; b.mesh.footprint_z=8.0f; b.mesh.height=11.5f;
    b.cylinder_y(b.mesh.foundation,{0,0.35f,0},0.70f,2.65f,16,true);
    for(float a:{0.0f,2.0944f,4.1888f}){
        const Vec3 foot{3.2f*std::cos(a),0.10f,3.2f*std::sin(a)};
        b.foot(foot,0.40f,0.18f);
        b.cylinder_between(b.mesh.details,{0,0.65f,0},{foot.x,0.20f,foot.z},0.11f,8,true,true);
    }
    // Telescopic communications mast.
    b.cylinder_y(b.mesh.body,{0,4.7f,0},8.2f,0.32f,12,true);
    b.cylinder_y(b.mesh.details,{0,8.2f,0},3.0f,0.20f,10,true);
    for(float y:{2.2f,4.3f,6.4f,8.4f}) b.cylinder_y(b.mesh.details,{0,y,0},0.18f,0.52f,12,true);
    // Angled dish support and parabolic wire-frame dish.
    const Vec3 hub{0.0f,7.7f,0.0f};
    const Vec3 dish_c{1.55f,8.65f,-0.75f};
    b.cylinder_between(b.mesh.details,hub,dish_c,0.18f,10,true,true);
    Vec3 n=normalize(Vec3{0.78f,0.48f,-0.38f});
    Vec3 u=normalize(cross(n,Vec3{0,1,0}));
    if(length(u)<0.1f) u={1,0,0};
    Vec3 v=normalize(cross(n,u));
    for(float rr:{0.7f,1.35f,2.0f}) ring(b,dish_c,u,v,rr,24);
    for(int i=0;i<12;++i){
        const float a=2.0f*PI*(float)i/12.0f;
        Vec3 rim=dish_c+(u*std::cos(a)+v*std::sin(a))*2.0f;
        b.cylinder_between(b.mesh.details,dish_c,rim,0.035f,6,true,true);
    }
    const Vec3 feed=dish_c+n*1.4f;
    b.cylinder_between(b.mesh.details,dish_c,feed,0.07f,8,true,true);
    b.box(b.mesh.details,feed,{0.32f,0.32f,0.32f});
    // Base comms/control boxes and cable conduits.
    b.box(b.mesh.details,{1.65f,1.15f,1.25f},{1.8f,1.55f,1.15f});
    b.box(b.mesh.details,{-1.55f,1.0f,-1.2f},{1.45f,1.25f,1.0f});
    b.cylinder_between(b.mesh.details,{1.0f,1.0f,0.6f},{0.15f,3.4f,0},0.09f,8,true,true);
    return std::move(b.mesh);
}
}
