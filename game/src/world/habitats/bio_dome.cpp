#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_module.hpp"

namespace game::habitats {
namespace {

// The greenhouse barrel is open at both X ends when represented only by the
// half-cylinder side surface. Generated corridor hatches project their complete
// doorway profile onto the pressure envelope, so those end faces must be real
// pressure surfaces as well. Close each end with a glazed semicircular cap;
// the low foundation supplies the rectangular portion below the arch spring.
void bio_dome_end_cap(Builder& b,float x,float base_y,float radius,int segments=20){
    const Vec3 center{x,base_y,0.0f};
    Vec3 previous{x,base_y,radius};
    for(int i=1;i<=segments;++i){
        const float a=PI*(float)i/(float)segments;
        const Vec3 current{x,base_y+std::sin(a)*radius,std::cos(a)*radius};
        if(x>=0.0f) b.tri(b.mesh.glazing,center,previous,current);
        else b.tri(b.mesh.glazing,center,current,previous);
        b.line(previous,current);
        if(i%4==0 && i<segments) b.line(center,current);
        previous=current;
    }
}

}

void build_bio_dome_outer(Builder& b){
    b.mesh.footprint_x=13.5f; b.mesh.footprint_z=8.0f; b.mesh.height=6.3f;
    constexpr float barrel_length=10.5f;
    constexpr float arch_base_y=1.20f;
    constexpr float arch_radius=3.3f;
    b.box(b.mesh.foundation,{0,0.62f,0},{barrel_length,1.25f,6.6f});
    b.half_cylinder_x(b.mesh.glazing,{0,arch_base_y,0},barrel_length,arch_radius,14,8,false);
    bio_dome_end_cap(b,-barrel_length*0.5f,arch_base_y,arch_radius);
    bio_dome_end_cap(b, barrel_length*0.5f,arch_base_y,arch_radius);
    // Repeated greenhouse arch ribs.
    for(float x:{-5.0f,-3.3f,-1.65f,0.0f,1.65f,3.3f,5.0f}) b.arch_rib_x(b.mesh.details,x,{0,1.20f,0},3.31f,0.085f,16);
    // Longitudinal frame rails.
    for(float a:{0.18f,0.42f,0.66f,0.84f}){
        const float theta=PI*a;
        const float y=1.20f+std::sin(theta)*3.31f;
        const float z=std::cos(theta)*3.31f;
        b.cylinder_between(b.mesh.details,{-5.25f,y,z},{5.25f,y,z},0.07f,6,true,true);
    }
    // Environmental control column and ducting.
    b.cylinder_y(b.mesh.details,{0,3.87f,0},.85f,0.38f,12,true);
    b.cylinder_between(b.mesh.details,{-4.7f,4.2f,0},{4.7f,4.2f,0},0.12f,8,true,true);
    // Support legs.
    b.supports(-4.2f,4.1f,2.65f,0.78f);
    b.supports(-4.2f,4.1f,-2.65f,0.78f);
}

} // namespace game::habitats
