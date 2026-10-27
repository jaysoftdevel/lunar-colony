#pragma once
#include "game/world/habitat_builder.hpp"

namespace game::habitats::interior_detail {
using namespace eng;

inline void console(Builder& b,Vec3 c,float width=1.2f,float facing_z=-1.0f){
    b.box(b.mesh.interior,c,{width,0.68f,0.46f},false);
    b.box(b.mesh.interior,{c.x,c.y+0.48f,c.z+0.18f*facing_z},{width*0.82f,0.42f,0.10f},false);
}
inline void locker_bank(Builder& b,Vec3 c,int count,float spacing=0.62f){
    for(int i=0;i<count;++i){
        const float x=c.x+((float)i-(float)(count-1)*0.5f)*spacing;
        b.box(b.mesh.interior,{x,c.y,c.z},{0.52f,1.55f,0.50f},false);
        b.box(b.mesh.interior,{x,c.y+0.18f,c.z-0.27f},{0.36f,0.05f,0.04f},false);
    }
}
inline void ceiling_light(Builder& b,Vec3 a,Vec3 c){ b.cylinder_between(b.mesh.interior,a,c,0.035f,6,true,false); }
inline void stool(Builder& b,Vec3 p){
    b.cylinder_y(b.mesh.interior,{p.x,p.y+0.35f,p.z},0.70f,0.07f,6,false);
    b.cylinder_y(b.mesh.interior,{p.x,p.y+0.72f,p.z},0.12f,0.26f,10,false);
}
inline void tank(Builder& b,Vec3 p,float h=1.9f,float r=0.32f){
    b.cylinder_y(b.mesh.interior,{p.x,p.y+h*0.5f,p.z},h,r,12,false);
}

} // namespace game::habitats::interior_detail
