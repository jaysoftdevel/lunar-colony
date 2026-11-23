#pragma once
#include "game/world/habitat_builder.hpp"
#include "game/world/interior_station.hpp"
namespace game::habitats::interior_detail {
using namespace game::runtime;
inline void bench(Builder& b,Vec3 feet,float width,float depth,float height=.78f){
    b.box(b.mesh.interior,feet+Vec3{0,height-.055f,0},{width,.11f,depth},false);
    for(float end:{-.37f,.37f}){
        b.box(b.mesh.interior,feet+Vec3{width*end,(height-.11f)*.5f,0},{.10f,height-.11f,depth*.82f},false);
        b.box(b.mesh.service_metal,feet+Vec3{width*end,.10f,0},{.16f,.07f,depth*.92f},false);
    }
    for(float sign:{-1.f,1.f})b.box(b.mesh.service_metal,feet+Vec3{0,height-.07f,sign*depth*.49f},{width,.065f,.035f},false);
    b.box(b.mesh.service_metal,feet+Vec3{0,height-.19f,0},{width*.88f,.16f,depth*.74f},false);
}
inline void console(Builder& b,Vec3 c,float width=1.2f,float facing_z=-1.f){
    interior_layout::StationScope station(b.mesh);
    b.box(b.mesh.interior,c,{width,.68f,.46f},false);
    b.box(b.mesh.interior,{c.x,c.y+.48f,c.z+.18f*facing_z},{width*.82f,.42f,.10f},false);
    structure_display::Mount screen{{c.x,c.y+.48f,c.z+.245f*facing_z},{facing_z,0,0},{0,1,0},width*.73f,.31f};
    structure_display::casing(b.mesh.service_metal,b.mesh.displays,screen);b.mesh.screens.push_back(screen);
    b.box(b.mesh.service_metal,{c.x,c.y+.27f,c.z+.16f*facing_z},{.10f,.24f,.08f},false);
    b.box(b.mesh.service_metal,{c.x,c.y+.20f,c.z+.25f*facing_z},{width*.80f,.035f,.16f},false);
    for(int key=0;key<5;++key)b.box(b.mesh.markings,{c.x+width*(key-2)*.10f,c.y+.222f,c.z+.26f*facing_z},{width*.055f,.006f,.045f},false);
}
inline void locker_bank(Builder& b,Vec3 c,int count,float spacing=.62f){
    for(int i=0;i<count;++i){
        interior_layout::StationScope station(b.mesh);
        const float x=c.x+(i-(count-1)*.5f)*spacing;
        b.box(b.mesh.interior,{x,c.y,c.z},{.52f,1.55f,.50f},false);
        b.box(b.mesh.service_metal,{x,c.y,c.z-.258f},{.47f,1.46f,.025f},false);
        b.box(b.mesh.service_metal,{x+.15f,c.y+.14f,c.z-.292f},{.038f,.25f,.045f},false);
        for(int vent=0;vent<4;++vent)b.box(b.mesh.markings,{x,c.y-.50f+vent*.045f,c.z-.275f},{.32f,.014f,.008f},false);
        b.box(b.mesh.markings,{x-.08f,c.y+.49f,c.z-.276f},{.19f,.075f,.008f},false);
    }
}
inline void ceiling_light(Builder& b,Vec3 a,Vec3 c){
    b.cylinder_between(b.mesh.service_metal,a,c,.052f,8,true,false);
    b.cylinder_between(b.mesh.lamps,a+Vec3{0,-.038f,0},c+Vec3{0,-.038f,0},.024f,8,true,false);
}
inline void stool(Builder& b,Vec3 p){
    b.cylinder_y(b.mesh.interior,p+Vec3{0,.35f,0},.70f,.07f,8,false);
    b.box(b.mesh.interior,p+Vec3{0,.72f,0},{.43f,.10f,.43f},false);
    b.box(b.mesh.fabric,p+Vec3{0,.778f,0},{.39f,.022f,.39f},false);
    b.box(b.mesh.service_metal,p+Vec3{0,.93f,.22f},{.38f,.38f,.055f},false);
    b.box(b.mesh.fabric,p+Vec3{0,.94f,.185f},{.34f,.30f,.025f},false);
}
inline void tank(Builder& b,Vec3 p,float h=1.9f,float r=.32f){
    b.cylinder_y(b.mesh.interior,p+Vec3{0,h*.5f,0},h,r,16,false);
    for(float f:{.12f,.87f})b.cylinder_y(b.mesh.service_metal,p+Vec3{0,h*f,0},.055f,r*1.06f,16,false);
    b.cylinder_y(b.mesh.service_metal,p+Vec3{0,h+.045f,0},.09f,r*.27f,10,false);
    b.box(b.mesh.markings,p+Vec3{0,h*.60f,-r-.008f},{r*1.05f,h*.21f,.012f},false);
}
}
