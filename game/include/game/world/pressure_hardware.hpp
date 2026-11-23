#pragma once
#include "game/world/resource_builder.hpp"
#include "game/world/structure_display.hpp"
namespace game::pressure_hardware {
using runtime::Vec3;
inline void leaf(runtime::SceneRenderer& r,Vec3 center,Vec3 right,Vec3 normal,float width,float height,float thickness){
    using namespace runtime;
    std::vector<Vec3> skin,metal,rubber,marks;
    const Vec3 up{0,1,0};const float hx=width*.5f,hy=height*.5f,bevel=std::min(.045f,width*.07f);
    const std::array<Vec2,8> rim{{{-hx+bevel,-hy},{hx-bevel,-hy},{hx,-hy+bevel},{hx,hy-bevel},{hx-bevel,hy},{-hx+bevel,hy},{-hx,hy-bevel},{-hx,-hy+bevel}}};
    for(float face:{-1.f,1.f}){
        const Vec3 n=normal*face;
        std::array<Vec3,8> outer,inner;
        for(int i=0;i<8;++i){
            outer[i]=center+right*rim[i].x+up*rim[i].y+n*(thickness*.5f-.020f);
            inner[i]=center+right*(rim[i].x*(1.f-bevel/hx))+up*(rim[i].y*(1.f-bevel/hy))+n*(thickness*.5f-.004f);
        }
        for(int i=0;i<8;++i){const int j=(i+1)%8;
            metal.insert(metal.end(),{outer[i],outer[j],inner[j],outer[i],inner[j],inner[i]});
            skin.insert(skin.end(),{center+n*(thickness*.5f-.004f),inner[i],inner[j]});
            if(face>0){const auto a=outer[i]-normal*(thickness-.04f),b=outer[j]-normal*(thickness-.04f);metal.insert(metal.end(),{outer[i],a,b,outer[i],b,outer[j]});}
        }
        for(float x:{-.36f,.36f})for(float y:{-.40f,.40f})
            structure_display::box(metal,center+right*(x*width)+up*(y*height)+n*(thickness*.5f-.003f),right,up,n,{.026f,.026f,.006f});
        for(float y:{-.23f,.23f})
            structure_display::box(metal,center+up*(y*height)+n*(thickness*.5f-.009f),right,up,n,{width*.75f,.022f,.014f});
        structure_display::box(metal,center+right*(width*.27f)+n*(thickness*.5f-.01f),right,up,n,{.055f,height*.13f,.018f});
        for(int i=0;i<5;++i)
            structure_display::box(marks,center+right*((i-2)*width*.14f)-up*(height*.42f)+n*(thickness*.5f),right,up,n,{width*.065f,.065f,.002f});
    }
    for(float x:{-1.f,1.f})structure_display::box(rubber,center+right*(x*(hx-.012f)),right,up,normal,{.024f,height,thickness*.82f});
    const LunarSurfaceMapping mapping{center,right,up,normal,4417};
    r.draw_mapped_lunar_surface(skin,LunarMaterial::PressureDoor,{.74f,.77f,.75f,1},mapping);
    r.draw_mapped_lunar_surface(metal,LunarMaterial::Alloy,{.29f,.34f,.36f,1},mapping);
    r.draw_mapped_lunar_surface(rubber,LunarMaterial::Rubber,{.035f,.044f,.048f,1},mapping);
    r.draw_mapped_lunar_surface(marks,LunarMaterial::Marking,{.89f,.66f,.22f,1},mapping);
}
}
