#pragma once
#include "game/world/resource_builder.hpp"
#include "game/world/structure_display.hpp"
#include "game/world/pressure_door.hpp"
#include "game/world/mesh_portals.hpp"
namespace game::pressure_hardware {
using runtime::Vec3;
// One pressure-leaf language is shared by all personnel and cargo hatches.
// Exposed surfaces have physical separation; no painted coplanar metal sheets.
inline void leaf(runtime::SceneRenderer& r,Vec3 center,Vec3 right,Vec3 normal,float width,float height,float thickness){
    using namespace runtime;
    if(dot(cross(right,Vec3{0,1,0}),normal)<0)right=right*-1.f;
    std::vector<Vec3> skin,metal,rubber,marks,glass;
    const Vec3 up{0,1,0};const float hx=width*.5f,hy=height*.5f,bevel=std::min(.065f,width*.09f);
    const std::array<Vec2,8> rim{{{-hx+bevel,-hy},{hx-bevel,-hy},{hx,-hy+bevel},{hx,hy-bevel},{hx-bevel,hy},{-hx+bevel,hy},{-hx,hy-bevel},{-hx,-hy+bevel}}};
    const float window_w=std::min(width*.52f,.60f),window_h=std::min(height*.15f,.38f);const auto window=center+up*(height*.17f);
    for(float face:{-1.f,1.f}){
        const Vec3 n=normal*face;std::array<Vec3,8> outer,inner;
        for(int i=0;i<8;++i){
            outer[i]=center+right*rim[i].x+up*rim[i].y+n*(thickness*.5f-.025f);
            inner[i]=center+right*(rim[i].x*(1.f-bevel/hx))+up*(rim[i].y*(1.f-bevel/hy))+n*(thickness*.5f-.006f);
        }
        const auto tri=[&](std::vector<Vec3>& out,Vec3 a,Vec3 b,Vec3 c){if(face<0)std::swap(b,c);out.insert(out.end(),{a,b,c});};
        for(int i=0;i<8;++i){const int j=(i+1)%8;
            tri(metal,outer[i],outer[j],inner[j]);tri(metal,outer[i],inner[j],inner[i]);
            tri(skin,center+n*(thickness*.5f-.006f),inner[i],inner[j]);
            if(face>0){const auto a=outer[i]-normal*(thickness-.05f),b=outer[j]-normal*(thickness-.05f);metal.insert(metal.end(),{outer[i],a,b,outer[i],b,outer[j]});}
        }
        // Recessed inspection window, captive perimeter fasteners and one
        // consistent vertical grip, kept clear of the leaf's meeting seal.
        for(float x:{-.34f,.34f})for(float y:{-.40f,.40f})
            structure_display::box(metal,center+right*(x*width)+up*(y*height)+n*(thickness*.5f+.010f),right,up,n,{.030f,.030f,.016f});
        const auto mount=window+n*(thickness*.5f+.006f);
        for(float sign:{-1.f,1.f}){
            structure_display::box(metal,mount+right*(sign*(window_w*.5f+.020f)),right,up,n,{.040f,window_h+.080f,.022f});
            structure_display::box(metal,mount+up*(sign*(window_h*.5f+.020f)),right,up,n,{window_w,.040f,.022f});
        }
        for(float y:{-.10f,.10f})structure_display::box(metal,center+right*(width*.28f)+up*(y*height)+n*(thickness*.5f+.024f),right,up,n,{.065f,.055f,.045f});
        structure_display::box(metal,center+right*(width*.28f)+n*(thickness*.5f+.055f),right,up,n,{.035f,height*.24f,.035f});
        for(float y:{-.29f,-.39f})structure_display::box(metal,center+up*(y*height)+n*(thickness*.5f+.010f),right,up,n,{width*.72f,.030f,.018f});
        for(int i=0;i<4;++i)structure_display::box(marks,center+right*((i-1.5f)*width*.17f)-up*(height*.45f)+n*(thickness*.5f+.018f),right,up,n,{width*.075f,.055f,.008f});
    }
    const mesh_portals::Aperture aperture{window,normal,thickness+.04f,window_w*.5f,window.y-window_h*.5f,window.y+window_h*.5f};
    skin=mesh_portals::subtract(skin,aperture);
    const auto a=window-right*(window_w*.5f)-up*(window_h*.5f),b=window+right*(window_w*.5f)-up*(window_h*.5f),c=window+right*(window_w*.5f)+up*(window_h*.5f),d=window-right*(window_w*.5f)+up*(window_h*.5f);
    glass={a,b,c,a,c,d};
    for(float x:{-1.f,1.f})structure_display::box(rubber,center+right*(x*(hx-.010f)),right,up,normal,{.020f,height,thickness*.74f});
    const LunarSurfaceMapping mapping{center,right,up,normal,4417};
    r.draw_mapped_lunar_surface(skin,LunarMaterial::PressureDoor,{.78f,.83f,.83f,1},mapping);
    r.draw_mapped_lunar_surface(metal,LunarMaterial::Alloy,{.35f,.42f,.45f,1},mapping);
    r.draw_mapped_lunar_surface(rubber,LunarMaterial::Rubber,{.025f,.035f,.04f,1},mapping);
    r.draw_mapped_lunar_surface(marks,LunarMaterial::Marking,{.93f,.70f,.25f,1},mapping);
    r.draw_mapped_lunar_surface(glass,LunarMaterial::Glass,{.22f,.46f,.56f,.52f},mapping);
}
// Airlock-specific latch gear is attached to each moving leaf. The double
// gasket stays on the fixed jamb; no equipment projects into the clear opening.
inline void airlock_gear(runtime::SceneRenderer& r,Vec3 center,Vec3 right,Vec3 normal,
                         float width,float height,float opening){
    using namespace runtime;
    if(dot(cross(right,Vec3{0,1,0}),normal)<0)right=right*-1.f;
    resources::Builder b;std::vector<Vec3> marks;
    const Vec3 up{0,1,0};
    for(float face:{-1.f,1.f}){
        const auto n=normal*face;const auto origin=center+n*(pressure_door::kThickness*.5f+.015f);
        for(float edge:{-1.f,1.f}){
            structure_display::box(b.mesh.details,origin+right*(edge*width*.40f),right,up,n,{.035f,height*.84f,.035f});
            for(float y:{-.36f,-.12f,.12f,.36f}){
                // Captive locking dogs retract before lateral opening.
                structure_display::box(b.mesh.details,origin+right*(edge*(width*.46f-.025f*std::min(opening*5,1.f)))+up*(y*height),right,up,n,{.080f,.065f,.042f});
            }
        }
        const auto hub=origin+up*.02f+n*.055f;
        b.cylinder_between(b.mesh.details,hub-n*.055f,hub,.034f,12,true,false);
        const float radius=.14f,turn=std::min(opening*5,1.f)*1.5707963f;
        for(int i=0;i<20;++i){const float a=6.2831853f*i/20,beta=6.2831853f*(i+1)/20;
            b.cylinder_between(b.mesh.details,hub+(right*std::cos(a)+up*std::sin(a))*radius,hub+(right*std::cos(beta)+up*std::sin(beta))*radius,.014f,6,true,false);
        }
        for(int i=0;i<3;++i){const float a=turn+6.2831853f*i/3;
            b.cylinder_between(b.mesh.details,hub,hub+(right*std::cos(a)+up*std::sin(a))*radius,.012f,6,true,false);
        }
        // Separate recessed equalization valve and grasp handle.
        const auto valve=origin+up*(height*.29f);
        b.cylinder_between(b.mesh.details,valve,valve+n*.035f,.055f,16,true,false);
        structure_display::box(marks,valve+n*.041f,right,up,n,{.085f,.018f,.014f});
        for(float y:{-.12f,.12f})b.cylinder_between(b.mesh.details,origin-right*(width*.25f)+up*y,origin-right*(width*.25f)+up*y+n*.045f,.018f,8,true,false);
        b.cylinder_between(b.mesh.details,origin-right*(width*.25f)-up*.12f+n*.045f,origin-right*(width*.25f)+up*.12f+n*.045f,.022f,10,true,false);
    }
    const LunarSurfaceMapping mapping{center,right,up,normal,4417};
    r.draw_mapped_lunar_surface(b.mesh.details,LunarMaterial::Alloy,{.36f,.42f,.44f,1},mapping);
    r.draw_mapped_lunar_surface(marks,LunarMaterial::Marking,{.88f,.63f,.16f,1},mapping);
}

}
