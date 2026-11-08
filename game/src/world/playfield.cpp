#include "game/world/playfield.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace game {
namespace {
float terrain_sample(int x,int z,std::uint32_t seed){
    std::uint32_t hash=static_cast<std::uint32_t>(x)*0x9e3779b1u ^
                       static_cast<std::uint32_t>(z)*0x85ebca77u ^ seed;
    hash=(hash^(hash>>16))*0x7feb352du;
    hash=(hash^(hash>>15))*0x846ca68bu;
    return static_cast<float>((hash^(hash>>16))>>8)/16777216.f;
}

float terrain_noise(float x,float z,std::uint32_t seed){
    const int ix=static_cast<int>(std::floor(x)),iz=static_cast<int>(std::floor(z));
    const auto smooth=[](float t){return t*t*(3.f-2.f*t);};
    const float tx=smooth(x-ix),tz=smooth(z-iz);
    return std::lerp(std::lerp(terrain_sample(ix,iz,seed),terrain_sample(ix+1,iz,seed),tx),
                     std::lerp(terrain_sample(ix,iz+1,seed),terrain_sample(ix+1,iz+1,seed),tx),tz);
}
}

void Playfield::rebuild(int size){
    size_=size;
    float h=size_*0.5f;
    floor_tris_={{-h,0,-h},{h,0,-h},{h,0,h},{-h,0,-h},{h,0,h},{-h,0,h}};
    // A bounded, flat mesh supplies inexpensive regolith shading in Classic
    // and on devices that cannot start the enhanced terrain pass.
    for(auto& shade:surface_) shade.clear();
    const int cells=std::clamp(size_/2,8,64);
    const float pitch=static_cast<float>(size_)/cells;
    const auto corner=[&](int x,int z){
        const bool edge=x==0 || z==0 || x==cells || z==cells;
        const float dx=edge?0.f:(terrain_sample(x,z,3)-.5f)*.44f;
        const float dz=edge?0.f:(terrain_sample(x,z,5)-.5f)*.44f;
        return eng::Vec3{(x+dx)*pitch-h,0,(z+dz)*pitch-h};
    };
    const auto add_triangle=[&](eng::Vec3 a,eng::Vec3 b,eng::Vec3 c){
        const auto p=(a+b+c)*(1.f/3.f);
        const float shade=.55f*terrain_noise(p.x/19,p.z/19,7)+.30f*terrain_noise(p.x/5,p.z/5,11)+
                          .15f*terrain_noise(p.x/pitch,p.z/pitch,13);
        const auto band=std::min(static_cast<std::size_t>(shade*kSurfaceShades),kSurfaceShades-1);
        surface_[band].insert(surface_[band].end(),{a,b,c});
    };
    for(int x=0;x<cells;++x) for(int z=0;z<cells;++z){
        const auto a=corner(x,z),b=corner(x+1,z),c=corner(x+1,z+1),d=corner(x,z+1);
        if((x+z)%2==0){
            add_triangle(a,c,b);
            add_triangle(a,d,c);
        }else{
            add_triangle(a,d,b);
            add_triangle(b,d,c);
        }
    }
    grid_lines_.clear();
    int step=1; if(size_>128)step=2; if(size_>256)step=4;
    for(int i=-size_/2;i<=size_/2;i+=step){
        float v=(float)i;
        grid_lines_.push_back({-h,0.01f,v}); grid_lines_.push_back({h,0.01f,v});
        grid_lines_.push_back({v,0.01f,-h}); grid_lines_.push_back({v,0.01f,h});
    }
}
void Playfield::draw_surface(eng::Renderer& renderer,float daylight) const {
    // Retain enough ambient contrast to read the surface on the night side.
    const float light=.52f+.48f*std::sqrt(std::clamp(daylight,0.f,1.f));
    for(std::size_t i=0;i<surface_.size();++i){
        if(surface_[i].empty()) continue;
        const float tone=(.25f+.036f*static_cast<float>(i))*light;
        renderer.draw_triangles_3d(surface_[i],{tone,tone*.97f,tone*.93f,1.f});
    }
}
}
