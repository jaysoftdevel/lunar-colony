#pragma once
#include "game/runtime/presentation.hpp"
#include <algorithm>
#include <cmath>
namespace game::sound {
// Continuous edge fade rather than a hard viewport toggle. Distance is measured
// from the actual eye so orbit zoom and FPV movement have the same acoustic law.
inline float driller_presence(game::runtime::Vec3 source,game::runtime::Vec3 eye,
                              const game::runtime::Mat4& vp){
    const auto& m=vp.m;
    const float x=m[0]*source.x+m[4]*source.y+m[8]*source.z+m[12];
    const float y=m[1]*source.x+m[5]*source.y+m[9]*source.z+m[13];
    const float w=m[3]*source.x+m[7]*source.y+m[11]*source.z+m[15];
    const float edge=w>.001f?std::max(std::abs(x/w),std::abs(y/w)):2.f;
    const float t=std::clamp((edge-.85f)/.40f,0.f,1.f);
    const float view=1.f-.82f*t*t*(3.f-2.f*t);
    const float distance=game::runtime::length(source-eye);
    return view/(1.f+(distance/32.f)*(distance/32.f));
}
inline float construction_presence(game::runtime::Vec3 source,game::runtime::Vec3 eye,const game::runtime::Mat4& vp){
    const auto& m=vp.m;
    const float x=m[0]*source.x+m[4]*source.y+m[8]*source.z+m[12];
    const float y=m[1]*source.x+m[5]*source.y+m[9]*source.z+m[13];
    const float z=m[2]*source.x+m[6]*source.y+m[10]*source.z+m[14];
    const float w=m[3]*source.x+m[7]*source.y+m[11]*source.z+m[15];
    if(w<=.001f||std::abs(x)>w||std::abs(y)>w||std::abs(z)>w)return 0;
    const float edge=std::max(std::abs(x/w),std::abs(y/w));
    return driller_presence(source,eye,vp)*std::clamp((1.f-edge)/.08f,0.f,1.f);
}

}
