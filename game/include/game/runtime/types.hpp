#pragma once
#include <frontier/math.hpp>

namespace game::runtime {
struct Vec2 { float x{},y{}; };
using Vec3=rf::V3;
using Mat4=rf::Mat4;
using rf::dot;
using rf::cross;
using rf::length;
using rf::perspective;
inline Vec3 normalize(Vec3 v){const float magnitude=rf::length(v);return magnitude>1e-6f?v/magnitude:Vec3{};}
inline float radians(float d){return d*rf::pi/180.f;}
inline Mat4 multiply(const Mat4& a,const Mat4& b){return a*b;}
inline Mat4 ortho(float l,float r,float b,float t,float n,float f){return rf::orthographic(l,r,b,t,n,f);}
// The game's orbit camera has an explicit up vector, including at the poles.
inline Mat4 look_at(Vec3 eye,Vec3 center,Vec3 up){
    const auto forward=normalize(center-eye),side=normalize(cross(forward,up)),vertical=cross(side,forward);
    auto matrix=Mat4::identity();
    matrix.m[0]=side.x;matrix.m[1]=vertical.x;matrix.m[2]=-forward.x;
    matrix.m[4]=side.y;matrix.m[5]=vertical.y;matrix.m[6]=-forward.y;
    matrix.m[8]=side.z;matrix.m[9]=vertical.z;matrix.m[10]=-forward.z;
    matrix.m[12]=-dot(side,eye);matrix.m[13]=-dot(vertical,eye);matrix.m[14]=dot(forward,eye);
    return matrix;
}
}
