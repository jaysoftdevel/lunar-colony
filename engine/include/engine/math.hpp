#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace eng {
struct Vec2 { float x{}, y{}; };
struct Vec3 {
    float x{}, y{}, z{};
    Vec3 operator+(const Vec3& r) const { return {x+r.x,y+r.y,z+r.z}; }
    Vec3 operator-(const Vec3& r) const { return {x-r.x,y-r.y,z-r.z}; }
    Vec3 operator*(float s) const { return {x*s,y*s,z*s}; }
    Vec3& operator+=(const Vec3& r) { x+=r.x; y+=r.y; z+=r.z; return *this; }
};
inline float dot(Vec3 a, Vec3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b){ return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline float length(Vec3 v){ return std::sqrt(dot(v,v)); }
inline Vec3 normalize(Vec3 v){ float l=length(v); return l>0.000001f ? v*(1.0f/l) : Vec3{}; }
inline float radians(float d){ return d*3.14159265358979323846f/180.0f; }

struct Mat4 {
    std::array<float,16> m{};
    static Mat4 identity(){ Mat4 r; r.m={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}; return r; }
};
inline Mat4 multiply(const Mat4& a,const Mat4& b){
    Mat4 r{};
    for(int c=0;c<4;++c) for(int row=0;row<4;++row){
        float v=0; for(int k=0;k<4;++k) v += a.m[k*4+row]*b.m[c*4+k];
        r.m[c*4+row]=v;
    }
    return r;
}
inline Mat4 perspective(float fovy,float aspect,float zn,float zf){
    float f=1.0f/std::tan(fovy*0.5f); Mat4 r{};
    r.m[0]=f/aspect; r.m[5]=f; r.m[10]=(zf+zn)/(zn-zf); r.m[11]=-1.0f; r.m[14]=(2.0f*zf*zn)/(zn-zf); return r;
}
inline Mat4 ortho(float l,float r,float b,float t,float n,float f){
    Mat4 m=Mat4::identity();
    m.m[0]=2.0f/(r-l); m.m[5]=2.0f/(t-b); m.m[10]=-2.0f/(f-n);
    m.m[12]=-(r+l)/(r-l); m.m[13]=-(t+b)/(t-b); m.m[14]=-(f+n)/(f-n); return m;
}
inline Mat4 look_at(Vec3 eye,Vec3 center,Vec3 up){
    Vec3 f=normalize(center-eye); Vec3 s=normalize(cross(f,up)); Vec3 u=cross(s,f);
    Mat4 r=Mat4::identity();
    r.m[0]=s.x; r.m[1]=u.x; r.m[2]=-f.x;
    r.m[4]=s.y; r.m[5]=u.y; r.m[6]=-f.y;
    r.m[8]=s.z; r.m[9]=u.z; r.m[10]=-f.z;
    r.m[12]=-dot(s,eye); r.m[13]=-dot(u,eye); r.m[14]=dot(f,eye); return r;
}
}
