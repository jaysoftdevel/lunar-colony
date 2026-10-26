#pragma once
#include "game/world/resource_mesh.hpp"
#include <algorithm>
#include <cmath>

namespace game::resources {
using namespace eng;
constexpr float PI=3.14159265358979323846f;

struct Builder {
    ResourceMesh mesh;

    static void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c){ dst.push_back(a); dst.push_back(b); dst.push_back(c); }
    void line(Vec3 a,Vec3 b){ mesh.edges.push_back(a); mesh.edges.push_back(b); }

    void box(std::vector<Vec3>& dst,Vec3 c,Vec3 s,bool outline=true){
        const Vec3 h=s*0.5f;
        Vec3 p[8]={
            {c.x-h.x,c.y-h.y,c.z-h.z},{c.x+h.x,c.y-h.y,c.z-h.z},
            {c.x+h.x,c.y+h.y,c.z-h.z},{c.x-h.x,c.y+h.y,c.z-h.z},
            {c.x-h.x,c.y-h.y,c.z+h.z},{c.x+h.x,c.y-h.y,c.z+h.z},
            {c.x+h.x,c.y+h.y,c.z+h.z},{c.x-h.x,c.y+h.y,c.z+h.z}
        };
        const int f[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
        for(const auto& q:f){ tri(dst,p[q[0]],p[q[1]],p[q[2]]); tri(dst,p[q[0]],p[q[2]],p[q[3]]); }
        if(outline){
            const int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
            for(const auto& q:e) line(p[q[0]],p[q[1]]);
        }
    }

    void oriented_box(std::vector<Vec3>& dst,Vec3 c,Vec3 axis_x,Vec3 axis_y,Vec3 axis_z,Vec3 size,bool outline=true){
        axis_x=normalize(axis_x); axis_y=normalize(axis_y); axis_z=normalize(axis_z);
        const Vec3 hx=axis_x*(size.x*0.5f), hy=axis_y*(size.y*0.5f), hz=axis_z*(size.z*0.5f);
        Vec3 p[8]={
            c-hx-hy-hz,c+hx-hy-hz,c+hx+hy-hz,c-hx+hy-hz,
            c-hx-hy+hz,c+hx-hy+hz,c+hx+hy+hz,c-hx+hy+hz
        };
        const int f[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
        for(const auto& q:f){ tri(dst,p[q[0]],p[q[1]],p[q[2]]); tri(dst,p[q[0]],p[q[2]],p[q[3]]); }
        if(outline){
            const int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
            for(const auto& q:e) line(p[q[0]],p[q[1]]);
        }
    }

    void cylinder_between(std::vector<Vec3>& dst,Vec3 a,Vec3 b,float radius,int segments=12,bool caps=true,bool outline=true){
        const Vec3 axis=b-a;
        const float len=length(axis);
        if(len<0.0001f) return;
        const Vec3 n=axis*(1.0f/len);
        const Vec3 ref=std::abs(n.y)<0.92f?Vec3{0,1,0}:Vec3{1,0,0};
        const Vec3 u=normalize(cross(n,ref));
        const Vec3 v=normalize(cross(n,u));
        const int seg=std::max(6,segments);
        for(int i=0;i<seg;++i){
            const float a0=2.0f*PI*(float)i/(float)seg;
            const float a1=2.0f*PI*(float)(i+1)/(float)seg;
            const Vec3 r0=u*std::cos(a0)*radius+v*std::sin(a0)*radius;
            const Vec3 r1=u*std::cos(a1)*radius+v*std::sin(a1)*radius;
            const Vec3 p0=a+r0,p1=a+r1,q0=b+r0,q1=b+r1;
            tri(dst,p0,q0,q1); tri(dst,p0,q1,p1);
            if(caps){ tri(dst,a,p1,p0); tri(dst,b,q0,q1); }
            if(outline){ line(p0,p1); line(q0,q1); if(i%(std::max(1,seg/4))==0) line(p0,q0); }
        }
    }

    void cylinder_x(std::vector<Vec3>& dst,Vec3 c,float len,float radius,int segments=12,bool outline=true){
        cylinder_between(dst,{c.x-len*0.5f,c.y,c.z},{c.x+len*0.5f,c.y,c.z},radius,segments,true,outline);
    }
    void cylinder_y(std::vector<Vec3>& dst,Vec3 c,float len,float radius,int segments=12,bool outline=true){
        cylinder_between(dst,{c.x,c.y-len*0.5f,c.z},{c.x,c.y+len*0.5f,c.z},radius,segments,true,outline);
    }
    void cylinder_z(std::vector<Vec3>& dst,Vec3 c,float len,float radius,int segments=12,bool outline=true){
        cylinder_between(dst,{c.x,c.y,c.z-len*0.5f},{c.x,c.y,c.z+len*0.5f},radius,segments,true,outline);
    }

    void foot(Vec3 c,float radius=0.42f,float height=0.18f){
        cylinder_y(mesh.foundation,{c.x,height*0.5f,c.z},height,radius,10,true);
    }
};

} // namespace game::resources
