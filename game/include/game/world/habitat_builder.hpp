#pragma once
#include "game/world/habitat_mesh.hpp"
#include <algorithm>
#include <cmath>

namespace game::habitats {
using namespace eng;
constexpr float PI=3.14159265358979323846f;

struct Builder {
    HabitatMesh mesh;

    static void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c){ dst.push_back(a);dst.push_back(b);dst.push_back(c); }
    void line(Vec3 a,Vec3 b){ mesh.edges.push_back(a);mesh.edges.push_back(b); }

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

    void cylinder_between(std::vector<Vec3>& dst,Vec3 a,Vec3 b,float radius,int segments=12,bool caps=true,bool outline=true){
        const Vec3 axis=b-a;
        const float len=length(axis);
        if(len<0.0001f) return;
        const Vec3 n=axis*(1.0f/len);
        Vec3 ref=std::abs(n.y)<0.92f?Vec3{0,1,0}:Vec3{1,0,0};
        Vec3 u=normalize(cross(n,ref));
        Vec3 v=normalize(cross(n,u));
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

    void cylinder_x(std::vector<Vec3>& dst,Vec3 c,float length_x,float radius,int segments=14,bool outline=true){
        cylinder_between(dst,{c.x-length_x*0.5f,c.y,c.z},{c.x+length_x*0.5f,c.y,c.z},radius,segments,true,outline);
    }
    void cylinder_y(std::vector<Vec3>& dst,Vec3 c,float height,float radius,int segments=12,bool outline=true){
        cylinder_between(dst,{c.x,c.y-height*0.5f,c.z},{c.x,c.y+height*0.5f,c.z},radius,segments,true,outline);
    }
    void cylinder_z(std::vector<Vec3>& dst,Vec3 c,float length_z,float radius,int segments=12,bool outline=true){
        cylinder_between(dst,{c.x,c.y,c.z-length_z*0.5f},{c.x,c.y,c.z+length_z*0.5f},radius,segments,true,outline);
    }

    void torus_x(std::vector<Vec3>& dst,Vec3 c,float major_r,float tube_r,int ring_segments=16,int tube_segments=6){
        for(int i=0;i<ring_segments;++i){
            const float a0=2.0f*PI*(float)i/(float)ring_segments;
            const float a1=2.0f*PI*(float)(i+1)/(float)ring_segments;
            const Vec3 p0{c.x,c.y+std::cos(a0)*major_r,c.z+std::sin(a0)*major_r};
            const Vec3 p1{c.x,c.y+std::cos(a1)*major_r,c.z+std::sin(a1)*major_r};
            cylinder_between(dst,p0,p1,tube_r,tube_segments,true,false);
            line(p0,p1);
        }
    }
    void torus_z(std::vector<Vec3>& dst,Vec3 c,float major_r,float tube_r,int ring_segments=16,int tube_segments=6){
        for(int i=0;i<ring_segments;++i){
            const float a0=2.0f*PI*(float)i/(float)ring_segments;
            const float a1=2.0f*PI*(float)(i+1)/(float)ring_segments;
            const Vec3 p0{c.x+std::cos(a0)*major_r,c.y+std::sin(a0)*major_r,c.z};
            const Vec3 p1{c.x+std::cos(a1)*major_r,c.y+std::sin(a1)*major_r,c.z};
            cylinder_between(dst,p0,p1,tube_r,tube_segments,true,false);
            line(p0,p1);
        }
    }

    void dome(std::vector<Vec3>& dst,Vec3 base,float radius,int rings=7,int segments=20,bool outline=true,float y_scale=1.0f){
        const int rs=std::max(3,rings), ss=std::max(8,segments);
        for(int r=0;r<rs;++r){
            const float t0=(PI*0.5f)*(float)r/(float)rs;
            const float t1=(PI*0.5f)*(float)(r+1)/(float)rs;
            const float rr0=radius*std::cos(t0), rr1=radius*std::cos(t1);
            const float y0=base.y+radius*std::sin(t0)*y_scale;
            const float y1=base.y+radius*std::sin(t1)*y_scale;
            for(int s=0;s<ss;++s){
                const float a0=2*PI*(float)s/(float)ss;
                const float a1=2*PI*(float)(s+1)/(float)ss;
                const Vec3 p00{base.x+rr0*std::cos(a0),y0,base.z+rr0*std::sin(a0)};
                const Vec3 p01{base.x+rr0*std::cos(a1),y0,base.z+rr0*std::sin(a1)};
                const Vec3 p10{base.x+rr1*std::cos(a0),y1,base.z+rr1*std::sin(a0)};
                const Vec3 p11{base.x+rr1*std::cos(a1),y1,base.z+rr1*std::sin(a1)};
                tri(dst,p00,p10,p11); tri(dst,p00,p11,p01);
                if(outline && (s%(std::max(1,ss/8))==0)) line(p00,p10);
                if(outline && s==0){ line(p00,p01); line(p10,p11); }
            }
        }
    }

    void half_cylinder_x(std::vector<Vec3>& dst,Vec3 base,float length_x,float radius,int arc_segments=12,int length_segments=6,bool outline=true){
        const int as=std::max(6,arc_segments), ls=std::max(1,length_segments);
        for(int ix=0;ix<ls;++ix){
            const float x0=base.x-length_x*0.5f+length_x*(float)ix/(float)ls;
            const float x1=base.x-length_x*0.5f+length_x*(float)(ix+1)/(float)ls;
            for(int ia=0;ia<as;++ia){
                const float a0=PI*(float)ia/(float)as;
                const float a1=PI*(float)(ia+1)/(float)as;
                const Vec3 p00{x0,base.y+std::sin(a0)*radius,base.z+std::cos(a0)*radius};
                const Vec3 p01{x0,base.y+std::sin(a1)*radius,base.z+std::cos(a1)*radius};
                const Vec3 p10{x1,base.y+std::sin(a0)*radius,base.z+std::cos(a0)*radius};
                const Vec3 p11{x1,base.y+std::sin(a1)*radius,base.z+std::cos(a1)*radius};
                tri(dst,p00,p10,p11); tri(dst,p00,p11,p01);
                if(outline && ia==0) line(p00,p10);
            }
        }
    }

    void arch_rib_x(std::vector<Vec3>& dst,float x,Vec3 base,float radius,float tube_r,int segments=14){
        Vec3 prev{x,base.y,base.z+radius};
        for(int i=1;i<=segments;++i){
            const float a=PI*(float)i/(float)segments;
            Vec3 cur{x,base.y+std::sin(a)*radius,base.z+std::cos(a)*radius};
            cylinder_between(dst,prev,cur,tube_r,6,true,false);
            line(prev,cur);
            prev=cur;
        }
    }

    void supports(float x0,float x1,float z,float y_top){
        const Vec3 pts[4]={
            {x0,y_top,z},{x0-0.65f,0.18f,z+0.35f},
            {x1,y_top,z},{x1+0.65f,0.18f,z+0.35f}
        };
        cylinder_between(mesh.details,pts[0],pts[1],0.13f,8,true,true);
        cylinder_between(mesh.details,pts[2],pts[3],0.13f,8,true,true);
        cylinder_y(mesh.details,{pts[1].x,0.10f,pts[1].z},0.18f,0.34f,10,true);
        cylinder_y(mesh.details,{pts[3].x,0.10f,pts[3].z},0.18f,0.34f,10,true);
    }
};

} // namespace game::habitats
