#pragma once
#include "game/world/resource_mesh.hpp"
#include <algorithm>
#include <cmath>
#include <utility>

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

    void dome(std::vector<Vec3>& dst,Vec3 base,float radius,int rings=7,int segments=20,bool outline=true,float y_scale=1.0f){
        const int rs=std::max(3,rings), ss=std::max(8,segments);
        for(int r=0;r<rs;++r){
            const float t0=(PI*0.5f)*(float)r/(float)rs;
            const float t1=(PI*0.5f)*(float)(r+1)/(float)rs;
            const float rr0=radius*std::cos(t0), rr1=radius*std::cos(t1);
            const float y0=base.y+radius*std::sin(t0)*y_scale;
            const float y1=base.y+radius*std::sin(t1)*y_scale;
            for(int s=0;s<ss;++s){
                const float a0=2.0f*PI*(float)s/(float)ss;
                const float a1=2.0f*PI*(float)(s+1)/(float)ss;
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

    void foot(Vec3 c,float radius=0.42f,float height=0.18f){
        cylinder_y(mesh.foundation,{c.x,height*0.5f,c.z},height,radius,10,true);
    }
};

inline ResourceMesh refine_future_resource(ResourceMesh source,ResourceKind kind){
    Builder b;
    b.mesh=std::move(source);
    const float hx=b.mesh.footprint_x*0.5f;
    const float hz=b.mesh.footprint_z*0.5f;

    // Shared maintainability layer: grounding/load lugs, redundant low-level
    // service runs and an externally reachable diagnostics enclosure.
    if(kind!=ResourceKind::LandingPad){
        const float ax=hx*0.36f,az=hz*0.36f;
        const float ar=std::clamp(std::min(hx,hz)*0.055f,0.18f,0.32f);
        for(float sx:{-1.0f,1.0f}) for(float sz:{-1.0f,1.0f}){
            b.cylinder_y(b.mesh.details,{sx*ax,0.10f,sz*az},0.20f,ar,10,true);
        }
        const float z=-hz*0.34f;
        b.cylinder_between(b.mesh.details,{-hx*0.28f,0.42f,z},{hx*0.28f,0.42f,z},0.050f,8,true,true);
        b.cylinder_between(b.mesh.details,{-hx*0.28f,0.30f,z+0.13f},{hx*0.28f,0.30f,z+0.13f},0.032f,8,true,true);
        b.box(b.mesh.details,{hx*0.22f,0.64f,z},{0.58f,0.62f,0.34f});
        for(int i=0;i<4;++i){
            const float y=0.45f+0.10f*(float)i;
            b.line({hx*0.22f-0.20f,y,z-0.18f},{hx*0.22f+0.20f,y,z-0.18f});
        }
    }

    switch(kind){
        case ResourceKind::SolarPanel:
            // Rotary drive bearings, redundant harnesses and a field-service inverter.
            b.cylinder_y(b.mesh.details,{0,3.38f,0},0.24f,0.94f,16,true);
            b.cylinder_between(b.mesh.details,{-4.55f,3.55f,0.48f},{4.55f,3.55f,0.48f},0.045f,8,true,true);
            b.cylinder_between(b.mesh.details,{-4.55f,3.47f,0.66f},{4.55f,3.47f,0.66f},0.030f,8,true,true);
            b.box(b.mesh.details,{-1.05f,1.38f,0.62f},{0.72f,0.86f,0.54f});
            break;
        case ResourceKind::WaterDriller:
            // Visible bypass manifold, valves and protected pressure instrumentation.
            b.cylinder_z(b.mesh.details,{1.30f,2.52f,0},4.80f,0.085f,10,true);
            for(float z:{-1.75f,0.0f,1.75f}){
                b.cylinder_y(b.mesh.details,{1.30f,2.70f,z},0.36f,0.15f,10,true);
                b.box(b.mesh.details,{1.55f,2.70f,z},{0.32f,0.32f,0.28f});
            }
            break;
        case ResourceKind::PowerStorage:
            // Independent DC bus ducts and emergency isolation cabinets.
            for(float z:{-1.62f,1.62f}) b.cylinder_x(b.mesh.details,{0,4.62f,z},7.8f,0.070f,8,true);
            b.box(b.mesh.details,{-4.18f,2.20f,0},{0.62f,1.55f,1.20f});
            b.box(b.mesh.details,{4.18f,3.35f,0},{0.58f,0.58f,1.35f});
            break;
        case ResourceKind::WaterStorage:
            // Cross-tank pressure equalisation and local valve manifolds.
            b.cylinder_z(b.mesh.details,{0,5.95f,0},3.55f,0.075f,8,true);
            for(float x:{-2.65f,2.65f}){
                b.cylinder_y(b.mesh.details,{x,6.65f,0},0.36f,0.17f,10,true);
                b.box(b.mesh.details,{x,1.14f,0},{0.58f,0.50f,0.58f});
            }
            break;
        case ResourceKind::OreDriller:
            // Secondary hydraulic lines and inspection/service platforms.
            b.cylinder_between(b.mesh.details,{-4.15f,9.65f,-0.68f},{-0.25f,4.60f,-0.68f},0.070f,8,true,true);
            b.cylinder_between(b.mesh.details,{-4.15f,9.65f,0.68f},{-0.25f,4.60f,0.68f},0.070f,8,true,true);
            b.box(b.mesh.details,{-3.55f,6.55f,-1.22f},{1.45f,0.16f,0.85f});
            break;
        case ResourceKind::WaterPurifier:
            // Bypass loop and sample/quality sensor bank.
            b.cylinder_x(b.mesh.details,{0,3.75f,2.35f},6.9f,0.075f,8,true);
            for(float x:{-2.5f,-0.8f,0.9f,2.6f}){
                b.cylinder_y(b.mesh.details,{x,3.98f,2.35f},0.34f,0.13f,10,true);
                b.box(b.mesh.details,{x,4.20f,2.35f},{0.24f,0.22f,0.24f});
            }
            break;
        case ResourceKind::Antenna:
            // Tension/guy lines make the tall mast read as a plausible low-mass structure.
            for(float a:{0.0f,2.0944f,4.1888f}){
                const Vec3 foot{3.15f*std::cos(a),0.26f,3.15f*std::sin(a)};
                b.line({0,8.25f,0},foot);
                b.cylinder_y(b.mesh.details,{foot.x,0.32f,foot.z},0.32f,0.16f,8,true);
            }
            break;
        case ResourceKind::LandingPad:
            // Expansion joints, embedded service trench and low flush approach markers.
            for(float x:{-5.7f,-2.85f,0.0f,2.85f,5.7f}) b.line({x,0.74f,-7.2f},{x,0.74f,7.2f});
            b.box(b.mesh.details,{0,0.59f,5.95f},{5.4f,0.12f,0.58f});
            for(float x:{-2.4f,-0.8f,0.8f,2.4f}) b.cylinder_y(b.mesh.details,{x,0.86f,-7.1f},0.28f,0.12f,8,true);
            break;
        case ResourceKind::StorageDome:
            // Four independent hull-health/service stations around the warehouse.
            for(int i=0;i<4;++i){
                const float a=PI*0.5f*(float)i;
                const Vec3 n{std::sin(a),0.0f,-std::cos(a)};
                const Vec3 c=n*(std::min(hx,hz)*0.78f)+Vec3{0,1.05f,0};
                b.box(b.mesh.details,c,{0.56f,0.88f,0.56f});
                b.cylinder_y(b.mesh.details,{c.x,1.62f,c.z},0.22f,0.14f,8,true);
            }
            break;
        case ResourceKind::Count: break;
    }
    return std::move(b.mesh);
}

} // namespace game::resources
