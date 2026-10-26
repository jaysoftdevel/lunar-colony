#include "game/world/habitat_mesh.hpp"
#include <algorithm>
#include <cmath>

namespace game {
using namespace eng;

namespace {
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

HabitatMesh make_airlock(){
    Builder b;
    b.mesh.footprint_x=10.0f; b.mesh.footprint_z=6.5f; b.mesh.height=5.8f;
    b.cylinder_x(b.mesh.shell,{0,2.55f,0},7.2f,2.15f,18,true);
    // Interior deck and equipment remain opaque when hull view modes are active.
    b.box(b.mesh.interior,{0,0.72f,0},{6.4f,0.16f,3.2f},false);
    b.box(b.mesh.interior,{-1.7f,1.55f,-1.35f},{1.6f,0.65f,0.48f},false);
    b.box(b.mesh.interior,{0.9f,1.55f,-1.35f},{1.6f,0.65f,0.48f},false);
    b.cylinder_y(b.mesh.interior,{2.35f,1.55f,0.95f},1.6f,0.30f,10,false);
    // Structural ribs and service conduits follow the cylindrical body.
    for(float x:{-2.5f,-0.8f,0.9f,2.6f}) b.torus_x(b.mesh.details,{x,2.55f,0},2.18f,0.08f,18,6);
    // Front pressure-lock vestibule and outer hatch.
    b.box(b.mesh.shell,{4.05f,2.30f,0},{1.45f,3.7f,3.75f});
    b.box(b.mesh.details,{4.83f,2.30f,0},{0.20f,3.15f,3.15f});
    b.torus_x(b.mesh.details,{4.96f,2.30f,0},0.72f,0.10f,18,6);
    b.cylinder_x(b.mesh.details,{5.02f,2.30f,0},0.16f,0.62f,16,true);
    // Roof service cap and side utility boxes.
    b.cylinder_y(b.mesh.details,{-1.7f,4.95f,0.35f},0.70f,0.52f,12,true);
    b.cylinder_y(b.mesh.details,{1.35f,4.82f,-0.55f},0.55f,0.38f,12,true);
    b.box(b.mesh.details,{-1.1f,2.3f,2.05f},{0.95f,0.90f,0.18f});
    b.box(b.mesh.details,{1.45f,1.65f,2.08f},{0.75f,0.72f,0.16f});
    // Side porthole.
    b.torus_z(b.mesh.details,{0.7f,2.65f,2.20f},0.55f,0.08f,16,6);
    b.cylinder_z(b.mesh.glazing,{0.7f,2.65f,2.24f},0.08f,0.46f,16,false);
    // Steps and handrail-like geometry.
    for(int i=0;i<3;++i) b.box(b.mesh.details,{5.2f+0.42f*i,0.28f+0.24f*i,0},{0.72f,0.18f,2.1f});
    b.cylinder_between(b.mesh.details,{5.05f,0.55f,1.25f},{6.0f,1.45f,1.25f},0.055f,6,true,true);
    b.cylinder_between(b.mesh.details,{5.05f,0.55f,-1.25f},{6.0f,1.45f,-1.25f},0.055f,6,true,true);
    b.supports(-2.7f,2.4f,1.55f,0.85f);
    b.supports(-2.7f,2.4f,-1.55f,0.85f);
    b.mesh.entrances.push_back({{5.25f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    return std::move(b.mesh);
}

HabitatMesh make_oxygen_dome(){
    Builder b;
    b.mesh.footprint_x=10.5f; b.mesh.footprint_z=10.0f; b.mesh.height=7.6f;
    b.cylinder_y(b.mesh.foundation,{0,0.55f,0},1.10f,4.35f,24,true);
    b.dome(b.mesh.shell,{0,1.0f,0},4.35f,8,24,true,0.92f);
    // Interior electrolyzer / buffer-tank cluster.
    b.box(b.mesh.interior,{0,1.10f,0},{5.6f,0.18f,5.6f},false);
    for(const Vec3 c: {Vec3{-1.25f,2.15f,-1.25f},Vec3{1.25f,2.15f,-1.25f},Vec3{-1.25f,2.15f,1.25f},Vec3{1.25f,2.15f,1.25f}})
        b.cylinder_y(b.mesh.interior,c,2.2f,0.46f,12,false);
    b.cylinder_y(b.mesh.interior,{0,2.45f,0},3.0f,0.34f,12,false);
    // Geodesic/service ribs.
    for(int s=0;s<8;++s){
        const float a=2*PI*(float)s/8.0f;
        Vec3 prev{4.38f*std::cos(a),1.0f,4.38f*std::sin(a)};
        for(int r=1;r<=7;++r){
            const float t=(PI*0.5f)*(float)r/7.0f;
            Vec3 cur{4.38f*std::cos(t)*std::cos(a),1.0f+4.0f*std::sin(t),4.38f*std::cos(t)*std::sin(a)};
            b.cylinder_between(b.mesh.details,prev,cur,0.065f,6,true,false); b.line(prev,cur); prev=cur;
        }
    }
    // Oxygen processing tanks / electrolyzer bank.
    b.cylinder_y(b.mesh.details,{-4.05f,2.05f,1.05f},3.4f,0.72f,14,true);
    for(int i=0;i<3;++i) b.cylinder_y(b.mesh.details,{-3.65f+i*0.72f,1.75f,2.75f},2.5f,0.26f,10,true);
    b.box(b.mesh.details,{-2.2f,1.55f,3.45f},{1.3f,1.8f,0.55f});
    // Piping from processing bank into dome.
    b.cylinder_between(b.mesh.details,{-4.05f,3.35f,1.05f},{-2.75f,4.05f,0.65f},0.10f,8,true,true);
    b.cylinder_between(b.mesh.details,{-2.75f,4.05f,0.65f},{-1.35f,4.7f,0.35f},0.10f,8,true,true);
    // Entry module.
    b.box(b.mesh.shell,{4.55f,2.0f,0},{1.9f,3.4f,3.15f});
    b.torus_x(b.mesh.details,{5.55f,2.0f,0},0.64f,0.09f,16,6);
    b.cylinder_x(b.mesh.details,{5.60f,2.0f,0},0.12f,0.54f,14,true);
    // Roof scrubber tower and vents.
    b.cylinder_y(b.mesh.details,{0,5.45f,0},1.45f,0.72f,16,true);
    b.cylinder_y(b.mesh.details,{2.2f,4.72f,-1.25f},0.75f,0.36f,12,true);
    b.cylinder_y(b.mesh.details,{-1.8f,4.85f,-1.65f},0.65f,0.32f,12,true);
    // Base feet.
    for(float a:{0.4f,2.0f,3.55f,5.1f}){
        Vec3 top{3.7f*std::cos(a),0.75f,3.7f*std::sin(a)};
        Vec3 foot{4.55f*std::cos(a),0.16f,4.55f*std::sin(a)};
        b.cylinder_between(b.mesh.details,top,foot,0.13f,8,true,true);
        b.cylinder_y(b.mesh.details,{foot.x,0.09f,foot.z},0.18f,0.36f,10,true);
    }
    b.mesh.entrances.push_back({{5.70f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    return std::move(b.mesh);
}

HabitatMesh make_cantine_dome(){
    Builder b;
    b.mesh.footprint_x=11.0f; b.mesh.footprint_z=10.5f; b.mesh.height=7.0f;
    b.cylinder_y(b.mesh.foundation,{0,0.60f,0},1.20f,4.55f,24,true);
    // Transparent-looking window dome plus substantial structural framing.
    b.dome(b.mesh.glazing,{0,1.05f,0},4.55f,7,24,false,0.88f);
    for(int s=0;s<10;++s){
        const float a=2*PI*(float)s/10.0f;
        Vec3 prev{4.56f*std::cos(a),1.05f,4.56f*std::sin(a)};
        for(int r=1;r<=9;++r){
            const float t=(PI*0.5f)*(float)r/9.0f;
            Vec3 cur{4.56f*std::cos(t)*std::cos(a),1.05f+4.0f*std::sin(t),4.56f*std::cos(t)*std::sin(a)};
            b.cylinder_between(b.mesh.details,prev,cur,0.095f,6,true,false); b.line(prev,cur); prev=cur;
        }
    }
    // Two horizontal window belt ribs around the communal dome.
    for(float t:{0.22f,0.48f}){
        const float theta=(PI*0.5f)*t;
        const float rr=4.57f*std::cos(theta);
        const float yy=1.05f+4.0f*std::sin(theta);
        for(int s=0;s<24;++s){
            const float a0=2*PI*s/24.0f,a1=2*PI*(s+1)/24.0f;
            Vec3 p0{rr*std::cos(a0),yy,rr*std::sin(a0)},p1{rr*std::cos(a1),yy,rr*std::sin(a1)};
            b.cylinder_between(b.mesh.details,p0,p1,0.075f,6,true,false); b.line(p0,p1);
        }
    }
    // Interior communal tables/core visible through the dome.
    b.cylinder_y(b.mesh.interior,{0,2.15f,0},2.2f,0.30f,12,false);
    b.cylinder_y(b.mesh.interior,{0,1.30f,0},0.22f,2.0f,18,false);
    b.cylinder_y(b.mesh.interior,{0,2.95f,0},0.18f,1.65f,18,false);
    for(float a:{0.0f,1.5708f,3.1416f,4.7124f}){
        const Vec3 c{2.35f*std::cos(a),1.28f,2.35f*std::sin(a)};
        b.cylinder_y(b.mesh.interior,c,0.16f,0.82f,14,false);
        b.cylinder_y(b.mesh.interior,{c.x,0.72f,c.z},0.95f,0.13f,8,false);
    }
    // Entry connector.
    b.box(b.mesh.shell,{4.85f,2.0f,0},{2.0f,3.35f,3.15f});
    b.torus_x(b.mesh.details,{5.9f,2.0f,0},0.62f,0.09f,16,6);
    b.cylinder_x(b.mesh.details,{5.95f,2.0f,0},0.12f,0.52f,14,true);
    b.cylinder_y(b.mesh.details,{0,5.50f,0},0.8f,0.55f,14,true);
    // Landing supports.
    for(float a:{0.45f,2.1f,3.65f,5.25f}){
        Vec3 top{3.95f*std::cos(a),0.75f,3.95f*std::sin(a)};
        Vec3 foot{4.8f*std::cos(a),0.15f,4.8f*std::sin(a)};
        b.cylinder_between(b.mesh.details,top,foot,0.13f,8,true,true);
        b.cylinder_y(b.mesh.details,{foot.x,0.09f,foot.z},0.18f,0.34f,10,true);
    }
    b.mesh.entrances.push_back({{6.00f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    return std::move(b.mesh);
}

HabitatMesh make_bio_dome(){
    Builder b;
    b.mesh.footprint_x=13.5f; b.mesh.footprint_z=8.0f; b.mesh.height=6.3f;
    b.box(b.mesh.foundation,{0,0.62f,0},{10.5f,1.25f,6.6f});
    b.half_cylinder_x(b.mesh.glazing,{0,1.20f,0},10.5f,3.3f,14,8,false);
    // Repeated greenhouse arch ribs.
    for(float x:{-5.0f,-3.3f,-1.65f,0.0f,1.65f,3.3f,5.0f}) b.arch_rib_x(b.mesh.details,x,{0,1.20f,0},3.31f,0.085f,16);
    // Longitudinal frame rails.
    for(float a:{0.18f,0.42f,0.66f,0.84f}){
        const float theta=PI*a;
        const float y=1.20f+std::sin(theta)*3.31f;
        const float z=std::cos(theta)*3.31f;
        b.cylinder_between(b.mesh.details,{-5.25f,y,z},{5.25f,y,z},0.07f,6,true,true);
    }
    // Interior crop racks visible as actual geometry.
    for(float z:{-1.7f,0.0f,1.7f}){
        for(float y:{1.25f,2.05f}){
            b.box(b.mesh.interior,{-0.5f,y,z},{8.3f,0.14f,0.75f},false);
            for(float x:{-3.8f,-1.9f,0.0f,1.9f,3.8f}) b.cylinder_y(b.mesh.interior,{x,y+0.35f,z},0.65f,0.06f,6,false);
        }
    }
    // Environmental control column and ducting.
    b.cylinder_y(b.mesh.details,{0,3.1f,0},3.6f,0.38f,12,true);
    b.cylinder_between(b.mesh.details,{-4.7f,4.2f,0},{4.7f,4.2f,0},0.12f,8,true,true);
    // Entry/service module on +X.
    b.box(b.mesh.shell,{6.05f,2.05f,0},{1.7f,3.5f,3.1f});
    b.torus_x(b.mesh.details,{6.92f,2.05f,0},0.58f,0.09f,16,6);
    b.cylinder_x(b.mesh.details,{6.97f,2.05f,0},0.12f,0.48f,14,true);
    b.box(b.mesh.details,{5.98f,2.15f,1.8f},{0.7f,1.2f,0.42f});
    // Support legs.
    b.supports(-4.2f,4.1f,2.65f,0.78f);
    b.supports(-4.2f,4.1f,-2.65f,0.78f);
    b.mesh.entrances.push_back({{7.00f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    return std::move(b.mesh);
}

HabitatMesh make_crew_quarters(){
    Builder b;
    b.mesh.footprint_x=14.0f; b.mesh.footprint_z=7.0f; b.mesh.height=6.0f;
    b.cylinder_x(b.mesh.shell,{0,2.65f,0},10.8f,2.35f,18,true);
    for(float x:{-4.4f,-2.2f,0.0f,2.2f,4.4f}) b.torus_x(b.mesh.details,{x,2.65f,0},2.38f,0.08f,18,6);
    // Repeated sleeping-compartment portholes on the visible side.
    for(float x:{-3.3f,0.0f,3.3f}){
        b.torus_z(b.mesh.details,{x,2.75f,2.39f},0.48f,0.075f,16,6);
        b.cylinder_z(b.mesh.glazing,{x,2.75f,2.43f},0.08f,0.40f,16,false);
        b.box(b.mesh.details,{x-0.75f,1.55f,2.38f},{0.55f,0.62f,0.16f});
    }
    // End vestibule and hatch.
    b.box(b.mesh.shell,{5.75f,2.35f,0},{1.55f,3.7f,3.55f});
    b.torus_x(b.mesh.details,{6.55f,2.35f,0},0.62f,0.09f,16,6);
    b.cylinder_x(b.mesh.details,{6.60f,2.35f,0},0.12f,0.52f,14,true);
    // Roof environmental units.
    b.cylinder_y(b.mesh.details,{-2.9f,5.1f,-0.4f},0.65f,0.52f,12,true);
    b.cylinder_y(b.mesh.details,{1.5f,5.0f,0.55f},0.70f,0.45f,12,true);
    b.box(b.mesh.details,{3.2f,4.65f,-0.65f},{1.2f,0.65f,0.9f});
    // Underfloor service trunk / basement remains opaque in view modes.
    b.box(b.mesh.foundation,{0,0.72f,0},{8.6f,0.65f,1.35f});
    // Interior center aisle and bunk modules.
    b.box(b.mesh.interior,{0,0.95f,0},{9.4f,0.16f,3.4f},false);
    for(float x:{-3.5f,-1.2f,1.2f,3.5f}){
        b.box(b.mesh.interior,{x,1.45f,-1.45f},{1.65f,0.55f,0.72f},false);
        b.box(b.mesh.interior,{x,2.25f,-1.45f},{1.65f,0.55f,0.72f},false);
        b.box(b.mesh.interior,{x,1.45f,1.45f},{1.65f,0.55f,0.72f},false);
        b.box(b.mesh.interior,{x,2.25f,1.45f},{1.65f,0.55f,0.72f},false);
    }
    b.supports(-4.0f,3.9f,1.7f,0.88f);
    b.supports(-4.0f,3.9f,-1.7f,0.88f);
    b.mesh.entrances.push_back({{6.65f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    return std::move(b.mesh);
}

} // namespace

HabitatModelLibrary::HabitatModelLibrary(){
    meshes_[(size_t)HabitatKind::AirLock]=make_airlock();
    meshes_[(size_t)HabitatKind::OxygenGeneratorDome]=make_oxygen_dome();
    meshes_[(size_t)HabitatKind::CantineDome]=make_cantine_dome();
    meshes_[(size_t)HabitatKind::BioDome]=make_bio_dome();
    meshes_[(size_t)HabitatKind::CrewQuarters]=make_crew_quarters();
}

const HabitatMesh& HabitatModelLibrary::mesh(HabitatKind kind) const {
    return meshes_[(size_t)kind];
}

} // namespace game
