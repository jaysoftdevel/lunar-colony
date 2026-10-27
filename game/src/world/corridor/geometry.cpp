#include "game/world/corridor/detail.hpp"
#include <array>
#include <vector>

namespace game::corridor_detail {
void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c){ dst.push_back(a); dst.push_back(b); dst.push_back(c); }
void edge(std::vector<Vec3>& dst,Vec3 a,Vec3 b){ dst.push_back(a); dst.push_back(b); }

std::vector<Vec2> corridor_cross_section(float scale){
    std::vector<Vec2> out;
    out.reserve(12);
    out.push_back({-CORRIDOR_HALF_WIDTH*scale,CORRIDOR_FLOOR_Y});
    out.push_back({ CORRIDOR_HALF_WIDTH*scale,CORRIDOR_FLOOR_Y});
    // Flat floor + short vertical walls + a rounded roof. This keeps the
    // pressure tube curved without producing a fully circular tunnel.
    constexpr int ARC=10;
    for(int i=0;i<=ARC;++i){
        const float a=(float)i/(float)ARC*PI;
        out.push_back({std::cos(a)*CORRIDOR_HALF_WIDTH*scale,
                       CORRIDOR_SIDE_Y+std::sin(a)*CORRIDOR_ROOF_RADIUS*scale});
    }
    return out;
}

Vec3 ring_vertex(Vec3 center,Vec3 side,const Vec2& q){
    return {center.x+side.x*q.x,center.y+q.y,center.z+side.z*q.x};
}

Vec3 path_tangent(const std::vector<Vec3>& path,std::size_t i){
    if(path.size()<2) return {1,0,0};
    if(i==0) return flat_normal(path[1]-path[0]);
    if(i+1>=path.size()) return flat_normal(path.back()-path[path.size()-2]);
    const Vec3 t=flat_normal(path[i+1]-path[i-1]);
    return length(t)>0.001f?t:flat_normal(path[i+1]-path[i]);
}

void append_swept_shell(CorridorMesh& mesh,const std::vector<Vec3>& path){
    if(path.size()<2) return;
    const auto section=corridor_cross_section();
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 t0=path_tangent(path,i);
        const Vec3 t1=path_tangent(path,i+1);
        const Vec3 s0=perp_xz(t0);
        const Vec3 s1=perp_xz(t1);
        const Vec3 c0{path[i].x,path[i].y,path[i].z};
        const Vec3 c1{path[i+1].x,path[i+1].y,path[i+1].z};
        for(std::size_t j=0;j<section.size();++j){
            const std::size_t k=(j+1)%section.size();
            const Vec3 a=ring_vertex(c0,s0,section[j]);
            const Vec3 b=ring_vertex(c1,s1,section[j]);
            const Vec3 c=ring_vertex(c1,s1,section[k]);
            const Vec3 d=ring_vertex(c0,s0,section[k]);
            tri(mesh.shell,a,b,c); tri(mesh.shell,a,c,d);
        }
    }
}

void append_walkway(CorridorMesh& mesh,const std::vector<Vec3>& path){
    if(path.size()<2) return;
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 t0=path_tangent(path,i),t1=path_tangent(path,i+1);
        const Vec3 s0=perp_xz(t0)*WALKWAY_HALF_WIDTH;
        const Vec3 s1=perp_xz(t1)*WALKWAY_HALF_WIDTH;
        const Vec3 p0{path[i].x,path[i].y+WALKWAY_Y,path[i].z};
        const Vec3 p1{path[i+1].x,path[i+1].y+WALKWAY_Y,path[i+1].z};
        tri(mesh.details,p0-s0,p1-s1,p1+s1);
        tri(mesh.details,p0-s0,p1+s1,p0+s0);
        edge(mesh.edges,p0-s0,p1-s1);
        edge(mesh.edges,p0+s0,p1+s1);
    }
}

void append_ring(CorridorMesh& mesh,Vec3 center,Vec3 tangent,float scale){
    tangent=flat_normal(tangent);
    if(length(tangent)<0.001f) return;
    const Vec3 side=perp_xz(tangent);
    const auto section=corridor_cross_section(scale);
    for(std::size_t j=0;j<section.size();++j){
        const std::size_t k=(j+1)%section.size();
        edge(mesh.edges,ring_vertex(center,side,section[j]),ring_vertex(center,side,section[k]));
    }
}

void append_ribs(CorridorMesh& mesh,const std::vector<Vec3>& path){
    if(path.size()<2) return;
    float next=1.7f;
    float travelled=0.0f;
    for(std::size_t i=0;i+1<path.size();++i){
        const float seg=distance_xz(path[i],path[i+1]);
        if(seg<0.001f) continue;
        while(travelled+seg>=next){
            const float t=(next-travelled)/seg;
            const Vec3 p=path[i]+(path[i+1]-path[i])*t;
            append_ring(mesh,{p.x,0,p.z},path[i+1]-path[i]);
            next+=1.7f;
        }
        travelled+=seg;
    }
}

} // namespace game::corridor_detail
