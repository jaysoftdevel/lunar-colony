#include "game/world/corridor/detail.hpp"
#include "game/world/structural_frame.hpp"
#include <array>
#include <limits>
#include <vector>

namespace game::corridor_detail {
void tri(std::vector<Vec3>& dst,Vec3 a,Vec3 b,Vec3 c){ dst.push_back(a); dst.push_back(b); dst.push_back(c); }
namespace {
float segment_distance_approx_for_self_test(Vec3 a0,Vec3 a1,Vec3 b0,Vec3 b1){
    float best=std::numeric_limits<float>::max();
    for(int i=0;i<=5;++i){
        const float t=(float)i/5.0f;
        const Vec3 p=a0+(a1-a0)*t;
        best=std::min(best,point_segment_distance_xz(p,b0,b1));
    }
    for(int i=0;i<=5;++i){
        const float t=(float)i/5.0f;
        const Vec3 p=b0+(b1-b0)*t;
        best=std::min(best,point_segment_distance_xz(p,a0,a1));
    }
    return best;
}
}

bool corridor_path_self_overlaps(const std::vector<Vec3>& path){
    constexpr float CLEARANCE=CORRIDOR_HALF_WIDTH*1.82f;
    if(path.size()<4) return false;
    std::vector<float> along(path.size(),0.0f);
    for(std::size_t i=1;i<path.size();++i) along[i]=along[i-1]+distance_xz(path[i-1],path[i]);
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 di=flat_normal(path[i+1]-path[i]);
        const float li=distance_xz(path[i],path[i+1]);
        for(std::size_t j=i+2;j+1<path.size();++j){
            const float lj=distance_xz(path[j],path[j+1]);
            const float route_separation=(along[j]+lj*0.5f)-(along[i]+li*0.5f);
            if(route_separation<CORRIDOR_HALF_WIDTH*1.35f) continue;

            Vec3 hit{};
            if(segment_intersection_xz(path[i],path[i+1],path[j],path[j+1],hit,0.001f)) return true;

            const float spacing=segment_distance_approx_for_self_test(path[i],path[i+1],path[j],path[j+1]);
            if(spacing>=CLEARANCE) continue;
            const Vec3 dj=flat_normal(path[j+1]-path[j]);
            // Catch a tight hairpin even when its two legs do not mathematically
            // cross. Their swept pressure shells still overlap and form the
            // impassable knot seen in the 0.40.25 triangle save.
            if(length(di)>0.001f && length(dj)>0.001f && dot(di,dj)<-0.35f) return true;

            // For same-direction/non-hairpin portions retain the wider route-
            // distance exemption so ordinary smooth bends are not rejected.
            if(route_separation>=CLEARANCE*2.4f) return true;
        }
    }
    return false;
}

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
        tri(mesh.details,p0-s0,p1+s1,p1-s1);
        tri(mesh.details,p0-s0,p0+s0,p1+s1);
        edge(mesh.edges,p0-s0,p1-s1);
        edge(mesh.edges,p0+s0,p1+s1);
    }
}


namespace {
void append_detail_tube(CorridorMesh& mesh,Vec3 a,Vec3 b,float radius,int segments=6){
    const Vec3 axis=b-a;
    const float len=length(axis);
    if(len<0.001f) return;
    const Vec3 n=axis*(1.0f/len);
    const Vec3 ref=std::abs(n.y)<0.92f?Vec3{0,1,0}:Vec3{1,0,0};
    const Vec3 u=normalize(cross(n,ref));
    const Vec3 v=normalize(cross(n,u));
    segments=std::max(6,segments);
    for(int i=0;i<segments;++i){
        const float a0=2.0f*PI*(float)i/(float)segments;
        const float a1=2.0f*PI*(float)(i+1)/(float)segments;
        const Vec3 r0=(u*std::cos(a0)+v*std::sin(a0))*radius;
        const Vec3 r1=(u*std::cos(a1)+v*std::sin(a1))*radius;
        tri(mesh.details,a+r0,b+r0,b+r1);
        tri(mesh.details,a+r0,b+r1,a+r1);
        if(i%2==0){ edge(mesh.edges,a+r0,b+r0); }
    }
}
}

void append_pressure_services(CorridorMesh& mesh,const std::vector<Vec3>& path){
    if(path.size()<2) return;
    // Corridors are part of the colony utility backbone.  A dedicated water
    // pipe and redundant power conduits run continuously through the pressure
    // shell and follow the exact swept corridor path through bends/junctions.
    for(std::size_t i=0;i+1<path.size();++i){
        const Vec3 t0=path_tangent(path,i),t1=path_tangent(path,i+1);
        const Vec3 s0=perp_xz(t0),s1=perp_xz(t1);
        const Vec3 a=path[i],b=path[i+1];

        // Water main: deliberately larger and low in the service void.
        append_detail_tube(mesh,a+s0*(-1.15f)+Vec3{0,0.48f,0},
                                b+s1*(-1.15f)+Vec3{0,0.48f,0},0.058f,8);

        // Power backbone: two smaller redundant conduits on the opposite side.
        append_detail_tube(mesh,a+s0*(1.15f)+Vec3{0,0.48f,0},
                                b+s1*(1.15f)+Vec3{0,0.48f,0},0.033f,6);
        append_detail_tube(mesh,a+s0*(0.88f)+Vec3{0,2.22f,0},
                                b+s1*(0.88f)+Vec3{0,2.22f,0},0.026f,6);
    }
}

void append_deck_panels(CorridorMesh& mesh,const std::vector<Vec3>& path){
    if(path.size()<2)return;
    std::vector<float> stations(path.size(),0.f);
    for(std::size_t i=1;i<path.size();++i)stations[i]=stations[i-1]+distance_xz(path[i-1],path[i]);
    const float total=stations.back();if(total<.01f)return;
    const auto at=[&](float distance,float lateral,float y){
        auto end=std::upper_bound(stations.begin(),stations.end(),distance);
        const auto i=std::min(std::size_t(end-stations.begin()),path.size()-1);
        const auto j=i?i-1:0;const float span=stations[i]-stations[j];
        const float t=span>.0001f?std::clamp((distance-stations[j])/span,0.f,1.f):0.f;
        const auto tangent=normalize(path_tangent(path,j)*(1-t)+path_tangent(path,i)*t);
        return path[j]+(path[i]-path[j])*t+perp_xz(tangent)*lateral+Vec3{0,y,0};
    };
    // Removable three-lane cassettes: real chamfered edges, recessed fasteners,
    // inset anti-slip strips and continuous support rails, swept in route space.
    // All relief stays below 12 mm; the authoritative walking plane stays .20 m.
    structural::Builder metal{mesh.details,mesh.edges,{}};
    constexpr float lane=2*WALKWAY_HALF_WIDTH/3;
    const int panels=std::max(1,int(std::ceil(total/.90f)));
    for(int panel=0;panel<panels;++panel){
        const float start=total*panel/panels+.006f,finish=total*(panel+1)/panels-.006f;
        if(finish<=start)continue;
        for(int col=0;col<3;++col){
            const float left=-WALKWAY_HALF_WIDTH+col*lane+.006f,right=left+lane-.012f;
            const float c=std::min(.025f,(finish-start)*.20f);
            const std::array<Vec2,8> rim{{{start+c,left},{finish-c,left},{finish,left+c},{finish,right-c},
                {finish-c,right},{start+c,right},{start,right-c},{start,left+c}}};
            const auto center=at((start+finish)*.5f,(left+right)*.5f,WALKWAY_Y+.004f);
            for(int i=0;i<8;++i){const int j=(i+1)%8;
                const auto a=at(rim[i].x,rim[i].y,WALKWAY_Y+.004f),b=at(rim[j].x,rim[j].y,WALKWAY_Y+.004f);
                // Route direction and lateral form a left-handed horizontal basis.
                tri(mesh.details,center,b,a);
                const auto lowa=a-Vec3{0,.024f,0},lowb=b-Vec3{0,.024f,0};
                tri(mesh.details,a,b,lowb);tri(mesh.details,a,lowb,lowa);
            }
            const auto tangent=flat_normal(at(finish,0,0)-at(start,0,0)),side=perp_xz(tangent);
            for(float distance:{start+.052f,finish-.052f})for(float lateral:{left+.045f,right-.045f}){
                // Low captive screw heads and a physical slot, not floating lines.
                const auto p=at(distance,lateral,WALKWAY_Y+.006f);
                metal.box(p,side*-1.f,{0,1,0},tangent,{.030f,.004f,.030f});
                metal.box(p+Vec3{0,.0025f,0},side*-1.f,{0,1,0},tangent,{.004f,.001f,.020f});
            }
            for(int strip=0;strip<6;++strip){
                const float distance=std::lerp(start+.15f,finish-.15f,float(strip)/5);
                if(finish-start<.32f)break;
                const auto p=at(distance,(left+right)*.5f,WALKWAY_Y+.008f);
                metal.box(p,side*-1.f,{0,1,0},tangent,{lane-.16f,.006f,.018f});
            }
            // Two recessed longitudinal grip channels outline each tread insert.
            for(float lateral:{left+.075f,right-.075f}){
                const auto a=at(start+.12f,lateral,WALKWAY_Y+.005f),b=at(finish-.12f,lateral,WALKWAY_Y+.005f);
                if(finish-start>.28f)metal.beam(a,b,.010f,.002f,{0,1,0});
            }
        }
    }
    for(std::size_t i=1;i<path.size();++i)for(float side:{-1.f,1.f}){
        const auto a=path[i-1]+perp_xz(path_tangent(path,i-1))*(side*(WALKWAY_HALF_WIDTH+.04f))+Vec3{0,.13f,0};
        const auto b=path[i]+perp_xz(path_tangent(path,i))*(side*(WALKWAY_HALF_WIDTH+.04f))+Vec3{0,.13f,0};
        metal.beam(a,b,.09f,.12f,{0,1,0},true);
    }
}

void append_ring(CorridorMesh& mesh,Vec3 center,Vec3 tangent,float scale){
    tangent=flat_normal(tangent);if(length(tangent)<.001f)return;
    const Vec3 side=perp_xz(tangent);const auto section=corridor_cross_section(std::min(scale,.94f));
    structural::Builder frame{mesh.details,mesh.edges,{}};
    const auto node=[&](const Vec2& q){
        auto p=ring_vertex(center,side,q);
        // Lower tie clears the ground and supports the underside of the deck.
        if(q.y<=CORRIDOR_FLOOR_Y+.001f)p.y= center.y+.10f;
        return p;
    };
    for(std::size_t j=0;j<section.size();++j){const auto k=(j+1)%section.size();
        const auto a=node(section[j]),b=node(section[k]);
        Vec3 normal=(a+b)*.5f-center-Vec3{0,CORRIDOR_SIDE_Y,0};
        if(j==0)normal={0,-1,0};
        frame.beam(a,b,.13f,.16f,normal,true);
    }
    for(float sign:{-1.f,1.f}){
        const auto foot=center+side*(sign*(CORRIDOR_HALF_WIDTH*.88f));
        // The route may be elevated or sloped. Feet bear on the lunar build
        // plane, while telescopic legs terminate at the actual deck frame.
        const Vec3 ground{foot.x,0,foot.z};
        frame.box(ground+Vec3{0,.024f,0},side*-1.f,{0,1,0},tangent,{.40f,.048f,.40f});
        frame.beam(ground+Vec3{0,.035f,0},foot+Vec3{0,.17f,0},.18f,.16f,side);
        if(center.y>.20f){
            frame.beam(ground+side*(-sign*.12f)+Vec3{0,.05f,0},
                       foot+side*(-sign*.20f)+Vec3{0,.12f,0},.065f,.07f,tangent);
        }
    }
}

void append_ribs(CorridorMesh& mesh,const std::vector<Vec3>& path){
    if(path.size()<2)return;
    float total=0;for(std::size_t i=1;i<path.size();++i)total+=distance_xz(path[i-1],path[i]);
    if(total<.01f)return;
    // A continuous ring at each end plus one in every <=1.35 m bay.
    const float end=std::min(.15f,total*.25f);const int bays=std::max(1,int(std::ceil((total-2*end)/1.35f)));
    for(int ring=0;ring<=bays;++ring){
        const float station=std::lerp(end,total-end,float(ring)/bays);float travelled=0;
        for(std::size_t i=1;i<path.size();++i){const float span=distance_xz(path[i-1],path[i]);
            if(span>.001f&&travelled+span>=station-.0001f){
                const float t=std::clamp((station-travelled)/span,0.f,1.f);const auto p=path[i-1]+(path[i]-path[i-1])*t;
                const auto tangent=normalize(path_tangent(path,i-1)*(1-t)+path_tangent(path,i)*t);
                append_ring(mesh,p,tangent);
                break;
            }
            travelled+=span;
        }
    }
    structural::Builder frame{mesh.details,mesh.edges,{}};
    // Stringers meet every hoop. They run inside the skin, outside the deck/head envelope, and
    // continue through each swept bend in the pressure shell's local basis.
    const std::array<Vec2,7> rails{{{-1.38f,.66f},{1.38f,.66f},{-1.38f,.89f},{1.38f,.89f},{-.98f,1.98f},{.98f,1.98f},{0,2.42f}}};
    for(std::size_t i=1;i<path.size();++i){
        const auto a=perp_xz(path_tangent(path,i-1)),b=perp_xz(path_tangent(path,i));
        for(const auto& q:rails)frame.beam(ring_vertex(path[i-1],a,q),ring_vertex(path[i],b,q),.075f,.105f,{0,q.y-CORRIDOR_SIDE_Y,0});
    }
}

} // namespace game::corridor_detail

namespace game {
void CorridorMesh::prepare_surfaces(const std::vector<runtime::Vec3>& path) const {
    using namespace runtime;
    if(surface_batches_ready&&deck_surface.size()+metal_surface.size()+grip_surface.size()==details.size())return;
    deck_surface.clear();metal_surface.clear();grip_surface.clear();
    for(std::size_t v=0;v+2<details.size();v+=3){
        const auto a=details[v],b=details[v+1],c=details[v+2],p=(a+b+c)/3.f;
        const auto n=normalize(cross(b-a,c-a));float base=0,best=std::numeric_limits<float>::max();
        for(std::size_t j=1;j<path.size();++j){const auto q=corridor_detail::closest_point_segment_xz(p,path[j-1],path[j]);const float d=corridor_detail::distance_xz(p,q);if(d<best){best=d;base=q.y;}}
        const bool horizontal=std::abs(n.y)>.80f;
        const float relief=p.y-base-geometry::kCorridorDeckY;
        auto& layer=horizontal&&relief>.0095f&&relief<.016f?grip_surface:
            horizontal&&relief<.12f?deck_surface:metal_surface;
        layer.insert(layer.end(),{a,b,c});
    }
    surface_batches_ready=true;
}
} // namespace game
