#pragma once
#include "game/world/habitat_mesh.hpp"
#include <array>
#include <algorithm>
namespace game::interior_finish {
using runtime::Vec3;
inline double side(Vec3 a,Vec3 b,Vec3 p){return double(b.x-a.x)*(p.z-a.z)-double(b.z-a.z)*(p.x-a.x);}
struct Edge {double low,high,m,b;double at(double x) const{return m*x+b;}};
// A planar sweep unions overlapping floor plates without recursive polygon
// fragmentation. Edge crossings split slabs so each union boundary is linear.
inline std::vector<Vec3> deck_union(const std::vector<Vec3>& source,float y){
    std::vector<double> events;std::vector<Edge> edges;
    for(std::size_t i=0;i+2<source.size();i+=3)for(int j=0;j<3;++j){
        const auto a=source[i+j],b=source[i+(j+1)%3];events.push_back(a.x);
        if(std::abs(b.x-a.x)>1e-7){const double m=double(b.z-a.z)/double(b.x-a.x);edges.push_back({std::min(a.x,b.x),std::max(a.x,b.x),m,a.z-m*a.x});}
    }
    for(std::size_t i=0;i<edges.size();++i)for(std::size_t j=i+1;j<edges.size();++j){
        const auto& a=edges[i];const auto& b=edges[j];if(std::abs(a.m-b.m)<1e-9)continue;
        const double x=(b.b-a.b)/(a.m-b.m);
        if(x>std::max(a.low,b.low)+1e-6&&x<std::min(a.high,b.high)-1e-6)events.push_back(x);
    }
    std::sort(events.begin(),events.end());events.erase(std::unique(events.begin(),events.end(),[](double a,double b){return std::abs(a-b)<1e-5;}),events.end());
    std::vector<Vec3> out;
    const auto triangle=[&](Vec3 a,Vec3 b,Vec3 c){const auto area=side(a,b,c);if(std::abs(area)<1e-7)return;if(area>0)std::swap(b,c);out.insert(out.end(),{a,b,c});};
    struct Interval {Edge low,high;};
    for(std::size_t k=1;k<events.size();++k){
        const auto x0=events[k-1],x1=events[k],mid=(x0+x1)*.5;std::vector<Interval> intervals;
        for(std::size_t i=0;i+2<source.size();i+=3){std::vector<Edge> crossed;
            for(int j=0;j<3;++j){const auto a=source[i+j],b=source[i+(j+1)%3];
                if(mid<=std::min(a.x,b.x)||mid>=std::max(a.x,b.x))continue;
                const double m=double(b.z-a.z)/double(b.x-a.x);crossed.push_back({std::min(a.x,b.x),std::max(a.x,b.x),m,a.z-m*a.x});
            }
            if(crossed.size()!=2)continue;
            if(crossed[0].at(mid)>crossed[1].at(mid))std::swap(crossed[0],crossed[1]);
            intervals.push_back({crossed[0],crossed[1]});
        }
        std::sort(intervals.begin(),intervals.end(),[&](const auto& a,const auto& b){return a.low.at(mid)<b.low.at(mid);});
        const auto emit=[&](const Interval& p){
            const Vec3 a{float(x0),y,float(p.low.at(x0))},b{float(x1),y,float(p.low.at(x1))},c{float(x1),y,float(p.high.at(x1))},d{float(x0),y,float(p.high.at(x0))};
            triangle(a,b,c);triangle(a,c,d);
        };
        if(intervals.empty())continue;
        auto merged=intervals[0];
        for(std::size_t i=1;i<intervals.size();++i){const auto& next=intervals[i];
            if(next.low.at(mid)<=merged.high.at(mid)+1e-6){if(next.high.at(mid)>merged.high.at(mid))merged.high=next.high;}
            else {emit(merged);merged=next;}
        }emit(merged);
    }return out;
}
inline void finish(HabitatMesh& mesh){
    const float floor=mesh.walkable_floor_y;std::vector<Vec3> floor_faces;
    for(std::size_t i=0;i+2<mesh.deck.size();i+=3)
        if(std::abs(mesh.deck[i].y-floor)<.025f&&std::abs(mesh.deck[i+1].y-floor)<.025f&&std::abs(mesh.deck[i+2].y-floor)<.025f)
            floor_faces.insert(floor_faces.end(),mesh.deck.begin()+i,mesh.deck.begin()+i+3);
    std::vector<Vec3> interior;std::vector<std::size_t> prefix(mesh.interior.size()+1);
    for(std::size_t i=0;i+2<mesh.interior.size();i+=3){
        bool flat=true;for(int j=0;j<3;++j)flat&=std::abs(mesh.interior[i+j].y-floor)<.015f;
        if(flat)floor_faces.insert(floor_faces.end(),mesh.interior.begin()+i,mesh.interior.begin()+i+3);
        else interior.insert(interior.end(),mesh.interior.begin()+i,mesh.interior.begin()+i+3);
        for(int j=1;j<=3;++j)prefix[i+j]=interior.size();
    }
    for(auto& station:mesh.stations){const auto begin=station.first[0],end=begin+station.count[0];station.first[0]=prefix[begin];station.count[0]=prefix[end]-prefix[begin];}
    mesh.interior=std::move(interior);mesh.deck=deck_union(floor_faces,floor+.012f);
}
} // namespace game::interior_finish
