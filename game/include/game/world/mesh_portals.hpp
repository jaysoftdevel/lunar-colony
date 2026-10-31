#pragma once
#include "engine/math.hpp"
#include <algorithm>
#include <array>
#include <vector>

namespace game::mesh_portals {
// Exact subtraction of an oriented aperture prism. Unlike dropping any
// triangle touched by a door, this preserves the pressure wall around it and
// also cuts a doorway out of a single large triangle with no vertex inside.
struct Aperture {
    eng::Vec3 center{},normal{1,0,0};
    float half_depth{0.85f},half_width{0.70f};
    float bottom{0.0f},top{2.45f};
};
inline std::vector<eng::Vec3> clip(const std::vector<eng::Vec3>& polygon,eng::Vec3 normal,float offset,bool outside){
    std::vector<eng::Vec3> out;
    if(polygon.empty()) return out;
    auto distance=[&](eng::Vec3 p){return eng::dot(p,normal)-offset;};
    auto previous=polygon.back();float pd=distance(previous);
    for(auto p:polygon){
        const float d=distance(p);
        const bool a=outside?pd>=0.0f:pd<=0.0f,b=outside?d>=0.0f:d<=0.0f;
        if(a!=b){const float t=pd/(pd-d);out.push_back(previous+(p-previous)*t);}
        if(b) out.push_back(p);
        previous=p;pd=d;
    }
    return out;
}
inline std::vector<eng::Vec3> subtract(const std::vector<eng::Vec3>& triangles,const Aperture& opening){
    using eng::Vec3;
    Vec3 n=eng::normalize({opening.normal.x,0,opening.normal.z});
    const Vec3 side{-n.z,0,n.x},up{0,1,0};
    const float axial=eng::dot(opening.center,n),lateral=eng::dot(opening.center,side);
    const std::array<std::pair<Vec3,float>,6> planes{{
        {n,axial+opening.half_depth},{n*-1,-axial+opening.half_depth},
        {side,lateral+opening.half_width},{side*-1,-lateral+opening.half_width},
        {up,opening.top},{up*-1,-opening.bottom}}};
    std::vector<Vec3> out;out.reserve(triangles.size());
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        bool disjoint=false;
        for(const auto& [normal,offset]:planes)
            if(eng::dot(triangles[i],normal)>offset && eng::dot(triangles[i+1],normal)>offset && eng::dot(triangles[i+2],normal)>offset){disjoint=true;break;}
        if(disjoint){out.insert(out.end(),triangles.begin()+i,triangles.begin()+i+3);continue;}
        std::vector<Vec3> inside{triangles[i],triangles[i+1],triangles[i+2]};
        for(const auto& [normal,offset]:planes){
            auto outside=clip(inside,normal,offset,true);
            for(std::size_t j=1;j+1<outside.size();++j){
                const auto a=outside[0],b=outside[j],c=outside[j+1];
                if(eng::length(eng::cross(b-a,c-a))<1.0e-7f) continue;
                out.insert(out.end(),{a,b,c});
            }
            inside=clip(inside,normal,offset,false);
            if(inside.empty()) break;
        }
    }
    return out;
}

inline std::vector<eng::Vec3> subtract_lines(const std::vector<eng::Vec3>& lines,const Aperture& opening){
    using eng::Vec3;
    const Vec3 n=eng::normalize({opening.normal.x,0,opening.normal.z}),side{-n.z,0,n.x};
    std::vector<Vec3> out;out.reserve(lines.size());
    for(std::size_t i=0;i+1<lines.size();i+=2){
        const auto a=lines[i],b=lines[i+1];float enter=0,leave=1;
        auto slab=[&](float from,float to,float lo,float hi){
            const float delta=to-from;
            if(std::abs(delta)<1.0e-7f) return from>=lo && from<=hi;
            float u=(lo-from)/delta,v=(hi-from)/delta;if(u>v) std::swap(u,v);
            enter=std::max(enter,u);leave=std::min(leave,v);return enter<=leave;
        };
        const auto ra=a-opening.center,rb=b-opening.center;
        const bool hit=slab(eng::dot(ra,n),eng::dot(rb,n),-opening.half_depth,opening.half_depth) &&
            slab(eng::dot(ra,side),eng::dot(rb,side),-opening.half_width,opening.half_width) &&
            slab(a.y,b.y,opening.bottom,opening.top);
        if(!hit){out.insert(out.end(),{a,b});continue;}
        if(enter>0.00001f) out.insert(out.end(),{a,a+(b-a)*enter});
        if(leave<0.99999f) out.insert(out.end(),{a+(b-a)*leave,b});
    }
    return out;
}
} // namespace game::mesh_portals
