#pragma once
#include "game/runtime/presentation.hpp"
#include <map>
#include <algorithm>
#include <cstring>
#include <tuple>
namespace game::hull_surface {
using runtime::Vec3;
struct Parts {std::vector<Vec3> lower,upper;};
inline const Parts& split(const std::vector<Vec3>& source,float height){
    using Key=std::tuple<std::uintptr_t,std::size_t,float>;
    struct Entry {Parts parts;std::vector<Vec3> source;};
    static std::map<Key,Entry> cache;
    const Key key{reinterpret_cast<std::uintptr_t>(source.data()),source.size(),height};
    auto& entry=cache[key];
    // Corridor junction rebuilds can reuse the same allocation and vertex count.
    // Compare retained bytes before reusing the split, just like GPU mesh uploads.
    if(entry.source.size()==source.size() && (source.empty() || !std::memcmp(entry.source.data(),source.data(),source.size()*sizeof(Vec3))))return entry.parts;
    entry.source=source;entry.parts={};
    auto& parts=entry.parts;
    for(std::size_t i=0;i+2<source.size();i+=3)for(bool upper:{false,true}){
        std::vector<Vec3> polygon;
        for(int edge=0;edge<3;++edge){
            const Vec3 a=source[i+edge],b=source[i+(edge+1)%3];
            const bool inside=upper?a.y>=height:a.y<=height,next=upper?b.y>=height:b.y<=height;
            if(inside)polygon.push_back(a);
            if(inside!=next)polygon.push_back(a+(b-a)*((height-a.y)/(b.y-a.y)));
        }
        auto& out=upper?parts.upper:parts.lower;
        for(std::size_t j=1;j+1<polygon.size();++j)out.insert(out.end(),{polygon[0],polygon[j],polygon[j+1]});
    }
    return parts;
}
inline const std::vector<Vec3>& lower_run(const std::vector<Vec3>& source,const std::vector<Vec3>& path,float above){
    using Key=std::tuple<std::uintptr_t,std::size_t,float>;
    struct Entry {std::vector<Vec3> source,path,lower;};static std::map<Key,Entry> cache;
    auto& e=cache[{reinterpret_cast<std::uintptr_t>(source.data()),source.size(),above}];
    const auto same=[](const auto& a,const auto& b){return a.size()==b.size()&&(a.empty()||!std::memcmp(a.data(),b.data(),a.size()*sizeof(Vec3)));};
    if(same(e.source,source)&&same(e.path,path))return e.lower;
    e.source=source;e.path=path;e.lower.clear();
    const auto distance=[&](Vec3 p){float best=1e30f,floor=path.empty()?0:path.front().y;
        for(std::size_t i=1;i<path.size();++i){const auto a=path[i-1],d=path[i]-a;const float len=d.x*d.x+d.z*d.z;
            const float t=len>1e-8f?std::clamp(((p.x-a.x)*d.x+(p.z-a.z)*d.z)/len,0.f,1.f):0;
            const auto q=a+d*t;const float ds=(q.x-p.x)*(q.x-p.x)+(q.z-p.z)*(q.z-p.z);
            if(ds<best){best=ds;floor=q.y;}
        }return p.y-floor-above;
    };
    for(std::size_t i=0;i+2<source.size();i+=3){std::vector<Vec3> polygon;
        for(int j=0;j<3;++j){const auto a=source[i+j],b=source[i+(j+1)%3];const float da=distance(a),db=distance(b);
            if(da<=0)polygon.push_back(a);
            if((da<=0)!=(db<=0))polygon.push_back(a+(b-a)*(da/(da-db)));
        }
        for(std::size_t j=1;j+1<polygon.size();++j)e.lower.insert(e.lower.end(),{polygon[0],polygon[j],polygon[j+1]});
    }return e.lower;
}
}
