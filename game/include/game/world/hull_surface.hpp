#pragma once
#include "game/runtime/presentation.hpp"
#include <map>
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
}
