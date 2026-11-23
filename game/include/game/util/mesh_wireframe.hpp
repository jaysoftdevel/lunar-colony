#pragma once
#include "game/runtime/presentation.hpp"
#include <vector>
#include <array>
#include <bit>
#include <cstring>
#include <unordered_set>
#include <algorithm>

namespace game::mesh_wireframe {

inline void append_triangle_edges(std::vector<game::runtime::Vec3>& out,const std::vector<game::runtime::Vec3>& triangles){
    out.reserve(out.size()+(triangles.size()/3)*6);
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        const game::runtime::Vec3& a=triangles[i];
        const game::runtime::Vec3& b=triangles[i+1];
        const game::runtime::Vec3& c=triangles[i+2];
        out.push_back(a); out.push_back(b);
        out.push_back(b); out.push_back(c);
        out.push_back(c); out.push_back(a);
    }
}

inline std::vector<game::runtime::Vec3> triangle_edges(const std::vector<game::runtime::Vec3>& triangles){
    std::vector<game::runtime::Vec3> out;
    append_triangle_edges(out,triangles);
    return out;
}

// Retain exact source content, never a borrowed address: most callers transform
// into a temporary vector and allocators reuse those addresses across objects.
inline const std::vector<game::runtime::Vec3>& unique_triangle_edges(const std::vector<game::runtime::Vec3>& triangles){
    using Vec3=game::runtime::Vec3;
    struct Entry {std::vector<Vec3> source,edges;std::uint64_t used{};};
    struct Hash {std::size_t operator()(const std::array<std::uint32_t,6>& key) const{
        std::size_t h=0;for(auto value:key)h^=value+0x9e3779b9u+(h<<6)+(h>>2);return h;
    }};
    struct Cache {std::vector<Entry> entries;std::size_t bytes{};std::uint64_t clock{};};
    thread_local Cache cache;
    ++cache.clock;
    for(auto& entry:cache.entries)if(entry.source.size()==triangles.size() &&
        !std::memcmp(entry.source.data(),triangles.data(),triangles.size()*sizeof(Vec3))){entry.used=cache.clock;return entry.edges;}
    Entry entry;entry.source=triangles;entry.used=cache.clock;
    std::unordered_set<std::array<std::uint32_t,6>,Hash> seen;seen.reserve(triangles.size());
    const auto bits=[](Vec3 p){return std::array<std::uint32_t,3>{std::bit_cast<std::uint32_t>(p.x==0?0.f:p.x),std::bit_cast<std::uint32_t>(p.y==0?0.f:p.y),std::bit_cast<std::uint32_t>(p.z==0?0.f:p.z)};};
    for(std::size_t i=0;i+2<triangles.size();i+=3)for(int j=0;j<3;++j){
        auto a=bits(triangles[i+j]),b=bits(triangles[i+(j+1)%3]);if(a==b)continue;if(b<a)std::swap(a,b);
        const std::array key{a[0],a[1],a[2],b[0],b[1],b[2]};
        if(seen.insert(key).second){entry.edges.push_back(triangles[i+j]);entry.edges.push_back(triangles[i+(j+1)%3]);}
    }
    const auto bytes=(entry.source.size()+entry.edges.size())*sizeof(Vec3);
    while(!cache.entries.empty() && (cache.entries.size()>=128 || cache.bytes+bytes>64u*1024u*1024u)){
        const auto old=std::min_element(cache.entries.begin(),cache.entries.end(),[](const auto& a,const auto& b){return a.used<b.used;});
        cache.bytes-=(old->source.size()+old->edges.size())*sizeof(Vec3);cache.entries.erase(old);
    }
    cache.bytes+=bytes;cache.entries.push_back(std::move(entry));return cache.entries.back().edges;
}

inline void draw_layer(game::runtime::SceneRenderer& r,const std::vector<game::runtime::Vec3>& triangles,game::runtime::Color color,float width=1.0f){
    if(triangles.empty()) return;
    r.draw_lines_3d(unique_triangle_edges(triangles),color,width);
}

} // namespace game::mesh_wireframe
