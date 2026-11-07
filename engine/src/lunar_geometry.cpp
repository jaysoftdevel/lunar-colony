#include "lunar_geometry.hpp"

#include <algorithm>
#include <map>
#include <limits>
#include <utility>

namespace eng::lunar {
namespace {

Float4 sphere(std::span<const TerrainVertex> vertices){
    const float inf=std::numeric_limits<float>::infinity();
    Vec3 low{inf,inf,inf},high{-inf,-inf,-inf};
    for(const auto& v:vertices){
        low={std::min(low.x,v.position.x),std::min(low.y,v.position.y),std::min(low.z,v.position.z)};
        high={std::max(high.x,v.position.x),std::max(high.y,v.position.y),std::max(high.z,v.position.z)};
    }
    const Vec3 center=(high+low)*0.5f;
    return {center.x,center.y,center.z,length(high-low)*0.5f+0.015f};
}

std::size_t add_cluster(TerrainPages& out,std::span<const TerrainVertex> vertices,Float4 bounds){
    const auto index=out.clusters.size();
    const auto start=static_cast<std::uint32_t>(out.coarse.size());
    out.clusters.push_back({bounds,{start,static_cast<std::uint32_t>(vertices.size()),0,0},{}});
    out.coarse.insert(out.coarse.end(),vertices.begin(),vertices.end());
    return index;
}

void coarse_rock(std::span<const TerrainVertex> rock,std::vector<TerrainVertex>& out){
    // The detailed boulder has two six-vertex rings and a peak.
    // A three-sided silhouette uses the same foot/peak vertices at long range.
    const Vec3 peak=rock[7].position;
    for(std::size_t side=0;side<6;side+=2){
        const Vec3 a=rock[side*9].position,c=rock[((side+2)%6)*9].position;
        const Vec3 n=normalize(cross(peak-a,c-a));
        out.insert(out.end(),{{a,n,1},{peak,n,1},{c,n,1}});
    }
}

} // namespace

TerrainPages make_terrain_pages(std::span<const TerrainVertex> landscape,std::size_t rocks_begin){
    TerrainPages out;
    rocks_begin=std::min(rocks_begin,landscape.size());
    // Terrain cells share the original vertex positions along their edges.
    // Keep those surfaces continuous when adjacent boulder pages change LOD.
    constexpr std::size_t surface_cluster_vertices=3*112;
    for(std::size_t first=0;first<rocks_begin;first+=surface_cluster_vertices){
        const auto tile=landscape.subspan(first,std::min(surface_cluster_vertices,rocks_begin-first));
        add_cluster(out,tile,sphere(tile));
    }

    constexpr std::size_t rock_vertices=18*3;
    constexpr float tile_size=32.0f;
    using Tile=std::pair<int,int>;
    std::map<Tile,std::vector<std::size_t>> tiles;
    for(std::size_t first=rocks_begin;first+rock_vertices<=landscape.size();first+=rock_vertices){
        const Vec3 p=landscape[first+7].position;
        tiles[{int(std::floor(p.x/tile_size)),int(std::floor(p.z/tile_size))}].push_back(first);
    }
    for(const auto& [tile,rocks]:tiles){
        (void)tile;
        for(std::size_t first=0;first<rocks.size();first+=8){
            const auto last=std::min(first+8,rocks.size());
            std::vector<TerrainVertex> coarse;
            GeometryPage page;
            page.vertices.reserve((last-first)*rock_vertices);
            coarse.reserve((last-first)*9);
            for(std::size_t i=first;i<last;++i){
                const auto rock=landscape.subspan(rocks[i],rock_vertices);
                coarse_rock(rock,coarse);
                page.vertices.insert(page.vertices.end(),rock.begin(),rock.end());
            }
            page.cluster=add_cluster(out,coarse,sphere(page.vertices));
            out.pages.push_back(std::move(page));
        }
    }
    return out;
}

std::vector<GeometryUpload> TerrainPages::update(Vec3 eye,Vec3 focus){
    struct Candidate { std::size_t page; float score; };
    std::vector<Candidate> near;
    for(std::size_t i=0;i<pages.size();++i){
        auto& page=pages[i];
        page.wanted=false;
        const auto& b=clusters[page.cluster].sphere;
        const Vec3 center{b[0],b[1],b[2]};
        const float distance=length(center-eye);
        if(distance<132.0f+b[3]){
            const float score=distance+0.18f*length(center-focus);
            near.push_back({i,score});
        }
    }
    const auto limit=std::min(near.size(),slots.size());
    std::partial_sort(near.begin(),near.begin()+limit,near.end(),
        [](const Candidate& a,const Candidate& b){ return a.score==b.score?a.page<b.page:a.score<b.score; });
    for(std::size_t i=0;i<limit;++i) pages[near[i].page].wanted=true;

    std::vector<GeometryUpload> uploads;
    constexpr std::size_t frame_upload_limit=8;
    for(std::size_t i=0;i<limit && uploads.size()<frame_upload_limit;++i){
        const auto index=near[i].page;
        const auto& page=pages[index];
        if(clusters[page.cluster].detail[1]) continue;
        const auto free=std::find_if(slots.begin(),slots.end(),
            [&](std::size_t resident){ return resident==0 || !pages[resident-1].wanted; });
        if(free==slots.end()) break;
        if(*free) clusters[pages[*free-1].cluster].detail={};
        const std::size_t start=coarse.size()+static_cast<std::size_t>(free-slots.begin())*detail_slot_vertices;
        clusters[page.cluster].detail={static_cast<std::uint32_t>(start),static_cast<std::uint32_t>(page.vertices.size()),0,0};
        *free=index+1;
        uploads.push_back({index,start});
    }
    return uploads;
}

} // namespace eng::lunar
