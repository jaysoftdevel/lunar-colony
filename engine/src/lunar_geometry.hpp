#pragma once

#include "lunar_bvh.hpp"
#include <array>
#include <span>
#include <vector>

namespace eng::lunar {

struct TerrainVertex {
    Vec3 position;
    Vec3 normal{0,1,0};
    float stone{};
};
static_assert(sizeof(TerrainVertex)==7*sizeof(float));

struct alignas(16) Cluster {
    Float4 sphere{};
    UInt4 coarse{};
    UInt4 detail{};
};
static_assert(sizeof(Cluster)==48);

inline constexpr std::size_t detail_slot_count=64;
inline constexpr std::size_t detail_slot_vertices=8*18*3;

struct GeometryPage {
    std::vector<TerrainVertex> vertices;
    std::size_t cluster{};
    bool wanted{};
};
struct GeometryUpload { std::size_t page{},first_vertex{}; };
struct TerrainPages {
    std::vector<TerrainVertex> coarse;
    std::vector<Cluster> clusters;
    std::vector<GeometryPage> pages;
    // Zero denotes a free slot. Resident page numbers are stored plus one.
    std::array<std::size_t,detail_slot_count> slots{};

    std::size_t buffer_vertices() const { return coarse.size()+detail_slot_count*detail_slot_vertices; }
    std::vector<GeometryUpload> update(Vec3 eye,Vec3 focus);
};

TerrainPages make_terrain_pages(std::span<const TerrainVertex> landscape,std::size_t rocks_begin);

} // namespace eng::lunar
