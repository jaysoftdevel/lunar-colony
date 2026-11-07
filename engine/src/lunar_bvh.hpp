#pragma once

#include "engine/engine.hpp"
#include <array>
#include <cstdint>
#include <vector>

namespace eng::lunar {

using Float4=std::array<float,4>;
using UInt4=std::array<std::uint32_t,4>;

struct alignas(16) RayTriangle {
    Float4 a{},b{},c{},tint{};
};
struct alignas(16) RayNode {
    Float4 minimum{},maximum{};
    UInt4 link{};
};
static_assert(sizeof(RayTriangle)==64 && sizeof(RayNode)==48);

struct RayTree {
    std::vector<RayTriangle> triangles;
    std::vector<RayNode> nodes;
    Vec3 focus{};
    std::uint64_t generation{};
};

RayTriangle ray_triangle(Vec3 a,Vec3 b,Vec3 c,Color tint,float material);
RayTree build_ray_tree(std::vector<RayTriangle> triangles,Vec3 focus,std::uint64_t generation);

} // namespace eng::lunar
