#pragma once
#include "engine/engine.hpp"
#include <array>
#include <vector>

namespace game {
class Playfield {
public:
    void rebuild(int size);
    void draw_surface(eng::Renderer& renderer,float daylight) const;
    int size() const { return size_; }
    const std::vector<eng::Vec3>& floor_triangles() const { return floor_tris_; }
    const std::vector<eng::Vec3>& grid_lines() const { return grid_lines_; }
private:
    static constexpr std::size_t kSurfaceShades=9;
    int size_=64;
    std::array<std::vector<eng::Vec3>,kSurfaceShades> surface_;
    std::vector<eng::Vec3> floor_tris_;
    std::vector<eng::Vec3> grid_lines_;
};
}
