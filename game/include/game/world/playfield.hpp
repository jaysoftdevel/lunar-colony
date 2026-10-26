#pragma once
#include "engine/engine.hpp"
#include <vector>

namespace game {
class Playfield {
public:
    void rebuild(int size);
    int size() const { return size_; }
    const std::vector<eng::Vec3>& floor_triangles() const { return floor_tris_; }
    const std::vector<eng::Vec3>& grid_lines() const { return grid_lines_; }
private:
    int size_=64;
    std::vector<eng::Vec3> floor_tris_;
    std::vector<eng::Vec3> grid_lines_;
};
}
