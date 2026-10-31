#pragma once

#include "engine/engine.hpp"
#include <functional>
#include <vector>

namespace game::robots {

// Geometry-only coarse route search. This route is advisory: the safety
// coordinator never authorizes movement from it directly. Every actual command
// must still pass short-horizon contract certification.
struct PathPlannerCallbacks {
    std::function<float(eng::Vec3)> floor_height;
    std::function<bool(eng::Vec3)> point_clear;
    std::function<bool(eng::Vec3,eng::Vec3)> segment_clear;
    std::function<bool(eng::Vec3,eng::Vec3)> neighbor_clear;
};

struct PathPlanRequest {
    eng::Vec3 start{};
    eng::Vec3 goal{};
    float stop_radius{0.0f};
    float corridor_half_width{9.0f};
    float cell_size{0.72f};
    int max_expansions{4200};
    std::size_t visibility_skip_limit{10};
    // Cheap direct/dogleg routing is always attempted first. Callers may turn
    // off the grid fallback when probing multiple clearance margins so only one
    // bounded A* search is paid for in the final fallback tier.
    bool allow_grid_fallback{true};
};

bool plan_corridor_path(const PathPlanRequest& request,
                        const PathPlannerCallbacks& callbacks,
                        std::vector<eng::Vec3>& geometric);

} // namespace game::robots
