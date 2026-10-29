#pragma once

#include "engine/engine.hpp"
#include <functional>
#include <vector>

namespace game::robots {

struct PathPlannerCallbacks {
    std::function<float(eng::Vec3)> floor_height;
    std::function<bool(eng::Vec3)> point_clear;
    std::function<bool(eng::Vec3,eng::Vec3)> segment_clear;
    // Cheap validation for one neighbouring search-grid edge. The reconstructed
    // and smoothed route is still validated with segment_clear before use.
    std::function<bool(eng::Vec3,eng::Vec3)> neighbor_clear;
    std::function<float(eng::Vec3)> traffic_cost;
    // Space-time fleet validation. The autonomy layer predicts other rover
    // hulls and rejects a candidate search edge if this rover would occupy the
    // same physical space during the estimated traversal interval. Keeping the
    // timing callback here lets A*/Theta* avoid a collision before publishing
    // the route instead of relying only on the execution-time speed governor.
    std::function<bool(eng::Vec3,eng::Vec3,float,float)> dynamic_segment_clear;
};

struct PathPlanRequest {
    eng::Vec3 start{};
    eng::Vec3 goal{};
    float stop_radius{0.0f};
    float corridor_half_width{9.0f};
    float cell_size{0.72f};
    int max_expansions{22000};
    float traffic_weight{0.75f};
    float alternating_side_preference{0.0f};
    std::size_t visibility_skip_limit{18};
    // Broad/full searches may precompute static grid clearance in parallel.
    // The autonomy layer supplies roughly half of the system's logical CPUs so
    // pathfinding can use spare cores without monopolising the whole machine.
    unsigned worker_count{1};
    // Used only for estimating arrival time during dynamic fleet avoidance.
    // Static geometry planning remains distance based and deterministic.
    float nominal_speed_mps{1.0f};
};

// Builds a geometry-only route. Dynamic execution, hull collision checks and
// speed negotiation stay in the robot-autonomy layer. Keeping A*/Theta*-style
// search isolated here lets future pathfinding work load this implementation
// target without pulling construction logistics, animations or UI context.
bool plan_corridor_path(const PathPlanRequest& request,
                        const PathPlannerCallbacks& callbacks,
                        std::vector<eng::Vec3>& geometric);

} // namespace game::robots
