#pragma once

#include "engine/engine.hpp"
#include <cstdint>
#include <functional>
#include <limits>
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
    // Optional soft navigation penalty. 0 means preferred free space; larger
    // values make the planner avoid the point when a reasonable alternative
    // exists. Infinity means blocked. This is used for MeshNav-style obstacle
    // inflation/clearance without turning useful narrow passages into hard
    // walls.
    std::function<float(eng::Vec3)> traversal_cost;
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

// Persistent state for the game-native D* Lite planner. The representation is
// deliberately engine-agnostic and contains only coarse navigation data; it is
// safe to keep one instance per rover. The global planner never owns motion
// authority: its output still goes through SafetyCoordinator contracts.
struct IncrementalPathPlannerState {
    struct OpenEntry {
        float k1{std::numeric_limits<float>::infinity()};
        float k2{std::numeric_limits<float>::infinity()};
        int index{-1};
        std::uint64_t serial{0};
    };

    bool initialized{false};
    float cell_size{1.25f};
    float origin_x{0.0f};
    float origin_z{0.0f};
    int nx{0};
    int nz{0};
    int last_start_index{-1};
    eng::Vec3 goal{};
    float km{0.0f};
    std::uint64_t world_epoch{0};
    std::uint64_t open_serial{0};

    std::vector<float> g{};
    std::vector<float> rhs{};
    std::vector<float> node_penalty{};
    std::vector<std::uint64_t> node_penalty_epoch{};
    std::vector<std::uint8_t> known_node{};
    std::vector<int> known_nodes{};
    std::vector<std::uint8_t> terminal{};

    // Directional edge cache (8 entries per grid node). Edge values are epoch
    // tagged so world revisions do not require clearing the whole table.
    std::vector<float> edge_cost{};
    std::vector<std::uint64_t> edge_cost_epoch{};
    std::vector<OpenEntry> open_heap{};

    std::uint32_t initialization_count{0};
    std::uint32_t incremental_repair_count{0};
    std::uint32_t repaired_node_count{0};
    std::uint32_t expanded_node_count{0};
};

void reset_incremental_path_state(IncrementalPathPlannerState& state);

// D* Lite–style persistent global planner. The grid is world-aligned (not tied
// to the current start->goal direction), so a moving rover can reuse g/rhs state
// while it advances. On a changed world epoch, already explored cells are
// re-sampled and only inconsistent vertices are repaired. Newly encountered
// cells are evaluated lazily. A bounded line-of-sight pass then turns the coarse
// grid route into a short smooth polyline.
enum class IncrementalPathResult : std::uint8_t {
    Success = 0,
    Pending,
    NoPath
};

IncrementalPathResult plan_incremental_path(const PathPlanRequest& request,
                                            std::uint64_t world_epoch,
                                            const PathPlannerCallbacks& callbacks,
                                            IncrementalPathPlannerState& state,
                                            std::vector<eng::Vec3>& geometric);

} // namespace game::robots
