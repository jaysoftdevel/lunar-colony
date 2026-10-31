#pragma once

#include "engine/engine.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace game::robots {

inline constexpr float kSafetyDtSeconds = 0.01f;
inline constexpr float kPlannerReplanSeconds = 0.05f;
inline constexpr float kPlannerHorizonSeconds = 0.50f;
inline constexpr float kSafetyNumericMarginM = 0.025f;
inline constexpr float kBlockingWaitTimeoutSeconds = 0.85f;
inline constexpr float kNeighborAwarenessRadiusM = 8.0f;
// Personal rover safety zone beyond the hard physical hull. Every rover owns
// this buffer and every other rover must respect it. The pairwise desired gap
// is therefore twice this value. The hard collision envelope remains separate.
inline constexpr float kRobotPersonalSafetyZoneM = 0.30f;
inline constexpr float kNeighborComfortClearanceM = 2.0f*kRobotPersonalSafetyZoneM;
// Saves may already contain rovers closer than the new personal-zone spacing.
// Such a rover may escape the zone only while it does not move closer.
inline constexpr float kPersonalZoneEscapeEpsilonM = 0.015f;
inline constexpr std::size_t kNoSafetyRobot = std::numeric_limits<std::size_t>::max();

enum class MotionIntent : std::uint8_t {
    Wait = 0,
    Navigate,
    Retreat
};

enum class SafetyMode : std::uint8_t {
    Idle = 0,
    Moving,
    Braking,
    Waiting,
    Retreating,
    Working,
    Blocked,
    Fault
};

enum class BlockReason : std::uint8_t {
    None = 0,
    StaticGeometry,
    RobotReservation,
    PassageReservation,
    NoRoute,
    UnsafeInitialGeometry,
    InvalidMotionBounds,
    ContractViolation,
    SceneChanged
};

inline const char* safety_mode_name(SafetyMode mode){
    switch(mode){
        case SafetyMode::Idle: return "IDLE HOLD";
        case SafetyMode::Moving: return "CERTIFIED MOVE";
        case SafetyMode::Braking: return "CERTIFIED BRAKE";
        case SafetyMode::Waiting: return "WAITING FOR CLEAR SPACE";
        case SafetyMode::Retreating: return "YIELDING / RETREATING";
        case SafetyMode::Working: return "WORK HOLD";
        case SafetyMode::Blocked: return "BLOCKED";
        case SafetyMode::Fault: return "SAFETY FAULT";
    }
    return "SAFETY FAULT";
}

inline const char* block_reason_name(BlockReason reason){
    switch(reason){
        case BlockReason::None: return "NONE";
        case BlockReason::StaticGeometry: return "STATIC GEOMETRY";
        case BlockReason::RobotReservation: return "ROBOT CONTRACT";
        case BlockReason::PassageReservation: return "RESERVED PASSAGE";
        case BlockReason::NoRoute: return "NO SAFE ROUTE";
        case BlockReason::UnsafeInitialGeometry: return "UNSAFE INITIAL GEOMETRY";
        case BlockReason::InvalidMotionBounds: return "INVALID MOTION BOUNDS";
        case BlockReason::ContractViolation: return "CONTRACT VIOLATION";
        case BlockReason::SceneChanged: return "WORLD CHANGED";
    }
    return "UNKNOWN";
}

struct TubePiece {
    float t0{0.0f};
    float t1{0.0f};
    eng::Vec3 start{};
    eng::Vec3 end{};
    float radius{0.0f};
};

struct TerminalHold {
    float start_time{0.0f};
    eng::Vec3 position{};
    float radius{0.0f};
};

struct MotionControl {
    float target_speed_mps{0.0f};
    float target_heading_degrees{0.0f};
    bool reverse{false};
};

struct MotionContract {
    bool valid{false};
    std::size_t robot_id{kNoSafetyRobot};
    float valid_start{0.0f};
    float command_expiry{0.0f};
    MotionControl permitted_command{};
    std::vector<TubePiece> occupancy_tube{};
    std::vector<TubePiece> brake_branches{};
    std::vector<TerminalHold> terminal_holds{};
    std::vector<std::int64_t> passage_keys{};
};

struct MotionProposal {
    MotionControl control{};
    std::vector<eng::Vec3> preview{};
    float cost{0.0f};
};

struct NavigationRequest {
    bool active{false};
    eng::Vec3 goal{};
    float stop_radius{0.15f};
    float speed_scale{1.0f};
    bool work_target{false};
    // Traffic arbitration priority. Occupied passage ownership and an active
    // certified retreat remain non-preemptible, but between otherwise equal
    // movers a larger value is considered first. Gameplay assigns BR-02 a
    // higher value than CR-01 so construction work is not starved by hauling.
    std::uint8_t task_priority{0};
};

struct SafetyRuntime {
    bool initialized{false};
    MotionIntent motion_intent{MotionIntent::Wait};
    SafetyMode mode{SafetyMode::Idle};
    BlockReason block_reason{BlockReason::None};
    std::size_t blocker_robot_id{kNoSafetyRobot};

    eng::Vec3 goal{};
    bool goal_valid{false};
    eng::Vec3 temporary_goal{};
    bool temporary_goal_valid{false};
    // A temporary waypoint selected specifically to route an active rover
    // around a stationary fleet obstacle.  The original goal stays intact and
    // is resumed automatically after this waypoint is reached.
    bool stationary_detour_active{false};
    std::size_t stationary_detour_blocker{kNoSafetyRobot};
    float stop_radius{0.15f};
    float speed_scale{1.0f};
    std::uint64_t goal_revision{0};

    std::vector<eng::Vec3> cached_route{};
    std::size_t route_cursor{0};
    std::uint64_t route_world_epoch{0};
    std::uint64_t route_goal_revision{0};
    std::vector<MotionProposal> proposals{};
    float planner_elapsed{0.0f};
    // A failed coarse route is not recomputed every 50 ms. Repeating a full
    // A* search while a stationary blocker has not changed was a major source
    // of frame-rate collapse in dense construction saves.
    bool route_failed{false};
    float route_retry_elapsed{0.0f};
    // Consecutive failed full-route searches. Used only for transient retry
    // backoff; it is deliberately not persisted in savegames.
    std::uint8_t route_failure_count{0};

    MotionContract active_contract{};
    float blocked_elapsed{0.0f};
    float wait_age{0.0f};
    float last_progress_time{0.0f};
    eng::Vec3 last_progress_position{};
    std::uint32_t local_replans{0};
    std::uint32_t retreat_count{0};
    std::uint32_t stationary_detour_count{0};
    std::uint32_t accepted_contracts{0};
    std::uint32_t brake_fallbacks{0};
    std::uint32_t rejected_contracts{0};

    // Local fleet awareness is refreshed for every rover each safety tick.
    // It is advisory telemetry/planning input only; the shared reservation
    // board remains the final movement authority.
    std::uint32_t nearby_robot_count{0};
    std::size_t nearest_robot_id{kNoSafetyRobot};
    float nearest_robot_distance_m{std::numeric_limits<float>::infinity()};
    float predicted_robot_clearance_m{std::numeric_limits<float>::infinity()};

    bool route_replan_required{true};
};

struct SafetyAgent {
    std::size_t id{kNoSafetyRobot};
    eng::Vec3 position{};
    float heading_degrees{0.0f};
    float speed_mps{0.0f};
    float radius_m{0.8f};
    float max_speed_mps{1.0f};
    float acceleration_mps2{1.0f};
    float braking_mps2{1.0f};
    float turn_rate_degrees_per_second{90.0f};
    SafetyRuntime* runtime{nullptr};
};

struct SafetyWorldCallbacks {
    std::function<float(eng::Vec3)> floor_height;
    std::function<bool(std::size_t,eng::Vec3)> point_clear;
    std::function<bool(std::size_t,eng::Vec3,eng::Vec3)> segment_clear;
    std::function<bool(std::size_t,eng::Vec3,eng::Vec3,float,std::vector<eng::Vec3>&)> coarse_route;
};

struct SafetyTickStats {
    std::uint32_t accepted_contracts{0};
    std::uint32_t rejected_contracts{0};
    std::uint32_t brake_fallbacks{0};
    bool invariant_fault{false};
};

bool moving_discs_overlap(const TubePiece& a,const TubePiece& b,float common_t0,float common_t1);
bool tubes_overlap(const std::vector<TubePiece>& a,const std::vector<TubePiece>& b);

// Shared authoritative movement tick. All agents are arbitrated against one
// reservation board and committed simultaneously. Expensive route planning is
// advisory; only a certified MotionContract can authorize a pose change.
SafetyTickStats safety_tick(float dt,
                            float absolute_time,
                            std::uint64_t world_epoch,
                            std::vector<SafetyAgent>& agents,
                            const std::vector<NavigationRequest>& requests,
                            const SafetyWorldCallbacks& world);

// Used by construction/door/scene mutation gates. A persistent scene change is
// not admitted while it would intersect any currently certified motion, braking
// branch, or terminal hold.
bool contract_intersects_region(const MotionContract& contract,
                                const std::function<bool(eng::Vec3,float)>& circle_intersects_region);

} // namespace game::robots
