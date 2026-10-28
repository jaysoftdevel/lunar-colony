#pragma once
#include "engine/engine.hpp"
#include "game/systems/resource_management.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace game {

enum class RobotRole : std::uint8_t {
    Constructor = 0,
    Carrier
};

inline const char* robot_role_name(RobotRole role){
    switch(role){
        case RobotRole::Constructor: return "BR-02 CONSTRUCTOR";
        case RobotRole::Carrier: return "CR-01 CARRIER";
    }
    return "ROBOT";
}

enum class BuilderRobotMode : std::uint8_t {
    Idle = 0,
    AwaitingMaterials,
    Fetching,
    Delivering,
    Building,
    DepotTransfer
};

enum class BuilderRobotAction : std::uint8_t {
    None = 0,
    Pickup,
    PutDown,
    ApplyResources,
    Welding,
    BuildUp
};

inline constexpr float kBuilderRobotBodyRadiusM = 0.48f;
inline constexpr float kCarrierRobotBodyRadiusM = 0.44f;
inline constexpr float kBuilderRobotTransitSpeedMps = 2.20f;
inline constexpr float kCarrierRobotTransitSpeedMps = kBuilderRobotTransitSpeedMps*1.10f;
inline constexpr std::size_t kNoRobotIndex = std::numeric_limits<std::size_t>::max();

inline float robot_body_radius(RobotRole role){
    return role==RobotRole::Carrier?kCarrierRobotBodyRadiusM:kBuilderRobotBodyRadiusM;
}

inline const char* builder_robot_action_name(BuilderRobotAction action){
    switch(action){
        case BuilderRobotAction::None: return "";
        case BuilderRobotAction::Pickup: return "PICK-UP";
        case BuilderRobotAction::PutDown: return "PUT-DOWN";
        case BuilderRobotAction::ApplyResources: return "APPLYING MATERIAL";
        case BuilderRobotAction::Welding: return "WELDING";
        case BuilderRobotAction::BuildUp: return "BUILD-UP";
    }
    return "";
}

enum class ConstructionTargetKind : std::uint8_t {
    None = 0,
    Habitat,
    Resource,
    Corridor,
    Utility,
    Walkway
};

struct ConstructionTargetRef {
    ConstructionTargetKind kind{ConstructionTargetKind::None};
    std::size_t index{0};
    bool valid() const { return kind!=ConstructionTargetKind::None; }
    bool same_as(const ConstructionTargetRef& other) const { return kind==other.kind && index==other.index; }
};

struct BuilderRobotState {
    RobotRole role{RobotRole::Constructor};
    eng::Vec3 position{};               // wheel contact / local floor position
    float heading_degrees{0.0f};
    BuilderRobotMode mode{BuilderRobotMode::Idle};
    ConstructionTargetRef target{};
    ResourceBundle cargo{};
    ResourceBundle site_buffer{};       // authoritative only for Constructor role

    // Exact logistics assignment for the current construction trip.  This is
    // the shared reservation seen by every robot: a material kilogram may be
    // either in the site buffer or reserved by exactly one robot, never both.
    // Keeping the reservation separate from cargo lets the fleet account for
    // resources before a CR-01 has physically reached the pile/depot.
    ResourceBundle logistics_reservation{};

    // Mechanical condition is exposed in the robot inspector now.  No wear
    // model consumes it yet, but it is persistent and ready for maintenance /
    // damage gameplay without changing the robot-state format again.
    float condition{1.0f};
    float work_elapsed{0.0f};
    float idle_elapsed{0.0f};

    // Cooperative-site routing. For constructors this points to the BR-02
    // coordinator that owns the shared site buffer/stage commit. For carriers
    // it points to that same coordinator while the CR-01 services the site.
    std::size_t support_robot_index{kNoRobotIndex};
    bool cargo_committed_to_build{false};

    // Transient locomotion state. These values deliberately are not required
    // for save compatibility: after loading, the robot accelerates smoothly
    // from rest instead of teleporting or resuming a stale steering impulse.
    float current_speed_mps{0.0f};
    float blocked_elapsed{0.0f};
    float build_orbit_phase{0.0f};
    // Legacy steering bias retained for save/source compatibility. The continuous
    // navigation controller no longer performs reactive steering or retreat
    // maneuvers; all fleet conflicts are resolved in the route schedule before
    // movement starts.
    float avoidance_offset_degrees{0.0f};

    // Render-only smoothing state. Navigation/collision always use position and
    // heading_degrees; the visible chassis follows those authoritative values
    // with critically damped interpolation so route replans never produce a
    // one-frame positional/heading snap. These fields are transient on load.
    eng::Vec3 visual_position{};
    float visual_heading_degrees{0.0f};
    eng::Vec3 visual_velocity{};
    bool visual_initialized{false};

    // Every trip is geometrically planned before motion begins, but execution is
    // continuous rather than quantized into stop/go time slots.  The route is a
    // dense, rounded polyline with cumulative arc-length metadata.  Fleet traffic
    // is predicted against the complete route before departure and continuously
    // negotiated with a speed governor while driving; steering never improvises a
    // new direction around another robot.  Routes are transient after save/load.
    std::vector<eng::Vec3> planned_route{};
    std::vector<float> planned_route_distances{};
    std::size_t route_cursor{0};
    eng::Vec3 route_destination{};
    float route_stop_radius{0.0f};
    float route_replan_cooldown{0.0f};
    float route_speed_scale{1.0f};
    float route_progress_m{0.0f};
    float route_total_length_m{0.0f};
    float route_cruise_speed_mps{0.0f};
    float route_priority_age{0.0f};

    // Kept as transient compatibility storage for older update/save code.  In the
    // continuous navigator route_segment_origin is the route's immutable start
    // pose; the elapsed-slot fields are no longer used as movement clocks.
    float route_segment_elapsed{0.0f};
    eng::Vec3 route_segment_origin{};
    bool route_segment_origin_valid{false};
    bool route_active{false};
    // True only while the smooth traffic governor is yielding to another rover.
    // A yield changes speed, never steering direction, so it cannot cause spins.
    bool route_waiting_for_clearance{false};
    float route_wait_elapsed{0.0f};

    // Stable operation station for pickup / delivery / construction work.
    // Interaction destinations must not be recomputed from the robot's current
    // position every frame: doing so makes a robot orbit a pallet or worksite
    // forever. The station is transient and can be invalidated/reselected when
    // world geometry changes.
    eng::Vec3 interaction_position{};
    eng::Vec3 interaction_focus_position{};
    MaterialResource interaction_material{MaterialResource::Iron};
    bool interaction_position_valid{false};

    // Independent high-level progress watchdog. Route blocking alone is not
    // sufficient to detect an operation deadlock (for example a valid route to
    // an interaction point whose source disappeared). Each robot monitors its
    // own progress and recovers without stopping or locking the rest of fleet.
    eng::Vec3 watchdog_position{};
    BuilderRobotMode watchdog_mode{BuilderRobotMode::Idle};
    BuilderRobotAction watchdog_action{BuilderRobotAction::None};
    float watchdog_elapsed{0.0f};
    std::uint8_t recovery_attempts{0};
    // Persistent alternate service slot selected by the local recovery logic.
    // Unlike recovery_attempts this must not reset after a few centimetres of
    // progress, otherwise a robot can oscillate back to the same blocked work
    // position forever.
    std::uint8_t service_slot_offset{0};

    // Stage commits are two-phase: after the visible build-up animation the
    // constructor first evacuates itself (and any fleet mate) from geometry
    // that the next construction step is about to create. These values are
    // transient and intentionally need no save-schema change.
    bool stage_commit_pending{false};
    eng::Vec3 stage_commit_safe_position{};

    // Action animation is deliberately transient. Save/load preserves the
    // underlying logistics transaction (cargo/site buffer/mode), then resumes
    // the visible action safely from its start without duplicating resources.
    BuilderRobotAction action{BuilderRobotAction::None};
    float action_elapsed{0.0f};
    float action_duration{0.0f};
    MaterialResource action_material{MaterialResource::Iron};
    std::int64_t action_amount{0};
    eng::Vec3 action_focus_position{};

    // Constructors prioritize assembly; CR-01 carriers trade arm reach for a
    // larger logistics deck and slightly higher transit speed.
    float move_speed_mps{kBuilderRobotTransitSpeedMps};
    std::int64_t payload_capacity_kg{1200};
};

inline const char* builder_robot_mode_name(BuilderRobotMode mode){
    switch(mode){
        case BuilderRobotMode::Idle: return "IDLE";
        case BuilderRobotMode::AwaitingMaterials: return "WAITING MATERIALS";
        case BuilderRobotMode::Fetching: return "FETCHING";
        case BuilderRobotMode::Delivering: return "DELIVERING";
        case BuilderRobotMode::Building: return "BUILDING";
        case BuilderRobotMode::DepotTransfer: return "DEPOT TRANSFER";
    }
    return "IDLE";
}

} // namespace game
