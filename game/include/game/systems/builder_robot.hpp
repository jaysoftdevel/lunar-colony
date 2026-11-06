#pragma once
#include "engine/engine.hpp"
#include "game/world/ore.hpp"
#include "game/systems/resource_management.hpp"
#include "game/robots/safety_coordinator.hpp"
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
    BuildUp,
    Dismantle
};

inline constexpr float kBuilderRobotBodyRadiusM = 0.48f;
inline constexpr float kCarrierRobotBodyRadiusM = 0.44f;
inline constexpr float kRobotMovementSpeedMultiplier = 2.00f;
// Both rover roles use one gameplay travel pace.  The constructor previously
// inherited the slower 2.20 m/s base while CR-01 used a 10% boost; with the
// global 200% movement multiplier that made BR-02 visibly lag behind logistics.
inline constexpr float kBuilderRobotTransitSpeedMps = 6.00f;
inline constexpr float kCarrierRobotTransitSpeedMps = 6.00f;

// Rover traction batteries. The capacities are deliberately sized around the
// compact Stage-1/Stage-2 bootstrap mission: one constructor and one carrier
// can establish power/water, the Air Lock and Oxygen Generator without an
// intermediate recharge, while retaining at least a 10% design reserve.
inline constexpr float kBuilderRobotBatteryCapacityKwh = 48.0f;
inline constexpr float kCarrierRobotBatteryCapacityKwh = 76.0f;
inline constexpr float kRobotBatteryReserveFraction = 0.10f;
inline constexpr float kRobotBatteryCriticalFraction = 0.025f;
inline constexpr float kRobotWorkshopChargePowerKw = 8.0f;
inline constexpr float kRobotWorkshopChargeEfficiency = 0.92f;
// Bootstrap charger beside the starter stash. It is available only until an
// operational Robot Workshop exists, preventing an early colony from becoming
// permanently unrecoverable because both construction rovers exhausted their packs.
inline constexpr float kStarterRobotChargePowerKw = 8.0f;
inline constexpr float kStarterRobotChargeReleaseFraction = 0.80f;
inline constexpr float kBuilderDriveEnergyKwhPerMeter = 0.0032f;
inline constexpr float kCarrierDriveEnergyKwhPerMeter = 0.0038f;
inline constexpr float kBuilderAuxiliaryPowerKw = 0.08f;
inline constexpr float kCarrierAuxiliaryPowerKw = 0.10f;
inline constexpr float kBuilderWorkPowerKw = 1.20f;
inline constexpr float kCarrierWorkPowerKw = 0.75f;
// Physical construction-drop package layout. Keeping these dimensions shared
// between rendering and navigation makes the visible material stacks real
// obstacles instead of decoration that a rover can drive through.
inline constexpr float kConstructionDropBoxColumnSpacingM = 0.46f;
inline constexpr float kConstructionDropBoxFirstRowOffsetM = 2.20f;
inline constexpr float kConstructionDropBoxRowSpacingM = 0.48f;
inline constexpr std::size_t kNoRobotIndex = std::numeric_limits<std::size_t>::max();

inline float robot_body_radius(RobotRole role){
    return role==RobotRole::Carrier?kCarrierRobotBodyRadiusM:kBuilderRobotBodyRadiusM;
}
inline float robot_battery_capacity_kwh(RobotRole role){
    return role==RobotRole::Carrier?kCarrierRobotBatteryCapacityKwh:kBuilderRobotBatteryCapacityKwh;
}
inline float robot_drive_energy_kwh_per_meter(RobotRole role){
    return role==RobotRole::Carrier?kCarrierDriveEnergyKwhPerMeter:kBuilderDriveEnergyKwhPerMeter;
}
inline float robot_auxiliary_power_kw(RobotRole role){
    return role==RobotRole::Carrier?kCarrierAuxiliaryPowerKw:kBuilderAuxiliaryPowerKw;
}
inline float robot_work_power_kw(RobotRole role){
    return role==RobotRole::Carrier?kCarrierWorkPowerKw:kBuilderWorkPowerKw;
}

inline const char* builder_robot_action_name(BuilderRobotAction action){
    switch(action){
        case BuilderRobotAction::None: return "";
        case BuilderRobotAction::Pickup: return "PICK-UP";
        case BuilderRobotAction::PutDown: return "PUT-DOWN";
        case BuilderRobotAction::ApplyResources: return "APPLYING MATERIAL";
        case BuilderRobotAction::Welding: return "WELDING";
        case BuilderRobotAction::BuildUp: return "BUILD-UP";
        case BuilderRobotAction::Dismantle: return "DISMANTLING";
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

inline constexpr std::uint8_t kRobotRelocationFailures=5;
inline constexpr std::uint64_t kLanderBlockerSequence=UINT64_MAX;

struct RobotOperationRetry {
    BuilderRobotAction action{BuilderRobotAction::None};
    BuilderRobotMode mode{BuilderRobotMode::Idle};
    std::uint64_t task{0};
    std::uint8_t failures{0};
    eng::Vec3 origin{}, goal{};
    float relocation_cooldown{0};
    bool relocation_pending() const { return failures>=kRobotRelocationFailures; }
};

struct BuilderRobotState {
    ore::Haul ore_haul{};
    RobotRole role{RobotRole::Constructor};
    eng::Vec3 position{};               // wheel contact / local floor position
    float heading_degrees{0.0f};
    BuilderRobotMode mode{BuilderRobotMode::Idle};
    ConstructionTargetRef target{};
    bool debug_move_active{false}; // transient, real assignments always preempt
    eng::Vec3 debug_move_destination{};
    ResourceBundle cargo{};
    ResourceBundle site_buffer{};       // legacy save field; blueprint site_buffer is authoritative since 0.40.12

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
    float battery_capacity_kwh{kBuilderRobotBatteryCapacityKwh};
    float battery_charge_kwh{kBuilderRobotBatteryCapacityKwh};
    float battery_last_draw_kw{0.0f};       // telemetry, transient
    bool battery_charging{false};            // telemetry, transient
    bool starter_charge_active{false};       // transient bootstrap/emergency return latch
    float work_elapsed{0.0f};
    float idle_elapsed{0.0f};
    std::size_t workshop_index{kNoRobotIndex};
    int workshop_bay{-1};
    bool workshop_parked{false};
    bool workshop_returning{false};
    bool workshop_entry_started{false};

    // Cooperative-site routing. For constructors this points to the BR-02
    // coordinator that owns the shared site buffer/stage commit. For carriers
    // it points to that same coordinator while the CR-01 services the site.
    std::size_t support_robot_index{kNoRobotIndex};
    bool cargo_committed_to_build{false};

    // Transient locomotion state. These values deliberately are not required
    // for save compatibility: after loading, the robot accelerates smoothly
    // from rest instead of teleporting or resuming a stale steering impulse.
    float current_speed_mps{0.0f};
    // Authoritative locomotion state. Since v0.40.30 every pose change is
    // produced by the shared safety coordinator from a certified motion
    // contract. Job logic may request a destination but cannot move a robot.
    robots::SafetyRuntime safety{};
    float blocked_elapsed{0.0f};
    float build_orbit_phase{0.0f};
    bool build_side_corner_reached{false};
    // Legacy navigation telemetry fields retained for debug rendering and save/update
    // source compatibility. They are mirrors of safety/cached proposal state only;
    // none of them authorizes motion or performs collision avoidance.
    float avoidance_offset_degrees{0.0f};

    eng::Vec3 visual_position{};
    float visual_heading_degrees{0.0f};
    eng::Vec3 visual_velocity{};
    bool visual_initialized{false};

    std::vector<eng::Vec3> planned_route{};
    std::vector<float> planned_route_distances{};
    std::size_t route_cursor{0};
    eng::Vec3 route_destination{};
    float route_stop_radius{0.0f};
    float route_replan_cooldown{0.0f};
    std::uint8_t route_search_tier{0};
    float route_speed_scale{1.0f};
    float route_progress_m{0.0f};
    float route_total_length_m{0.0f};
    float route_cruise_speed_mps{0.0f};
    // Preferred extra edge-to-hull clearance around completed habitat/resource
    // structures for the active route. The planner starts at 1 m and relaxes
    // it only when that halo would close the only usable passage. Hard physical
    // hull collision remains enforced independently.
    float route_structure_clearance_margin_m{1.0f};
    float route_priority_age{0.0f};
    float route_segment_elapsed{0.0f};
    eng::Vec3 route_segment_origin{};
    bool route_segment_origin_valid{false};
    bool route_active{false};
    bool route_waiting_for_clearance{false};
    float route_wait_elapsed{0.0f};
    std::size_t traffic_blocker_index{kNoRobotIndex};
    eng::Vec3 traffic_wait_position{};
    bool traffic_wait_position_valid{false};
    bool traffic_detour_active{false};
    std::size_t traffic_detour_avoid_index{kNoRobotIndex};
    float traffic_clear_elapsed{0.0f};
    std::vector<eng::Vec3> traffic_motion_history{};
    float route_resolution_elapsed{0.0f};
    bool route_force_full_replan{false};
    std::uint16_t route_full_replan_count{0};
    eng::Vec3 traffic_escape_position{};
    bool traffic_escape_active{false};

    // Stable operation station for pickup / delivery / construction work.
    // Interaction destinations must not be recomputed from the robot's current
    // position every frame: doing so makes a robot orbit a pallet or worksite
    // forever. The station is transient and can be invalidated/reselected when
    // world geometry changes.
    eng::Vec3 interaction_position{};
    eng::Vec3 interaction_focus_position{};
    MaterialResource interaction_material{MaterialResource::Iron};
    bool interaction_position_valid{false};

    // Exclusive ownership of the narrow starter-depot pallet throat. This is
    // transient and acquired only once the rover actually reaches the lane
    // mouth, so two approaching robots cannot swap priority every tick.
    bool pickup_lane_claimed{false};
    // Stable FIFO age for a starter-depot material lane. Dynamic distance-based
    // ranking can reorder two approaching rovers every frame and put both into
    // the throat. Older assignments keep priority until pickup/egress completes.
    float pickup_queue_age{0.0f};

    // Runtime hysteresis for leaving the starter depot with cargo. Once a
    // rover has crossed the outbound gate for the current load, it may not
    // re-enter the exit phase just because a later route briefly crosses the
    // old x-threshold. These flags are transient and are reconstructed from
    // cargo/position after load; they do not need save-schema persistence.
    bool starter_exit_pending{false};
    bool starter_exit_completed{false};

    // After unloading at a construction site's single material drop, a CR-01
    // must physically clear the apron before the site reservation is released.
    // Otherwise an "idle" carrier can remain parked on the only legal drop bay
    // and block the rest of the logistics fleet.
    bool drop_exit_pending{false};
    bool post_delivery_hold{false};

    // Independent high-level progress watchdog. Route blocking alone is not
    // sufficient to detect an operation deadlock (for example a valid route to
    // an interaction point whose source disappeared). Each robot monitors its
    // own progress and recovers without stopping or locking the rest of fleet.
    eng::Vec3 watchdog_position{};
    BuilderRobotMode watchdog_mode{BuilderRobotMode::Idle};
    BuilderRobotAction watchdog_action{BuilderRobotAction::None};
    float watchdog_elapsed{0.0f};
    std::uint8_t recovery_attempts{0};
    RobotOperationRetry operation_retry{};
    std::uint64_t blocking_structure{0};
    float blocker_display_seconds{0};
    float watchdog_action_elapsed{0};
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
    // Latched evacuation destination for the current stage transition. Reusing
    // one validated point prevents the destination from hopping between nearby
    // perimeter samples while the rover is moving, which otherwise invalidates
    // its route and can trigger repeated A* replans. Transient by design.
    bool stage_commit_safe_position_valid{false};
    bool stage_commit_alternate_attempted{false};
    // Independent wall-clock for one stage evacuation. Do not derive hard
    // recovery from blocked_elapsed: turning/replanning can legitimately reset
    // that locomotion counter while the rover is still trapped inside the
    // future construction envelope.
    float stage_commit_evacuate_elapsed{0.0f};

    // Current construction-step material handoff. Carriers always place
    // delivered resources at the site's single drop zone. Before assembly can
    // start, one BR-02 must physically visit that drop zone and pick the
    // current step's materials. This flag is transient: after load the builder
    // simply repeats the handoff animation against the still-authoritative
    // site_buffer, so resources can never be duplicated or lost.
    bool stage_materials_picked{false};

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
    std::int64_t payload_capacity_kg{2400};
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
