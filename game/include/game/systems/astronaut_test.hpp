#pragma once

#include "engine/math.hpp"
#include <array>
#include "game/world/geometry_standards.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace game {

inline constexpr float kAstronautSeparation=0.90f;
inline constexpr float kIndoorAstronautSeparation=.70f;
inline constexpr double kSuitOxygenCapacityKg=.28; // Eight in-game hours at the nominal breathing rate.
inline constexpr double kCrewFoodKgPerDay=.85;
inline constexpr float kCrewFullAir=99.95f;
inline constexpr float kCrewBreathableAir=95.f;
inline constexpr float kCrewTired=.80f;
inline constexpr float kCrewExhausted=1.20f;
inline constexpr float kCrewCacheRadius=5.0f;

enum class AstronautActivity : std::uint8_t { Idle, Manual, Shelter, Duty, ToBed, Sleeping, Knockout, Resupply, Construction };

struct AstronautCache {
    double oxygen_kg{42.0};
    double food_kg{32.0};
    double water_litres{128.0};
};

struct AstronautRestPose {
    eng::Vec3 feet{};
    float heading_degrees{};
};
inline constexpr float kAstronautFieldMargin=0.45f;
inline constexpr float kAstronautMeshHeight=1.91f*geometry::kAstronautScale;

struct AstronautRole {
    const char* name;
    const char* job;
    std::array<float,3> color;
};
inline constexpr std::array<AstronautRole,4> kAstronautRoles{{
    {"AMARA OKAFOR","WORKER",{.96f,.62f,.15f}},
    {"LUCA REYES","ENGINEER",{.22f,.57f,.97f}},
    {"MEI TANAKA","BIOLOGIST",{.26f,.80f,.46f}},
    {"INEZ SILVA","MEDIC",{.96f,.32f,.40f}},
}};

struct AstronautTestActor {
    eng::Vec3 position{};
    float heading_degrees{};
    float stride_phase{};
    float speed{};
    float health{1.f};
    double suit_oxygen_kg{kSuitOxygenCapacityKg};
    double food_pack_kg{};
    float hunger{},thirst{},fatigue{};
    AstronautActivity activity{AstronautActivity::Idle};
    std::uint64_t construction_job{}; // Transient assignment to a blueprint's placement ID.
    std::uint64_t bed_owner{};
    std::uint8_t bed_slot{};
    bool suited{true};
    float decision_delay{};
    float order_timeout{};
    float passing_cooldown{};
    bool waiting_for_door{};
    std::optional<eng::Vec3> destination{};
    std::array<eng::Vec3,2> entry_waypoints{};
    std::uint8_t entry_step{2};
    std::vector<eng::Vec3> route{};
    std::size_t route_step{};
    float retry_seconds{};
};

inline bool astronaut_resting(const AstronautTestActor& actor){
    return actor.activity==AstronautActivity::Sleeping || actor.activity==AstronautActivity::Knockout || actor.health<=0.f;
}

inline constexpr float astronaut_stride(bool suited){
    // Distance covered by the complete baked gait cycle at game scale.
    return (suited?1.24f:1.28f*1.035f)*geometry::kAstronautScale;
}

inline constexpr float astronaut_walk_speed(bool suited){
    constexpr float base=astronaut_stride(false)/1.12f;
    return suited?base*.70f:base;
}

} // namespace game
