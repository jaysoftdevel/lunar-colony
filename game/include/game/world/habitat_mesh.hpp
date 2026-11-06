#pragma once
#include "game/world/supply_state.hpp"
#include "game/world/hydroponics.hpp"
#include "engine/engine.hpp"
#include "game/world/construction.hpp"
#include "game/systems/resource_management.hpp"
#include <array>
#include <cstdint>
#include <vector>

namespace game {

enum class HabitatKind {
    AirLock = 0,
    OxygenGeneratorDome,
    CantineDome,
    BioDome,
    CrewQuarters,
    ProcessingFacility,
    Factory,
    Manufactory,
    SickBay,
    Laboratory,
    Count
};

enum class HabitatSizeTier {
    Standard = 0,
    Mid,
    Big,
    // Appended to preserve the serialized numeric values of all existing
    // 0.40.x saves. Small scales only X/Z; vertical dimensions stay unchanged.
    Small,
    Count
};

inline bool habitat_has_extended_size_tiers(HabitatKind kind){
    return kind==HabitatKind::OxygenGeneratorDome ||
           kind==HabitatKind::CantineDome ||
           kind==HabitatKind::BioDome ||
           kind==HabitatKind::CrewQuarters;
}

// Every habitat now supports at least SMALL + STANDARD. The four historically
// scalable domes additionally retain MID + BIG.
inline bool habitat_has_size_tiers(HabitatKind){ return true; }

inline std::size_t hydroponics_bay_count(HabitatSizeTier size){
    switch(size){
        case HabitatSizeTier::Small: return 2;
        case HabitatSizeTier::Standard: return 4;
        case HabitatSizeTier::Mid: return 6;
        case HabitatSizeTier::Big: return 8;
        default: return 4;
    }
}

inline const char* habitat_size_name(HabitatSizeTier size){
    switch(size){
        case HabitatSizeTier::Standard: return "STANDARD";
        case HabitatSizeTier::Mid: return "MID";
        case HabitatSizeTier::Big: return "BIG";
        case HabitatSizeTier::Small: return "SMALL";
        default: return "STANDARD";
    }
}

struct HabitatEntrance {
    eng::Vec3 local_position{};
    eng::Vec3 local_normal{1.0f,0.0f,0.0f};
};

struct HabitatEntranceAccess {
    std::vector<eng::Vec3> details;
    std::vector<eng::Vec3> edges;
};

inline constexpr int crew_quarters_bed_count(HabitatSizeTier size){
    switch(size){
        case HabitatSizeTier::Small:return 2;
        case HabitatSizeTier::Standard:return 4;
        case HabitatSizeTier::Mid:return 6;
        case HabitatSizeTier::Big:return 8;
        default:return 0;
    }
}

struct CrewBunk {
    eng::Vec3 access;
    eng::Vec3 feet;
};

struct HabitatMesh {
    std::vector<eng::Vec3> shell;
    std::vector<eng::Vec3> foundation;
    std::vector<eng::Vec3> interior;
    std::vector<eng::Vec3> details;
    std::vector<eng::Vec3> glazing;
    std::vector<eng::Vec3> edges;
    float footprint_x{1.0f};
    float footprint_z{1.0f};
    float height{1.0f};
    // Authoritative POV/navigation deck height supplied by the habitat module.
    // Do not infer this from furniture/foundation triangles.
    float walkable_floor_y{0.20f};
    bool crew_accessible{true};
    std::vector<CrewBunk> crew_bunks{};
    std::vector<HabitatEntrance> entrances{};
    // Optional exterior access geometry indexed like entrances. This is kept
    // separate from the permanent habitat mesh so stairs/ramps can disappear
    // when that exact entrance is pressure-docked to a corridor.
    std::vector<HabitatEntranceAccess> entrance_access{};
};

struct PlacedHabitat {
    SupplyState supply{};
    HabitatKind kind{HabitatKind::AirLock};
    // Stable serial within this exact habitat type/size. It is assigned when
    // the blueprint is placed and remains unchanged after construction.
    std::uint32_t instance_serial{0};
    // Global FIFO blueprint order across every construction kind.
    std::uint64_t placement_sequence{0};
    eng::Vec3 position{};
    float rotation_degrees{0.0f};
    HabitatSizeTier size{HabitatSizeTier::Standard};
    ConstructionState construction{};
    ConstructionResourcePlan resource_plan{};
    ResourceBundle site_buffer{}; // delivered construction material staged at this blueprint
    std::array<HydroponicsBay,kMaxHydroponicsBays> hydroponics{};
    bool power_enabled{true};
};

class HabitatModelLibrary {
public:
    HabitatModelLibrary();
    const HabitatMesh& mesh(HabitatKind kind,HabitatSizeTier size=HabitatSizeTier::Standard) const;
    const HabitatMesh& mesh(const PlacedHabitat& placed) const { return mesh(placed.kind,placed.size); }
private:
    std::array<std::array<HabitatMesh,(size_t)HabitatSizeTier::Count>,(size_t)HabitatKind::Count> meshes_{};
};

} // namespace game
