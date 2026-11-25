#pragma once
#include "game/world/structural_frame.hpp"
#include "game/world/supply_state.hpp"
#include "game/world/structure_display.hpp"
#include "game/world/hydroponics.hpp"
#include "game/world/interior_unit.hpp"
#include "game/runtime/presentation.hpp"
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
    VerySmall, // Legacy 0.79 dome tier; no longer offered for new construction.
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
        case HabitatSizeTier::Small: case HabitatSizeTier::VerySmall: return 2;
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
        case HabitatSizeTier::VerySmall: return "VERY SMALL";
        default: return "STANDARD";
    }
}
inline const char* habitat_size_name(HabitatKind kind,HabitatSizeTier size){
    (void)kind;
    return habitat_size_name(size);
}

struct HabitatEntrance {
    game::runtime::Vec3 local_position{};
    game::runtime::Vec3 local_normal{1.0f,0.0f,0.0f};
};

struct HabitatEntranceAccess {
    std::vector<game::runtime::Vec3> details;
    std::vector<game::runtime::Vec3> edges;
};

inline constexpr int crew_quarters_bed_count(HabitatSizeTier size){
    switch(size){
        case HabitatSizeTier::Small:case HabitatSizeTier::VerySmall:return 2;
        case HabitatSizeTier::Standard:return 4;
        case HabitatSizeTier::Mid:return 6;
        case HabitatSizeTier::Big:return 8;
        default:return 0;
    }
}

enum class StationKind { Equipment, Grow, Bed, Screen, Furniture };
struct InteriorStation {
    StationKind kind{StationKind::Equipment};std::size_t slot{};
    std::array<std::size_t,8> first{},count{};
    std::size_t screen_first{},screen_count{};
    game::runtime::Vec3 low{},high{};bool active{true};
};

struct CrewBunk {
    game::runtime::Vec3 access;
    game::runtime::Vec3 feet;
    bool available{true};
};

struct HabitatMesh {
    structural::Metrics reinforcement;
    std::vector<game::runtime::Vec3> hull_frames;
    std::vector<game::runtime::Vec3> overhead_services,overhead_lamps; // Roof-mounted; hidden in cutaway.
    game::runtime::Vec3 dome_radii{}; // X radius, roof rise, Z radius.
    float dome_spring_y{};
    std::vector<game::runtime::Vec3> shell;
    std::vector<game::runtime::Vec3> foundation;
    std::vector<game::runtime::Vec3> interior;
    std::vector<game::runtime::Vec3> details;
    std::vector<game::runtime::Vec3> glazing;
    std::vector<game::runtime::Vec3> edges;
    std::vector<game::runtime::Vec3> lining,deck,service_metal,fabric,displays,lamps,markings;
    std::vector<structure_display::Mount> screens;
    std::vector<InteriorStation> stations;
    std::uint16_t blocked_grow_bays{};
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
    game::runtime::Vec3 position{};
    float rotation_degrees{0.0f};
    HabitatSizeTier size{HabitatSizeTier::Standard};
    ConstructionState construction{};
    ConstructionResourcePlan resource_plan{};
    ResourceBundle site_buffer{}; // delivered construction material staged at this blueprint
    std::array<HydroponicsBay,kMaxHydroponicsBays> hydroponics{};
    std::uint16_t blocked_grow_bays{}; // Derived doorway reservations; not serialized.
    std::vector<InteriorUnit> units;
    bool oxygen_unit_fitted{false}; // Included equipment is fitted only once.
    std::array<double,4> unit_ore_kg{};
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
