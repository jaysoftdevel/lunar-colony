#pragma once
#include "game/world/ore.hpp"
#include "game/world/supply_state.hpp"
#include "engine/engine.hpp"
#include "game/world/construction.hpp"
#include "game/systems/resource_management.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>
#include <limits>

namespace game {

enum class ResourceKind {
    SolarPanel = 0,
    WaterDriller,
    PowerStorage,
    WaterStorage,
    OreDriller,
    WaterPurifier,
    Antenna,
    LandingPad,
    StorageDome,
    RobotWorkshop,
    StorageGate,
    HydrogenGenerator,
    HydrogenTank, // Append for backward-compatible saved ResourceKind ordinals.
    OxygenTank,
    Count
};


enum class ResourceSizeTier {
    Standard = 0,
    Small,
    Count
};

inline const char* resource_size_name(ResourceSizeTier size){
    switch(size){
        case ResourceSizeTier::Standard: return "STANDARD";
        case ResourceSizeTier::Small: return "SMALL";
        default: return "STANDARD";
    }
}

// Shared electrical-storage capacity definition. Keep simulation, status bar
// and inspectors on the same source of truth. v0.43.32 raises the dedicated
// Power Storage battery by another 50%: Standard 90 -> 135 kWh,
// Small 45 -> 67.5 kWh.
inline constexpr double power_storage_capacity_kwh(ResourceSizeTier size){
    return size==ResourceSizeTier::Small?67.5:135.0;
}

// Small local output buffers keep generators useful during short storage/network
// interruptions without turning them into replacements for dedicated storage.
// The standard solar buffer holds 15 minutes at peak output; small keeps the
// same duration. Water drillers buffer a fraction of a game-day production.
inline constexpr double solar_panel_buffer_capacity_kwh(ResourceSizeTier size){
    return size==ResourceSizeTier::Small?2.0:4.0;
}
// Lunar cryogenic tank geometry: a 3.35 m-radius vacuum-insulated sphere.
// LH2 is ~71 kg/m^3 at ~20 K.  Its 157.5 m^3 gross volume with a 75%
// usable/fill allowance holds ~8,386 kg; nominal nameplate capacity 8,000 kg.
// The SMALL mesh has 0.63x linear dimensions => one quarter the usable volume.
inline constexpr double kLiquidHydrogenDensityKgPerM3=71.0;
inline constexpr double kHydrogenTankStandardVesselRadiusM=3.35;
inline constexpr double kHydrogenTankUsableVolumeFraction=0.75;
inline constexpr double kHydrogenTankSmallLinearScale=0.63;
inline constexpr double oxygen_tank_capacity_kg(ResourceSizeTier size){
    return size==ResourceSizeTier::Small?250.0:1000.0;
}
inline constexpr double hydrogen_tank_capacity_kg(ResourceSizeTier size){
    return size==ResourceSizeTier::Small?2000.0:8000.0;
}

// Conceptual reusable LH2/LOX lunar-return vehicles (not Earth launchers).
// NASA-derived lunar ascent ~1.92 km/s and trans-Earth injection/correction
// ~0.975 km/s; design allowance rounds the combined manoeuvres to 3 km/s.
// Vacuum engine Isp 450 s, oxidizer/fuel mass ratio 6:1. Tsiolkovsky:
// m_prop = m_dry+payload * (exp(deltaV/(Isp*g0)) - 1).
// A 30 t dry+payload return rocket and an 8 t small return lander are assumed.
// LOX must also be supplied for any real departure. Existing Stage-2 fueling
// only transfers LH2; the oxidizer is displayed but NOT falsely manufactured.
inline constexpr double kLunarReturnDeltaVMps=3000.0;
inline constexpr double kLunarReturnEngineIspSeconds=450.0;
inline constexpr double kStandardGravityMps2=9.80665;
inline constexpr double kHydroloxOxygenToHydrogenMassRatio=6.0;
inline constexpr double kRocketReturnDryAndPayloadKg=30000.0;
inline constexpr double kLanderReturnDryAndPayloadKg=8000.0;
inline double lunar_return_hydrogen_requirement_kg(double dry_and_payload_kg){
    const double ratio=std::exp(kLunarReturnDeltaVMps/(kLunarReturnEngineIspSeconds*kStandardGravityMps2));
    const double hydrogen=dry_and_payload_kg*(ratio-1.0)/(1.0+kHydroloxOxygenToHydrogenMassRatio);
    return std::ceil(hydrogen/50.0)*50.0; // 50 kg loading reserve/rounding
}
inline double landing_pad_rocket_fuel_capacity_kg(){
    return lunar_return_hydrogen_requirement_kg(kRocketReturnDryAndPayloadKg);
}
inline double landing_pad_lander_fuel_capacity_kg(){
    return lunar_return_hydrogen_requirement_kg(kLanderReturnDryAndPayloadKg);
}
inline double landing_pad_rocket_oxidizer_requirement_kg(){
    return landing_pad_rocket_fuel_capacity_kg()*kHydroloxOxygenToHydrogenMassRatio;
}
inline double landing_pad_lander_oxidizer_requirement_kg(){
    return landing_pad_lander_fuel_capacity_kg()*kHydroloxOxygenToHydrogenMassRatio;
}
inline constexpr double water_driller_buffer_capacity_litres(ResourceSizeTier size){
    return size==ResourceSizeTier::Small?50.0:100.0;
}

enum class StorageDomeSize {
    Small = 0,
    Mid,
    Big,
    Huge,
    Count
};

inline const char* storage_dome_size_name(StorageDomeSize size){
    switch(size){
        case StorageDomeSize::Small: return "SMALL";
        case StorageDomeSize::Mid: return "MID";
        case StorageDomeSize::Big: return "BIG";
        case StorageDomeSize::Huge: return "HUGE";
        default: return "SMALL";
    }
}

inline constexpr float storage_dome_diameter_m(StorageDomeSize size){
    switch(size){
        case StorageDomeSize::Small: return 9.0f;
        case StorageDomeSize::Mid: return 13.0f;
        case StorageDomeSize::Big: return 18.0f;
        case StorageDomeSize::Huge: return 24.0f;
        default: return 9.0f;
    }
}

inline constexpr float storage_dome_height_m(StorageDomeSize size){
    switch(size){
        case StorageDomeSize::Small: return 5.3f;
        case StorageDomeSize::Mid: return 7.0f;
        case StorageDomeSize::Big: return 9.3f;
        case StorageDomeSize::Huge: return 12.0f;
        default: return 5.3f;
    }
}

inline constexpr std::uint32_t kVisibleOreBoxesPerType=12;

// Storage uses a conservative cylindrical volume completely inside the curved
// pressure hull. This keeps even upper pallet layers away from the dome skin.
inline constexpr float storage_dome_storage_radius_m(StorageDomeSize size){
    return storage_dome_diameter_m(size)*0.30f;
}
inline constexpr float storage_dome_storage_height_m(StorageDomeSize size){
    return storage_dome_height_m(size)*0.43f;
}

inline constexpr std::int64_t storage_dome_capacity_kg(StorageDomeSize size){
    // Capacity follows the usable interior volume instead of arbitrary tier
    // constants. 600 kg/m^3 represents mixed packaged cargo including aisles,
    // pallets and handling clearance. Round down to whole metric tonnes.
    constexpr float pi=3.14159265358979323846f;
    const float r=storage_dome_storage_radius_m(size);
    const float h=storage_dome_storage_height_m(size);
    const float kg=pi*r*r*h*600.0f;
    return static_cast<std::int64_t>(kg/1000.0f)*1000;
}

struct ResourceMesh {
    std::vector<eng::Vec3> body;
    std::vector<eng::Vec3> foundation;
    std::vector<eng::Vec3> details;
    std::vector<eng::Vec3> edges;
    float footprint_x{1.0f};
    float footprint_z{1.0f};
    float height{1.0f};
    float walkable_floor_y{0.20f};
};

struct StoredResourceBox {
    MaterialResource resource{MaterialResource::Iron};
    std::int64_t amount{0};
    float condition{1.0f};
};

struct PlacedResource {
    ore::Mix mined_ore_kg{};
    std::array<std::uint32_t,ore::materials.size()> stored_ore_boxes{};
    SupplyState supply{};
    double stored_oxygen_kg{0.0}; // Separate from the pressurized supply manifold.
    // Hydrogen already transferred into the two docked vehicle fuel allocations
    // at a landing pad. Separate from H2 still inside the network storage tank.
    double rocket_fuel_kg{0.0}, lander_fuel_kg{0.0};
    ResourceKind kind{ResourceKind::SolarPanel};
    // Stable serial within this exact structure type/size. It is assigned when
    // the blueprint is placed and remains unchanged after construction.
    std::uint32_t instance_serial{0};
    // Global FIFO blueprint order across every construction kind.
    std::uint64_t placement_sequence{0};
    eng::Vec3 position{};
    float rotation_degrees{0.0f};
    ConstructionState construction{};
    ConstructionResourcePlan resource_plan{};
    ResourceBundle site_buffer{}; // delivered construction material staged at this blueprint
    ResourceBundle stored_resources{};
    std::vector<StoredResourceBox> stored_boxes{};
    StorageDomeSize storage_size{StorageDomeSize::Small};
    // Storage gates are optional child construction tasks. The owning dome
    // keeps a derived three-slot view for routing/rendering compatibility:
    // storage_loading_face is slot 1, storage_extra_faces are slots 2/3.
    // Legacy saves may still persist a built-in primary face; loading migrates
    // that face to a StorageGate child and synchronization derives these fields.
    std::int8_t storage_loading_face{-1};
    // Faces 8..10 are freely positioned gate slots, with continuous local yaw.
    float storage_gate_angle_degrees{0};
    std::array<float,2> storage_extra_angles{}; // derived from child gates
    bool storage_loading_ready{false}; // derived from first StorageGate child
    std::size_t storage_loading_index{std::numeric_limits<std::size_t>::max()}; // derived
    std::uint64_t storage_parent_sequence{0}; // StorageGate -> owning dome
    std::size_t storage_parent_index{std::numeric_limits<std::size_t>::max()}; // derived
    std::array<std::int8_t,2> storage_extra_faces{{-1,-1}}; // derived from child slots 2/3
    std::array<bool,2> storage_extra_ready{};
    std::array<std::size_t,2> storage_extra_indices{{std::numeric_limits<std::size_t>::max(),std::numeric_limits<std::size_t>::max()}};
    float storage_extension_open{0.0f}; // transient shutter animation
    float workshop_gate_open{0.0f}; // transient; restored from live occupancy
    float workshop_gate_hold{0.0f};
    bool workshop_gate_initialized{false};
    int workshop_entry_owner{-1};
    // Solar collectors track the star by default to preserve the established
    // maximum-output behaviour. Players can disable this per collector from
    // its status panel; the collector then remains at its authored fixed pose.
    bool solar_tracking_enabled{true};
    // Transient visual tracker pose. The simulation output may react instantly,
    // but the physical collector slews continuously toward its target so neither
    // time progression nor toggling tracking can make the panel jump.
    eng::Vec3 solar_visual_normal{};
    bool solar_visual_initialized{false};
    // Non-storage outdoor structures support STANDARD + SMALL. Storage domes
    // retain their dedicated SMALL/MID/BIG/HUGE tiers.
    ResourceSizeTier size{ResourceSizeTier::Standard};
    bool power_enabled{true};
};

inline ResourceBundle stored_resource_box_totals(const PlacedResource& placed){
    ResourceBundle totals{};
    for(const auto& box:placed.stored_boxes){
        if(box.amount>0) totals.amount(box.resource)+=box.amount;
    }
    return totals;
}

inline void sync_stored_resource_totals(PlacedResource& placed){
    placed.stored_resources=stored_resource_box_totals(placed);
}

inline constexpr double kStorageDomeFullWarning=0.85;

inline std::int64_t stored_ore_mass_kg(const PlacedResource& placed){
    std::int64_t amount=0;
    for(const auto count:placed.stored_ore_boxes)amount+=ore::box_kg*count;
    return amount;
}

inline std::int64_t storage_dome_used_kg(const PlacedResource& placed){
    return placed.stored_resources.total_amount()+stored_ore_mass_kg(placed);
}

class ResourceModelLibrary {
public:
    ResourceModelLibrary();
    const ResourceMesh& mesh(ResourceKind kind,ResourceSizeTier size=ResourceSizeTier::Standard) const;
    const ResourceMesh& mesh(ResourceKind kind,StorageDomeSize storage_size) const;
    const ResourceMesh& mesh(const PlacedResource& placed) const;
private:
    std::array<ResourceMesh,(size_t)ResourceKind::Count> meshes_{};
    std::array<ResourceMesh,(size_t)ResourceKind::Count> small_meshes_{};
    std::array<ResourceMesh,(size_t)StorageDomeSize::Count> storage_domes_{};
    std::array<ResourceMesh,(size_t)StorageDomeSize::Count> storage_gates_{};
};

} // namespace game
