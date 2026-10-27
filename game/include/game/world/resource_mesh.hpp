#pragma once
#include "engine/engine.hpp"
#include "game/world/construction.hpp"
#include "game/systems/resource_management.hpp"
#include <array>
#include <vector>

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
    Count
};

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
};

struct StoredResourceBox {
    MaterialResource resource{MaterialResource::Iron};
    std::int64_t amount{0};
    float condition{1.0f};
};

struct PlacedResource {
    ResourceKind kind{ResourceKind::SolarPanel};
    eng::Vec3 position{};
    float rotation_degrees{0.0f};
    ConstructionState construction{};
    ConstructionResourcePlan resource_plan{};
    ResourceBundle stored_resources{};
    std::vector<StoredResourceBox> stored_boxes{};
    StorageDomeSize storage_size{StorageDomeSize::Small};
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

class ResourceModelLibrary {
public:
    ResourceModelLibrary();
    const ResourceMesh& mesh(ResourceKind kind) const;
    const ResourceMesh& mesh(ResourceKind kind,StorageDomeSize storage_size) const;
    const ResourceMesh& mesh(const PlacedResource& placed) const { return mesh(placed.kind,placed.storage_size); }
private:
    std::array<ResourceMesh,(size_t)ResourceKind::Count> meshes_{};
    std::array<ResourceMesh,(size_t)StorageDomeSize::Count> storage_domes_{};
};

} // namespace game
