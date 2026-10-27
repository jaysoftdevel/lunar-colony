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

inline constexpr std::int64_t storage_dome_capacity_kg(StorageDomeSize size){
    switch(size){
        case StorageDomeSize::Small: return 25000;
        case StorageDomeSize::Mid: return 100000;
        case StorageDomeSize::Big: return 250000;
        case StorageDomeSize::Huge: return 600000;
        default: return 25000;
    }
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
