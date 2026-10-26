#pragma once
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include "game/world/utility_network.hpp"
#include <string>
#include <vector>

namespace game {

enum class BuildIcon {
    CategoryHabitats,
    CategoryResources,
    CategoryFacilities,
    AirLock,
    OxygenDome,
    CantineDome,
    BioDome,
    CrewQuarters,
    ProcessingFacility,
    Factory,
    Manufactory,
    SickBay,
    Laboratory,
    Corridor,
    SolarPanel,
    WaterDriller,
    PowerStorage,
    WaterStorage,
    OreDriller,
    WaterPurifier,
    Antenna,
    LandingPad,
    SupplyChannel,
    Walkway
};

enum class BuildCategory {
    Habitats,
    Resources,
    Facilities
};

enum class BuildObjectKind {
    Habitat,
    Corridor,
    Resource,
    Utility,
    Walkway
};

struct BuildObjectDefinition {
    std::string id;
    std::string name;
    std::string type;
    BuildIcon icon;
    BuildCategory category{BuildCategory::Habitats};
    BuildObjectKind kind{BuildObjectKind::Habitat};
    HabitatKind habitat_kind{HabitatKind::AirLock};
    HabitatSizeTier habitat_size{HabitatSizeTier::Standard};
    ResourceKind resource_kind{ResourceKind::SolarPanel};
    UtilityKind utility_kind{UtilityKind::SupplyChannel};
};

class BuildCatalog {
public:
    BuildCatalog();
    const std::vector<BuildObjectDefinition>& items() const { return items_; }
    int selected_index() const { return selected_index_; }
    const BuildObjectDefinition* selected() const;
    void select(int index);
    void clear_selection(){ selected_index_=-1; }
    BuildCategory category() const { return category_; }
    void set_category(BuildCategory category){
        if(category_==category) return;
        category_=category;
        selected_index_=-1;
    }
private:
    std::vector<BuildObjectDefinition> items_;
    int selected_index_=-1;
    BuildCategory category_{BuildCategory::Habitats};
};
}
