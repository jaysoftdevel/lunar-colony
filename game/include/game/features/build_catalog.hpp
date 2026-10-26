#pragma once
#include "game/world/habitat_mesh.hpp"
#include <string>
#include <vector>

namespace game {

enum class BuildIcon {
    AirLock,
    OxygenDome,
    CantineDome,
    BioDome,
    CrewQuarters,
    Corridor
};

enum class BuildObjectKind {
    Habitat,
    Corridor
};

struct BuildObjectDefinition {
    std::string id;
    std::string name;
    std::string type;
    BuildIcon icon;
    BuildObjectKind kind{BuildObjectKind::Habitat};
    HabitatKind habitat_kind{HabitatKind::AirLock};
};

class BuildCatalog {
public:
    BuildCatalog();
    const std::vector<BuildObjectDefinition>& items() const { return items_; }
    int selected_index() const { return selected_index_; }
    const BuildObjectDefinition* selected() const;
    void select(int index);
    void clear_selection(){ selected_index_=-1; }
private:
    std::vector<BuildObjectDefinition> items_;
    int selected_index_=-1;
};
}
