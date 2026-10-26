#pragma once
#include <string>
#include <vector>

namespace game {
enum class BuildIcon {
    Habitat,
    Solar,
    Water,
    Storage,
    Workshop,
    Farm
};

struct BuildObjectDefinition {
    std::string id;
    std::string name;
    std::string type;
    BuildIcon icon;
};

class BuildCatalog {
public:
    BuildCatalog();
    const std::vector<BuildObjectDefinition>& items() const { return items_; }
    int selected_index() const { return selected_index_; }
    void select(int index);
private:
    std::vector<BuildObjectDefinition> items_;
    int selected_index_=0;
};
}
