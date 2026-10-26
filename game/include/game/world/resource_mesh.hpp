#pragma once
#include "engine/engine.hpp"
#include <array>
#include <vector>

namespace game {

enum class ResourceKind {
    SolarPanel = 0,
    WaterDriller,
    PowerStorage,
    WaterStorage,
    Count
};

struct ResourceMesh {
    std::vector<eng::Vec3> body;
    std::vector<eng::Vec3> foundation;
    std::vector<eng::Vec3> details;
    std::vector<eng::Vec3> edges;
    float footprint_x{1.0f};
    float footprint_z{1.0f};
    float height{1.0f};
};

struct PlacedResource {
    ResourceKind kind{ResourceKind::SolarPanel};
    eng::Vec3 position{};
    float rotation_degrees{0.0f};
};

class ResourceModelLibrary {
public:
    ResourceModelLibrary();
    const ResourceMesh& mesh(ResourceKind kind) const;
private:
    std::array<ResourceMesh,(size_t)ResourceKind::Count> meshes_{};
};

} // namespace game
