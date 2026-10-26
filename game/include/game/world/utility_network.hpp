#pragma once
#include "engine/engine.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include <optional>
#include <vector>

namespace game {

enum class UtilityKind {
    PowerCable = 0,
    WaterPipe
};

enum class UtilityEndpointKind {
    Habitat = 0,
    Resource
};

struct UtilityBuildAnchor {
    UtilityEndpointKind kind{UtilityEndpointKind::Habitat};
    std::size_t index{0};
    eng::Vec3 hit{};
};

struct UtilityEndpoint {
    UtilityEndpointKind kind{UtilityEndpointKind::Habitat};
    std::size_t index{0};
    eng::Vec3 position{};
    eng::Vec3 normal{1.0f,0.0f,0.0f};
};

struct UtilityMesh {
    std::vector<eng::Vec3> shell;
    std::vector<eng::Vec3> details;
    std::vector<eng::Vec3> edges;
};

struct UtilityConnection {
    UtilityKind kind{UtilityKind::PowerCable};
    UtilityEndpoint endpoint_a{};
    UtilityEndpoint endpoint_b{};
    std::vector<eng::Vec3> path{};
    UtilityMesh mesh{};
};

class UtilityNetwork {
public:
    void clear(){ connections_.clear(); }
    const std::vector<UtilityConnection>& connections() const { return connections_; }

    bool anchor_compatible(UtilityKind kind,const UtilityBuildAnchor& anchor,
                           const std::vector<PlacedHabitat>& habitats,
                           const std::vector<PlacedResource>& resources) const;

    std::optional<UtilityConnection> preview_connection(
        UtilityKind kind,const UtilityBuildAnchor& a,const UtilityBuildAnchor& b,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const;

    bool build_connection(
        UtilityKind kind,const UtilityBuildAnchor& a,const UtilityBuildAnchor& b,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library);

    float connection_length(std::size_t index) const;
    std::size_t endpoint_connection_count(UtilityEndpointKind kind,std::size_t index) const;

private:
    std::vector<UtilityConnection> connections_{};
};

const char* utility_kind_name(UtilityKind kind);
const char* utility_endpoint_name(UtilityEndpointKind kind);

} // namespace game
