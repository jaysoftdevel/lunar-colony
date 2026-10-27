#pragma once
#include "engine/engine.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include "game/world/construction.hpp"
#include "game/systems/resource_management.hpp"
#include <algorithm>
#include <optional>
#include <vector>
#include <utility>

namespace game {

// v0.15.0 merges the former electrical and water build tools into one physical
// supply channel. A channel carries both services through one shared tray/pipe
// assembly; individual structures simply consume the services they need.
enum class UtilityKind {
    SupplyChannel = 0
};

enum class UtilityEndpointKind {
    Habitat = 0,
    Resource,
    Open
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
    UtilityKind kind{UtilityKind::SupplyChannel};
    UtilityEndpoint endpoint_a{};
    UtilityEndpoint endpoint_b{};
    std::vector<eng::Vec3> path{};
    // Supply channels form a shared manifold wherever two channel centerlines
    // cross. The junction is common to both power and water services.
    std::vector<eng::Vec3> junctions{};
    UtilityMesh mesh{};
    StagedConstructionState construction{kSupplyConstructionSteps};
    ConstructionResourcePlan resource_plan{};
};

class UtilityNetwork {
public:
    void clear(){ connections_.clear(); }
    void set_playfield_half_extent(float half_extent){ playfield_half_extent_=std::max(8.0f,half_extent); }
    const std::vector<UtilityConnection>& connections() const { return connections_; }
    void restore_connections(std::vector<UtilityConnection> connections){ connections_=std::move(connections); }

    bool anchor_compatible(UtilityKind kind,const UtilityBuildAnchor& anchor,
                           const std::vector<PlacedHabitat>& habitats,
                           const std::vector<PlacedResource>& resources) const;

    // Resolve one snapped structure anchor to the actual mesh-surface service
    // port used by a connection. This is also used before the first click so
    // the source-port blueprint can float over the selected structure.
    std::optional<UtilityEndpoint> preview_anchor(
        UtilityKind kind,const UtilityBuildAnchor& anchor,eng::Vec3 toward,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const;

    UtilityMesh preview_endpoint_mesh(const UtilityEndpoint& endpoint) const;

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
    void remove_connection(std::size_t index);
    void remove_habitat(std::size_t habitat_index);
    void remove_resource(std::size_t resource_index);
    void update_construction(float dt,ResourceManagement& resources,bool allow_negative);

private:
    void rebuild_supply_junctions();
    std::vector<UtilityConnection> connections_{};
    float playfield_half_extent_{32.0f};
};

const char* utility_kind_name(UtilityKind kind);
const char* utility_endpoint_name(UtilityEndpointKind kind);

} // namespace game
