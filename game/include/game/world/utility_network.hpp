#pragma once
#include "game/world/supply_state.hpp"
#include "engine/engine.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include "game/world/corridor_network.hpp"
#include "game/world/construction.hpp"
#include "game/systems/resource_management.hpp"
#include <algorithm>
#include <array>
#include <optional>
#include <limits>
#include <vector>
#include <utility>

namespace game {

// v0.15.0 merges the former electrical and water build tools into one physical
// supply channel. A channel carries both services through one shared tray/pipe
// assembly; individual structures simply consume the services they need.
enum class UtilityKind {
    SupplyChannel = 0,
    ConveyorBelt
};

enum class UtilityEndpointKind {
    Habitat = 0,
    Resource,
    Open,
    // Appended to preserve the serialized values used by existing saves. A
    // supply channel may terminate on a corridor wall and feed the service
    // pipes carried inside the pressurized corridor network.
    Corridor
};

struct UtilityBuildAnchor {
    UtilityEndpointKind kind{UtilityEndpointKind::Habitat};
    std::size_t index{0};
    eng::Vec3 hit{};
    // Outward direction for network snaps or a conveyor start already picked.
    eng::Vec3 normal_hint{};
};

struct UtilityEndpoint {
    UtilityEndpointKind kind{UtilityEndpointKind::Habitat};
    std::size_t index{0};
    eng::Vec3 position{};
    eng::Vec3 normal{1.0f,0.0f,0.0f};
};

enum class ConveyorRoute { Invalid, OreProcessor, OreStorage, Trade };

ConveyorRoute conveyor_route(const UtilityEndpoint& source,const UtilityEndpoint& receiver,
                            const std::vector<PlacedHabitat>& habitats,const std::vector<PlacedResource>& resources);

struct UtilityMesh {
    std::vector<eng::Vec3> shell;
    std::vector<eng::Vec3> details;
    std::vector<eng::Vec3> edges;
};


inline constexpr std::size_t kNoUtilityComponent=std::numeric_limits<std::size_t>::max();

struct UtilityServiceState {
    bool connected{false};
    bool power{false};
    bool water{false};
    // Runtime-only topology identity. Structures with the same component id can
    // exchange utility-fed resources without any global/teleporting shortcut.
    std::size_t component{kNoUtilityComponent};
};

struct UtilityServiceSnapshot {
    std::vector<UtilityServiceState> habitats{};
    std::vector<UtilityServiceState> resources{};
    // Live graph in habitat/resource/channel/corridor order, including junctions.
    std::vector<std::pair<std::size_t,std::size_t>> links{};
    std::vector<std::size_t> node_components{};
    std::vector<double> run_lengths{};
    std::vector<double> air_link_lengths{}; // same order as links
};

struct UtilityConnection {
    std::uint8_t ore_cargo{ore::no_cargo};
    std::uint8_t ore_next{0};
    float ore_distance_m{0};
    eng::Vec3 cargo_position(float distance) const;
    eng::Vec3 ore_box_position() const { return cargo_position(ore_distance_m); }
    SupplyState supply{};
    UtilityKind kind{UtilityKind::SupplyChannel};
    std::uint64_t placement_sequence{0};
    UtilityEndpoint endpoint_a{};
    UtilityEndpoint endpoint_b{};
    std::vector<eng::Vec3> path{};
    // Supply channels form a shared manifold wherever two channel centerlines
    // cross. The junction is common to both power and water services.
    std::vector<eng::Vec3> junctions{};
    UtilityMesh mesh{};
    StagedConstructionState construction{kSupplyConstructionSteps};
    ConstructionResourcePlan resource_plan{};
    ResourceBundle site_buffer{}; // delivered construction material staged at this blueprint
};

class UtilityNetwork {
public:
    void clear(){ connections_.clear(); }
    void set_playfield_half_extent(float half_extent){ playfield_half_extent_=std::max(8.0f,half_extent); }
    const std::vector<UtilityConnection>& connections() const { return connections_; }
    std::vector<UtilityConnection>& mutable_connections() { return connections_; }
    void restore_connections(std::vector<UtilityConnection> connections,bool rebuild_geometry=false){
        connections_=std::move(connections);
        if(rebuild_geometry) rebuild_supply_junctions();
    }

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

    std::optional<UtilityConnection> preview_conveyor_start(const UtilityEndpoint& endpoint) const;

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
    void append_connection(UtilityConnection connection); // validated preview
    void remove_connection(std::size_t index);
    void remove_habitat(std::size_t habitat_index);
    void remove_resource(std::size_t resource_index);
    void remove_corridor(std::size_t corridor_index);

    // Reuse the completed service graph until its inputs change. The returned
    // snapshot belongs to this network and is replaced when the graph changes.
    const UtilityServiceSnapshot& evaluate_services(
        const CorridorNetwork& corridors,
        const std::vector<PlacedHabitat>& habitats,
        const std::vector<PlacedResource>& resources) const;

    void update_construction(float dt,ResourceManagement& resources,bool allow_negative);

private:
    struct ServiceStructureInput {
        bool complete{},powered{};
        ResourceKind kind{};
        std::array<float,3> position{};
        bool operator==(const ServiceStructureInput&) const = default;
    };
    struct ServiceLinkInput {
        bool active{};
        int a_kind{},b_kind{};
        std::size_t a_index{},b_index{},points{};
        bool operator==(const ServiceLinkInput&) const = default;
    };
    struct ServiceInputs {
        std::size_t habitat_count{},utility_count{};
        std::vector<ServiceStructureInput> structures;
        std::vector<ServiceLinkInput> links;
        std::vector<std::array<float,3>> centerlines;
        bool operator==(const ServiceInputs&) const = default;
    };

    UtilityServiceSnapshot build_service_snapshot(
        const CorridorNetwork& corridors,
        const std::vector<PlacedHabitat>& habitats,
        const std::vector<PlacedResource>& resources) const;
    bool conveyor_source_used(std::size_t resource_index) const;
    void rebuild_supply_junctions();
    std::vector<UtilityConnection> connections_{};
    mutable ServiceInputs service_inputs_,next_service_inputs_;
    mutable UtilityServiceSnapshot service_snapshot_;
    float playfield_half_extent_{32.0f};
};

const char* utility_kind_name(UtilityKind kind);
const char* utility_endpoint_name(UtilityEndpointKind kind);

} // namespace game
