#pragma once
#include "engine/engine.hpp"
#include "game/world/corridor_network.hpp"
#include "game/world/utility_network.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include "game/world/construction.hpp"
#include "game/systems/resource_management.hpp"
#include <optional>
#include <vector>
#include <utility>

namespace game {

enum class WalkwayEndpointKind { Habitat = 0, Resource, Corridor, Walkway, Open };

struct WalkwayBuildAnchor {
    WalkwayEndpointKind kind{WalkwayEndpointKind::Open};
    std::size_t index{0};
    eng::Vec3 hit{};
};

struct WalkwayEndpoint {
    WalkwayEndpointKind kind{WalkwayEndpointKind::Open};
    std::size_t index{0};
    eng::Vec3 position{};
    eng::Vec3 normal{1.0f,0.0f,0.0f};
    CorridorDoor door{}; // populated for habitat endpoints
    bool landing_pad{false};
    bool generated_access{false}; // exterior/generated access gate; never pressurized
};

struct WalkwayJunction {
    eng::Vec3 position{};
    std::vector<eng::Vec3> arms{}; // outward centerline directions participating in the merge
    float radius{1.55f};            // centerline trim radius used by every participating segment
    bool owns_patch{false};        // exactly one connection renders the shared junction deck
};


struct WalkwayCorridorCrossing {
    std::size_t corridor_index{0};
    eng::Vec3 center{};
    eng::Vec3 walkway_tangent{1.0f,0.0f,0.0f};
    eng::Vec3 corridor_tangent{0.0f,0.0f,1.0f};
    eng::Vec3 entrance_a{};
    eng::Vec3 entrance_b{};
};

struct WalkwaySupplyBridge {
    std::size_t utility_index{0};
    eng::Vec3 center{};
    eng::Vec3 tangent{1.0f,0.0f,0.0f};
    float path_distance{0.0f};
};

struct WalkwayMesh {
    std::vector<eng::Vec3> deck;
    std::vector<eng::Vec3> details;
    std::vector<eng::Vec3> edges;
};

struct WalkwayConnection {
    WalkwayEndpoint endpoint_a{};
    WalkwayEndpoint endpoint_b{};
    std::vector<eng::Vec3> path{};
    std::vector<WalkwayJunction> junctions{};
    std::vector<WalkwayCorridorCrossing> corridor_crossings{};
    std::vector<WalkwaySupplyBridge> supply_bridges{};
    WalkwayMesh mesh{};
    StagedConstructionState construction{kWalkwayConstructionSteps};
    ConstructionResourcePlan resource_plan{};
    bool carries_power{true};
    bool carries_water{true};
    bool pressurized{false}; // exterior crew access only; intentionally no atmosphere
};

class WalkwayNetwork {
public:
    void clear(){ connections_.clear(); }
    const std::vector<WalkwayConnection>& connections() const { return connections_; }
    void restore_connections(std::vector<WalkwayConnection> connections){ connections_=std::move(connections); }

    std::optional<WalkwayConnection> preview_connection(
        const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,
        const CorridorNetwork& corridors,const UtilityNetwork& utilities,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const;

    // Returns a best-effort geometric preview even when the requested target is
    // invalid. It is used only to draw the attempted route in red; it is never
    // committed to the world.
    std::optional<WalkwayConnection> preview_invalid_connection(
        const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,
        const CorridorNetwork& corridors,const UtilityNetwork& utilities,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const;

    bool build_connection(
        const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,
        const CorridorNetwork& corridors,const UtilityNetwork& utilities,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library);

    float connection_length(std::size_t index) const;
    std::size_t endpoint_connection_count(WalkwayEndpointKind kind,std::size_t index) const;
    void remove_connection(std::size_t index);
    void remove_habitat(std::size_t habitat_index);
    void remove_resource(std::size_t resource_index);
    void remove_corridor(std::size_t corridor_index);
    void remove_utility(std::size_t utility_index);
    void update_construction(float dt,ResourceManagement& resources,bool allow_negative);

private:
    void rebuild_walkway_merges();
    std::vector<WalkwayConnection> connections_{};
};

const char* walkway_endpoint_name(WalkwayEndpointKind kind);

} // namespace game
