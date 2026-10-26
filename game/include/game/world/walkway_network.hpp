#pragma once
#include "engine/engine.hpp"
#include "game/world/corridor_network.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include <optional>
#include <vector>

namespace game {

enum class WalkwayEndpointKind { Habitat = 0, Resource, Open };

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
    bool generated_access{false}; // external access gate; never pressurized
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
    WalkwayMesh mesh{};
    bool carries_power{true};
    bool carries_water{true};
    bool pressurized{false}; // exterior crew access only; intentionally no atmosphere
};

class WalkwayNetwork {
public:
    void clear(){ connections_.clear(); }
    const std::vector<WalkwayConnection>& connections() const { return connections_; }

    std::optional<WalkwayConnection> preview_connection(
        const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,
        const CorridorNetwork& corridors,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const;

    bool build_connection(
        const WalkwayBuildAnchor& source,const WalkwayBuildAnchor& target,
        const CorridorNetwork& corridors,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& habitat_library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library);

    float connection_length(std::size_t index) const;
    std::size_t endpoint_connection_count(WalkwayEndpointKind kind,std::size_t index) const;
    void remove_connection(std::size_t index);
    void remove_habitat(std::size_t habitat_index);
    void remove_resource(std::size_t resource_index);

private:
    std::vector<WalkwayConnection> connections_{};
};

const char* walkway_endpoint_name(WalkwayEndpointKind kind);

} // namespace game
