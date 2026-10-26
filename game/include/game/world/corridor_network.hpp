#pragma once
#include "game/world/habitat_mesh.hpp"
#include <cstddef>
#include <optional>
#include <vector>

namespace game {

struct CorridorDoor {
    eng::Vec3 position{};
    eng::Vec3 wall_position{};
    eng::Vec3 normal{1.0f,0.0f,0.0f};
    bool generated{false};
};

struct CorridorMesh {
    std::vector<eng::Vec3> shell;
    std::vector<eng::Vec3> details; // ribs, docking hardware and internal walkway
    std::vector<eng::Vec3> edges;
};

enum class CorridorEndpointKind {
    Habitat,
    Corridor,
    OpenDoor
};

struct CorridorBuildAnchor {
    CorridorEndpointKind kind{CorridorEndpointKind::OpenDoor};
    std::size_t index{};
    eng::Vec3 position{};
};

struct CorridorEndpoint {
    CorridorEndpointKind kind{CorridorEndpointKind::OpenDoor};
    std::size_t index{};
    eng::Vec3 position{}; // actual corridor centerline endpoint
    eng::Vec3 tangent{1.0f,0.0f,0.0f}; // outward direction from endpoint
    CorridorDoor door{}; // valid for habitat endpoints
};

enum class CorridorConnectionKind {
    Automatic,
    NetworkLink,
    BuiltSection
};

struct CorridorConnection {
    CorridorConnectionKind kind{CorridorConnectionKind::Automatic};
    CorridorEndpoint endpoint_a{};
    CorridorEndpoint endpoint_b{};
    std::vector<eng::Vec3> path{};
    CorridorMesh mesh{};
};

class CorridorNetwork {
public:
    void clear(){ connections_.clear(); }

    void connect_new_habitat(std::size_t new_index,
                             const std::vector<PlacedHabitat>& habitats,
                             const HabitatModelLibrary& library);

    std::optional<CorridorConnection> preview_connection(
        std::size_t new_index,
        const std::vector<PlacedHabitat>& habitats,
        const HabitatModelLibrary& library) const;

    std::optional<CorridorConnection> preview_corridor_link(
        std::size_t corridor_a,
        std::size_t corridor_b,
        const std::vector<PlacedHabitat>& habitats,
        const HabitatModelLibrary& library) const;

    bool connect_corridors(std::size_t corridor_a,
                           std::size_t corridor_b,
                           const std::vector<PlacedHabitat>& habitats,
                           const HabitatModelLibrary& library);

    std::optional<CorridorConnection> preview_built_corridor(
        const CorridorBuildAnchor& a,
        const CorridorBuildAnchor& b,
        const std::vector<PlacedHabitat>& habitats,
        const HabitatModelLibrary& library) const;

    bool build_corridor(const CorridorBuildAnchor& a,
                        const CorridorBuildAnchor& b,
                        const std::vector<PlacedHabitat>& habitats,
                        const HabitatModelLibrary& library);

    CorridorBuildAnchor corridor_anchor(std::size_t corridor_index,eng::Vec3 near_point) const;
    eng::Vec3 nearest_point_on_corridor(std::size_t corridor_index,eng::Vec3 p) const;

    std::size_t habitat_connection_count(std::size_t habitat_index) const;
    float connection_length(std::size_t corridor_index) const;
    const std::vector<CorridorConnection>& connections() const { return connections_; }

private:
    std::vector<CorridorConnection> connections_{};
};

} // namespace game
