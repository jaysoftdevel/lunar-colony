#pragma once
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include "game/world/construction.hpp"
#include "game/systems/resource_management.hpp"
#include <cstddef>
#include <optional>
#include <vector>

namespace game {

struct CorridorDoor {
    eng::Vec3 position{};
    eng::Vec3 wall_position{};
    eng::Vec3 normal{1.0f,0.0f,0.0f};
    bool generated{false};
    int entrance_index{-1}; // built-in habitat entrance index; -1 for generated doors
    // For generated entrances this stores the exact pressure-hull contact ring
    // in world space. The vestibule transitions from these surface points to
    // its outer door profile, so curved/faceted structures remain airtight.
    std::vector<eng::Vec3> wall_profile{};
};

struct CorridorMesh {
    std::vector<eng::Vec3> shell;
    std::vector<eng::Vec3> details; // ribs, docking hardware and internal walkway
    std::vector<eng::Vec3> edges;
};

enum class CorridorEndpointKind {
    Habitat,
    Resource,
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
    eng::Vec3 tangent{1.0f,0.0f,0.0f}; // outward direction from endpoint / new route
    eng::Vec3 host_tangent{0.0f,0.0f,0.0f}; // existing corridor direction at a corridor junction
    eng::Vec3 host_position{}; // exact host corridor centerline point for surface-matched branches
    CorridorDoor door{}; // valid for habitat endpoints
    bool suppress_open_door{false}; // open corridor end consumed by a close corridor connection
    bool seamless_splice{false}; // corridor end-to-end splice: no junction/entrance fitting at this endpoint
    bool surface_matched_junction{false}; // branch terminates on the exact host D-profile surface
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
    StagedConstructionState construction{kCorridorConstructionSteps};
    ConstructionResourcePlan resource_plan{};
};

class CorridorNetwork {
public:
    void clear(){ connections_.clear(); }

    std::optional<CorridorConnection> preview_corridor_link(
        std::size_t corridor_a,
        std::size_t corridor_b,
        const std::vector<PlacedHabitat>& habitats,
        const HabitatModelLibrary& library) const;

    bool connect_corridors(std::size_t corridor_a,
                           std::size_t corridor_b,
                           const std::vector<PlacedHabitat>& habitats,
                           const HabitatModelLibrary& library);

    std::optional<CorridorEndpoint> preview_anchor(
        const CorridorBuildAnchor& anchor,eng::Vec3 other,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library) const;
    std::optional<CorridorEndpoint> preview_anchor(
        const CorridorBuildAnchor& anchor,eng::Vec3 other,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const;

    CorridorMesh preview_endpoint_mesh(const CorridorEndpoint& endpoint) const;

    std::optional<CorridorConnection> preview_built_corridor(
        const CorridorBuildAnchor& a,const CorridorBuildAnchor& b,
        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library) const;

    bool build_corridor(const CorridorBuildAnchor& a,const CorridorBuildAnchor& b,
                        const std::vector<PlacedHabitat>& habitats,const HabitatModelLibrary& library,
                        const std::vector<PlacedResource>& resources,const ResourceModelLibrary& resource_library);

    CorridorBuildAnchor corridor_anchor(std::size_t corridor_index,eng::Vec3 near_point) const;
    eng::Vec3 nearest_point_on_corridor(std::size_t corridor_index,eng::Vec3 p) const;

    std::size_t habitat_connection_count(std::size_t habitat_index) const;
    std::size_t resource_connection_count(std::size_t resource_index) const;
    bool habitat_entrance_connected(std::size_t habitat_index,std::size_t entrance_index) const;
    float connection_length(std::size_t corridor_index) const;
    void remove_corridor(std::size_t corridor_index);
    void remove_habitat(std::size_t habitat_index);
    void remove_resource(std::size_t resource_index);
    void update_construction(float dt,ResourceManagement& resources,bool allow_negative);
    const std::vector<CorridorConnection>& connections() const { return connections_; }

private:
    std::vector<CorridorConnection> connections_{};
};

} // namespace game
