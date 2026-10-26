#pragma once
#include "engine/engine.hpp"
#include <array>
#include <vector>

namespace game {

enum class HabitatKind {
    AirLock = 0,
    OxygenGeneratorDome,
    CantineDome,
    BioDome,
    CrewQuarters,
    Count
};

struct HabitatEntrance {
    eng::Vec3 local_position{};
    eng::Vec3 local_normal{1.0f,0.0f,0.0f};
};

struct HabitatEntranceAccess {
    std::vector<eng::Vec3> details;
    std::vector<eng::Vec3> edges;
};

struct HabitatMesh {
    std::vector<eng::Vec3> shell;
    std::vector<eng::Vec3> foundation;
    std::vector<eng::Vec3> interior;
    std::vector<eng::Vec3> details;
    std::vector<eng::Vec3> glazing;
    std::vector<eng::Vec3> edges;
    float footprint_x{1.0f};
    float footprint_z{1.0f};
    float height{1.0f};
    std::vector<HabitatEntrance> entrances{};
    // Optional exterior access geometry indexed like entrances. This is kept
    // separate from the permanent habitat mesh so stairs/ramps can disappear
    // when that exact entrance is pressure-docked to a corridor.
    std::vector<HabitatEntranceAccess> entrance_access{};
};

struct PlacedHabitat {
    HabitatKind kind{HabitatKind::AirLock};
    eng::Vec3 position{};
    float rotation_degrees{0.0f};
};

class HabitatModelLibrary {
public:
    HabitatModelLibrary();
    const HabitatMesh& mesh(HabitatKind kind) const;
private:
    std::array<HabitatMesh,(size_t)HabitatKind::Count> meshes_{};
};

} // namespace game
