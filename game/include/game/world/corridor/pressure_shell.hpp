#pragma once
#include "game/world/corridor_network.hpp"
namespace game::corridor_detail {
struct PressureChamber {
    game::runtime::Vec3 position{};
    std::vector<game::runtime::Vec3> footprint;
    std::size_t owner{};
};
std::vector<PressureChamber> pressure_chambers(const std::vector<CorridorConnection>& connections);
// Combine closed convex pressure cells by polygon clipping, retaining the
// exact exposed wall/roof boundary rather than deleting intersecting triangles.
void rebuild_pressure_shells(std::vector<CorridorConnection>& connections);
}
