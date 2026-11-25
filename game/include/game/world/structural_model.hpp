#pragma once
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
namespace game::structural {
Metrics reinforce(HabitatMesh&,HabitatKind);
Metrics reinforce(ResourceMesh&,ResourceKind);
}
