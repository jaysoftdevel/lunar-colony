#pragma once
#include "game/systems/resource_management.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"

namespace game::construction_costs {

ConstructionResourcePlan habitat(HabitatKind kind,HabitatSizeTier size,const HabitatMesh& mesh);
ConstructionResourcePlan structure(ResourceKind kind,const ResourceMesh& mesh);
ConstructionResourcePlan corridor(float length_m);
ConstructionResourcePlan supply_channel(float length_m);
ConstructionResourcePlan walkway(float length_m);

} // namespace game::construction_costs
