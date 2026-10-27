#pragma once
#include "game/systems/resource_management.hpp"
#include "game/world/habitat_mesh.hpp"
#include "game/world/resource_mesh.hpp"
#include "engine/engine.hpp"
#include <cstddef>
#include <vector>

namespace game::construction_costs {

ConstructionResourcePlan habitat(HabitatKind kind,HabitatSizeTier size,const HabitatMesh& mesh);
ConstructionResourcePlan structure(ResourceKind kind,const ResourceMesh& mesh);
ConstructionResourcePlan corridor(float length_m);
ConstructionResourcePlan corridor(const std::vector<eng::Vec3>& path,std::size_t network_junctions=0);
ConstructionResourcePlan supply_channel(float length_m);
ConstructionResourcePlan supply_channel(const std::vector<eng::Vec3>& path,std::size_t junctions=0);
ConstructionResourcePlan walkway(float length_m);

} // namespace game::construction_costs
