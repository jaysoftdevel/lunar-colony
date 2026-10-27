#pragma once
#include "game/world/structure_module.hpp"
#include "game/world/corridor_network.hpp"
#include <vector>

namespace game::corridor_detail {

// Corridors participate in the same structural access contract as habitats.
// Their pressure shell/detail frame is the outer module; their continuous
// walking deck is the inner module.
class CorridorModule final : public game::StructureModule {
public:
    const char* module_id() const override { return "corridor"; }
    game::StructureAccessProfile access_profile() const override;

    void build_outer(CorridorMesh& mesh,const std::vector<eng::Vec3>& path) const;
    void build_inner(CorridorMesh& mesh,const std::vector<eng::Vec3>& path) const;
};

const CorridorModule& corridor_module();

} // namespace game::corridor_detail
