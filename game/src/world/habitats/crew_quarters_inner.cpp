#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_crew_quarters_inner(Builder& b){
    const float floor=b.mesh.walkable_floor_y;
    b.box(b.mesh.interior,{0,floor-.08f,0},{10.8f,.16f,3.5f},false);
    // Small quarters have a staggered pair. Each larger tier adds
    // another pair while keeping a full-width aisle between bunk bays.
    constexpr std::array<Vec2,8> bunks{{
        {-1.58f,-1.48f},{1.58f,1.48f},{-1.58f,1.48f},{1.58f,-1.48f},
        {-4.03f,-1.48f},{4.03f,1.48f},{-4.03f,1.48f},{4.03f,-1.48f},
    }};
    for(const auto& bunk:bunks){
        b.mesh.crew_bunks.push_back({{bunk.x,floor,std::copysign(.45f,bunk.y)},
                                     {bunk.x-.78f,1.59f,bunk.y}});
    }
    for(float x:{-3.25f,0.f,3.25f})ceiling_light(b,{x,4.23f,-.75f},{x,4.23f,.75f});
}

} // namespace game::habitats
