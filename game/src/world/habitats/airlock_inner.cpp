#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_airlock_inner(Builder& b){
    // Three bays, with a real pressure bulkhead on either side of the center.
    // The shared post-scaling crew-interface pass carves human-sized apertures;
    // runtime sliding leaves occupy those same apertures in both size tiers.
    const float floor=b.mesh.walkable_floor_y;
    b.box(b.mesh.interior,{0,floor-.08f,0},{9.5f,.16f,3.2f},false);
    for(float x:{-1.6f,1.6f}){
        b.cylinder_x(b.mesh.interior,{x,2.55f,0},0.14f,2.12f,18,false);
        b.torus_x(b.mesh.interior,{x,2.55f,0},2.06f,0.055f,18,6);
    }
    // Both end cabins have suit/isolation equipment, so either authored hatch
    // can face the colony without rebuilding or mirroring the structure.
    for(float x:{-3.15f,3.15f}){
        locker_bank(b,{x,floor+.775f,-1.55f},2,.57f);
        bench(b,{x,floor,1.54f},.75f,.38f,.56f);
        tank(b,{x,floor+.56f,1.54f},.88f,.14f);
        ceiling_light(b,{x-0.55f,4.0f,0},{x+0.55f,4.0f,0});
    }
    // Central pressure-control/recovery manifold stays beside the clear aisle.
    console(b,{0,floor+.34f,1.55f},.86f,-1.f);
    for(float x:{-.44f,.44f})tank(b,{x,floor,-1.55f},1.48f,.18f);
    ceiling_light(b,{-1.2f,4.15f,0},{1.2f,4.15f,0});
    b.cylinder_between(b.mesh.interior,{-3.0f,3.82f,1.3f},{3.0f,3.82f,1.3f},0.06f,6,true,false);
}
} // namespace game::habitats
