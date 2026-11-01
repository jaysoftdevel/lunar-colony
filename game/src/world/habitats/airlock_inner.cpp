#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"

namespace game::habitats {
using namespace eng;
using namespace interior_detail;

void build_airlock_inner(Builder& b){
    // Three bays, with a real pressure bulkhead on either side of the center.
    // The shared post-scaling crew-interface pass carves human-sized apertures;
    // runtime sliding leaves occupy those same apertures in both size tiers.
    b.box(b.mesh.interior,{0,0.72f,0},{9.5f,0.16f,3.2f},false);
    for(float x:{-1.6f,1.6f}){
        b.cylinder_x(b.mesh.interior,{x,2.55f,0},0.14f,2.12f,18,false);
        b.torus_x(b.mesh.interior,{x,2.55f,0},2.06f,0.055f,18,6);
        console(b,{x+0.32f,1.38f,1.52f},0.60f,1.0f);
    }
    // Both end cabins have suit/isolation equipment, so either authored hatch
    // can face the colony without rebuilding or mirroring the structure.
    for(float x:{-3.15f,3.15f}){
        locker_bank(b,{x,1.75f,-1.65f},2);
        b.cylinder_y(b.mesh.interior,{x,1.75f,1.6f},1.70f,0.20f,10,false);
        ceiling_light(b,{x-0.55f,4.0f,0},{x+0.55f,4.0f,0});
    }
    // Central pressure-control/recovery manifold stays beside the clear aisle.
    console(b,{0,1.38f,1.52f},1.0f,1.0f);
    tank(b,{-0.45f,0.8f,-1.65f},1.8f,0.24f);
    tank(b,{0.45f,0.8f,-1.65f},1.8f,0.24f);
    ceiling_light(b,{-1.2f,4.15f,0},{1.2f,4.15f,0});
    b.cylinder_between(b.mesh.interior,{-3.0f,3.82f,1.3f},{3.0f,3.82f,1.3f},0.06f,6,true,false);
}
} // namespace game::habitats
