#include "game/world/habitats/habitat_module.hpp"
#include "game/world/habitats/habitat_interior_builder.hpp"
namespace game::habitats {
using namespace game::runtime;
void build_airlock_inner(Builder& b){
    auto& m=b.mesh;const float floor=m.walkable_floor_y;
    // Integral recovery pumps, filters, valves and vents: no floor machinery.
    // Even the Small variant keeps a clear 1.8 m longitudinal transfer aisle.
    for(float sign:{-1.f,1.f}){
        const float z=sign*2.05f;
        for(float x:{-.40f,.40f})for(float y:{.90f,1.50f})
            b.box(m.service_metal,{x,floor+y,sign*2.34f},{.07f,.07f,.30f},false);

        b.box(m.service_metal,{0,floor+1.18f,z},{1.08f,.90f,.30f},false);
        b.box(m.fabric,{0,floor+1.18f,z-sign*.157f},{.90f,.68f,.018f},false);
        for(float x:{-.30f,0.f,.30f}){
            b.cylinder_y(m.service_metal,{x,floor+.57f,z},.50f,.105f,12,false);
            b.cylinder_between(m.service_metal,{x,floor+.82f,z},{x,floor+1.77f,z},.022f,8,true,false);
        }
        b.cylinder_between(m.service_metal,{-.78f,floor+1.82f,z},{.78f,floor+1.82f,z},.038f,10,true,false);
        for(float x:{-3.05f,3.05f}){
            const float wall_z=sign*1.67f;
            b.box(m.service_metal,{x,floor+1.86f,wall_z},{.64f,.24f,.08f},false);
            for(int slot=0;slot<7;++slot)b.box(m.fabric,{x+(slot-3)*.075f,floor+1.86f,wall_z-sign*.045f},{.035f,.16f,.012f},false);
            b.cylinder_between(m.service_metal,{x-.36f,floor+1.04f,wall_z-sign*.08f},{x+.36f,floor+1.04f,wall_z-sign*.08f},.027f,10,true,false);
        }
    }
    for(float x:{-3.05f,0.f,3.05f})
        b.cylinder_between(m.lamps,{x-.40f,floor+2.78f,0},{x+.40f,floor+2.78f,0},.028f,8,true,false);
}
} // namespace game::habitats
