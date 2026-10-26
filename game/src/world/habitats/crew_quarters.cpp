#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_generators.hpp"
#include "game/world/habitats/habitat_interiors.hpp"

namespace game::habitats {
HabitatMesh make_crew_quarters(){
    Builder b;
    b.mesh.footprint_x=14.0f; b.mesh.footprint_z=7.0f; b.mesh.height=6.0f;
    b.cylinder_x(b.mesh.shell,{0,2.65f,0},10.8f,2.35f,18,true);
    for(float x:{-4.4f,-2.2f,0.0f,2.2f,4.4f}) b.torus_x(b.mesh.details,{x,2.65f,0},2.38f,0.08f,18,6);
    // Repeated sleeping-compartment portholes on the visible side.
    for(float x:{-3.3f,0.0f,3.3f}){
        b.torus_z(b.mesh.details,{x,2.75f,2.39f},0.48f,0.075f,16,6);
        b.cylinder_z(b.mesh.glazing,{x,2.75f,2.43f},0.08f,0.40f,16,false);
        b.box(b.mesh.details,{x-0.75f,1.55f,2.38f},{0.55f,0.62f,0.16f});
    }
    // End vestibule and hatch.
    b.box(b.mesh.shell,{5.75f,2.35f,0},{1.55f,3.7f,3.55f});
    b.torus_x(b.mesh.details,{6.55f,2.35f,0},0.62f,0.09f,16,6);
    b.cylinder_x(b.mesh.details,{6.60f,2.35f,0},0.12f,0.52f,14,true);
    // Roof environmental units.
    b.cylinder_y(b.mesh.details,{-2.9f,5.1f,-0.4f},0.65f,0.52f,12,true);
    b.cylinder_y(b.mesh.details,{1.5f,5.0f,0.55f},0.70f,0.45f,12,true);
    b.box(b.mesh.details,{3.2f,4.65f,-0.65f},{1.2f,0.65f,0.9f});
    // Underfloor service trunk / basement remains opaque in view modes.
    b.box(b.mesh.foundation,{0,0.72f,0},{8.6f,0.65f,1.35f});
    // Interior center aisle and bunk modules.
    b.box(b.mesh.interior,{0,0.95f,0},{9.4f,0.16f,3.4f},false);
    for(float x:{-3.5f,-1.2f,1.2f,3.5f}){
        b.box(b.mesh.interior,{x,1.45f,-1.45f},{1.65f,0.55f,0.72f},false);
        b.box(b.mesh.interior,{x,2.25f,-1.45f},{1.65f,0.55f,0.72f},false);
        b.box(b.mesh.interior,{x,1.45f,1.45f},{1.65f,0.55f,0.72f},false);
        b.box(b.mesh.interior,{x,2.25f,1.45f},{1.65f,0.55f,0.72f},false);
    }
    b.supports(-4.0f,3.9f,1.7f,0.88f);
    b.supports(-4.0f,3.9f,-1.7f,0.88f);
    b.mesh.entrances.push_back({{6.65f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    interiors::add_crew_quarters(b);
    return std::move(b.mesh);
}
} // namespace game::habitats
