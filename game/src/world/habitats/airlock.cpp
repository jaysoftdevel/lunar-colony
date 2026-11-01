#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_module.hpp"

namespace game::habitats {
void build_airlock_outer(Builder& b){
    b.mesh.footprint_x=10.0f; b.mesh.footprint_z=6.5f; b.mesh.height=5.8f;
    b.cylinder_x(b.mesh.shell,{0,2.55f,0},7.2f,2.15f,18,true);
    // Structural ribs and service conduits follow the cylindrical body.
    for(float x:{-2.5f,-0.8f,0.9f,2.6f}) b.torus_x(b.mesh.details,{x,2.55f,0},2.18f,0.08f,18,6);

    // The three-chamber pressure body keeps closed structural end pods, but
    // they contain no authored hatch. The first actual connection cuts/owns
    // the generated base-side doorway; the opposite outside seal is derived
    // dynamically from that choice.
    b.box(b.mesh.shell,{4.05f,2.30f,0},{1.45f,3.7f,3.75f});
    b.box(b.mesh.shell,{-4.05f,2.30f,0},{1.45f,3.7f,3.75f});

    // Roof service cap and side utility boxes.
    b.cylinder_y(b.mesh.details,{-1.7f,4.95f,0.35f},0.70f,0.52f,12,true);
    b.cylinder_y(b.mesh.details,{1.35f,4.82f,-0.55f},0.55f,0.38f,12,true);
    b.box(b.mesh.details,{-1.1f,2.3f,2.05f},{0.95f,0.90f,0.18f});
    b.box(b.mesh.details,{1.45f,1.65f,2.08f},{0.75f,0.72f,0.16f});
    // Side porthole.
    b.torus_z(b.mesh.details,{0.7f,2.65f,2.20f},0.55f,0.08f,16,6);
    b.cylinder_z(b.mesh.glazing,{0.7f,2.65f,2.24f},0.08f,0.46f,16,false);

    b.supports(-2.7f,2.4f,1.55f,0.85f);
    b.supports(-2.7f,2.4f,-1.55f,0.85f);

}

} // namespace game::habitats
