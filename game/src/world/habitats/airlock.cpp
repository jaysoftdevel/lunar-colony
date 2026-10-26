#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_generators.hpp"

namespace game::habitats {
HabitatMesh make_airlock(){
    Builder b;
    b.mesh.footprint_x=10.0f; b.mesh.footprint_z=6.5f; b.mesh.height=5.8f;
    b.cylinder_x(b.mesh.shell,{0,2.55f,0},7.2f,2.15f,18,true);
    // Interior deck and equipment remain opaque when hull view modes are active.
    b.box(b.mesh.interior,{0,0.72f,0},{6.4f,0.16f,3.2f},false);
    b.box(b.mesh.interior,{-1.7f,1.55f,-1.35f},{1.6f,0.65f,0.48f},false);
    b.box(b.mesh.interior,{0.9f,1.55f,-1.35f},{1.6f,0.65f,0.48f},false);
    b.cylinder_y(b.mesh.interior,{2.35f,1.55f,0.95f},1.6f,0.30f,10,false);
    // Structural ribs and service conduits follow the cylindrical body.
    for(float x:{-2.5f,-0.8f,0.9f,2.6f}) b.torus_x(b.mesh.details,{x,2.55f,0},2.18f,0.08f,18,6);
    // Front pressure-lock vestibule and outer hatch.
    b.box(b.mesh.shell,{4.05f,2.30f,0},{1.45f,3.7f,3.75f});
    b.box(b.mesh.details,{4.83f,2.30f,0},{0.20f,3.15f,3.15f});
    b.torus_x(b.mesh.details,{4.96f,2.30f,0},0.72f,0.10f,18,6);
    b.cylinder_x(b.mesh.details,{5.02f,2.30f,0},0.16f,0.62f,16,true);
    // Roof service cap and side utility boxes.
    b.cylinder_y(b.mesh.details,{-1.7f,4.95f,0.35f},0.70f,0.52f,12,true);
    b.cylinder_y(b.mesh.details,{1.35f,4.82f,-0.55f},0.55f,0.38f,12,true);
    b.box(b.mesh.details,{-1.1f,2.3f,2.05f},{0.95f,0.90f,0.18f});
    b.box(b.mesh.details,{1.45f,1.65f,2.08f},{0.75f,0.72f,0.16f});
    // Side porthole.
    b.torus_z(b.mesh.details,{0.7f,2.65f,2.20f},0.55f,0.08f,16,6);
    b.cylinder_z(b.mesh.glazing,{0.7f,2.65f,2.24f},0.08f,0.46f,16,false);
    // Steps and handrail-like geometry.
    for(int i=0;i<3;++i) b.box(b.mesh.details,{5.2f+0.42f*i,0.28f+0.24f*i,0},{0.72f,0.18f,2.1f});
    b.cylinder_between(b.mesh.details,{5.05f,0.55f,1.25f},{6.0f,1.45f,1.25f},0.055f,6,true,true);
    b.cylinder_between(b.mesh.details,{5.05f,0.55f,-1.25f},{6.0f,1.45f,-1.25f},0.055f,6,true,true);
    b.supports(-2.7f,2.4f,1.55f,0.85f);
    b.supports(-2.7f,2.4f,-1.55f,0.85f);
    b.mesh.entrances.push_back({{5.25f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    return std::move(b.mesh);
}

} // namespace game::habitats
