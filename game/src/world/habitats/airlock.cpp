#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_generators.hpp"
#include "game/world/habitats/habitat_interiors.hpp"

namespace game::habitats {
namespace {
HabitatEntranceAccess make_stairs(const HabitatEntrance& entrance){
    Builder b;
    const eng::Vec3 n=eng::normalize(entrance.local_normal);

    // Highest tread is nearest the hatch; each following tread descends in the
    // outward direction. The previous air-lock mesh accidentally did this in
    // reverse, making the stair set climb away from the door.
    for(int i=0;i<3;++i){
        eng::Vec3 c=entrance.local_position+n*(0.05f+0.42f*(float)i);
        c.y=0.76f-0.24f*(float)i;
        b.box(b.mesh.details,c,{0.72f,0.18f,2.10f});
    }

    // Handrails follow the same down-and-away direction on both sides.
    for(float z:{-1.25f,1.25f}){
        eng::Vec3 near=entrance.local_position-n*0.15f;
        near.y=1.42f; near.z=z;
        eng::Vec3 far=entrance.local_position+n*1.20f;
        far.y=0.54f; far.z=z;
        b.cylinder_between(b.mesh.details,near,far,0.055f,6,true,true);
    }

    HabitatEntranceAccess out;
    out.details=std::move(b.mesh.details);
    out.edges=std::move(b.mesh.edges);
    return out;
}
}

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

    // Front pressure-lock vestibule and entry hatch.
    b.box(b.mesh.shell,{4.05f,2.30f,0},{1.45f,3.7f,3.75f});
    b.box(b.mesh.details,{4.83f,2.30f,0},{0.20f,3.15f,3.15f});
    b.torus_x(b.mesh.details,{4.96f,2.30f,0},0.72f,0.10f,18,6);
    b.cylinder_x(b.mesh.details,{5.02f,2.30f,0},0.16f,0.62f,16,true);

    // Rear pressure-lock vestibule and exit hatch. It mirrors the front hatch
    // and is a real corridor attachment point, not a generated side entrance.
    b.box(b.mesh.shell,{-4.05f,2.30f,0},{1.45f,3.7f,3.75f});
    b.box(b.mesh.details,{-4.83f,2.30f,0},{0.20f,3.15f,3.15f});
    b.torus_x(b.mesh.details,{-4.96f,2.30f,0},0.72f,0.10f,18,6);
    b.cylinder_x(b.mesh.details,{-5.02f,2.30f,0},0.16f,0.62f,16,true);

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

    // Entrance 0 = front entry, entrance 1 = rear exit.
    b.mesh.entrances.push_back({{5.25f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    b.mesh.entrances.push_back({{-5.25f,1.18f,0.0f},{-1.0f,0.0f,0.0f}});
    b.mesh.entrance_access.push_back(make_stairs(b.mesh.entrances[0]));
    b.mesh.entrance_access.push_back(make_stairs(b.mesh.entrances[1]));
    interiors::add_airlock(b);
    return std::move(b.mesh);
}

} // namespace game::habitats
