#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_water_purifier(){
    Builder b;
    b.mesh.footprint_x=10.5f; b.mesh.footprint_z=8.5f; b.mesh.height=6.8f;
    b.box(b.mesh.foundation,{0,0.42f,0},{8.8f,0.84f,6.6f});
    for(Vec3 p:std::initializer_list<Vec3>{{-4.4f,0,-3.1f},{4.4f,0,-3.1f},{-4.4f,0,3.1f},{4.4f,0,3.1f}}) b.foot(p,0.44f,0.18f);

    // Three-stage recycling plant: settling/filter columns, membrane block and clean-water buffer.
    for(float x:{-3.15f,-1.85f,-0.55f}){
        b.cylinder_y(b.mesh.body,{x,2.45f,-1.35f},3.7f,0.58f,16,true);
        b.cylinder_y(b.mesh.details,{x,4.45f,-1.35f},0.22f,0.33f,10,true);
    }
    b.box(b.mesh.body,{1.45f,2.15f,-1.25f},{2.5f,3.2f,1.8f});
    for(int i=0;i<6;++i) b.box(b.mesh.details,{0.55f+0.35f*i,2.15f,-2.19f},{0.12f,2.55f,0.10f});
    b.cylinder_y(b.mesh.body,{3.2f,2.55f,1.15f},4.25f,1.05f,18,true);
    b.cylinder_y(b.mesh.details,{3.2f,4.80f,1.15f},0.25f,0.50f,12,true);

    // Pumps and manifold gallery.
    for(float z:{0.15f,1.45f}){
        b.cylinder_x(b.mesh.details,{-1.6f,1.25f,z},1.85f,0.38f,12,true);
        b.box(b.mesh.details,{-2.72f,1.25f,z},{0.55f,0.85f,0.85f});
    }
    b.cylinder_x(b.mesh.details,{0.2f,1.05f,2.55f},6.6f,0.14f,10,true);
    b.cylinder_x(b.mesh.details,{0.2f,1.55f,2.55f},6.6f,0.14f,10,true);
    for(float x:{-3.0f,-1.0f,1.0f,3.0f})
        b.cylinder_between(b.mesh.details,{x,1.05f,2.55f},{x,2.0f,1.1f},0.11f,8,true,true);

    // UV/sterilization cabinet and controls.
    b.box(b.mesh.details,{1.0f,1.45f,2.25f},{1.65f,1.55f,0.75f});
    b.box(b.mesh.details,{-3.4f,1.45f,2.20f},{1.45f,1.45f,0.75f});
    b.cylinder_y(b.mesh.details,{0.0f,4.9f,-0.2f},1.45f,0.32f,12,true);
    b.cylinder_y(b.mesh.details,{0.75f,4.7f,-0.1f},1.1f,0.24f,10,true);
    return std::move(b.mesh);
}
}
