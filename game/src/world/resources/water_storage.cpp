#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_water_storage(){
    Builder b;
    b.mesh.footprint_x=11.0f; b.mesh.footprint_z=8.5f; b.mesh.height=7.2f;

    b.box(b.mesh.foundation,{0,0.38f,0},{9.4f,0.76f,7.0f});
    for(Vec3 p:std::initializer_list<Vec3>{{-4.7f,0,-3.3f},{4.7f,0,-3.3f},{-4.7f,0,3.3f},{4.7f,0,3.3f}}) b.foot(p,0.42f,0.18f);

    const Vec3 tanks[4]={{-2.65f,3.55f,-1.75f},{2.65f,3.55f,-1.75f},{-2.65f,3.55f,1.75f},{2.65f,3.55f,1.75f}};
    for(const Vec3& t:tanks){
        b.cylinder_y(b.mesh.body,t,5.55f,1.28f,16,true);
        b.cylinder_y(b.mesh.details,{t.x,6.38f,t.z},0.28f,0.38f,10,true);
        b.cylinder_y(b.mesh.details,{t.x,0.92f,t.z},0.32f,0.24f,10,true);
        // Vertical support rails.
        for(float sx:{-1.0f,1.0f}){
            b.box(b.mesh.details,{t.x+sx*1.18f,3.35f,t.z},{0.10f,4.5f,0.12f});
        }
    }
    // Shared lower manifold and pump cluster.
    b.cylinder_x(b.mesh.details,{0,1.05f,0},8.2f,0.16f,10,true);
    b.cylinder_z(b.mesh.details,{0,1.05f,0},5.6f,0.16f,10,true);
    b.cylinder_x(b.mesh.details,{0,1.55f,2.85f},4.0f,0.34f,12,true);
    b.box(b.mesh.details,{-3.8f,1.5f,2.85f},{1.4f,1.7f,1.4f});
    b.box(b.mesh.details,{3.8f,1.5f,2.85f},{1.4f,1.7f,1.4f});
    // Overhead equalization pipe.
    b.cylinder_x(b.mesh.details,{0,6.55f,0},5.35f,0.12f,8,true);
    return std::move(b.mesh);
}
}
