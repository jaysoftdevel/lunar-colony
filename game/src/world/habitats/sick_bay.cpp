#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_module.hpp"

namespace game::habitats {
void build_sick_bay_outer(Builder& b){
    b.mesh.footprint_x=13.0f; b.mesh.footprint_z=11.0f; b.mesh.height=8.2f;
    b.cylinder_y(b.mesh.foundation,{0,0.55f,0},1.1f,4.55f,24,true);

    // Low medical dome with broad observation glazing band.
    b.dome(b.mesh.shell,{0,1.0f,0},4.55f,8,24,true,0.78f);
    const float window_y=3.65f,window_base_y=2.45f;
    const auto skin_radius=[](float y){
        const float rise=(y-1.0f)/0.78f;
        return std::sqrt(4.55f*4.55f-rise*rise)+.04f;
    };
    const float top_radius=skin_radius(window_y),base_radius=skin_radius(window_base_y);
    for(int s=0;s<12;++s){
        const float a0=2*PI*(float)s/12.0f;
        const float a1=2*PI*(float)(s+1)/12.0f;
        Vec3 p0{top_radius*std::cos(a0),window_y,top_radius*std::sin(a0)};
        Vec3 p1{top_radius*std::cos(a1),window_y,top_radius*std::sin(a1)};
        b.cylinder_between(b.mesh.details,p0,p1,0.09f,6,true,false);
        b.line(p0,p1);
        Vec3 low0{base_radius*std::cos(a0),window_base_y,base_radius*std::sin(a0)};
        b.cylinder_between(b.mesh.details,low0,p0,0.075f,6,true,false);
        b.line(low0,p0);
        Builder::tri(b.mesh.glazing,low0,p0,p1);
        Vec3 low1{base_radius*std::cos(a1),window_base_y,base_radius*std::sin(a1)};
        Builder::tri(b.mesh.glazing,low0,p1,low1);
    }

    // Exterior medical support pods / life support tanks.
    for(float z:{-2.6f,-0.9f,0.9f,2.6f}) b.cylinder_y(b.mesh.details,{-4.85f,2.35f,z},2.8f,0.46f,12,true);
    b.box(b.mesh.details,{-4.2f,1.55f,-3.65f},{1.35f,1.35f,0.55f});
    b.cylinder_y(b.mesh.details,{0,6.1f,0},1.35f,0.82f,16,true);
    for(Vec3 c:std::initializer_list<Vec3>{{-2.5f,5.25f,-1.8f},{2.6f,5.0f,1.9f}}) b.cylinder_y(b.mesh.details,c,0.95f,0.35f,12,true);

    for(float a:{0.5f,2.05f,3.65f,5.2f}){
        Vec3 top{3.85f*std::cos(a),0.8f,3.85f*std::sin(a)};
        Vec3 foot{4.7f*std::cos(a),0.15f,4.7f*std::sin(a)};
        b.cylinder_between(b.mesh.details,top,foot,0.14f,8,true,true);
        b.cylinder_y(b.mesh.details,{foot.x,0.09f,foot.z},0.18f,0.36f,10,true);
    }
}
}
