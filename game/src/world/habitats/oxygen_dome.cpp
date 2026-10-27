#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_module.hpp"

namespace game::habitats {
void build_oxygen_dome_outer(Builder& b){
    b.mesh.footprint_x=10.5f; b.mesh.footprint_z=10.0f; b.mesh.height=7.6f;
    b.cylinder_y(b.mesh.foundation,{0,0.55f,0},1.10f,4.35f,24,true);
    b.dome(b.mesh.shell,{0,1.0f,0},4.35f,8,24,true,0.92f);
    // Geodesic/service ribs.
    for(int s=0;s<8;++s){
        const float a=2*PI*(float)s/8.0f;
        Vec3 prev{4.38f*std::cos(a),1.0f,4.38f*std::sin(a)};
        for(int r=1;r<=7;++r){
            const float t=(PI*0.5f)*(float)r/7.0f;
            Vec3 cur{4.38f*std::cos(t)*std::cos(a),1.0f+4.0f*std::sin(t),4.38f*std::cos(t)*std::sin(a)};
            b.cylinder_between(b.mesh.details,prev,cur,0.065f,6,true,false); b.line(prev,cur); prev=cur;
        }
    }
    // Oxygen processing tanks / electrolyzer bank.
    b.cylinder_y(b.mesh.details,{-4.05f,2.05f,1.05f},3.4f,0.72f,14,true);
    for(int i=0;i<3;++i) b.cylinder_y(b.mesh.details,{-3.65f+i*0.72f,1.75f,2.75f},2.5f,0.26f,10,true);
    b.box(b.mesh.details,{-2.2f,1.55f,3.45f},{1.3f,1.8f,0.55f});
    // Piping from processing bank into dome.
    b.cylinder_between(b.mesh.details,{-4.05f,3.35f,1.05f},{-2.75f,4.05f,0.65f},0.10f,8,true,true);
    b.cylinder_between(b.mesh.details,{-2.75f,4.05f,0.65f},{-1.35f,4.7f,0.35f},0.10f,8,true,true);
    // Entry module.
    b.box(b.mesh.shell,{4.55f,2.0f,0},{1.9f,3.4f,3.15f});
    b.torus_x(b.mesh.details,{5.55f,2.0f,0},0.64f,0.09f,16,6);
    b.cylinder_x(b.mesh.details,{5.60f,2.0f,0},0.12f,0.54f,14,true);
    // Roof scrubber tower and vents.
    b.cylinder_y(b.mesh.details,{0,5.45f,0},1.45f,0.72f,16,true);
    b.cylinder_y(b.mesh.details,{2.2f,4.72f,-1.25f},0.75f,0.36f,12,true);
    b.cylinder_y(b.mesh.details,{-1.8f,4.85f,-1.65f},0.65f,0.32f,12,true);
    // Base feet.
    for(float a:{0.4f,2.0f,3.55f,5.1f}){
        Vec3 top{3.7f*std::cos(a),0.75f,3.7f*std::sin(a)};
        Vec3 foot{4.55f*std::cos(a),0.16f,4.55f*std::sin(a)};
        b.cylinder_between(b.mesh.details,top,foot,0.13f,8,true,true);
        b.cylinder_y(b.mesh.details,{foot.x,0.09f,foot.z},0.18f,0.36f,10,true);
    }
    b.mesh.entrances.push_back({{5.70f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
}

} // namespace game::habitats
