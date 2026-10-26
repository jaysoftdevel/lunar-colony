#include "game/world/habitat_builder.hpp"
#include "game/world/habitats/habitat_generators.hpp"

namespace game::habitats {
HabitatMesh make_cantine_dome(){
    Builder b;
    b.mesh.footprint_x=11.0f; b.mesh.footprint_z=10.5f; b.mesh.height=7.0f;
    b.cylinder_y(b.mesh.foundation,{0,0.60f,0},1.20f,4.55f,24,true);
    // Transparent-looking window dome plus substantial structural framing.
    b.dome(b.mesh.glazing,{0,1.05f,0},4.55f,7,24,false,0.88f);
    for(int s=0;s<10;++s){
        const float a=2*PI*(float)s/10.0f;
        Vec3 prev{4.56f*std::cos(a),1.05f,4.56f*std::sin(a)};
        for(int r=1;r<=9;++r){
            const float t=(PI*0.5f)*(float)r/9.0f;
            Vec3 cur{4.56f*std::cos(t)*std::cos(a),1.05f+4.0f*std::sin(t),4.56f*std::cos(t)*std::sin(a)};
            b.cylinder_between(b.mesh.details,prev,cur,0.095f,6,true,false); b.line(prev,cur); prev=cur;
        }
    }
    // Two horizontal window belt ribs around the communal dome.
    for(float t:{0.22f,0.48f}){
        const float theta=(PI*0.5f)*t;
        const float rr=4.57f*std::cos(theta);
        const float yy=1.05f+4.0f*std::sin(theta);
        for(int s=0;s<24;++s){
            const float a0=2*PI*s/24.0f,a1=2*PI*(s+1)/24.0f;
            Vec3 p0{rr*std::cos(a0),yy,rr*std::sin(a0)},p1{rr*std::cos(a1),yy,rr*std::sin(a1)};
            b.cylinder_between(b.mesh.details,p0,p1,0.075f,6,true,false); b.line(p0,p1);
        }
    }
    // Interior communal tables/core visible through the dome.
    b.cylinder_y(b.mesh.interior,{0,2.15f,0},2.2f,0.30f,12,false);
    b.cylinder_y(b.mesh.interior,{0,1.30f,0},0.22f,2.0f,18,false);
    b.cylinder_y(b.mesh.interior,{0,2.95f,0},0.18f,1.65f,18,false);
    for(float a:{0.0f,1.5708f,3.1416f,4.7124f}){
        const Vec3 c{2.35f*std::cos(a),1.28f,2.35f*std::sin(a)};
        b.cylinder_y(b.mesh.interior,c,0.16f,0.82f,14,false);
        b.cylinder_y(b.mesh.interior,{c.x,0.72f,c.z},0.95f,0.13f,8,false);
    }
    // Entry connector.
    b.box(b.mesh.shell,{4.85f,2.0f,0},{2.0f,3.35f,3.15f});
    b.torus_x(b.mesh.details,{5.9f,2.0f,0},0.62f,0.09f,16,6);
    b.cylinder_x(b.mesh.details,{5.95f,2.0f,0},0.12f,0.52f,14,true);
    b.cylinder_y(b.mesh.details,{0,5.50f,0},0.8f,0.55f,14,true);
    // Landing supports.
    for(float a:{0.45f,2.1f,3.65f,5.25f}){
        Vec3 top{3.95f*std::cos(a),0.75f,3.95f*std::sin(a)};
        Vec3 foot{4.8f*std::cos(a),0.15f,4.8f*std::sin(a)};
        b.cylinder_between(b.mesh.details,top,foot,0.13f,8,true,true);
        b.cylinder_y(b.mesh.details,{foot.x,0.09f,foot.z},0.18f,0.34f,10,true);
    }
    b.mesh.entrances.push_back({{6.00f,1.18f,0.0f},{1.0f,0.0f,0.0f}});
    return std::move(b.mesh);
}

} // namespace game::habitats
