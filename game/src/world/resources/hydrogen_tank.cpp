#include "game/world/resource_builder.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_hydrogen_tank(){
    Builder b;
    // A freestanding spherical cryogenic vessel, rather than the previous two
    // 1.6 m-radius cylinders that could not geometrically justify 8 t LH2.
    // Radius 3.35 m = 157.5 m^3 gross, ~8.4 t at 75% fill and 71 kg/m^3.
    // The second tier is uniformly scaled to 0.63 in the model library.
    constexpr float radius=3.35f;
    constexpr float center_y=4.05f;
    b.mesh.footprint_x=12.4f;
    b.mesh.footprint_z=12.4f;
    b.mesh.height=9.1f;
    b.box(b.mesh.foundation,{0,0.28f,0},{11.6f,0.56f,11.6f},false);
    // Lower hemisphere: mirror the upper hemisphere and reverse winding to
    // retain outward-facing triangles for culled renderers.
    b.dome(b.mesh.body,{0,center_y,0},radius,12,32,false);
    const auto top=b.mesh.body;
    for(std::size_t i=0;i+2<top.size();i+=3){
        auto a=top[i], c=top[i+1], d=top[i+2];
        a.y=2*center_y-a.y;
        c.y=2*center_y-c.y;
        d.y=2*center_y-d.y;
        Builder::tri(b.mesh.body,a,d,c);
    }
    // Three structural saddles and four launch-site mounting points.
    for(float x:{-2.15f,2.15f}){
        b.box(b.mesh.foundation,{x,0.91f,0},{0.75f,1.12f,4.65f},false);
    }
    for(float x:{-4.75f,4.75f})for(float z:{-4.75f,4.75f})
        b.foot({x,0,z},0.52f,0.21f);
    // Equatorial insulation/thermal shield rings (laid just outside the hull).
    for(float y:{center_y-0.46f,center_y+0.46f}){
        for(int i=0;i<32;++i){
            const float a=2.0f*PI*(float)i/32.0f, n=2.0f*PI*(float)(i+1)/32.0f;
            const float rr=std::sqrt(radius*radius-(y-center_y)*(y-center_y))+0.055f;
            b.line({rr*std::cos(a),y,rr*std::sin(a)},
                   {rr*std::cos(n),y,rr*std::sin(n)});
        }
    }
    // Cryocooler / refrigeration package, pressure relief vent, vacuum jacket
    // service port and hydrogen-compatible fueling valves.
    b.box(b.mesh.body,{4.62f,1.36f,0},{2.0f,2.0f,2.1f},false);
    b.box(b.mesh.details,{5.70f,1.47f,0},{0.14f,1.45f,1.72f},false);
    for(float z:{-0.55f,0.0f,0.55f})
        b.cylinder_y(b.mesh.details,{5.82f,1.47f,z},0.24f,0.11f,10,false);
    b.cylinder_y(b.mesh.details,{-4.55f,3.05f,0},5.75f,0.12f,12,false);
    b.cylinder_y(b.mesh.details,{-4.55f,5.95f,0},0.18f,0.36f,14,false);
    b.cylinder_x(b.mesh.details,{4.62f,0.84f,-2.65f},2.35f,0.16f,12,false);
    b.cylinder_z(b.mesh.details,{3.95f,0.84f,-1.65f},1.96f,0.14f,12,false);
    for(float z:{-2.5f,-1.72f})
        b.cylinder_y(b.mesh.details,{4.8f,0.93f,z},0.38f,0.20f,12,false);
    return std::move(b.mesh);
}
} // namespace game::resources
