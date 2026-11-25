#include "game/world/resource_builder.hpp"
#include "game/world/structural_frame.hpp"
#include "game/world/resources/resource_generators.hpp"

namespace game::resources {
ResourceMesh make_solar_panel(){
    Builder b;
    b.mesh.footprint_x=11.0f; b.mesh.footprint_z=8.0f; b.mesh.height=7.0f;

    b.box(b.mesh.foundation,{0,0.30f,0},{3.4f,0.60f,2.5f});
    for(Vec3 p:std::initializer_list<Vec3>{{-2.9f,0,-2.2f},{2.9f,0,-2.2f},{-2.9f,0,2.2f},{2.9f,0,2.2f}}){
        b.foot(p,0.48f,0.18f);
        b.cylinder_between(b.mesh.details,{p.x,0.18f,p.z},{p.x*0.42f,0.82f,p.z*0.42f},0.16f,8,true,true);
    }

    b.cylinder_y(b.mesh.body,{0,1.65f,0},2.8f,0.55f,14,true);
    b.cylinder_y(b.mesh.details,{0,3.15f,0},0.45f,0.78f,14,true);
    b.box(b.mesh.details,{0.85f,1.55f,0.55f},{1.1f,1.2f,0.75f});

    const float pitch=game::runtime::radians(24.0f);
    const Vec3 ax{1,0,0};
    const Vec3 ay{0,std::cos(pitch),-std::sin(pitch)};
    const Vec3 az{0,std::sin(pitch),std::cos(pitch)};
    const auto body_begin=b.mesh.body.size(),detail_begin=b.mesh.details.size(),edge_begin=b.mesh.edges.size();
    for(int i=0;i<4;++i){
        const float x=-4.35f+2.90f*(float)i;
        const Vec3 c{x,4.55f+std::sin(pitch)*0.45f,0.25f};
        b.oriented_box(b.mesh.body,c,ax,ay,az,{2.62f,0.12f,4.35f});
        b.cylinder_between(b.mesh.details,{x,3.32f,0},{x,4.15f,-0.30f},0.10f,8,true,true);
        // The perimeter and underside cross-battens belong to the moving
        // collector range, so they track together with the photovoltaic skin.
        structural::Builder frame{b.mesh.details,b.mesh.edges,{}};
        const auto p0=c-ax*1.31f-az*2.175f-ay*.13f,p1=c+ax*1.31f-az*2.175f-ay*.13f;
        const auto p2=c+ax*1.31f+az*2.175f-ay*.13f,p3=c-ax*1.31f+az*2.175f-ay*.13f;
        for(const auto& pair:std::array<std::pair<Vec3,Vec3>,4>{{{p0,p1},{p1,p2},{p2,p3},{p3,p0}}})frame.beam(pair.first,pair.second,.10f,.13f,ay);
        for(float z:{-1.f,1.f})frame.beam(c-ax*1.31f+az*z-ay*.18f,c+ax*1.31f+az*z-ay*.18f,.09f,.14f,ay);
        // Cell grid as mesh lines only.
        for(int gx=-2;gx<=2;++gx){
            const Vec3 a=c+ax*(0.50f*(float)gx)-az*2.0f+ay*0.08f;
            const Vec3 d=c+ax*(0.50f*(float)gx)+az*2.0f+ay*0.08f;
            b.line(a,d);
        }
        for(int gz=-3;gz<=3;++gz){
            const Vec3 a=c-ax*1.20f+az*(0.55f*(float)gz)+ay*0.08f;
            const Vec3 d=c+ax*1.20f+az*(0.55f*(float)gz)+ay*0.08f;
            b.line(a,d);
        }
    }
    b.cylinder_x(b.mesh.details,{0,3.45f,0},9.2f,0.14f,10,true);
    b.mesh.solar_head.body.push_back({body_begin,b.mesh.body.size()});
    b.mesh.solar_head.details.push_back({detail_begin,b.mesh.details.size()});
    b.mesh.solar_head.edges.push_back({edge_begin,b.mesh.edges.size()});
    b.mesh.solar_head.center={0,4.55f+std::sin(pitch)*.45f,.25f};
    return std::move(b.mesh);
}
}
