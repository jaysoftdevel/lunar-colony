#include "game/game_app.hpp"
#include "game/util/world_math.hpp"
#include "game/world/resource_builder.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
namespace game {
namespace {
using eng::Vec3;using eng::Color;using resources::Builder;
std::vector<Vec3> pose(const std::vector<Vec3>& src,Vec3 position,float rotation,float scale=1){std::vector<Vec3> out;out.reserve(src.size());for(auto p:src)out.push_back(world_math::rotate_y(p*scale,rotation)+position);return out;}
void draw(eng::Renderer& r,const Builder& b,Vec3 p,bool solid,float yaw=0,float s=1,Color color={.42f,.58f,.65f,1},bool wire=false){
    if(solid){
        const eng::LunarSurfaceMapping mapping{p,world_math::rotate_y({1,0,0},yaw),{0,1,0},
            world_math::rotate_y({0,0,1},yaw)};
        r.draw_mapped_lunar_surface(pose(b.mesh.foundation,p,yaw,s),eng::LunarMaterial::Alloy,{.44f,.49f,.52f,1},mapping);
        r.draw_mapped_lunar_surface(pose(b.mesh.body,p,yaw,s),eng::LunarMaterial::Paint,color,mapping);
        r.draw_mapped_lunar_surface(pose(b.mesh.details,p,yaw,s),eng::LunarMaterial::Alloy,{.79f,.58f,.27f,1},mapping);
        return;
    }
    if(wire){r.draw_lines_3d(pose(b.mesh.edges,p,yaw,s),color,1);return;}
    r.draw_triangles_3d(pose(b.mesh.foundation,p,yaw,s),{.16f,.20f,.23f,1});
    r.draw_triangles_3d(pose(b.mesh.body,p,yaw,s),color);
    r.draw_triangles_3d(pose(b.mesh.details,p,yaw,s),{.75f,.56f,.22f,1});
    r.draw_lines_3d(pose(b.mesh.edges,p,yaw,s),{.09f,.13f,.16f,1},1);
}
Color package_color(TradeGood good){
    const auto index=(std::size_t)good;
    if(index>=kLegacyTradeGoodCount && index<kTradeGoodCount)return ore::colors[index-kLegacyTradeGoodCount];
    if(good==TradeGood::Oxygen)return {.70f,.80f,.92f,1};
    if(good==TradeGood::Hydrogen)return {.30f,.62f,.86f,1};
    return {.72f,.45f,.20f,1};
}
void package(Builder& b,TradeGood good,Vec3 p,float scale=1){
    if(good==TradeGood::Oxygen||good==TradeGood::Hydrogen){
        for(float x:{-.13f,.13f}){b.cylinder_y(b.mesh.body,p+Vec3{x,.28f*scale,0},.52f*scale,.105f*scale,10);b.box(b.mesh.details,p+Vec3{x,.57f*scale,0},{.07f,.08f,.07f});}
    }else{b.box(b.mesh.body,p+Vec3{0,.23f*scale,0},{.50f*scale,.46f*scale,.50f*scale});for(float x:{-.18f,.18f})b.box(b.mesh.details,p+Vec3{x,.23f*scale,0},{.04f,.49f*scale,.54f*scale});}
}
}
void GameApp::render_trader(eng::Renderer& r){
    const bool solid=!mesh_layout_active();
    // Reserved source crates and rover-deck cargo are views of the authoritative
    // lot/haul records, not extra resource objects that can be picked up twice.
    auto pickup=trader_.dock;
    if(trader_.phase==TradePhase::Loading){
        for(std::size_t i=0;i<trader_.lots.size();++i){
            const auto& l=trader_.lots[i];if(l.remaining<=0)continue;
            if(l.conveyor_sequence){
                if(l.conveyor_distance_m<=0.f)continue;
                const auto& connections=utility_network_.connections();
                const auto belt=std::find_if(connections.begin(),connections.end(),[&](const auto& link){
                    return link.kind==UtilityKind::ConveyorBelt && link.placement_sequence==l.conveyor_sequence;
                });
                if(belt==connections.end())continue;
                const auto position=belt->cargo_position(l.conveyor_distance_m);
                Builder crate;
                package(crate,l.good,{});
                draw(r,crate,position,solid,0,1,package_color(l.good));
                if(world_math::distance_xz(position,belt->endpoint_b.position)<2.f)pickup=belt->endpoint_b.position;
                continue;
            }
            // Several materials can share one source packing bay. Draw its
            // aggregate package once, avoiding overlapping duplicate meshes.
            bool drawn=false;for(std::size_t j=0;j<i;++j)if(trader_.lots[j].remaining>0&&!trader_.lots[j].conveyor_sequence&&eng::length(trader_.lots[j].position-l.position)<.1f)drawn=true;
            if(drawn)continue;
            Builder crate;package(crate,l.good,{});draw(r,crate,l.position,solid,0,1,package_color(l.good));
        }
        for(const auto& h:trader_.hauls)if(h.cargo>0&&h.robot<builder_robots_.size()){
            const auto& robot=builder_robots_[h.robot];Builder cargo;package(cargo,trader_.lots[h.lot].good,{0,.82f,.12f});draw(r,cargo,robot.visual_initialized?robot.visual_position:robot.position,solid,robot.heading_degrees);
        }
    }
    if(!trader_.active())return;
    const auto pi=trader_pad_index();if(pi>=placed_resources_.size())return;
    const auto& pad=placed_resources_[pi];const auto& pm=resource_models_.mesh(pad);const float size=pm.footprint_x/20;
    float height=0,x=0;
    if(trader_.phase==TradePhase::Approach){float u=std::clamp(trader_.elapsed/24,0.0f,1.0f);height=75*(1-u)*(1-u);x=-26*(1-u)*(1-u);}
    if(trader_.phase==TradePhase::Departing){float u=std::clamp(trader_.elapsed/18,0.0f,1.0f);height=100*u*u;x=38*u*u*u;}
    const Vec3 origin=pad.position+Vec3{x,height+.73f*size,0};
    Builder ship;
    ship.box(ship.mesh.foundation,{0,1.2f,0},{11.7f,.6f,6.5f});
    ship.box(ship.mesh.body,{-4.4f,2.8f,0},{3.2f,2.6f,4.9f});
    ship.box(ship.mesh.body,{-5.6f,2.35f,0},{1.4f,1.5f,4.2f});
    ship.box(ship.mesh.foundation,{-5.9f,3.15f,0},{.1f,.65f,3.0f});
    ship.box(ship.mesh.body,{-1.3f,4.4f,0},{3.6f,.3f,6.3f});
    ship.box(ship.mesh.body,{-1.3f,2.65f,3.0f},{3.6f,3.2f,.2f});
    for(float x:{-3.0f,.4f})for(float z:{-3.0f,3.0f})ship.cylinder_between(ship.mesh.body,{x,1.3f,z},{x,4.4f,z},.14f,8);
    for(float x:{1.8f,4.3f}){
        ship.cylinder_x(ship.mesh.details,{x,3.0f,0},2.15f,1.8f,24);
        for(float dx:{-.95f,.95f})ship.cylinder_x(ship.mesh.foundation,{x+dx,3.0f,0},.12f,1.84f,24);
        ship.box(ship.mesh.body,{x,3.0f,-1.82f},{1.2f,.7f,.08f});
    }
    for(float x:{-4.5f,0.0f,4.5f})for(float z:{-3.25f,3.25f}){
        ship.cylinder_between(ship.mesh.body,{x,1.4f,z*.75f},{x,.1f,z},.18f,10);
        ship.cylinder_y(ship.mesh.foundation,{x,.1f,z},.2f,.46f,12);
    }
    for(float z:{-1.8f,1.8f})ship.cylinder_x(ship.mesh.foundation,{6.0f,2.5f,z},1.5f,.72f,18);
    ship.cylinder_between(ship.mesh.body,{-4.0f,4.1f,0},{-4.0f,5.4f,0},.055f,6);
    const auto loaded=std::accumulate(trader_.loaded.begin(),trader_.loaded.end(),std::int64_t{0});
    const int crates=loaded>0?std::clamp((int)((loaded+999)/1000),1,12):0;
    for(int i=0;i<crates;++i)ship.box(ship.mesh.details,{-2.25f+(float)(i%2)*1.5f,1.9f+.62f*(float)(i/6),-1.7f+(float)((i/2)%3)*1.5f},{1.22f,.56f,1.16f});
    draw(r,ship,origin,solid,pad.rotation_degrees,size,{.77f,.79f,.78f,1},true);
    if(height<.1f){
        Builder crane;const Vec3 pivot=pad.position+Vec3{0,4.7f*size,0},head=pickup+Vec3{0,3.4f,0};
        crane.cylinder_between(crane.mesh.details,pivot,head,.10f,8);
        crane.cylinder_between(crane.mesh.foundation,head,pickup+Vec3{0,1.1f,0},.025f,6);
        draw(r,crane,{},solid,0,1,{.72f,.75f,.78f,1},true);
        const auto q=trader_.dock;const float half=1.2f;
        r.draw_lines_3d({{q.x-half,.035f,q.z-half},{q.x+half,.035f,q.z-half},{q.x+half,.035f,q.z-half},{q.x+half,.035f,q.z+half},{q.x+half,.035f,q.z+half},{q.x-half,.035f,q.z+half},{q.x-half,.035f,q.z+half},{q.x-half,.035f,q.z-half}},{.25f,.85f,.72f,1},2);
    }else{
        if(r.draw_lunar_volume({origin+Vec3{0,-0.1f,0},{0,-1,0},3.9f*size,1.22f*size,
            {0.25f,0.52f,0.96f,1},0.76f,3.3f,true})) return;
        Builder plume;plume.cylinder_y(plume.mesh.body,{0,-1.6f,0},4.0f,.6f,16,true);
        if(r.lunar_scene_active())
            r.draw_lunar_surface(pose(plume.mesh.body,origin,pad.rotation_degrees,size),eng::LunarMaterial::Emissive,{.32f,.67f,1.0f,.48f});
        else r.draw_lines_3d(pose(plume.mesh.edges,origin,pad.rotation_degrees,size),{.35f,.65f,.95f,.55f},1);
    }
}
} // namespace game
