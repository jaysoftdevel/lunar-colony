#pragma once
#include "game/world/habitat_builder.hpp"
namespace game::habitats::interior_detail {
inline void finish_station_interior(Builder& b,HabitatKind kind){
    auto& m=b.mesh;const float floor=m.walkable_floor_y;
    const auto& envelope=m.shell.empty()?m.glazing:m.shell;
    if(envelope.empty())return;
    Vec3 low=envelope.front(),high=low;
    for(const auto p:envelope){low={std::min(low.x,p.x),std::min(low.y,p.y),std::min(low.z,p.z)};high={std::max(high.x,p.x),std::max(high.y,p.y),std::max(high.z,p.z)};}
    // Greenhouses keep their transparent canopy; only their low service skirts
    // receive an opaque pressure liner.
    if(m.shell.empty())for(float sign:{-1.f,1.f})
        b.box(m.lining,{0,floor+.22f,sign*high.z*.95f},{(high.x-low.x)*.94f,.42f,.06f},false);
    const float center_y=(low.y+high.y)*.5f;
    // The inner pressure liner follows the actual hull rather than an unrelated box.
    if(!m.shell.empty())m.lining=m.shell;
    for(auto& p:m.lining){p.x*=.96f;p.z*=.96f;p.y=center_y+(p.y-center_y)*.96f;}
    for(std::size_t i=0;i+2<m.interior.size();i+=3){
        if(std::abs(m.interior[i].y-floor)>.015f || std::abs(m.interior[i+1].y-floor)>.015f || std::abs(m.interior[i+2].y-floor)>.015f)continue;
        for(std::size_t j=i;j<i+3;++j)m.deck.push_back(m.interior[j]+Vec3{0,.009f,0});
    }
    if(m.deck.empty())b.cylinder_y(m.deck,{0,floor,0},.018f,std::min(high.x,high.z)*.88f,40,false);
    const float hx=std::min(std::abs(low.x),std::abs(high.x))*.74f;
    const float side=std::min(std::abs(low.z),std::abs(high.z))*.77f;
    const float ceiling=std::min(floor+2.48f,high.y-.52f);
    // Separate electrical/data and air/water runs, with accessible ceiling hangers.
    for(float sign:{-1.f,1.f}){
        for(int cable=0;cable<3;++cable)
            b.cylinder_between(m.service_metal,{-hx,ceiling-.13f*cable,sign*side},{hx,ceiling-.13f*cable,sign*side},cable==0?.052f:.030f,8,true,false);
        for(int i=0;i<5;++i){
            const float x=std::lerp(-hx,hx,float(i)/4);
            b.box(m.service_metal,{x,ceiling-.14f,sign*side},{.055f,.40f,.19f},false);
        }
        // Wall service/access panels stay outside the central crew circulation lane.
        for(int i=0;i<3;++i){
            const float x=std::lerp(-hx*.72f,hx*.72f,float(i)/2);
            b.box(m.service_metal,{x,floor+1.34f,sign*(side+.04f)},{.72f,.58f,.065f},false);
            b.box(m.displays,{x,floor+1.40f,sign*(side-.002f)},{.58f,.30f,.018f},false);
            for(int vent=0;vent<4;++vent)b.box(m.service_metal,{x,floor+1.12f+vent*.035f,sign*(side-.008f)},{.43f,.012f,.026f},false);
        }
        b.box(m.lamps,{0,ceiling+.06f,sign*side*.60f},{hx*1.45f,.026f,.075f},false);
    }
    // Wayfinding is physical paint on the deck, not a collision obstacle.
    for(float sign:{-1.f,1.f})b.box(m.markings,{0,floor+.019f,sign*.62f},{std::max(1.f,hx*1.65f),.006f,.035f},false);
    if(kind==HabitatKind::AirLock){
        for(float x:{-1.6f,1.6f})for(int i=0;i<7;++i)
            b.box(m.markings,{x+(i%2)*.045f,floor+.024f,-.60f+i*.20f},{.11f,.008f,.095f},false);
    }
}
}
