#pragma once
#include "game/world/corridor_network.hpp"
#include "game/world/geometry_standards.hpp"
#include <cmath>

namespace game {
struct PressureDoorState {
    CorridorDoor door{};
    float open_fraction{0};
    float hold_seconds{0};
    bool seen{false};
};
namespace pressure_door {
inline constexpr float kSealOverlap=0.06f;
inline constexpr float kPanelHalfWidth=geometry::kDoorClearWidth*0.25f+kSealOverlap;
inline constexpr float kSlide=geometry::kDoorClearWidth*0.5f+2*kSealOverlap;
inline constexpr float kThickness=0.12f;
inline float clear_half_width(float fraction){
    return std::max(0.0f,kSlide*std::clamp(fraction,0.0f,1.0f)-kSealOverlap);
}
inline bool sealed(float fraction){return fraction<=0.0001f;}
// A closing door reopens before a body reaches the swept leaf envelope.
inline void advance(PressureDoorState& state,float dt,bool requested,bool occupied){
    dt=std::clamp(dt,0.0f,0.10f);
    if(requested || occupied) state.hold_seconds=0.85f;
    else state.hold_seconds=std::max(0.0f,state.hold_seconds-dt);
    const bool open=requested || occupied || state.hold_seconds>0;
    const float delta=(open?2.8f:-1.4f)*dt;
    state.open_fraction=std::clamp(state.open_fraction+delta,0.0f,1.0f);
}
// Continuous body-vs-leaf slab test. Includes a body already straddling the
// plane and long steps whose endpoints both lie beyond the panel thickness.
inline bool blocks(const CorridorDoor& door,float fraction,eng::Vec3 from,eng::Vec3 to,float radius){
    auto n=door.normal;n.y=0;n=eng::normalize(n);
    const eng::Vec3 side{-n.z,0,n.x};
    const auto a=from-door.wall_position,b=to-door.wall_position;
    const float da=eng::dot(a,n),db=eng::dot(b,n);
    const float slab=radius+kThickness*0.5f;
    if(std::min(da,db)>slab || std::max(da,db)<-slab) return false;
    const float dv=db-da;
    float t0=0,t1=1;
    if(std::abs(dv)>0.00001f){
        t0=std::max(0.0f,std::min((-slab-da)/dv,(slab-da)/dv));
        t1=std::min(1.0f,std::max((-slab-da)/dv,(slab-da)/dv));
    }
    const float u0=eng::dot(a+(b-a)*t0,side),u1=eng::dot(a+(b-a)*t1,side);
    const float aperture=geometry::kDoorClearWidth*0.5f;
    if(std::min(u0,u1)>aperture+radius || std::max(u0,u1)<-aperture-radius) return false;
    const float clear=clear_half_width(fraction);
    return std::max(std::abs(u0),std::abs(u1))+radius+0.015f>clear;
}
}
}
