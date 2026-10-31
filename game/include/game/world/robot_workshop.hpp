#pragma once
#include "game/world/resource_mesh.hpp"
#include "game/util/world_math.hpp"
#include <array>
#include <cmath>

namespace game::workshop {
inline constexpr int kBays=4;
inline constexpr float kHalf=7.0f,kDeck=0.40f,kRampEnd=-12.0f,kRampHalf=3.0f,kGateHalf=2.60f;
struct Box { eng::Vec3 center;float half_x,half_z; };
inline const std::array<Box,9> kSolids{{
    {{-6.90f,0,0},0.10f,7},{{6.90f,0,0},0.10f,7},{{0,0,6.90f},7,0.10f},
    {{-4.80f,0,-6.90f},2.20f,0.10f},{{4.80f,0,-6.90f},2.20f,0.10f},
    {{-6.10f,0,-3},0.35f,1.2f},{{-6.10f,0,3},0.35f,1.2f},
    {{6.10f,0,-3},0.35f,1.2f},{{6.10f,0,3},0.35f,1.2f}
}};
inline eng::Vec3 local(const PlacedResource& w,eng::Vec3 p){return world_math::rotate_y(p-w.position,-w.rotation_degrees);}
inline eng::Vec3 world(const PlacedResource& w,eng::Vec3 p){return w.position+world_math::rotate_y(p,w.rotation_degrees);}
inline eng::Vec3 bay_local(int bay){return {(bay%2)?4.15f:-4.15f,kDeck,(bay/2)?3.0f:-3.0f};}
inline eng::Vec3 bay(const PlacedResource& w,int bay_index){return world(w,bay_local(bay_index));}
inline bool inside(const PlacedResource& w,eng::Vec3 p){const auto q=local(w,p);return std::abs(q.x)<kHalf && std::abs(q.z)<kHalf;}
inline float floor(const PlacedResource& w,eng::Vec3 p){
    const auto q=local(w,p);
    if(std::abs(q.x)<kHalf && std::abs(q.z)<kHalf) return w.position.y+kDeck;
    if(std::abs(q.x)<=kRampHalf && q.z>=kRampEnd && q.z<=-kHalf)
        return w.position.y+kDeck*(q.z-kRampEnd)/(-kHalf-kRampEnd);
    return 0;
}
inline bool rect_hit(eng::Vec3 a,eng::Vec3 b,Box box,float margin){
    a=a-box.center;b=b-box.center;float t0=0,t1=1;
    auto clip=[&](float p,float d,float half){
        if(std::abs(d)<0.000001f) return std::abs(p)<=half;
        float u=(-half-p)/d,v=(half-p)/d;if(u>v)std::swap(u,v);
        t0=std::max(t0,u);t1=std::min(t1,v);return t0<=t1;
    };
    return clip(a.x,b.x-a.x,box.half_x+margin) && clip(a.z,b.z-a.z,box.half_z+margin);
}
inline bool clear(const PlacedResource& w,eng::Vec3 from,eng::Vec3 to,float radius){
    const auto a=local(w,from),b=local(w,to);
    for(const auto& box:kSolids) if(rect_hit(a,b,box,radius)) return false;
    if(w.workshop_gate_open<0.999f && rect_hit(a,b,{{0,0,-kHalf},kGateHalf,0.10f},radius)) return false;
    // The raised deck is entered through the ramp, never over a deck side.
    for(float sign:{-1.0f,1.0f})
        if(rect_hit(a,b,{{sign*(kRampHalf+0.10f),0,-9.50f},0.10f,2.50f},radius)) return false;
    return true;
}
inline world_math::OrientedFootprint approach(const PlacedResource& w){
    return world_math::footprint(world(w,{0,0,-12.50f}),w.rotation_degrees,11.0f,13.0f);
}
inline bool approach_hits(const PlacedResource& w,eng::Vec3 a,eng::Vec3 b,float margin){
    return w.kind==ResourceKind::RobotWorkshop && rect_hit(local(w,a),local(w,b),{{0,0,-12.50f},5.50f,6.50f},margin);
}
}
