#pragma once
#include "game/world/resource_mesh.hpp"
#include "game/util/world_math.hpp"
#include <array>
#include <cmath>

namespace game::workshop {
inline constexpr int kBays=4;
// Compact four-rover workshop.  The 11 x 10 m hall still leaves more than a
// rover diameter around every bay, a full central aisle and a 4.5 m gate.
inline constexpr float kHalfX=5.50f,kHalfZ=5.00f,kDeck=0.40f;
inline constexpr float kRampEnd=-9.80f,kRampHalf=2.60f,kGateHalf=2.25f;
inline constexpr float kApproachHalfWidth=4.50f,kApproachFar=-15.0f;
struct Box { eng::Vec3 center;float half_x,half_z; };
inline const std::array<Box,9> kSolids{{
    {{-5.40f,0,0},0.10f,5.0f},{{5.40f,0,0},0.10f,5.0f},{{0,0,4.90f},5.5f,0.10f},
    {{-3.88f,0,-4.90f},1.62f,0.10f},{{3.88f,0,-4.90f},1.62f,0.10f},
    {{-4.72f,0,-2.1f},0.30f,1.0f},{{-4.72f,0,2.1f},0.30f,1.0f},
    {{4.72f,0,-2.1f},0.30f,1.0f},{{4.72f,0,2.1f},0.30f,1.0f}
}};
inline eng::Vec3 local(const PlacedResource& w,eng::Vec3 p){return world_math::rotate_y(p-w.position,-w.rotation_degrees);}
inline eng::Vec3 world(const PlacedResource& w,eng::Vec3 p){return w.position+world_math::rotate_y(p,w.rotation_degrees);}
inline eng::Vec3 bay_local(int bay){return {(bay%2)?3.25f:-3.25f,kDeck,(bay/2)?2.05f:-2.05f};}
inline eng::Vec3 bay(const PlacedResource& w,int bay_index){return world(w,bay_local(bay_index));}
inline bool inside(const PlacedResource& w,eng::Vec3 p){const auto q=local(w,p);return std::abs(q.x)<kHalfX && std::abs(q.z)<kHalfZ;}
inline float floor(const PlacedResource& w,eng::Vec3 p){
    const auto q=local(w,p);
    if(std::abs(q.x)<kHalfX && std::abs(q.z)<kHalfZ) return w.position.y+kDeck;
    if(std::abs(q.x)<=kRampHalf && q.z>=kRampEnd && q.z<=-kHalfZ)
        return w.position.y+kDeck*(q.z-kRampEnd)/(-kHalfZ-kRampEnd);
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
    if(w.workshop_gate_open<0.999f && rect_hit(a,b,{{0,0,-kHalfZ},kGateHalf,0.10f},radius)) return false;
    // The raised deck is entered through the ramp, never over a deck side.
    for(float sign:{-1.0f,1.0f})
        if(rect_hit(a,b,{{sign*(kRampHalf+0.10f),0,(kRampEnd-kHalfZ)*0.5f},0.10f,(-kHalfZ-kRampEnd)*0.5f},radius)) return false;
    return true;
}
inline world_math::OrientedFootprint approach(const PlacedResource& w){
    const float near=-kHalfZ;
    return world_math::footprint(world(w,{0,0,(kApproachFar+near)*0.5f}),w.rotation_degrees,
                                 2.0f*kApproachHalfWidth,near-kApproachFar);
}
inline bool approach_hits(const PlacedResource& w,eng::Vec3 a,eng::Vec3 b,float margin){
    const float near=-kHalfZ;
    return w.kind==ResourceKind::RobotWorkshop &&
        rect_hit(local(w,a),local(w,b),{{0,0,(kApproachFar+near)*0.5f},kApproachHalfWidth,(near-kApproachFar)*0.5f},margin);
}
}
