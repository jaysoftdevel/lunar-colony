#pragma once
#include "game/world/resource_mesh.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <cmath>

namespace game::storage_loading {
inline constexpr float kHalfWidth=1.20f;
inline constexpr float kHeight=1.65f;
inline constexpr float kServiceOffset=1.55f;
inline constexpr float kReach=0.55f;
inline constexpr float kApronDepth=4.40f;
inline constexpr float kApronHalfWidth=2.15f;

inline eng::Vec3 local_normal(int face){
    switch(face){
        case 0:return {0,0,-1};
        case 2:return {0,0,1};
        case 3:return {-1,0,0};
        case 4:return {0.70710678f,0,0.70710678f};
        default:return {1,0,0};
    }
}
inline eng::Vec3 normal(const PlacedResource& item){
    return world_math::rotate_y(local_normal(item.storage_loading_face),item.rotation_degrees);
}
inline float outer_radius(const PlacedResource& item){
    const auto n=local_normal(item.storage_loading_face);
    const float half=storage_dome_diameter_m(item.storage_size)*0.5f;
    return (half-(item.storage_loading_face==4?1.10f:0.0f))/std::max(std::abs(n.x),std::abs(n.z));
}
inline eng::Vec3 wall(const PlacedResource& item){
    return item.position+normal(item)*(storage_dome_diameter_m(item.storage_size)*(5.35f/13.0f));
}
inline eng::Vec3 mouth(const PlacedResource& item){
    return item.position+normal(item)*(outer_radius(item)-0.20f);
}
inline eng::Vec3 service(const PlacedResource& item){
    const float radius=item.storage_loading_face==4?
        storage_dome_diameter_m(item.storage_size)*0.5f/0.70710678f+1.85f:
        outer_radius(item)+kServiceOffset;
    return item.position+normal(item)*radius;
}
// The complete approach bay is forbidden to pressure tubes, including routes
// passing a different structure. Call before an own-dock clearance exception.
inline bool reserved(const PlacedResource& item,eng::Vec3 point,float margin=0.0f){
    if(item.kind!=ResourceKind::StorageDome) return false;
    const auto n=normal(item);const eng::Vec3 side{-n.z,0,n.x};
    auto d=point-wall(item);d.y=0;
    return eng::dot(d,n)>=-0.25f-margin &&
           eng::dot(point-item.position,n)<=outer_radius(item)+kApronDepth+margin &&
           std::abs(eng::dot(d,side))<=kApronHalfWidth+margin;
}
inline bool at_service(const PlacedResource& item,eng::Vec3 point){
    return world_math::distance_xz(point,service(item))<=kReach;
}
inline bool body_hits_buffer(const PlacedResource& item,eng::Vec3 point,float radius){
    const auto n=normal(item);const eng::Vec3 side{-n.z,0,n.x};
    const auto d=point-wall(item);
    const float depth=eng::dot(mouth(item)-wall(item),n);
    return eng::dot(d,n)>=-radius && eng::dot(d,n)<=depth+0.15f+radius &&
        std::abs(eng::dot(d,side))<=kHalfWidth+0.12f+radius;
}
inline bool segment_reserved(const PlacedResource& item,eng::Vec3 a,eng::Vec3 b,float margin){
    if(item.kind!=ResourceKind::StorageDome) return false;
    const auto n=normal(item);const eng::Vec3 side{-n.z,0,n.x};
    const auto center=wall(item);const auto d=b-a;
    float t0=0,t1=1;
    auto clip=[&](float start,float delta,float lo,float hi){
        if(std::abs(delta)<0.000001f) return start>=lo && start<=hi;
        float u=(lo-start)/delta,v=(hi-start)/delta;if(u>v) std::swap(u,v);
        t0=std::max(t0,u);t1=std::min(t1,v);return t0<=t1;
    };
    return clip(eng::dot(a-center,n),eng::dot(d,n),-0.25f-margin,
                outer_radius(item)+kApronDepth-eng::dot(center-item.position,n)+margin) &&
           clip(eng::dot(a-center,side),eng::dot(d,side),-kApronHalfWidth-margin,kApronHalfWidth+margin);
}
inline world_math::OrientedFootprint apron(const PlacedResource& item){
    const auto n=normal(item);const auto start=wall(item)-n*0.25f;
    const auto end=item.position+n*(outer_radius(item)+kApronDepth);
    const float rotation=std::atan2(n.x,n.z)*180.0f/3.14159265358979323846f;
    return world_math::footprint((start+end)*0.5f,rotation,2*kApronHalfWidth,eng::length(end-start));
}
}
