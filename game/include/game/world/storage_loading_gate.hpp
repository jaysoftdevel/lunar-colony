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
inline constexpr std::array<int,4> kCargoFaces{{4,5,6,7}};

inline eng::Vec3 local_normal(int face){
    switch(face){
        case 0:return {0,0,-1};
        case 2:return {0,0,1};
        case 3:return {-1,0,0};
        // Cargo gates occupy the four sectors between the fixed cardinal crew
        // pressure entrances.  Keeping them between those doors means a user
        // never has to sacrifice a corridor entrance just to access storage.
        case 4:return {0.70710678f,0,0.70710678f};
        case 5:return {-0.70710678f,0,0.70710678f};
        case 6:return {-0.70710678f,0,-0.70710678f};
        case 7:return {0.70710678f,0,-0.70710678f};
        default:return {1,0,0};
    }
}
inline int face(const PlacedResource& item,int selected=-1){return selected<0?item.storage_loading_face:selected;}
inline std::array<int,3> faces(const PlacedResource& item,bool only_ready=false){
    return {(!only_ready || item.storage_loading_ready)?item.storage_loading_face:-1,
            (!only_ready || item.storage_extra_ready[0])?item.storage_extra_faces[0]:-1,
            (!only_ready || item.storage_extra_ready[1])?item.storage_extra_faces[1]:-1};
}
inline int gate_count(const PlacedResource& item){
    int count=0;for(int f:faces(item)) if(f>=0) ++count;return count;
}
inline bool has_ready_gate(const PlacedResource& item){
    for(int f:faces(item,true)) if(f>=0) return true;
    return false;
}
inline eng::Vec3 local_normal(const PlacedResource& item,int selected=-1){
    const int f=face(item,selected);
    if(f<8)return local_normal(f);
    float angle=item.storage_gate_angle_degrees;
    for(std::size_t i=0;i<item.storage_extra_faces.size();++i)if(item.storage_extra_faces[i]==f)angle=item.storage_extra_angles[i];
    const float a=angle*3.14159265358979323846f/180.0f;return {std::sin(a),0,std::cos(a)};
}
inline void set_angle(PlacedResource& owner,int f,float angle){
    if(owner.storage_loading_face<0 || owner.storage_loading_face==f){owner.storage_loading_face=(std::int8_t)f;owner.storage_gate_angle_degrees=angle;return;}
    for(std::size_t i=0;i<owner.storage_extra_faces.size();++i)if(owner.storage_extra_faces[i]<0 || owner.storage_extra_faces[i]==f){owner.storage_extra_faces[i]=(std::int8_t)f;owner.storage_extra_angles[i]=angle;return;}
}
inline eng::Vec3 normal(const PlacedResource& item,int selected=-1){
    return world_math::rotate_y(local_normal(item,selected),item.rotation_degrees);
}
inline float outer_radius(const PlacedResource& item,int selected=-1){
    const auto n=local_normal(item,selected);
    const float half=storage_dome_diameter_m(item.storage_size)*0.5f;
    // Extend the physical loading sleeve out to the rover-safe envelope.
    // Its pad is then immediately beside the real gate, with no route through
    // the warehouse collision footprint.
    return half/std::max(std::abs(n.x),std::abs(n.z))+(face(item,selected)>=4?0.50f:0.20f);
}
inline eng::Vec3 wall(const PlacedResource& item,int selected=-1){
    return item.position+normal(item,selected)*(storage_dome_diameter_m(item.storage_size)*(5.35f/13.0f));
}
inline eng::Vec3 mouth(const PlacedResource& item,int selected=-1){
    return item.position+normal(item,selected)*(outer_radius(item,selected)-0.20f);
}
inline eng::Vec3 service(const PlacedResource& item,int selected=-1){
    return mouth(item,selected)+normal(item,selected)*kServiceOffset;
}
// The complete approach bay is forbidden to pressure tubes, including routes
// passing a different structure. Call before an own-dock clearance exception.
inline bool reserved_face(const PlacedResource& item,eng::Vec3 point,float margin,int selected){
    if(item.kind!=ResourceKind::StorageDome) return false;
    const auto n=normal(item,selected);const eng::Vec3 side{-n.z,0,n.x};
    auto d=point-wall(item,selected);d.y=0;
    return eng::dot(d,n)>=-0.25f-margin &&
           eng::dot(point-item.position,n)<=outer_radius(item,selected)+kApronDepth+margin &&
           std::abs(eng::dot(d,side))<=kApronHalfWidth+margin;
}
inline bool at_service_face(const PlacedResource& item,eng::Vec3 point,int selected){
    return world_math::distance_xz(point,service(item,selected))<=kReach;
}
inline bool at_service(const PlacedResource& item,eng::Vec3 point){
    for(int f:faces(item,true)) if(f>=0 && at_service_face(item,point,f)) return true;
    return false;
}
inline bool body_hits_buffer_face(const PlacedResource& item,eng::Vec3 point,float radius,int selected){
    const auto n=normal(item,selected);const eng::Vec3 side{-n.z,0,n.x};
    const auto d=point-wall(item,selected);
    const float depth=eng::dot(mouth(item,selected)-wall(item,selected),n);
    return eng::dot(d,n)>=-radius && eng::dot(d,n)<=depth+0.15f+radius &&
        std::abs(eng::dot(d,side))<=kHalfWidth+0.12f+radius;
}
inline bool segment_reserved_face(const PlacedResource& item,eng::Vec3 a,eng::Vec3 b,float margin,int selected){
    if(item.kind!=ResourceKind::StorageDome) return false;
    const auto n=normal(item,selected);const eng::Vec3 side{-n.z,0,n.x};
    const auto center=wall(item,selected);const auto d=b-a;
    float t0=0,t1=1;
    auto clip=[&](float start,float delta,float lo,float hi){
        if(std::abs(delta)<0.000001f) return start>=lo && start<=hi;
        float u=(lo-start)/delta,v=(hi-start)/delta;if(u>v) std::swap(u,v);
        t0=std::max(t0,u);t1=std::min(t1,v);return t0<=t1;
    };
    return clip(eng::dot(a-center,n),eng::dot(d,n),-0.25f-margin,
                outer_radius(item,selected)+kApronDepth-eng::dot(center-item.position,n)+margin) &&
           clip(eng::dot(a-center,side),eng::dot(d,side),-kApronHalfWidth-margin,kApronHalfWidth+margin);
}
inline world_math::OrientedFootprint apron(const PlacedResource& item,int selected=-1){
    const auto n=normal(item,selected);const auto start=wall(item,selected)-n*0.25f;
    const auto end=item.position+n*(outer_radius(item,selected)+kApronDepth);
    const float rotation=std::atan2(n.x,n.z)*180.0f/3.14159265358979323846f;
    return world_math::footprint((start+end)*0.5f,rotation,2*kApronHalfWidth,eng::length(end-start));
}
inline bool reserved(const PlacedResource& item,eng::Vec3 p,float margin=0){
    if(item.kind!=ResourceKind::StorageDome) return false;
    for(int f:faces(item)) if(f>=0 && reserved_face(item,p,margin,f)) return true;
    return false;
}
inline bool segment_reserved(const PlacedResource& item,eng::Vec3 a,eng::Vec3 b,float margin){
    if(item.kind!=ResourceKind::StorageDome) return false;
    for(int f:faces(item)) if(f>=0 && segment_reserved_face(item,a,b,margin,f)) return true;
    return false;
}
inline bool body_hits_buffer(const PlacedResource& item,eng::Vec3 p,float radius){
    for(int f:faces(item,true)) if(f>=0 && body_hits_buffer_face(item,p,radius,f)) return true;
    return false;
}

}
