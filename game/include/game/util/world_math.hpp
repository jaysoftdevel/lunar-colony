#pragma once
#include "engine/engine.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace game::world_math {
using namespace eng;

inline Vec3 rotate_y(Vec3 v,float degrees){
    const float a=radians(degrees);
    const float c=std::cos(a),s=std::sin(a);
    return {v.x*c+v.z*s,v.y,-v.x*s+v.z*c};
}
inline float wrap_degrees(float degrees){
    degrees=std::fmod(degrees,360.0f);
    if(degrees<0.0f) degrees+=360.0f;
    return degrees;
}
struct OrientedFootprint {
    Vec2 center{};
    Vec2 axis_x{1,0};
    Vec2 axis_z{0,1};
    float half_x{};
    float half_z{};
};
inline OrientedFootprint footprint(Vec3 position,float rotation_degrees,float size_x,float size_z){
    const float a=radians(rotation_degrees);
    const float c=std::cos(a),s=std::sin(a);
    return {{position.x,position.z},{c,-s},{s,c},size_x*0.5f,size_z*0.5f};
}
inline float projected_radius(const OrientedFootprint& box,Vec2 axis){
    return box.half_x*std::abs(box.axis_x.x*axis.x+box.axis_x.y*axis.y)
         + box.half_z*std::abs(box.axis_z.x*axis.x+box.axis_z.y*axis.y);
}
inline bool overlaps(const OrientedFootprint& a,const OrientedFootprint& b,float clearance){
    const Vec2 axes[4]={a.axis_x,a.axis_z,b.axis_x,b.axis_z};
    const Vec2 delta{b.center.x-a.center.x,b.center.y-a.center.y};
    for(const Vec2 axis:axes){
        const float separation=std::abs(delta.x*axis.x+delta.y*axis.y);
        if(separation>=projected_radius(a,axis)+projected_radius(b,axis)+clearance) return false;
    }
    return true;
}
inline float distance_xz(Vec3 a,Vec3 b){
    const float dx=a.x-b.x,dz=a.z-b.z;
    return std::sqrt(dx*dx+dz*dz);
}
inline float point_segment_distance_xz(Vec3 p,Vec3 a,Vec3 b){
    const float dx=b.x-a.x,dz=b.z-a.z;
    const float den=dx*dx+dz*dz;
    if(den<0.0001f) return distance_xz(p,a);
    const float t=std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/den,0.0f,1.0f);
    return distance_xz(p,{a.x+dx*t,0.0f,a.z+dz*t});
}
inline std::vector<Vec3> transformed_vertices(const std::vector<Vec3>& src,Vec3 p,float rotation_degrees){
    std::vector<Vec3> out;
    out.reserve(src.size());
    // Mesh rendering transforms thousands of vertices per frame. Computing
    // sin/cos inside rotate_y() for every single vertex dominated CPU time for
    // large construction blueprints. Evaluate the fixed instance transform once
    // per batch and apply it to every vertex.
    const float a=radians(rotation_degrees);
    const float c=std::cos(a),s=std::sin(a);
    for(const Vec3& v:src) out.push_back({v.x*c+v.z*s+p.x,v.y+p.y,-v.x*s+v.z*c+p.z});
    return out;
}
} // namespace game::world_math
