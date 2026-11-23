#pragma once
#include "game/world/resource_mesh.hpp"
#include "game/util/world_math.hpp"
namespace game::solar_collector {
using runtime::Vec3;
inline bool moving(std::size_t vertex,const std::vector<MeshRange>& ranges){for(const auto range:ranges)if(vertex>=range.first&&vertex<range.last)return true;return false;}
struct Pose {
    Vec3 position{},pivot{},x{},normal{},z{},authored_x{1,0,0},authored_normal{},authored_z{};
    float base_rotation{};
    Pose(const SolarHead& head,Vec3 origin,float rotation,Vec3 desired):position(origin),base_rotation(rotation){
        constexpr float pitch=24.f*3.14159265358979323846f/180.f;
        authored_z=runtime::normalize({0,std::sin(pitch),std::cos(pitch)*head.horizontal_scale});
        authored_normal=runtime::normalize(runtime::cross(authored_z,authored_x));
        normal=runtime::length(desired)>.5f?runtime::normalize(desired):world_math::rotate_y(authored_normal,rotation);
        x=runtime::normalize(runtime::cross(normal,{0,1,0}));
        if(runtime::length(x)<.001f)x=world_math::rotate_y(authored_x,rotation);
        z=runtime::normalize(runtime::cross(x,normal));
        pivot=position+world_math::rotate_y(head.pivot,rotation);
    }
    Vec3 point(Vec3 p,const SolarHead& head,bool tracked) const {
        if(!tracked)return position+world_math::rotate_y(p,base_rotation);
        const auto q=p-head.pivot;return pivot+x*runtime::dot(q,authored_x)+normal*runtime::dot(q,authored_normal)+z*runtime::dot(q,authored_z);
    }
    runtime::LunarSurfaceMapping mapping() const {return {pivot,x,normal,z,3107};}
};
inline std::vector<Vec3> vertices(const std::vector<Vec3>& source,const std::vector<MeshRange>& ranges,const SolarHead& head,const Pose& pose){
    std::vector<Vec3> out;out.reserve(source.size());
    for(std::size_t i=0;i<source.size();++i)out.push_back(pose.point(source[i],head,moving(i,ranges)));
    return out;
}
}
