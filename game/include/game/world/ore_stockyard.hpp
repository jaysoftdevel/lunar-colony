#pragma once
#include "game/world/utility_network.hpp"
#include "game/world/geometry_standards.hpp"
#include "game/util/world_math.hpp"
#include <functional>
#include <optional>

namespace game::ore {
struct Stockyard {
    game::runtime::Vec3 center{},side{},outward{};
    game::runtime::Vec3 box(std::size_t kind,int index) const {
        return center+side*((float(kind)-1.5f)*.5f)+outward*((index%3-1)*.46f)+
            game::runtime::Vec3{0,.035f+(index/3)*.32f,0};
    }
    game::runtime::Vec3 pickup(std::size_t kind,float offset=0) const {
        return center+side*((float(kind)-1.5f)*.5f+offset)+outward*1.76f;
    }
};
// Planned links reserve exactly the same space as installed ones. A full pad is
// moved as a unit, so no crate row can fall into a pipe/conveyor blueprint.
inline std::optional<Stockyard> find_stockyard(game::runtime::Vec3 origin,float rotation,float width,float depth,
    const std::vector<UtilityConnection>& links,const std::function<bool(const world_math::OrientedFootprint&)>& available={}) {
    using namespace game::runtime;using namespace world_math;
    for(int ring=0;ring<3;++ring)for(float turn:{0.f,90.f,-90.f,180.f}){
        const float yaw=rotation+turn;
        const Vec3 side=rotate_y({1,0,0},yaw),out=rotate_y({0,0,1},yaw);
        const float extent=std::abs(turn)==90.f?width*.5f:depth*.5f;
        Stockyard pad{origin+out*(extent+1.02f+ring*1.65f),side,out};
        const auto area=footprint(pad.center,yaw,2.12f,1.64f);
        if(available && !available(area))continue;
        bool clear=true;
        for(const auto& link:links)for(std::size_t i=1;i<link.path.size();++i){
            const auto delta=link.path[i]-link.path[i-1];const float length=std::hypot(delta.x,delta.z);
            const float half=link.kind==UtilityKind::ConveyorBelt?belt_half_width:geometry::kSupplyJunctionWidth*.5f;
            const auto tray=footprint((link.path[i]+link.path[i-1])*.5f,std::atan2(delta.x,delta.z)*57.29578f,2*half,length+2*half);
            if(overlaps(area,tray,.12f)){clear=false;break;}
        }
        if(clear)return pad;
    }
    return std::nullopt; // Retain stock in the driller's buffer until an outlet is clear.
}
} // namespace game::ore
