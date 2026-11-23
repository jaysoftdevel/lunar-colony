#pragma once
#include "game/world/habitat_mesh.hpp"
#include "game/world/interior_station.hpp"
#include <optional>

namespace game::habitats {
// Intersect the real skin at each bracket, including capsule/dome curvature.
inline std::optional<game::runtime::Vec3> screen_wall(const HabitatMesh& mesh,float x,float y,float sign){
    using namespace game::runtime;
    const auto& skin=mesh.shell.empty()?mesh.glazing:mesh.shell;
    const Vec3 origin{x,y,0},ray{0,0,sign};float nearest=1e6f;
    for(std::size_t i=0;i+2<skin.size();i+=3){
        const auto a=skin[i],e1=skin[i+1]-a,e2=skin[i+2]-a,p=cross(ray,e2);const float det=dot(e1,p);
        if(std::abs(det)<1e-7f)continue;
        const auto t=origin-a;const float u=dot(t,p)/det;if(u<0||u>1)continue;
        const auto q=cross(t,e1);const float v=dot(ray,q)/det;if(v<0||u+v>1)continue;
        const float distance=dot(e2,q)/det;if(distance>.01f)nearest=std::min(nearest,distance);
    }
    if(nearest>1e5f)return std::nullopt;
    return origin+ray*nearest;
}
inline void finish_screen_mounts(HabitatMesh& mesh){
    using namespace game::runtime;
    const auto& skin=mesh.shell.empty()?mesh.glazing:mesh.shell;
    if(skin.empty())return;
    float min_x=skin.front().x,max_x=min_x;
    for(auto p:skin){min_x=std::min(min_x,p.x);max_x=std::max(max_x,p.x);}
    for(float sign:{-1.f,1.f})for(float x:{std::lerp(min_x,max_x,.26f),std::lerp(min_x,max_x,.74f)}){
        constexpr float w=.90f,h=.48f;
        const float y=mesh.walkable_floor_y+1.72f;
        std::array<Vec3,4> anchors;float inset=1e5f;int index=0;bool valid=true;
        for(float dx:{-w*.37f,w*.37f})for(float dy:{-h*.35f,h*.35f}){
            const auto wall=screen_wall(mesh,x+dx,y+dy,sign);if(!wall){valid=false;break;}
            anchors[index++]=*wall;inset=std::min(inset,wall->z*sign);
        }
        if(!valid)continue;
        const Vec3 normal{0,0,-sign},right{-sign,0,0};
        structure_display::Mount screen{{x,y,sign*(inset-.14f)},right,{0,1,0},w,h};
        interior_layout::StationScope station(mesh,StationKind::Screen);
        structure_display::casing(mesh.service_metal,mesh.displays,screen);mesh.screens.push_back(screen);
        for(auto anchor:anchors){
            const Vec3 rear{anchor.x,anchor.y,screen.center.z+sign*.08f};
            const float depth=std::abs(anchor.z-rear.z);
            structure_display::box(mesh.service_metal,(anchor+rear)*.5f,right,{0,1,0},normal,{.055f,.055f,std::max(.02f,depth)});
            structure_display::box(mesh.service_metal,anchor-Vec3{0,0,sign*.018f},right,{0,1,0},normal,{.12f,.12f,.035f});
        }
    }
}
}
