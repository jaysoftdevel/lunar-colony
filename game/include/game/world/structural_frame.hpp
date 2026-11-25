#pragma once
#include "game/runtime/types.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace game::structural {
using namespace runtime;
inline constexpr float pi=3.14159265358979323846f;
struct Metrics {std::size_t members{},vertices{};unsigned frames{};float max_bay_m{};std::size_t first_vertex{};};

// Closed solid geometry, generated once with the model. Primary members have
// two flanges and a web; secondary members represent closed box sections.
struct Builder {
    std::vector<Vec3>& solid;std::vector<Vec3>& lines;Metrics metrics;
    void box(Vec3 c,Vec3 x,Vec3 y,Vec3 z,Vec3 size){
        x=x*(size.x*.5f);y=y*(size.y*.5f);z=z*(size.z*.5f);
        const Vec3 p[]{c-x-y-z,c+x-y-z,c+x+y-z,c-x+y-z,c-x-y+z,c+x-y+z,c+x+y+z,c-x+y+z};
        constexpr int faces[][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
        for(const auto& f:faces)solid.insert(solid.end(),{p[f[0]],p[f[2]],p[f[1]],p[f[0]],p[f[3]],p[f[2]]});
    }
    void beam(Vec3 a,Vec3 b,float width,float depth,Vec3 normal={0,1,0},bool primary=false){
        const float span=length(b-a);if(span<.001f)return;
        const auto axis=(b-a)/span;
        normal=normal-axis*dot(normal,axis);
        if(length(normal)<.01f){normal=std::abs(axis.y)<.9f?Vec3{0,1,0}:Vec3{0,0,1};normal=normal-axis*dot(normal,axis);}
        normal=normalize(normal);const auto side=normalize(cross(normal,axis));const auto c=(a+b)*.5f;
        if(primary){
            const float flange=std::clamp(depth*.045f,.005f,.012f);
            box(c,side,normal,axis,{std::clamp(width*.05f,.004f,.009f),depth-2*flange,span});
            for(float sign:{-1.f,1.f})box(c+normal*(sign*(depth-flange)*.5f),side,normal,axis,{width,flange,span});
        }else box(c,side,normal,axis,{width,depth,span});
        lines.insert(lines.end(),{a+normal*(depth*.5f),b+normal*(depth*.5f)});++metrics.members;
    }
    void plate(Vec3 c,Vec3 normal,float width=.34f,float height=.27f){
        normal=normalize(normal);const auto ref=std::abs(normal.y)<.9f?Vec3{0,1,0}:Vec3{0,0,1};
        const auto x=normalize(cross(ref,normal)),y=cross(normal,x);
        box(c,x,y,normal,{width,height,.012f});
        // Four raised hex bolt heads. Their shanks terminate in the doubler.
        for(float sx:{-1.f,1.f})for(float sy:{-1.f,1.f}){
            const auto center=c+x*(sx*width*.32f)+y*(sy*height*.30f)+normal*.008f;
            for(int i=0;i<6;++i){
                const float a=2*pi*i/6,b=2*pi*(i+1)/6;
                const auto p=center+(x*std::cos(a)+y*std::sin(a))*.018f;
                const auto q=center+(x*std::cos(b)+y*std::sin(b))*.018f;
                const auto d=normal*.012f;
                solid.insert(solid.end(),{p,q,q+d,p,q+d,p+d,center+d,p+d,q+d});
            }
        }
    }
    void bay(float span){metrics.max_bay_m=std::max(metrics.max_bay_m,span);}
};
} // namespace game::structural
