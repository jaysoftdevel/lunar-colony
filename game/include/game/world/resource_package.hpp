#pragma once
#include "game/systems/resource_management.hpp"
#include "game/runtime/types.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include <cstdint>
namespace game::packages {
inline constexpr float lift=.025f;
struct Box {
    runtime::Vec3 base{};float half{},height{};MaterialResource material{};
    float bottom() const{return base.y+lift;}
    float top() const{return bottom()+height;}
    bool intersects(runtime::Vec3 feet,float radius,float body_height) const {
        if(top()<=feet.y+.035f||bottom()>=feet.y+body_height-.025f)return false;
        const float dx=std::max(0.f,std::abs(feet.x-base.x)-half);
        const float dz=std::max(0.f,std::abs(feet.z-base.z)-half);
        return dx*dx+dz*dz<radius*radius;
    }
};
template<class Pile,class Emit> bool each(const Pile& pile,Emit emit){
    using Kind=decltype(pile.kind);
    const auto capacity=pile.kind==Kind::Loose?pile.amount:resource_package_capacity_kg(pile.resource);
    if(capacity<=0)return true;
    auto base=pile.position;
    for(auto left=pile.amount;left>0;left-=capacity){
        const auto kg=std::min(left,capacity);
        Box box{base,resource_package_half_width(pile.resource,kg),resource_package_height(pile.resource,kg),pile.resource};
        if(!emit(box))return false;
        base.y+=box.height+.02f;
    }
    return true;
}
inline const std::array<std::uint8_t,8>& icon(MaterialResource material){
    static constexpr std::array<std::array<std::uint8_t,8>,8> masks{{
        {{0x7e,0x7e,0x18,0x18,0x18,0x18,0x7e,0x7e}}, // iron: I beam
        {{0x7e,0x42,0x7e,0x42,0x7e,0x42,0x7e,0x42}}, // copper: winding
        {{0x24,0x7e,0x5a,0xdb,0xdb,0x5a,0x7e,0x24}}, // silicon: chip
        {{0x18,0x3c,0x24,0x42,0xc3,0x81,0xe7,0x7e}}, // plastic: recycling
        {{0x18,0x18,0x3c,0x3c,0x7e,0x7e,0x3c,0x18}}, // water: droplet
        {{0x02,0x0e,0x3e,0x7c,0xf8,0xf0,0xc0,0x80}}, // food: leaf
        {{0x18,0x18,0x18,0xff,0xff,0x18,0x18,0x18}}, // medical: cross
        {{0x00,0x7e,0x66,0x00,0x7e,0x66,0x00,0x7e}}  // aluminum: stacked sheets
    }};
    return masks.at(static_cast<std::size_t>(material));
}
inline void markings(const Box& box,std::vector<runtime::Vec3>& backing,std::vector<runtime::Vec3>& ink){
    using namespace runtime;
    const auto quad=[](auto& out,Vec3 c,Vec3 right,Vec3 up,float w,float h){
        const auto x=right*(w*.5f),y=up*(h*.5f);
        out.insert(out.end(),{c-x-y,c+x-y,c+x+y,c-x-y,c+x+y,c-x+y});
    };
    const Vec3 center=box.base+Vec3{0,box.height*.5f+lift,0};
    for(const Vec3 n:std::array<Vec3,6>{{{1,0,0},{-1,0,0},{0,0,1},{0,0,-1},{0,1,0},{0,-1,0}}}){
        const Vec3 up=n.y==0?Vec3{0,1,0}:Vec3{0,0,-n.y};
        const auto right=cross(up,n);
        const float extent=n.y==0?box.half:box.height*.5f;
        const float w=box.half*1.38f,h=n.y==0?std::min(box.height*.72f,w):w;
        const auto c=center+n*(extent+.003f);
        quad(backing,c,right,up,w,h);
        const float step=std::min(w,h)*.10f;
        const auto& mask=icon(box.material);
        for(int row=0;row<8;++row)for(int col=0;col<8;++col)if(mask[row]&(1u<<(7-col)))
            quad(ink,c+n*.0015f+right*((col-3.5f)*step)+up*((3.5f-row)*step),right,up,step*1.01f,step*1.01f);
    }
}
}

