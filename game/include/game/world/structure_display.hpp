#pragma once
#include "game/runtime/presentation.hpp"
#include <array>
#include <string>

namespace game::structure_display {
using runtime::Vec3;
struct Mount {
    Vec3 center{},right{1,0,0},up{0,1,0};
    float width{.90f},height{.48f};
    Vec3 normal() const {return runtime::normalize(runtime::cross(right,up));}
};
struct Content {
    std::string title;
    std::array<std::string,3> rows;
    runtime::Color accent{.27f,.82f,.90f,1};
};
inline void quad(std::vector<Vec3>& out,Vec3 c,Vec3 right,Vec3 up,float w,float h){
    const Vec3 a=c-right*(w*.5f)-up*(h*.5f),b=c+right*(w*.5f)-up*(h*.5f),d=c-right*(w*.5f)+up*(h*.5f),e=c+right*(w*.5f)+up*(h*.5f);
    out.insert(out.end(),{a,b,e,a,e,d});
}
inline void box(std::vector<Vec3>& out,Vec3 c,Vec3 right,Vec3 up,Vec3 normal,Vec3 size){
    quad(out,c+normal*(size.z*.5f),right,up,size.x,size.y);
    quad(out,c-normal*(size.z*.5f),right*-1,up,size.x,size.y);
    quad(out,c+right*(size.x*.5f),normal*-1,up,size.z,size.y);
    quad(out,c-right*(size.x*.5f),normal,up,size.z,size.y);
    quad(out,c+up*(size.y*.5f),right,normal*-1,size.x,size.z);
    quad(out,c-up*(size.y*.5f),right,normal,size.x,size.z);
}
inline void casing(std::vector<Vec3>& metal,std::vector<Vec3>& glass,const Mount& m){
    const auto n=m.normal();
    box(metal,m.center-n*.045f,m.right,m.up,n,{m.width+.09f,m.height+.09f,.08f});
    for(float sign:{-1.f,1.f}){
        box(metal,m.center+m.right*(sign*(m.width*.5f+.023f)),m.right,m.up,n,{.045f,m.height+.09f,.045f});
        box(metal,m.center+m.up*(sign*(m.height*.5f+.023f)),m.right,m.up,n,{m.width,.045f,.045f});
    }
    quad(glass,m.center,m.right,m.up,m.width,m.height);
}
std::string value(double number,int decimals=0);
// CPU glyph strokes are cached; text remains a depth-tested part of the model.
void draw(runtime::SceneRenderer&,const Mount&,Vec3 position,float rotation,const Content&,bool powered,Vec3 eye);
}
