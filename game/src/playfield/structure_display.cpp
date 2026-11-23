#include "game/world/structure_display.hpp"
#include "game/runtime/ui_font.hpp"
#include "game/util/world_math.hpp"
#include <iomanip>
#include <map>
#include <sstream>

namespace game::structure_display {
std::string value(double number,int decimals){
    if(!std::isfinite(number))return "--";
    std::ostringstream out;out<<std::fixed<<std::setprecision(decimals)<<number;return out.str();
}
namespace {
const std::vector<Vec3>& glyphs(const std::string& text){
    static std::map<std::string,std::vector<Vec3>> cache;
    if(const auto it=cache.find(text);it!=cache.end())return it->second;
    if(cache.size()>512)cache.clear();
    auto& out=cache[text];
    for(std::size_t character=0;character<text.size();++character){
        const auto it=runtime::ui::kBitmapFont.find(char(std::toupper(static_cast<unsigned char>(text[character]))));
        if(it==runtime::ui::kBitmapFont.end())continue;
        for(int y=0;y<7;++y)for(int x=0;x<5;){
            if(!(it->second[y]&(1<<(4-x)))){++x;continue;}
            const int start=x;while(x<5&&(it->second[y]&(1<<(4-x))))++x;
            quad(out,{float(character*6+start)+(x-start)*.5f,6.5f-y,0},{1,0,0},{0,1,0},float(x-start),1);
        }
    }
    return out;
}
}
void draw(runtime::SceneRenderer& r,const Mount& m,Vec3 position,float rotation,const Content& content,bool powered,Vec3 eye){
    const Vec3 center=world_math::rotate_y(m.center,rotation)+position;
    const Vec3 n=world_math::rotate_y(m.normal(),rotation),right=world_math::rotate_y(m.right,rotation),up=world_math::rotate_y(m.up,rotation);
    const float distance=runtime::length(eye-center);
    if(distance>24.f || runtime::dot(n,eye-center)<=0 || m.height/std::max(.1f,distance)<.012f)return;
    const auto line=[&](const std::string& text,int row,runtime::Color tint){
        if(text.empty())return;
        const auto& source=glyphs(text);
        const float scale=std::min(m.width*.91f/std::max(1.f,float(text.size()*6)),m.height/40.f);
        const float y=m.height*.38f-row*m.height*.245f;
        const Vec3 origin=center+n*.004f-right*(m.width*.455f)+up*(y-3.5f*scale);
        std::vector<Vec3> vertices;vertices.reserve(source.size());
        for(auto p:source)vertices.push_back(origin+right*(p.x*scale)+up*(p.y*scale));
        r.draw_lunar_surface(vertices,powered?runtime::LunarMaterial::DisplayText:runtime::LunarMaterial::Marking,tint);
    };
    line(content.title,0,powered?content.accent:runtime::Color{.52f,.56f,.58f,1});
    if(!powered){line("POWER OFF",2,{.55f,.57f,.58f,1});return;}
    for(int i=0;i<3;++i)line(content.rows[i],i+1,{.82f,.91f,.91f,1});
}
}
