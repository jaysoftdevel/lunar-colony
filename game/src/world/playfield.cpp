#include "game/world/playfield.hpp"
#include <algorithm>
#include <cmath>

namespace game {
void Playfield::rebuild(int size){
    size_=size;
    float h=size_*0.5f;
    floor_tris_={{-h,0,-h},{h,0,h},{h,0,-h},{-h,0,-h},{-h,0,h},{h,0,h}};
    grid_lines_.clear();
    int step=1; if(size_>128)step=2; if(size_>256)step=4;
    for(int i=-size_/2;i<=size_/2;i+=step){
        float v=(float)i;
        grid_lines_.push_back({-h,0.01f,v}); grid_lines_.push_back({h,0.01f,v});
        grid_lines_.push_back({v,0.01f,-h}); grid_lines_.push_back({v,0.01f,h});
    }
}
void Playfield::draw_surface(eng::Renderer& renderer,float daylight) const {
    // Retain enough ambient contrast to read the surface on the night side.
    const float light=.52f+.48f*std::sqrt(std::clamp(daylight,0.f,1.f));
    renderer.draw_lunar_surface(floor_tris_,eng::LunarMaterial::Regolith,{.40f*light,.39f*light,.37f*light,1.f});
}
}
