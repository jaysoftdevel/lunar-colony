#include "game/world/playfield.hpp"
namespace game {
void Playfield::rebuild(int size){
    size_=size;
    float h=size_*0.5f;
    floor_tris_={{-h,0,-h},{h,0,-h},{h,0,h},{-h,0,-h},{h,0,h},{-h,0,h}};
    grid_lines_.clear();
    int step=1; if(size_>128)step=2; if(size_>256)step=4;
    for(int i=-size_/2;i<=size_/2;i+=step){
        float v=(float)i;
        grid_lines_.push_back({-h,0.01f,v}); grid_lines_.push_back({h,0.01f,v});
        grid_lines_.push_back({v,0.01f,-h}); grid_lines_.push_back({v,0.01f,h});
    }
}
}
